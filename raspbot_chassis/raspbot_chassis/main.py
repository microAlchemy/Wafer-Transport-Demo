"""Simple camera PID with a CSV trace for measured tuning."""

import csv
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node

from .hardware import Hardware
from .pid import PID


BASE_SPEED = 0.02       # metres per second; deliberately slow for camera tracking
MAX_CORRECTION = 0.02   # maximum left/right speed correction, m/s
KP, KI, KD = 0.04, 0.0, 0.002


class LineController(Node):
    def __init__(self):
        super().__init__('line_controller')
        for name, value in [('speed', BASE_SPEED), ('max_correction', MAX_CORRECTION),
                            ('kp', KP), ('ki', KI), ('kd', KD), ('steering_sign', 1.0),
                            ('telemetry_file', str(Path.home() / 'raspbot_pid.csv'))]:
            self.declare_parameter(name, value)
        self.speed = float(self.get_parameter('speed').value)
        self.max_correction = float(self.get_parameter('max_correction').value)
        self.steering_sign = float(self.get_parameter('steering_sign').value)
        if self.speed <= 0 or self.max_correction < 0 or self.steering_sign not in (-1.0, 1.0):
            raise ValueError('speed must be positive, max_correction nonnegative, steering_sign ±1')
        self.hardware = Hardware(self)
        self.pid = PID(*(float(self.get_parameter(name).value) for name in ('kp', 'ki', 'kd')))
        path = Path(self.get_parameter('telemetry_file').value).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        self.telemetry = path.open('w', newline='')
        self.writer = csv.writer(self.telemetry)
        self.writer.writerow(['time_s', 'status', 'error', 'line_age_s', 'p', 'i', 'd',
                              'correction_mps', 'speed_cmd_mps', 'yaw_cmd_rad_s',
                              'yaw_odom_rad_s', 'x_odom_m', 'y_odom_m',
                              'kp', 'ki', 'kd', 'steering_sign'])
        self.start = self.previous = time.monotonic()
        self.get_logger().info(f'PID telemetry: {path}')
        self.create_timer(0.05, self.tick)

    def tick(self):
        now = time.monotonic()
        dt = now - self.previous
        self.previous = now
        error = self.hardware.read_line_error()
        if error is None:
            self.pid.reset()
            command = self.hardware.stop()
            self.record(float('nan'), 0.0, command, 'lost')
            return
        raw_error = error
        if abs(error) < 0.03:
            error = 0.0
        correction = self.steering_sign * self.pid.update(error, dt)
        correction = max(-self.max_correction, min(self.max_correction, correction))
        # Positive camera error means tape right: left wheels run faster.
        command = self.hardware.set_motors(self.speed + correction, self.speed - correction)
        self.record(raw_error, correction, command, 'tracking')

    def record(self, error, correction, command, status):
        odom = self.hardware.odom
        if odom is None:
            yaw_odom = x_odom = y_odom = math.nan
        else:
            yaw_odom = odom.twist.twist.angular.z
            x_odom = odom.pose.pose.position.x
            y_odom = odom.pose.pose.position.y
        self.writer.writerow([
            time.monotonic() - self.start, status, error, self.hardware.line_age(),
            self.pid.kp * (self.pid.prev_error or 0.0), self.pid.ki * self.pid.integral,
            self.pid.kd * self.pid.derivative, correction, command.linear.x,
            command.angular.z, yaw_odom, x_odom, y_odom,
            self.pid.kp, self.pid.ki, self.pid.kd, self.steering_sign,
        ])
        self.telemetry.flush()

    def destroy_node(self):
        try:
            self.hardware.cleanup()
        finally:
            self.telemetry.close()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
