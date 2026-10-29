"""Simulation hardware adapter: camera deviation in, chassis velocity out."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


WHEEL_SEPARATION = 0.13583  # Raspbot V2 MecanumDrive setting, metres
CAMERA_TIMEOUT = 0.4


class Hardware:
    def __init__(self, node):
        self.error = None
        self.received_at = None
        self.odom = None
        self.publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        node.create_subscription(Float32, '/line/offset_normalized', self._on_error, 10)
        node.create_subscription(Odometry, '/odom', self._on_odom, 10)

    def _on_error(self, message):
        self.error = message.data if math.isfinite(message.data) else None
        self.received_at = time.monotonic()

    def read_line_error(self):
        if self.received_at is None or time.monotonic() - self.received_at > CAMERA_TIMEOUT:
            return None
        return self.error

    def line_age(self):
        return time.monotonic() - self.received_at if self.received_at is not None else float('nan')

    def _on_odom(self, message):
        self.odom = message

    def set_motors(self, left, right):
        """Convert left/right wheel-side speeds (m/s) to Gazebo's Twist input."""
        command = Twist()
        command.linear.x = (left + right) / 2
        command.angular.z = (right - left) / WHEEL_SEPARATION
        self.publisher.publish(command)
        return command

    def stop(self):
        command = Twist()
        self.publisher.publish(command)
        return command

    def cleanup(self):
        self.stop()
