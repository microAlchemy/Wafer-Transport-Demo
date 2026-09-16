"""Ubuntu integration runner: launches Gazebo and asserts observed behavior.

Run through ros2 run. Image fault injection is confined to this test process.
Actual QR pixels and physical simulation remain the source of mission evidence.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import yaml
from .common import DemoNode, LATCHED
from .core import placed


class Observer(DemoNode):
    def __init__(self, destination, scenario):
        super().__init__('integration_check')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.destination = destination
        self.scenario = scenario
        self.history = {'/wafer_handler/state': [], '/transport/state': []}
        self.qrs = set()
        self.frames = {'camera': 0, 'down_camera': 0}
        self.errors = []
        self.drop_error = None
        self.fault_seen_wall = None
        self.complete_wall = None
        self.injected = False
        self.bridge = CvBridge()
        self.watch('/carrier_ready', Bool, LATCHED)
        self.watch('/carrier_delivered', Bool, LATCHED)
        self.watch('/system/state', String, LATCHED)
        self.watch('/system/fault', String, LATCHED)
        self.watch('/cmd_vel', Twist)
        self.watch('/odom', Odometry)
        self.watch_pose('carrier')
        self.watch_pose('wafer')
        for topic in self.history:
            self.create_subscription(String, topic, lambda msg, t=topic: self.state_msg(t, msg), LATCHED)
        self.create_subscription(String, '/detected_station', lambda m: self.qrs.add(m.data), 10)
        self.out = {name: self.create_publisher(Image, f'/{name}/image_raw', qos_profile_sensor_data)
                    for name in self.frames}
        for name in self.frames:
            self.create_subscription(Image, f'/test/{name}/image_raw',
                                     lambda m, n=name: self.image(n, m), qos_profile_sensor_data)

    def state_msg(self, topic, msg):
        if not self.history[topic] or self.history[topic][-1] != msg.data:
            self.history[topic].append(msg.data)
        self.values[topic] = msg

    def image(self, name, msg):
        self.frames[name] += 1
        moving = self.value('/transport/state') in {'FOLLOW_PATH', 'SCAN_QR', 'CONTINUE'}
        if moving:
            self.injected = True
        if self.injected and self.scenario == 'missing_images':
            return
        erase = ((self.scenario == 'lost_line' and self.injected and name == 'down_camera') or
                 (self.scenario == 'missing_target' and name == 'camera'))
        if erase:
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            image[:] = 255
            replacement = self.bridge.cv2_to_imgmsg(image, 'bgr8')
            replacement.header = msg.header
            msg = replacement
        self.out[name].publish(msg)

    def check(self):
        cmd = self.values.get('/cmd_vel', Twist())
        moving = abs(cmd.linear.x) + abs(cmd.angular.z) > 1e-5
        if moving and not self.value('/carrier_ready'):
            self.errors.append('Motion before carrier_ready')
        if self.value('/transport/state') == 'DROP_CARRIER' and self.drop_error is None:
            positions = {'STATION_A': .28, 'STATION_B': .60, 'STATION_C': .92}
            if '/odom' in self.values:
                self.drop_error = abs(self.values['/odom'].pose.pose.position.x - positions[self.destination])
        if self.value('/system/fault') and self.fault_seen_wall is None:
            self.fault_seen_wall = time.monotonic()
        if self.value('/system/state') == 'COMPLETE' and self.complete_wall is None:
            self.complete_wall = time.monotonic()
        # Keep observing after success to verify physical support after release.
        if self.complete_wall and time.monotonic() - self.complete_wall > 2:
            return True
        if self.fault_seen_wall and time.monotonic() - self.fault_seen_wall > 2:
            return True
        return False

    def verdict(self):
        if self.scenario != 'nominal':
            if not self.value('/system/fault'):
                self.errors.append('Expected a fault')
            if self.value('/carrier_delivered'):
                self.errors.append('False delivery success during injected failure')
            cmd = self.values.get('/cmd_vel', Twist())
            if not self.fresh('/cmd_vel') or abs(cmd.linear.x) + abs(cmd.angular.z) > 1e-5:
                self.errors.append('Missing or nonzero stop command after failure')
        else:
            if self.value('/system/state') != 'COMPLETE':
                self.errors.append('Mission did not complete')
            if self.destination not in self.qrs:
                self.errors.append('Target QR was not observed')
            if self.destination == 'STATION_C' and 'CONTINUE' not in self.history['/transport/state']:
                self.errors.append('Default mission did not reject a non-target QR')
            if self.drop_error is None or self.drop_error > .0101:
                self.errors.append('Docking error exceeds 10 mm or drop never occurred')
            y = {'STATION_A': .4, 'STATION_B': .72, 'STATION_C': 1.04}[self.destination]
            if not placed(self.position('carrier'), (1.145, y, .134)):
                self.errors.append('Carrier not supported on destination shelf')
            if not placed(self.position('wafer'), (1.145, y, .140), .022, .010):
                self.errors.append('Wafer not retained inside delivered carrier')
            expected = ['HOME', 'MOVE_TO_WAFER', 'LOWER_WAND', 'VACUUM_ON',
                        'VERIFY_VACUUM', 'LIFT_PICKUP', 'MOVE_TO_BOX', 'LOWER',
                        'VACUUM_OFF', 'LIFT_RETRACT', 'TRANSFER_COMPLETE']
            states = self.history['/wafer_handler/state']
            if [s for s in states if s != 'IDLE'] != expected:
                self.errors.append('Wafer state order incomplete or incorrect')
        if not all(self.frames.values()):
            self.errors.append('One or both Gazebo cameras never produced an image')
        return {'passed': not self.errors, 'destination': self.destination,
                'scenario': self.scenario, 'errors': sorted(set(self.errors)),
                'states': self.history, 'qr_ids': sorted(self.qrs),
                'image_frames': self.frames, 'dock_error_m': self.drop_error,
                'fault': self.value('/system/fault')}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', choices=['STATION_A', 'STATION_B', 'STATION_C'], default='STATION_C')
    parser.add_argument('--scenario', choices=['nominal', 'missing_images', 'lost_line',
                                              'failed_pickup', 'missing_target'], default='nominal')
    parser.add_argument('--timeout', type=float, default=420)
    parser.add_argument('--output', default='log/integration')
    opts = parser.parse_args(args)
    output = Path(opts.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    stem = f'{opts.destination}_{opts.scenario}'
    share = Path(get_package_share_directory('wafer_transport_sim'))
    config = yaml.safe_load((share / 'config/bridge.yaml').read_text())
    for entry in config:
        if entry['ros_topic_name'] in ('/camera/image_raw', '/down_camera/image_raw'):
            entry['ros_topic_name'] = '/test' + entry['ros_topic_name']
    if opts.scenario == 'failed_pickup':
        config = [entry for entry in config if entry['ros_topic_name'] != '/attachments/vacuum/attach']
    rclpy.init()
    node = Observer(opts.destination, opts.scenario)
    process = None
    report = None
    try:
        with tempfile.TemporaryDirectory(prefix='wafer-integration-') as directory:
            bridge_file = Path(directory) / 'bridge.yaml'
            bridge_file.write_text(yaml.safe_dump(config))
            with (output / (stem + '.log')).open('w') as log:
                process = subprocess.Popen([
                    'ros2', 'launch', 'wafer_transport_sim', 'demo.launch.py',
                    'gui:=false', f'destination_station:={opts.destination}',
                    f'bridge_config:={bridge_file}'], stdout=log, stderr=subprocess.STDOUT,
                    start_new_session=True)
                deadline = time.monotonic() + opts.timeout
                while time.monotonic() < deadline and process.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.05)
                    if node.check():
                        break
                else:
                    node.errors.append('Launch exited early or wall-clock integration timeout')
                report = node.verdict()
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        node.destroy_node()
        rclpy.shutdown()
    (output / (stem + '.json')).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)
