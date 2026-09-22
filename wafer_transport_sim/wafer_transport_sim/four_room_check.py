"""Launch and observe the real four-room simulation on Ubuntu, with fault cases."""
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
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
import yaml
from .common import DemoNode, LATCHED
from .core import placed
from .four_room_layout import ROOMS, ROOM_CODES, START

RECOVERY_SCENARIOS = {'lost_line', 'missing_ir', 'missing_images', 'missing_room_qr'}

class Observer(DemoNode):
    def __init__(self, scenario):
        super().__init__('four_room_check')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.scenario = scenario
        self.history = []
        self.qrs = set()
        self.placements = set()
        self.frames = {'camera': 0, 'down_camera': 0}
        self.injected = False
        self.finished = None
        self.errors = []
        self.bridge = CvBridge()
        for topic in ('/transport/state', '/rooms/active', '/rooms/completed', '/system/fault',
                      '/transport/steering_mode'):
            self.watch(topic, String, LATCHED)
        self.watch('/wafer_delivered', Bool, LATCHED)
        self.watch('/cmd_vel', Twist)
        self.cycles = set()
        self.saw_simulated_ir = False
        self.watch_attachment('vacuum')
        self.watch_attachment('carrier')
        for name in ('wafer', 'carrier', 'transport_robot'):
            self.watch_pose(name)
        self.watch_joints('/robot/joint_states')
        for room in ROOMS:
            for side in ('entry', 'exit'):
                self.watch_joints(f'/doors/room_{room.number}/{side}/joint_states')
        self.create_subscription(String, '/detected_station', lambda m: self.qrs.add(m.data), 10)
        self.out = {name: self.create_publisher(Image, f'/{name}/image_raw', qos_profile_sensor_data)
                    for name in self.frames}
        for name in self.frames:
            self.create_subscription(Image, f'/four_room_test/{name}/image_raw',
                                     lambda msg, n=name: self.image(n, msg), qos_profile_sensor_data)

    def image(self, name, msg):
        self.frames[name] += 1
        if self.value('/transport/state') == 'ENTER_ROOM' and self.scenario in {'lost_line', 'missing_images'}:
            self.injected = True
        if self.injected and self.scenario == 'missing_images':
            return
        if ((self.injected and self.scenario == 'lost_line' and name == 'down_camera') or
                (self.scenario == 'missing_room_qr' and name == 'camera')):
            pixels = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            pixels[:] = 255
            new = self.bridge.cv2_to_imgmsg(pixels, 'bgr8')
            new.header = msg.header
            msg = new
            self.injected = True
        self.out[name].publish(msg)

    def check(self):
        room, state = self.value('/rooms/active'), self.value('/transport/state')
        if self.value('/transport/steering_mode') == 'SIMULATED_IR':
            self.saw_simulated_ir = True
            if self.scenario in {'lost_line', 'missing_ir'}:
                self.injected = True
        if self.scenario == 'lost_line':
            self.send('/ir/healthy', Bool, True)
            self.send('/ir/valid', Bool, False)
            self.pub('/ir/cmd_vel', Twist).publish(Twist())
        cmd = self.values.get('/cmd_vel', Twist())
        if state in {'OPEN_ENTRY', 'HOLD_ENTRY', 'CLOSE_ENTRY', 'OPEN_EXIT', 'HOLD_EXIT', 'CLOSE_EXIT'}:
            if abs(cmd.linear.x)+abs(cmd.angular.z) > 1e-5:
                self.errors.append('Motion commanded during door cycle')
        if state in {'HOLD_ENTRY', 'HOLD_EXIT'} and room in ROOM_CODES:
            side = 'entry' if state == 'HOLD_ENTRY' else 'exit'
            target = ROOMS[ROOM_CODES.index(room)].door_joint(side)
            if self.at_joint(target, .52):
                self.cycles.add((room, side))
        if room and state and (not self.history or self.history[-1] != [room, state]):
            self.history.append([room, state])
        if state == 'VERIFY_PLACE' and room in ROOM_CODES and self.fresh('/poses/wafer') and self.attachment('vacuum') == 'detached':
            if placed(self.position('wafer'), ROOMS[ROOM_CODES.index(room)].table, .012, .005):
                self.placements.add((room, 'table'))
            cx, cy, cz = self.position('carrier')
            if self.fresh('/poses/carrier') and placed(self.position('wafer'), (cx, cy, cz+.006), .012, .005):
                self.placements.add((room, 'carrier'))
        if self.scenario == 'failed_pickup' and state == 'ATTACH':
            self.injected = True
        if self.scenario == 'stuck_door' and state == 'OPEN_ENTRY':
            self.injected = True
        if self.value('/system/fault') or self.value('/wafer_delivered'):
            if self.finished is None:
                self.finished = self.now()
            # Continue for two simulation seconds to check stable support and stop.
            if self.value('/wafer_delivered') and not placed(self.position('wafer'), ROOMS[-1].table, .012, .005):
                self.errors.append('Wafer left final supporting table after delivery')
            return self.now()-self.finished >= 2.
        return False

    def report(self):
        if self.scenario == 'nominal' or self.scenario in RECOVERY_SCENARIOS:
            if self.value('/transport/state') != 'COMPLETE' or not self.value('/wafer_delivered'):
                self.errors.append('Four-room mission did not complete')
            if self.value('/system/fault'):
                self.errors.append('Unexpected mission fault: ' + self.value('/system/fault'))
            if self.value('/rooms/completed') != ','.join(ROOM_CODES):
                self.errors.append('Rooms were not completed in order')
            if self.scenario in RECOVERY_SCENARIOS and not self.injected:
                self.errors.append('Recovery scenario was never exercised')
            if self.scenario in {'lost_line', 'missing_ir'} and not self.saw_simulated_ir:
                self.errors.append('Simulated IR recovery was not observed')
            if self.cycles != {(r.code, side) for r in ROOMS for side in ('entry', 'exit')}:
                self.errors.append('Not all eight door openings were verified')
            required = {('ROOM_1', 'carrier'), ('ROOM_2', 'table'), ('ROOM_2', 'carrier'),
                        ('ROOM_3', 'table'), ('ROOM_3', 'carrier'), ('ROOM_4', 'table')}
            if not required <= self.placements:
                self.errors.append('Missing observed supported wafer placements')
            if not self.fresh('/poses/transport_robot') or not placed(self.position('transport_robot'), (*START, 0.), .04, .02):
                self.errors.append('Robot did not return outside all four rooms')
            if self.attachment('vacuum') != 'detached':
                self.errors.append('Final wafer still attached to wand')
            if self.attachment('carrier') != 'attached':
                self.errors.append('Onboard carrier attachment lost')
            for room in ROOMS:
                if [room.code, 'CLOSE_EXIT'] not in self.history:
                    self.errors.append('No verified exit sequence for ' + room.code)
                for side in ('entry', 'exit'):
                    if not self.at_joint(room.door_joint(side), 0.):
                        self.errors.append('Door not closed: ' + room.door_joint(side))
        else:
            if not self.injected:
                self.errors.append('Fault injection stage was never reached')
            if not self.value('/system/fault'):
                self.errors.append('Expected a reported fault')
            if self.value('/wafer_delivered'):
                self.errors.append('False delivery success after injected failure')
        command = self.values.get('/cmd_vel', Twist())
        if (not self.fresh('/cmd_vel') or
                abs(command.linear.x)+abs(command.linear.y)+abs(command.angular.z) > 1e-5):
            self.errors.append('Final zero velocity command missing')
        return dict(passed=not self.errors, scenario=self.scenario, errors=sorted(set(self.errors)),
                    states=self.history, qrs=sorted(self.qrs), placements=sorted(self.placements),
                    frames=self.frames, door_cycles=sorted(self.cycles),
                    simulated_ir=self.saw_simulated_ir, fault=self.value('/system/fault'))


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=['nominal', 'lost_line', 'missing_ir', 'missing_images', 'missing_room_qr',
                                              'failed_pickup', 'stuck_door'], default='nominal')
    parser.add_argument('--timeout', type=float, default=1800., help='Wall-clock limit; VMs may render slowly')
    parser.add_argument('--gui', action='store_true')
    parser.add_argument('--output', default='log/four_rooms')
    opts = parser.parse_args(args)
    output = Path(opts.output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    share = Path(get_package_share_directory('wafer_transport_sim'))
    mappings = yaml.safe_load((share/'config/four_rooms_bridge.yaml').read_text())
    if opts.scenario in {'missing_images', 'missing_room_qr'}:
        removed = {'/camera/image_raw'}
        if opts.scenario == 'missing_images':
            removed.add('/down_camera/image_raw')
        mappings = [m for m in mappings if m['ros_topic_name'] not in removed]
    for entry in mappings:
        if entry['ros_topic_name'] in ('/camera/image_raw', '/down_camera/image_raw'):
            entry['ros_topic_name'] = '/four_room_test' + entry['ros_topic_name']
    omitted = {'failed_pickup': '/attachments/vacuum/attach', 'stuck_door': '/actuators/room_1_entry_z'}.get(opts.scenario)
    mappings = [m for m in mappings if m['ros_topic_name'] != omitted]
    rclpy.init()
    node = Observer(opts.scenario)
    if opts.scenario in {'missing_images', 'missing_room_qr'}:
        node.injected = True  # Camera bridges were omitted from this launch.
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix='wafer-four-rooms-') as temp:
            bridge = Path(temp)/'bridge.yaml'
            bridge.write_text(yaml.safe_dump(mappings))
            with (output/(opts.scenario+'.log')).open('w') as log:
                process = subprocess.Popen(['ros2', 'launch', 'wafer_transport_sim', 'demo.launch.py',
                    'layout:=four_rooms', f'gui:={str(opts.gui).lower()}', f'bridge_config:={bridge}',
                    f'ir_enabled:={str(opts.scenario not in {"lost_line", "missing_ir"}).lower()}'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic()+opts.timeout
                while time.monotonic() < deadline and process.poll() is None:
                    rclpy.spin_once(node, timeout_sec=.05)
                    if node.check():
                        break
                else:
                    node.errors.append('Launch exited or wall-clock test timeout')
                report = node.report()
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
    (output/(opts.scenario+'.json')).write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if report['passed'] else 1)
