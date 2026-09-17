"""Four-room asset, real-pixel vision and controller regression checks.

The kinematic harness has ideal wheels, joints and support; it is not Gazebo.
"""
import math
from pathlib import Path
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET
import cv2
import numpy as np
import pytest
import yaml
from test_controllers import ros, Scalar, Twist, Odometry, pose
from wafer_transport_sim.four_room_layout import ROOMS, ROOM_CODES, PATH, START, LENGTH, project, room_stop, door_clear
from wafer_transport_sim.vision import line_command, station_codes
from wafer_transport_sim.core import DetectionFilter, clamp

ROOT = Path(__file__).resolve().parents[1]


def floor_image(x, y, heading):
    # Pinhole image of the actual 18 mm tape, at the SDF camera pose.
    image = np.full((240, 320, 3), 240, dtype=np.uint8)
    focal = 160/math.tan(1.3/2)
    c, s = math.cos(heading), math.sin(heading)
    points = []
    for px, py in PATH:
        forward = (px-x)*c + (py-y)*s - .13
        left = -(px-x)*s + (py-y)*c
        points.append((round(160-left*focal/.0807), round(120-forward*focal/.0807)))
    cv2.polylines(image, [np.int32(points)], False, (5, 5, 5), round(.018*focal/.0807))
    return image


def test_loop_layout_and_separate_door_bridges():
    assert math.dist(PATH[0], PATH[-1]) < 1e-9
    assert 10 < LENGTH < 12
    stops = [room_stop(r, p) for r in ROOMS for p in ('entry', 'work', 'exit')]
    assert stops == sorted(stops)
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')
    assert len(world.findall('.//include')) == 11  # robot, carrier, wafer, eight doors
    names = [i.findtext('name') for i in world.findall('.//include')]
    assert len(names) == len(set(names))
    assert len([m for m in world.findall('.//model') if m.get('name').startswith('room_')]) == 4
    mappings = yaml.safe_load((ROOT/'config/four_rooms_bridge.yaml').read_text())
    topics = {m['ros_topic_name']: m for m in mappings}
    assert len(topics) == len(mappings)
    for room in ROOMS:
        for side in ('entry', 'exit'):
            path = ROOT/f'models/room_{room.number}_{side}_door/model.sdf'
            door = ET.parse(path)
            joint = room.door_joint(side)
            assert door.find(f".//joint[@name='{joint}']") is not None
            assert topics['/actuators/'+joint]['direction'] == 'ROS_TO_GZ'
            assert topics[f'/doors/room_{room.number}/{side}/joint_states']['direction'] == 'GZ_TO_ROS'


@pytest.mark.parametrize('room', ROOMS)
def test_room_qr_decodes_from_approach(room):
    qr = cv2.imread(str(ROOT/f'textures/room_{room.number}_qr.png'))
    assert room.code in station_codes(cv2.QRCodeDetector(), qr, 55.)
    # Project the actual planar sign when 8 cm before the work stop. Its normal
    # faces back along travel, so horizontal texture U points toward image right.
    focal = 480/math.tan(1.7/2)
    depth = .28+.08-.095
    corners = [[480+focal*(offset-.14)/depth, 360-focal*z/depth]
               for offset, z in [(-.08, .08), (.08, .08), (.08, -.08), (-.08, -.08)]]
    h, w = qr.shape[:2]
    matrix = cv2.getPerspectiveTransform(np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]]), np.float32(corners))
    image = cv2.warpPerspective(qr, matrix, (960, 720), borderValue=(240, 240, 240))
    assert room.code in station_codes(cv2.QRCodeDetector(), image, 55.)
    confirmation = DetectionFilter(3, ROOM_CODES)
    assert [confirmation.update(room.code) for _ in range(3)] == ['', '', room.code]


def test_doors_do_not_close_over_oriented_robot():
    for room in ROOMS:
        for side in ('entry', 'exit'):
            x = room.door_x(side)
            assert not door_clear(x, room.y, room.heading, x)
            assert not door_clear(x+.1, room.y, math.pi/4, x)
            assert door_clear(x+.20, room.y, room.heading, x)


@pytest.mark.parametrize('room', ROOMS)
def test_door_swept_volume_clear_of_room_and_frame(room):
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')

    def bounds(link, collision, origin=(0., 0., 0.)):
        lp = list(map(float, link.findtext('pose').split()))
        cp = list(map(float, collision.findtext('pose').split()))
        assert lp[3:] == cp[3:] == [0., 0., 0.]
        size = list(map(float, collision.findtext('geometry/box/size').split()))
        center = [origin[i]+lp[i]+cp[i] for i in range(3)]
        return ([center[i]-size[i]/2 for i in range(3)], [center[i]+size[i]/2 for i in range(3)])

    obstacles = []
    for link in world.findall('.//model/link'):
        for collision in link.findall('collision'):
            obstacles.append((collision.get('name'), bounds(link, collision)))
    for side in ('entry', 'exit'):
        door = ET.parse(ROOT/f'models/room_{room.number}_{side}_door/model.sdf')
        origin = (room.door_x(side), room.y-.155, 0.)
        frame, panel = door.find(".//link[@name='frame']"), door.find(".//link[@name='panel']")
        fixed = obstacles + [(c.get('name'), bounds(frame, c, origin)) for c in frame.findall('collision')]
        for collision in panel.findall('collision'):
            lo, hi = bounds(panel, collision, origin)
            lo[2] -= .004
            hi[2] += .535
            for name, (flo, fhi) in fixed:
                assert not all(min(hi[i], fhi[i])-max(lo[i], flo[i]) > 1e-9 for i in range(3)), name


class Harness:
    def __init__(self, bus):
        self.bus = bus
        self.node = bus.four_room_controller.FourRoomController()
        self.node.params['state_timeout'] = 60.
        self.x, self.y = START
        self.heading = 0.
        self.joints = dict.fromkeys(('reach_1', 'reach_2', 'wand_x', 'wand_z', 'tray_z', 'tray_y'), 0.)
        self.joints.update({r.door_joint(side): 0. for r in ROOMS for side in ('entry', 'exit')})
        self.vacuum, self.carrier = 'detached', 'attached'
        self.wafer = ROOMS[0].table
        self.cmd = Twist()
        self.history = []
        self.releases = []
        self.automatic_qr = True
        self.failed_door = None
        self.failed_pickup = False
        self.blank = False

    def feedback(self):
        b = self.bus
        odom = Odometry()
        odom.twist.twist = self.cmd
        # Deliberately biased wheel yaw; room alignment uses a consistent world frame.
        odom.pose.pose.orientation.z = math.sin((self.heading+.25)/2)
        odom.pose.pose.orientation.w = math.cos((self.heading+.25)/2)
        b.emit('/odom', odom)
        b.emit('/poses/transport_robot', pose(self.x, self.y, 0., self.heading))
        b.emit('/poses/carrier', pose(self.x, self.y, .15+self.joints['tray_z'], self.heading))
        b.emit('/poses/wafer', pose(*self.wafer))
        b.emit('/robot/joint_states', NS(name=list(self.joints), position=list(self.joints.values())))
        for r in ROOMS:
            for side in ('entry', 'exit'):
                name = r.door_joint(side)
                b.emit(f'/doors/room_{r.number}/{side}/joint_states', NS(name=[name], position=[self.joints[name]]))
        b.emit('/attachments/vacuum/state', Scalar(self.vacuum))
        b.emit('/attachments/carrier/state', Scalar(self.carrier))
        result = None if self.blank else line_command(floor_image(self.x, self.y, self.heading), .08, .004, .6, 55, .35)
        line = Twist()
        if result:
            line.linear.x, line.angular.z = result
        b.emit('/line/healthy', Scalar(True))
        b.emit('/line/valid', Scalar(result is not None))
        b.emit('/line/cmd_vel', line)
        b.emit('/qr/healthy', Scalar(True))
        if self.automatic_qr and self.node.state in {'ENTER_ROOM', 'CONFIRM_ROOM'}:
            b.emit('/detected_station', Scalar(self.node.room.code))

    def advance(self):
        b, n = self.bus, self.node
        dt = .05
        self.x += self.cmd.linear.x*math.cos(self.heading)*dt
        self.y += self.cmd.linear.x*math.sin(self.heading)*dt
        self.heading += self.cmd.angular.z*dt
        for name in self.joints:
            target = b.outputs.get('/actuators/'+name, Scalar(self.joints[name])).data
            if name != self.failed_door:
                self.joints[name] += clamp(target-self.joints[name], -.1*dt, .1*dt)
        # Commands are events. Consume once so an old detach cannot override a
        # later attach. Support/attachment motion here is ideal, not physics.
        for name in ('carrier', 'vacuum'):
            for action in ('detach', 'attach'):
                if b.outputs.pop(f'/attachments/{name}/{action}', None) is not None:
                    if name == 'vacuum' and action == 'attach' and self.failed_pickup:
                        continue
                    setattr(self, name, 'attached' if action == 'attach' else 'detached')
                    if name == 'vacuum' and action == 'detach' and n.state == 'RELEASE':
                        self.releases.append((n.room.code, n.destination))
        if self.vacuum == 'attached':
            reach = sum(self.joints[k] for k in ('reach_1', 'reach_2', 'wand_x'))
            self.wafer = (self.x+reach*math.cos(self.heading), self.y+reach*math.sin(self.heading),
                          .295+self.joints['wand_z'])
        elif n.payload == 'carrier':
            self.wafer = (self.x, self.y, .156+self.joints['tray_z'])
        elif n.payload == 'table':
            self.wafer = n.payload_table
        b.time += dt
        self.feedback()
        n.tick()
        if not self.history or self.history[-1] != (n.room.code, n.state):
            self.history.append((n.room.code, n.state))
        self.cmd = b.outputs['/cmd_vel']


def test_full_four_room_loop_with_camera_steering(ros):
    h = Harness(ros)
    for _ in range(12000):
        h.advance()
        assert h.node.state != 'FAULT', (h.history[-5:], ros.outputs.get('/system/fault').data,
                                        h.x, h.y, h.heading)
        if h.node.state == 'COMPLETE':
            break
    assert h.node.state == 'COMPLETE', h.history[-5:]
    assert h.node.completed == list(ROOM_CODES)
    assert ros.outputs['/wafer_delivered'].data
    assert math.dist((h.x, h.y), START) < .015
    assert h.wafer == ROOMS[-1].table
    for r in ROOMS:
        assert (r.code, 'CLOSE_EXIT') in h.history
        for side in ('entry', 'exit'):
            assert h.joints[r.door_joint(side)] == pytest.approx(0., abs=.002)
    assert set(h.releases) == {('ROOM_1', 'carrier'), ('ROOM_2', 'table'), ('ROOM_2', 'carrier'),
                               ('ROOM_3', 'table'), ('ROOM_3', 'carrier'), ('ROOM_4', 'table')}


@pytest.mark.parametrize('failure', ['lost_line', 'stale_image', 'stale_pose', 'stuck_door', 'missing_qr', 'failed_pickup'])
def test_four_room_failures_stop_without_delivery(ros, failure):
    h = Harness(ros)
    target = {'stuck_door': 'OPEN_ENTRY', 'missing_qr': 'CONFIRM_ROOM', 'failed_pickup': 'ATTACH'}.get(failure, 'ENTER_ROOM')
    if failure == 'missing_qr':
        h.automatic_qr = False
    for _ in range(2000):
        h.advance()
        if h.node.state == target:
            break
        assert h.node.state != 'FAULT'
    assert h.node.state == target
    if failure == 'lost_line':
        ros.emit('/line/valid', Scalar(False))
    elif failure == 'stale_image':
        h.node.received['/line/healthy'] -= 2.
    elif failure == 'stale_pose':
        h.node.received['/poses/transport_robot'] -= 2.
    else:
        if failure == 'missing_qr':
            ros.emit('/detected_station', Scalar('ROOM_4'))
            assert not h.node.confirmed
        # A timeout is a failure, never a substitute for attachment/door/QR evidence.
        h.node.entered = ros.time - 61.
    h.node.tick()
    assert h.node.state == 'FAULT'
    assert ros.outputs['/cmd_vel'].linear.x == ros.outputs['/cmd_vel'].angular.z == 0.
    assert not ros.outputs['/wafer_delivered'].data


@pytest.mark.parametrize('failure', ['door', 'vacuum'])
def test_unacknowledged_actuator_never_advances(ros, failure):
    h = Harness(ros)
    if failure == 'door':
        h.failed_door = ROOMS[0].door_joint('entry')
    else:
        h.failed_pickup = True
    blocked = 'OPEN_ENTRY' if failure == 'door' else 'ATTACH'
    for _ in range(3000):
        h.advance()
        if h.node.state == 'FAULT':
            break
    assert h.node.state == 'FAULT'
    assert ('ROOM_1', blocked) in h.history
    assert not h.node.completed
    assert not ros.outputs['/wafer_delivered'].data
    assert ros.outputs['/cmd_vel'].linear.x == 0.
    assert not any(state == ('ENTER_ROOM' if failure == 'door' else 'LIFT') for _, state in h.history)


def test_final_delivery_requires_supported_wafer_and_low_velocity(ros):
    h = Harness(ros)
    n = h.node
    n.index = 3
    n.completed = list(ROOM_CODES)
    n.payload_table = ROOMS[-1].table
    h.feedback()
    n.transition('VERIFY_DELIVERY')
    for _ in range(30):
        ros.time += .05
        h.feedback()
        ros.emit('/poses/wafer', pose(0., 0., 0.))
        n.tick()
        assert not ros.outputs['/wafer_delivered'].data
    h.wafer = ROOMS[-1].table
    h.cmd.linear.x = .02
    for _ in range(30):
        ros.time += .05
        h.feedback()
        n.tick()
        assert not ros.outputs['/wafer_delivered'].data


def test_launch_keeps_command_owners_exclusive():
    # Confirm the branch is exclusive without requiring ROS launch on this Mac.
    import ast
    launch = ast.parse((ROOT/'launch/demo.launch.py').read_text())
    assignment = next(n for n in ast.walk(launch) if isinstance(n, ast.Assign) and
                      any(isinstance(t, ast.Name) and t.id == 'executables' for t in n.targets))
    assert isinstance(assignment.value, ast.IfExp)
    new = ast.literal_eval(assignment.value.body)
    legacy = ast.literal_eval(assignment.value.orelse)
    assert 'four_room_controller' in new and 'transport_controller' not in new
    assert 'transport_controller' in legacy and 'four_room_controller' not in legacy
