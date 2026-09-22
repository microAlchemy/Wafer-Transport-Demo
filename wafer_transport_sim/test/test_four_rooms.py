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
from wafer_transport_sim.four_room_layout import (ROOMS, ROOM_CODES, PATH, START, LENGTH,
                                                  ROOM_LENGTH, ROOM_DEPTH, ROOM_HEIGHT,
                                                  CORRIDOR_WIDTH, project, room_stop, door_clear)
from wafer_transport_sim.vision import line_command, station_codes
from wafer_transport_sim.core import DetectionFilter, clamp
from wafer_transport_sim.raspbot_spec import CAMERA_OFFSET, CAMERA_PIVOT, MISSION_CAMERA_TILT

ROOT = Path(__file__).resolve().parents[1]


def floor_image(x, y, heading):
    # Pinhole image of the actual 18 mm tape, at the SDF camera pose.
    image = np.full((240, 320, 3), 240, dtype=np.uint8)
    focal = 160/math.tan(1.8/2)
    c, s = math.cos(heading), math.sin(heading)
    radius = max(1, round(.018*focal/.041/2))
    # Rasterize only the local ground footprint. Projecting an entire closed
    # loop as one polyline incorrectly draws behind-camera segments across the
    # image when the route contains several tight exterior turns.
    for a, b in zip(PATH, PATH[1:]):
        count = max(2, math.ceil(math.dist(a, b)/.004))
        for i in range(count+1):
            t = i/count
            px, py = a[0]+t*(b[0]-a[0]), a[1]+t*(b[1]-a[1])
            forward = (px-x)*c + (py-y)*s - .070
            left = -(px-x)*s + (py-y)*c
            if abs(forward) <= .06 and abs(left) <= .06:
                point = (round(160-left*focal/.041), round(120-forward*focal/.041))
                cv2.circle(image, point, radius, (5, 5, 5), -1)
    return image


def test_loop_layout_and_separate_door_bridges():
    assert math.dist(PATH[0], PATH[-1]) < 1e-9
    assert 10 < LENGTH < 30
    stops = [room_stop(r, p) for r in ROOMS for p in ('entry', 'work', 'exit')]
    assert stops == sorted(stops)
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')
    floor = world.find(".//collision[@name='floor_collision']/geometry/box/size")
    fx, fy, _ = map(float, floor.text.split())
    assert all(abs(x)+.15 < fx/2 and abs(y)+.15 < fy/2 for x, y in PATH)
    assert ROOM_LENGTH == pytest.approx(4*.3048)
    assert ROOM_DEPTH == pytest.approx(3*.3048)
    assert ROOM_HEIGHT == pytest.approx(4*.3048)
    assert ROOMS[1].x-ROOMS[0].x-ROOM_LENGTH >= CORRIDOR_WIDTH
    assert ROOMS[0].y-ROOMS[3].y-ROOM_DEPTH >= CORRIDOR_WIDTH
    for room in ROOMS:
        for x, y in PATH:
            dx = max(abs(x-room.x)-ROOM_LENGTH/2, 0.)
            dy = max(abs(y-room.y)-ROOM_DEPTH/2, 0.)
            assert math.hypot(dx, dy) >= CORRIDOR_WIDTH/2-1e-9
    source_pose = list(map(float, world.findtext(".//include[name='wafer']/pose").split()))
    assert source_pose[:2] == list(ROOMS[0].table[:2])
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


def test_raspbot_ir_array_centres_and_steers_toward_tape(ros):
    ir = ros.ir_line_follower
    centered = ir.probe_values(*START, 0.)
    assert centered[1] == pytest.approx(centered[2])
    assert ir.steering(centered)[1] == pytest.approx(0.)
    below_tape = ir.probe_values(START[0], START[1]-.012, 0.)
    assert ir.steering(below_tape)[1] > 0.  # tape is to the robot's left


@pytest.mark.parametrize('room', ROOMS)
@pytest.mark.parametrize('before_stop', [0., .08, .20])
def test_room_qr_decodes_from_approach(room, before_stop):
    qr = cv2.imread(str(ROOT/f'textures/room_{room.number}_qr.png'))
    assert room.code in station_codes(cv2.QRCodeDetector(), qr, 55.)
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')
    sign = world.find(f".//model[@name='room_{room.number}']//visual[@name='room_qr']")
    sx, sy, sz, *_ = map(float, sign.findtext('pose').split())
    sw, sh = map(float, sign.findtext('geometry/plane/size').split())
    robot = ET.parse(ROOT/'models/raspbot_v2/model.sdf')
    camera = robot.find(".//sensor[@name='forward_camera']/camera")
    iw, ih = (int(camera.findtext('image/'+name)) for name in ('width', 'height'))
    focal = iw/2/math.tan(float(camera.findtext('horizontal_fov'))/2)
    q = MISSION_CAMERA_TILT
    camera_forward = CAMERA_PIVOT[0] + math.cos(q)*CAMERA_OFFSET
    camera_height = CAMERA_PIVOT[2] - math.sin(q)*CAMERA_OFFSET
    forward = room.direction*(sx-(room.x-room.direction*before_stop))-camera_forward
    lateral = room.direction*(sy-room.route_y)
    vertical = sz-camera_height
    depth = math.cos(q)*forward-math.sin(q)*vertical
    camera_z = math.sin(q)*forward+math.cos(q)*vertical
    corners = [[iw/2-focal*(lateral+u)/depth, ih/2-focal*(camera_z+v)/depth]
               for u, v in [(-sw/2, sh/2), (sw/2, sh/2), (sw/2, -sh/2), (-sw/2, -sh/2)]]
    h, w = qr.shape[:2]
    matrix = cv2.getPerspectiveTransform(np.float32([[0, 0], [w-1, 0], [w-1, h-1], [0, h-1]]), np.float32(corners))
    image = cv2.warpPerspective(qr, matrix, (iw, ih), borderValue=(240, 240, 240))
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
        origin = (room.door_x(side), room.y-(CORRIDOR_WIDTH/2+.015), 0.)
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
        cfg = yaml.safe_load((ROOT/'config/four_rooms.yaml').read_text())
        self.node.params.update(cfg['/**']['ros__parameters'])
        self.node.params.update(cfg['transport_controller']['ros__parameters'])
        self.line_speed = cfg['line_follower']['ros__parameters']['linear_speed']
        self.line_crop = cfg['line_follower']['ros__parameters']['camera_crop_ratio']
        self.line_kp = cfg['line_follower']['ros__parameters']['kp']
        self.line_max_angular = cfg['line_follower']['ros__parameters']['max_angular_speed']
        self.x, self.y = START
        self.heading = 0.
        self.joints = dict.fromkeys(('reach_1', 'reach_2', 'wand_x', 'wand_z', 'tray_z', 'tray_y',
                                    'camera_pan', 'camera_tilt'), 0.)
        self.joints.update({r.door_joint(side): 0. for r in ROOMS for side in ('entry', 'exit')})
        self.vacuum, self.carrier = 'detached', 'attached'
        self.wafer = ROOMS[0].table
        self.cmd = Twist()
        self.applied = Twist()
        self.history = []
        self.releases = []
        self.automatic_qr = True
        self.failed_door = None
        self.stuck_joints = set()
        self.failed_pickup = False
        self.blank = False
        self.ir_failure = None
        self.door_cycles = set()

    def feedback(self):
        b = self.bus
        odom = Odometry()
        odom.twist.twist = self.applied
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
        # No camera or QR messages at all: the mission must depend only on IR
        # steering plus physics feedback for doors, payload and stops.
        ir_values = b.ir_line_follower.probe_values(self.x, self.y, self.heading)
        ir_result = b.ir_line_follower.steering(ir_values, self.line_speed, 25., 1.4, .09)
        ir_command = Twist()
        if ir_result:
            ir_command.linear.x, ir_command.angular.z = ir_result
        if self.ir_failure != 'missing':
            b.emit('/ir/healthy', Scalar(True))
            b.emit('/ir/valid', Scalar(ir_result is not None and self.ir_failure != 'lost'))
            b.emit('/ir/cmd_vel', Twist() if self.ir_failure == 'zero' else ir_command)

    def advance(self):
        b, n = self.bus, self.node
        dt = .05
        # Match the Raspbot MecanumDrive acceleration limit in its generated SDF.
        self.applied.linear.x += clamp(self.cmd.linear.x-self.applied.linear.x, -.8*dt, .8*dt)
        self.applied.angular.z += clamp(self.cmd.angular.z-self.applied.angular.z, -1.5*dt, 1.5*dt)
        self.x += self.applied.linear.x*math.cos(self.heading)*dt
        self.y += self.applied.linear.x*math.sin(self.heading)*dt
        self.heading += self.applied.angular.z*dt
        for name in self.joints:
            target = b.outputs.get('/actuators/'+name, Scalar(self.joints[name])).data
            if name != self.failed_door and name not in self.stuck_joints:
                speed = .20 if name.startswith('room_') else .1
                self.joints[name] += clamp(target-self.joints[name], -speed*dt, speed*dt)
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
        if n.state in {'OPEN_ENTRY', 'HOLD_ENTRY', 'CLOSE_ENTRY', 'OPEN_EXIT', 'HOLD_EXIT', 'CLOSE_EXIT'}:
            assert n.stopped(), n.state
            assert self.cmd.linear.x == self.cmd.angular.z == 0.
        if n.state in {'HOLD_ENTRY', 'HOLD_EXIT'}:
            side = 'entry' if n.state == 'HOLD_ENTRY' else 'exit'
            assert self.joints[n.room.door_joint(side)] == pytest.approx(.52, abs=.002)
            self.door_cycles.add((n.room.code, side))
        if self.cmd.linear.x > 0.:
            assert all(abs(self.joints[r.door_joint(side)]) <= .002
                       for r in ROOMS for side in ('entry', 'exit'))


@pytest.mark.parametrize('ir_failure', [None, 'missing', 'lost', 'zero'])
def test_full_four_room_loop_with_ir_steering(ros, ir_failure):
    h = Harness(ros)
    h.ir_failure = ir_failure
    h.automatic_qr = False
    for _ in range(12000):
        h.advance()
        assert h.node.state != 'FAULT', (h.history[-5:], ros.outputs.get('/system/fault').data,
                                        h.x, h.y, h.heading)
        if h.node.state == 'COMPLETE':
            break
    assert h.node.state == 'COMPLETE', h.history[-5:]
    assert h.node.completed == list(ROOM_CODES)
    assert ros.outputs['/wafer_delivered'].data
    assert math.dist((h.x, h.y), START) < .04
    assert h.wafer == ROOMS[-1].table
    assert h.door_cycles == {(r.code, side) for r in ROOMS for side in ('entry', 'exit')}
    for r in ROOMS:
        states = [state for code, state in h.history if code == r.code]
        assert states.index('OPEN_ENTRY') < states.index('HOLD_ENTRY') < states.index('CLOSE_ENTRY') < states.index('ENTER_ROOM')
        assert states.index('EXIT_ROOM') < states.index('OPEN_EXIT') < states.index('HOLD_EXIT') < states.index('CLOSE_EXIT')
        assert (r.code, 'CLOSE_EXIT') in h.history
        for side in ('entry', 'exit'):
            assert h.joints[r.door_joint(side)] == pytest.approx(0., abs=.002)
    assert set(h.releases) == {('ROOM_1', 'carrier'), ('ROOM_2', 'table'), ('ROOM_2', 'carrier'),
                               ('ROOM_3', 'table'), ('ROOM_3', 'carrier'), ('ROOM_4', 'table')}


def test_cosmetic_camera_and_tray_joints_do_not_block_driving(ros):
    h = Harness(ros)
    h.joints.update(camera_pan=.31, camera_tilt=-.44, tray_y=.003, tray_z=.001)
    h.stuck_joints.update(('camera_pan', 'camera_tilt', 'tray_y', 'tray_z'))
    for _ in range(300):
        h.advance()
        if h.node.state == 'APPROACH_ENTRY':
            break
    assert h.node.state == 'APPROACH_ENTRY'
    h.advance()
    assert h.cmd.linear.x > 0.


@pytest.mark.parametrize('failure', ['lost_line', 'stale_ir', 'stale_pose', 'stuck_door', 'failed_pickup'])
def test_four_room_failures_stop_without_delivery(ros, failure):
    h = Harness(ros)
    h.node.params['allow_sensor_fallback'] = False
    target = {'stuck_door': 'OPEN_ENTRY', 'failed_pickup': 'ATTACH'}.get(failure, 'ENTER_ROOM')
    for _ in range(2000):
        h.advance()
        if h.node.state == target:
            break
        assert h.node.state != 'FAULT'
    assert h.node.state == target
    if failure == 'lost_line':
        ros.emit('/ir/valid', Scalar(False))
    elif failure == 'stale_ir':
        h.node.received['/ir/healthy'] -= 2.
    elif failure == 'stale_pose':
        h.node.received['/poses/transport_robot'] -= 2.
    else:
        # A timeout is a failure, never a substitute for attachment/door evidence.
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
    h.applied.linear.x = .02
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
    assert 'ir_line_follower' in new
    assert 'line_follower' not in new and 'qr_detector' not in new
    assert 'transport_controller' in legacy and 'four_room_controller' not in legacy


def test_ir_steered_entry_has_no_camera_subscriptions(ros):
    h = Harness(ros)
    h.automatic_qr = False
    for _ in range(1200):
        h.advance()
        if h.node.state == 'ENTER_ROOM':
            break
    assert h.node.state == 'ENTER_ROOM'
    h.node.tick()
    assert h.node.state == 'ENTER_ROOM'
    assert ros.outputs['/cmd_vel'].linear.x > 0.
    assert '/qr/healthy' not in ros.subscribers
    assert '/line/cmd_vel' not in ros.subscribers
    assert '/detected_station' not in ros.subscribers


@pytest.mark.parametrize('failure', ['missing', 'lost', 'zero'])
def test_mid_route_ir_failure_switches_to_simulated_tracking_and_recovers(ros, failure):
    h = Harness(ros)
    for _ in range(2000):
        h.advance()
        if h.node.state == 'ENTER_ROOM':
            break
        assert h.node.state != 'FAULT'
    assert h.node.state == 'ENTER_ROOM'
    h.ir_failure = failure
    if failure == 'missing':
        h.node.received['/ir/cmd_vel'] -= 2.
    h.feedback()
    h.node.tick()
    assert h.node.state == 'ENTER_ROOM'
    assert ros.outputs['/transport/steering_mode'].data == 'SIMULATED_IR'
    assert ros.outputs['/cmd_vel'].linear.x > 0.
    h.ir_failure = None
    h.feedback()
    h.node.tick()
    assert ros.outputs['/transport/steering_mode'].data == 'IR'
    assert not ros.outputs['/wafer_delivered'].data


def test_configured_speed_reaches_wheel_command_and_brakes_near_stop(ros):
    h = Harness(ros)
    h.joints['tray_z'] = .004
    h.feedback()
    n = h.node
    n.state = 'APPROACH_ENTRY'
    n.drive(1.)
    assert n.command.linear.x == pytest.approx(.20)
    n.command = Twist()
    n.drive(.05)
    assert .07 <= n.command.linear.x < .20
    n.command = Twist()
    n.drive(.003)
    assert n.command.linear.x == 0.
