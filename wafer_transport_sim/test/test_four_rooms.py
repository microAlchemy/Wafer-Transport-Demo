"""Ten-glovebox U-shaped two-track assets, AprilTags and controller regression checks.

The kinematic harness has ideal wheels, joints and support; it is not Gazebo.
Marker visibility is checked by projecting the generated marker geometry
through the generated camera intrinsics, which is static evidence, not a
rendered-camera measurement.
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
from wafer_transport_sim.core import clamp
from wafer_transport_sim.four_room_layout import (
    CORRIDOR_WIDTH, DECISION_OFFSET, EDGES, JUNCTION_OFFSET, LANE_PITCH, MARKER_HEIGHT,
    MARKER_SIZE, MISSION_LENGTH, PATH, PROCESS_CONTACT_Z, PROCESS_TRAVEL,
    PROCESS_TRANSIT_WAFER_Z, ROOMS, ROOM_CODES, ROOM_DEPTH, ROOM_HEIGHT, ROOM_LENGTH, START,
    TAPE_SEGMENTS, TRACK_1_Y, TRACK_2_Y, TURN_X, MAIN_PATH, SERVICE_PATH, HOME_EDGE, TAIL_EDGE, edge_point, edge_project,
    entry_decision_stop, entry_junction_x, entry_rung, exit_junction_x, exit_rung,
    oriented_door_clear, main_edge, marker_pose, room_stop, route_plan, service_edge,
    tag_decision_stop, tape_distance)
from wafer_transport_sim.raspbot_spec import (CAMERA_OFFSET, CAMERA_PIVOT,
                                              LENGTH as ROBOT_LENGTH, WIDTH as ROBOT_WIDTH)

ROOT = Path(__file__).resolve().parents[1]
TRANSFER_STATES = {'PICK_POSITION', 'PICK_LOWER', 'ATTACH', 'LIFT', 'PLACE_POSITION',
                   'PLACE_LOWER', 'RELEASE', 'RETRACT', 'VERIFY_PLACE'}
# Conservative rotating envelope of the published Raspbot chassis.
TURN_ENVELOPE = math.hypot(ROBOT_LENGTH/2, ROBOT_WIDTH/2)
# Door leaf and its guide posts, read from the generated door geometry.
DOOR_HALF_OPENING = CORRIDOR_WIDTH/2
DOOR_POST_HALF_DEPTH = .013


def tape_visuals(world):
    return [v for v in world.findall(".//model[@name='loop_floor']//visual")
            if v.get('name', '').startswith('tape_')]


def forward_camera():
    """Forward camera pose in the robot frame, from the generated model.

    SDF link poses are model-relative (none of these links sets `relative_to`),
    so the mount is the `camera_tilt_link` pose plus the sensor pose inside that
    link.  The pan link is where the pan servo sits; at zero pan/tilt it is not
    part of the camera translation chain.
    """
    model = ET.parse(ROOT/'models/raspbot_v2/model.sdf')

    def pose_of(path):
        return list(map(float, model.findtext(path + '/pose').split()))

    base = pose_of(".//link[@name='base_link']")
    pan = pose_of(".//link[@name='camera_pan_link']")
    tilt = list(map(float, model.findtext(".//link[@name='camera_tilt_link']/pose").split()))
    sensor = list(map(float, model.findtext(".//sensor[@name='forward_camera']/pose").split()))
    # The mission commands neither camera joint, so both stay at their zero
    # position and the joint frames coincide with their child link frames.
    for joint in ('camera_pan', 'camera_tilt'):
        assert model.find(f".//joint[@name='{joint}']/pose") is None, joint
    # Cross-check the modelled parts against the documented Raspbot spec.
    assert tilt[:3] == pytest.approx(CAMERA_PIVOT)
    assert sensor[:3] == pytest.approx((CAMERA_OFFSET, 0., 0.))
    assert tilt[3:6] == [0., 0., 0.] and sensor[3:6] == [0., 0., 0.]
    # Model-relative frames: the base link and the pan servo are separate
    # placements, not translations to add to the tilt-plus-lens mount.
    x, y, z = tilt[0]+sensor[0], tilt[1]+sensor[1], tilt[2]+sensor[2]
    assert (x, y, z) == pytest.approx((CAMERA_PIVOT[0]+CAMERA_OFFSET, 0., CAMERA_PIVOT[2]))
    assert base[2] < z and pan[2] < z and pan[0] < x
    camera = model.find(".//sensor[@name='forward_camera']/camera")
    width = int(camera.findtext('image/width'))
    height = int(camera.findtext('image/height'))
    hfov = float(camera.findtext('horizontal_fov'))
    return x, y, z, width, height, hfov


def project_marker(camera, robot, marker):
    """Project the marker corners, in committed-texture order, into the image.

    Corner order matches `textures/room_N_side_apriltag.png`: (u, v) = (0, 0),
    (1, 0), (1, 1), (0, 1).  The marker faces back down the approach lane, so
    +u runs along world -y and +v along world -z.
    """
    cam_x, cam_y, cam_z, width, height, hfov = camera
    x, y, heading = robot
    cx = x + cam_x*math.cos(heading)
    cy = y + cam_x*math.sin(heading)
    cz = cam_z
    focal = (width/2)/math.tan(hfov/2)
    mx, my, mz, normal = marker
    half = MARKER_SIZE/2
    yaw = normal-math.pi
    ux, uy = math.sin(yaw), -math.cos(yaw)
    corners = [(mx-ux*half, my-uy*half, mz+half),
               (mx+ux*half, my+uy*half, mz+half),
               (mx+ux*half, my+uy*half, mz-half),
               (mx-ux*half, my-uy*half, mz-half)]
    projected = []
    c, s = math.cos(heading), math.sin(heading)
    for px, py, pz in corners:
        forward = (px-cx)*c + (py-cy)*s
        left = -(px-cx)*s + (py-cy)*c
        up = pz-cz
        projected.append((width/2 - left*focal/forward, height/2 - up*focal/forward, forward))
    return projected, focal


def test_two_track_tape_network_geometry():
    assert len(ROOMS) == 10
    assert [round(r.yaw, 6) for r in ROOMS] == [round(math.pi, 6)]*4 + \
        [round(-math.pi/2, 6)]*2 + [0.]*4
    assert len(TAPE_SEGMENTS) == 6+2*len(ROOMS)
    assert tuple(a for a, b in TAPE_SEGMENTS[:3]) == MAIN_PATH[:-1]
    assert tuple(b for a, b in TAPE_SEGMENTS[:3]) == MAIN_PATH[1:]
    assert tuple(a for a, b in TAPE_SEGMENTS[3:6]) == SERVICE_PATH[:-1]
    assert tuple(b for a, b in TAPE_SEGMENTS[3:6]) == SERVICE_PATH[1:]
    assert LANE_PITCH == pytest.approx(CORRIDOR_WIDTH)
    for room in ROOMS:
        assert entry_rung(room).length == pytest.approx(CORRIDOR_WIDTH)
        assert exit_rung(room).length == pytest.approx(CORRIDOR_WIDTH)
        assert service_edge(room).length == pytest.approx(2*JUNCTION_OFFSET)
        assert main_edge(room).length > DECISION_OFFSET
        assert entry_rung(room).points[0] == pytest.approx(room.lane_point(-JUNCTION_OFFSET, 1))
        assert exit_rung(room).points[-1] == pytest.approx(room.lane_point(JUNCTION_OFFSET, 1))
    plan = route_plan()
    assert len(plan) == 4*len(ROOMS)+2
    assert plan[-2:] == ('main_tail', 'main_home')
    assert len(set(plan)) == len(plan)
    assert MISSION_LENGTH == pytest.approx(sum(EDGES[name].length for name in plan))
    assert 48 < MISSION_LENGTH < 53


def test_geometry_keeps_robot_lane_and_door_clearance():
    assert ROOM_LENGTH == pytest.approx(4*.3048)
    assert ROOM_DEPTH == pytest.approx(3*.3048)
    assert ROOM_HEIGHT == pytest.approx(4*.3048)
    for room in ROOMS:
        for track in (1, 2):
            for along in (-JUNCTION_OFFSET, JUNCTION_OFFSET):
                px, py = room.lane_point(along, track)
                # Project to the rotated room frame: even the service tape is
                # outside the front wall, with room for the Raspbot chassis.
                lx = (px-room.x)*math.cos(room.yaw)+(py-room.y)*math.sin(room.yaw)
                ly = -(px-room.x)*math.sin(room.yaw)+(py-room.y)*math.cos(room.yaw)
                assert abs(lx) < ROOM_LENGTH/2
                assert ly <= -ROOM_DEPTH/2-CORRIDOR_WIDTH/2+1e-8
                assert oriented_door_clear(px, py, room.work_heading, room, 'entry')
        assert room.door_position('entry') != room.door_position('exit')
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')
    floor = world.find(".//collision[@name='floor_collision']/geometry/box/size")
    fx, fy, _ = map(float, floor.text.split())
    assert all(abs(x+.65)+.15 < fx/2 and abs(y)+.15 < fy/2
               for segment in TAPE_SEGMENTS for x, y in segment)


def test_world_tape_and_markers_match_the_shared_geometry():
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')
    visuals = tape_visuals(world)
    assert len(visuals) == len(TAPE_SEGMENTS)
    drawn = []
    for visual in visuals:
        values = list(map(float, visual.findtext('pose').split()))
        size = list(map(float, visual.findtext('geometry/box/size').split()))
        drawn.append((round(values[0], 6), round(values[1], 6), round(values[5], 6),
                      round(size[0], 3), round(size[1], 3), visual.findtext('material/diffuse')))
    expected = []
    for a, b in TAPE_SEGMENTS:
        length = math.dist(a, b)
        expected.append((round((a[0]+b[0])/2, 6), round((a[1]+b[1])/2, 6),
                         round(math.atan2(b[1]-a[1], b[0]-a[0]), 6),
                         round(length+.001, 3), .018, '.005 .005 .005 1'))
    assert sorted(drawn) == sorted(expected)
    room = world.find(".//model[@name='room_1']")
    for side in ('entry', 'exit'):
        sign_x, sign_y, sign_z, _ = marker_pose(ROOMS[0], side)
        visual = next(v for v in room.findall(".//visual")
                      if v.get('name') == side+'_apriltag')
        values = list(map(float, visual.findtext('pose').split()))
        # textured_plane stores the yaw plus its own quarter-turn.
        assert values[:3] == pytest.approx([sign_x, sign_y, sign_z])
        assert list(map(float, visual.findtext('geometry/plane/size').split())) == \
            pytest.approx([MARKER_SIZE, MARKER_SIZE])
        assert visual.findtext('material/pbr/metal/albedo_map') == \
            f'../textures/room_1_{side}_apriltag.png'
    assert len(world.findall('.//include')) == 33
    names = [i.findtext('name') for i in world.findall('.//include')]
    assert len(names) == len(set(names))
    assert len([m for m in world.findall('.//model') if m.get('name', '').startswith('room_')]) == 10
    mappings = yaml.safe_load((ROOT/'config/four_rooms_bridge.yaml').read_text())
    topics = {m['ros_topic_name']: m for m in mappings}
    assert len(topics) == len(mappings)
    for room in ROOMS:
        for side in ('entry', 'exit'):
            door = ET.parse(ROOT/f'models/room_{room.number}_{side}_door/model.sdf')
            joint = room.door_joint(side)
            assert door.find(f".//joint[@name='{joint}']") is not None
            assert topics['/actuators/'+joint]['direction'] == 'ROS_TO_GZ'
            assert topics[f'/doors/room_{room.number}/{side}/joint_states']['direction'] == 'GZ_TO_ROS'
        assert (ROOT/f'models/room_{room.number}_process_robot/model.sdf').exists()
        assert topics[f'/attachments/process_{room.number}/state']['direction'] == 'GZ_TO_ROS'


def test_raspbot_ir_array_centres_and_steers_toward_tape(ros):
    ir = ros.ir_line_follower
    centered = ir.probe_values(*START, ROOMS[0].yaw)
    assert centered[1] == pytest.approx(centered[2])
    assert ir.steering(centered)[1] == pytest.approx(0.)
    offset = ir.probe_values(START[0], START[1]-.012, ROOMS[0].yaw)
    assert ir.steering(offset)[1] < 0.  # tape is on robot's right


def test_ir_probes_reflect_both_lanes_and_the_transverse_junctions(ros):
    ir = ros.ir_line_follower
    for room in ROOMS:
        for track in (1, 2):
            x, y = room.lane_point(0., track)
            values = ir.probe_values(x, y, room.yaw)
            assert ir.steering(values) is not None
        a, b = entry_rung(room).points
        middle = ((a[0]+b[0])/2, (a[1]+b[1])/2)
        assert tape_distance(*middle) == pytest.approx(0., abs=1e-8)
        assert max(ir.probe_values(*middle, room.yaw+math.pi/2)) > .5


@pytest.mark.parametrize('room', ROOMS)
@pytest.mark.parametrize('side,offset', [('entry', 0), ('exit', 1)])
def test_real_apriltag_textures_decode(room, side, offset):
    image = cv2.imread(str(ROOT/f'textures/room_{room.number}_{side}_apriltag.png'))
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
        cv2.aruco.DetectorParameters())
    _, ids, _ = detector.detectMarkers(image)
    assert ids.flatten().tolist() == [2*(room.number-1)+offset]


@pytest.mark.parametrize('room', ROOMS)
@pytest.mark.parametrize('side', ['entry', 'exit'])
def test_marker_is_visible_from_the_pre_junction_decision_stop(room, side):
    """Static projection of the generated marker through the generated camera.

    This is geometry evidence for the Ubuntu VM: contact dynamics and the
    rendered 60 Hz camera still have to be validated there.
    """
    camera = forward_camera()
    stop = tag_decision_stop(room, side)
    marker = marker_pose(room, side)
    projected, focal = project_marker(camera, stop, marker)
    width, height = camera[3], camera[4]
    for u, v, forward in projected:
        assert forward > .1, (room.number, side, forward)
        assert 0. < u < width and 0. < v < height, (room.number, side, u, v)
    sides = [math.dist(projected[i][:2], projected[(i+1) % 4][:2]) for i in range(4)]
    # 36h11 needs several pixels per module. The 20 cm marker is ~0.3 m in
    # front of the mounted lens at the decision stop, which leaves well over
    # 6 px per module in this static projection.
    assert min(sides) >= 85., (room.number, side, min(sides))
    assert max(sides) <= 260.
    assert focal > 200.
    # The marker stands in front of the decision point, before the junction.
    junction = entry_junction_x(room) if side == 'entry' else exit_junction_x(room)
    assert (marker[0]-stop[0])*math.cos(room.yaw) + (marker[1]-stop[1])*math.sin(room.yaw) > 0
    assert abs(marker[2]-MARKER_HEIGHT) < 1e-9


def test_marker_lead_clears_the_decision_window():
    for room in ROOMS:
        for side in ('entry', 'exit'):
            stop = tag_decision_stop(room, side)
            marker = marker_pose(room, side)
            assert .35 < math.dist(stop[:2], marker[:2]) < .60
            assert entry_decision_stop(room) > .12
            assert room_stop(room, 'exit') > DECISION_OFFSET


def test_overhead_preview_draws_two_lanes_and_transverse_junctions(tmp_path):
    import importlib.util
    from PIL import Image
    spec = importlib.util.spec_from_file_location('preview_two_track', ROOT/'scripts'/'preview_two_track.py')
    preview = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preview)
    path = tmp_path/'overhead.png'
    info = preview.render(path, scale=100)
    image = Image.open(path).convert('RGB')
    pixels = image.load()
    def black(x, y):
        col = round((x-info['left'])*info['scale'])
        row = round((info['top']-y)*info['scale'])
        return max(pixels[col, row]) <= 10
    for a, b in TAPE_SEGMENTS:
        assert black((a[0]+b[0])/2, (a[1]+b[1])/2)
    assert image.size[0] > 700 and image.size[1] > 600


def test_doors_do_not_close_over_oriented_robot():
    for room in ROOMS:
        for side in ('entry', 'exit'):
            x, y = room.door_position(side)
            assert not oriented_door_clear(x, y, room.work_heading, room, side)
            px, py = room.lane_point(0., 2)
            assert oriented_door_clear(px, py, room.work_heading, room, side)


@pytest.mark.parametrize('room', ROOMS)
def test_door_models_are_rotated_into_front_wall(room):
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')

    for side in ('entry', 'exit'):
        include = world.find(f".//include[name='room_{room.number}_{side}_door']")
        values = list(map(float, include.findtext('pose').split()))
        assert values[5] == pytest.approx(room.yaw+math.pi/2)
        door = ET.parse(ROOT/f'models/room_{room.number}_{side}_door/model.sdf')
        assert door.find(f".//joint[@name='{room.door_joint(side)}']") is not None


class Harness:
    def __init__(self, bus):
        self.bus = bus
        self.node = bus.four_room_controller.FourRoomController()
        cfg = yaml.safe_load((ROOT/'config/four_rooms.yaml').read_text())
        self.node.params.update(cfg['/**']['ros__parameters'])
        self.node.params.update(cfg['transport_controller']['ros__parameters'])
        ir_cfg = cfg['ir_line_follower']['ros__parameters']
        self.line_speed = ir_cfg['linear_speed']
        self.line_crop = .35
        self.line_kp = ir_cfg['kp']
        self.line_max_angular = ir_cfg['max_angular_speed']
        self.x, self.y = START
        self.heading = ROOMS[0].yaw
        self.joints = dict.fromkeys(('reach_1', 'reach_2', 'wand_x', 'wand_z', 'tray_z', 'tray_y',
                                    'camera_pan', 'camera_tilt'), 0.)
        self.joints.update({r.door_joint(side): 0. for r in ROOMS for side in ('entry', 'exit')})
        self.vacuum, self.carrier = 'detached', 'attached'
        self.wafer = (self.x, self.y, .156)
        self.cmd = Twist()
        self.applied = Twist()
        self.history = []
        self.releases = []
        self.automatic_tags = True
        self.tag_wrong = None
        self.tag_override = {}
        self.tag_sides = ('entry', 'exit')
        self.tags_emitted = []
        self.route = []
        self.ir_override = None
        self.failed_door = None
        self.stuck_joints = set()
        self.failed_pickup = False
        self.blank = False
        self.ir_failure = None
        self.door_cycles = set()
        self.cells_detached = True

    def emit_tags(self, b):
        if not self.automatic_tags:
            return
        states = self.node.state
        for side in self.tag_sides:
            room = self.node.room
            if self.tag_wrong and self.tag_wrong[0] == side:
                room = ROOMS[self.tag_wrong[1]-1]
            tag = self.tag_override.get(side) or room.tag(side)
            if side == 'entry' and states in {'APPROACH_ENTRY', 'CONFIRM_ENTRY_TAG',
                                              'ADVANCE_ENTRY_JUNCTION', 'ALIGN_ENTRY_JUNCTION'}:
                self.tags_emitted.append((tag, b.time))
                b.emit('/detected_apriltag', Scalar(tag))
            if side == 'exit' and states in {'TRAVEL_TO_EXIT', 'CONFIRM_EXIT_TAG'}:
                self.tags_emitted.append((tag, b.time))
                b.emit('/detected_apriltag', Scalar(tag))

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
        b.emit('/apriltag/healthy', Scalar(True))
        self.emit_tags(b)
        if self.node.payload == 'process':
            self.wafer = self.node.room.handoff('exit')
            b.emit('/process/state', Scalar('READY_EXIT'))
            b.emit('/process/ready', Scalar(self.node.room.code))
            b.emit('/process/cells_detached', Scalar(self.cells_detached))
        else:
            b.emit('/process/state', Scalar('IDLE'))
            b.emit('/process/ready', Scalar(''))
            b.emit('/process/cells_detached', Scalar(self.cells_detached))
        # No camera images: the fake mission provides confirmed tags and IR
        # steering plus physics feedback for doors, payload and stops.
        ir_values = b.ir_line_follower.probe_values(self.x, self.y, self.heading)
        ir_result = b.ir_line_follower.steering(ir_values, self.line_speed, 25., 1.4, .09)
        ir_command = Twist()
        if ir_result:
            ir_command.linear.x, ir_command.angular.z = ir_result
        if self.ir_override is not None:
            ir_command = self.ir_override
            ir_result = (ir_command.linear.x, ir_command.angular.z)
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
        elif n.payload in {'entry', 'exit'}:
            self.wafer = n.payload_table
        b.time += dt
        self.feedback()
        n.tick()
        if not self.history or self.history[-1] != (n.room.code, n.state):
            self.history.append((n.room.code, n.state))
        if not self.route or self.route[-1] != (b.outputs['/transport/track'].data,
                                                b.outputs['/transport/route_segment'].data):
            self.route.append((b.outputs['/transport/track'].data,
                               b.outputs['/transport/route_segment'].data))
        self.cmd = b.outputs['/cmd_vel']
        if n.state in {'OPEN_ENTRY', 'CLOSE_ENTRY', 'OPEN_EXIT', 'CLOSE_EXIT'}:
            assert n.stopped(), n.state
            assert self.cmd.linear.x == self.cmd.angular.z == 0.
        if n.state in TRANSFER_STATES and (abs(self.joints[n.room.door_joint('entry')]-.52) < .002 or
                                    abs(self.joints[n.room.door_joint('exit')]-.52) < .002):
            side = 'entry' if abs(self.joints[n.room.door_joint('entry')]-.52) < .002 else 'exit'
            assert self.joints[n.room.door_joint(side)] == pytest.approx(.52, abs=.002)
            self.door_cycles.add((n.room.code, side))
        if self.cmd.linear.x > 0.:
            assert all(abs(self.joints[r.door_joint(side)]) <= .002
                       for r in ROOMS for side in ('entry', 'exit'))

    def run(self, ticks=40000, stop=('COMPLETE',)):
        for _ in range(ticks):
            self.advance()
            assert self.node.state != 'FAULT', (self.history[-6:],
                                                self.bus.outputs.get('/system/fault').data,
                                                self.x, self.y, self.heading, self.node.leg)
            if self.node.state in stop:
                break
        return self.node.state


def test_full_ten_glovebox_two_track_mission(ros):
    h = Harness(ros)
    assert h.run() == 'COMPLETE'
    n = h.node
    assert n.completed == list(ROOM_CODES)
    assert ros.outputs['/wafer_delivered'].data
    assert math.dist((h.x, h.y), START) < .04
    assert math.dist(h.wafer[:2], (h.x, h.y)) < .01
    assert h.door_cycles == {(r.code, side) for r in ROOMS for side in ('entry', 'exit')}
    tracks = [track for track, segment in h.route]
    assert 'TRACK_1' in tracks and 'TRACK_2' in tracks
    # Every room is entered on Track 1, serviced on Track 2 and left on Track 1.
    for room in ROOMS:
        states = [state for code, state in h.history if code == room.code]
        assert (states.index('CONFIRM_ENTRY_TAG') < states.index('ADVANCE_ENTRY_JUNCTION')
                < states.index('CROSS_ENTRY_JUNCTION') < states.index('TRAVEL_TO_ENTRY')
                < states.index('OPEN_ENTRY') < states.index('CLOSE_ENTRY')
                < states.index('TRAVEL_TO_EXIT') < states.index('CONFIRM_EXIT_TAG')
                < states.index('OPEN_EXIT') < states.index('CLOSE_EXIT')
                < states.index('TRAVEL_TO_RETURN_JUNCTION')
                < states.index('CROSS_RETURN_JUNCTION'))
        assert (room.code, 'CLOSE_EXIT') in h.history
        for side in ('entry', 'exit'):
            assert h.joints[room.door_joint(side)] == pytest.approx(0., abs=.002)
    assert set(h.releases) == {(r.code, 'entry') for r in ROOMS} | {(r.code, 'carrier') for r in ROOMS}
    # Each room's service excursion selects that room's own Track 2 edge, and
    # the return home never leaves Track 1.
    segments = [segment for track, segment in h.route]
    for room in ROOMS:
        assert service_edge(room).name in segments
        assert segments.index(main_edge(room).name) < segments.index(entry_rung(room).name)
        assert segments.index(entry_rung(room).name) < segments.index(exit_rung(room).name)
    tail = h.route[segments.index('main_tail'):]
    assert tail and all(track == 'TRACK_1' for track, segment in tail)
    assert all(segment in {'main_tail', 'main_home'} for track, segment in tail)


def entry_rung_name(room):
    return EDGES[service_edge(room).start].name


def test_return_home_stays_on_track_1_without_projection_jumps(ros):
    h = Harness(ros)
    while h.node.state != 'RETURN_HOME':
        h.advance()
        assert h.node.state != 'FAULT', h.history[-6:]
    assert h.bus.outputs['/transport/track'].data == 'TRACK_1'
    samples = []
    while h.node.state != 'COMPLETE':
        h.advance()
        assert h.node.state != 'FAULT', h.history[-6:]
        assert h.bus.outputs['/transport/track'].data == 'TRACK_1'
        samples.append((h.x, h.y, h.node.leg))
    # Reverse main-lane return: one straight lane, monotone along-edge travel.
    assert {leg for _, _, leg in samples} <= {'main_tail', 'main_home'}
    distance = [edge_project(EDGES[leg], x, y)[0] for x, y, leg in samples]
    assert max(edge_project(HOME_EDGE if leg == 'main_home' else TAIL_EDGE, x, y)[1] for x, y, leg in samples) < .03
    assert len(distance) == len(samples)
    home = [d for d, (x, y, leg) in zip(distance, samples) if leg == 'main_home']
    assert home == sorted(home)
    assert max(b-a for a, b in zip(home, home[1:])) < .20
    assert math.dist((h.x, h.y), START) < .04


@pytest.mark.parametrize('side', ['entry', 'exit'])
def test_wrong_room_tags_are_ignored_at_the_decision(ros, side):
    h = Harness(ros)
    n = h.node
    target = 'CONFIRM_ENTRY_TAG' if side == 'entry' else 'CONFIRM_EXIT_TAG'
    # Everything except the expected tag of the expected room keeps arriving,
    # including the neighbouring room's marker of the same kind.
    h.tag_wrong = (side, 2 if side == 'entry' else 3)
    while n.state != target:
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    for _ in range(40):
        h.advance()
        assert n.state == target, n.history[-3:]
        assert not n.tags_authorized(n.room, side)
    assert h.tags_emitted
    # The same marker of the correct room and decision does authorise.
    h.tag_wrong = None
    for _ in range(40):
        h.advance()
        if n.state != target:
            break
    assert n.state not in {target, 'FAULT'}


@pytest.mark.parametrize('side', ['entry', 'exit'])
def test_tag_observed_before_the_decision_scope_does_not_authorise(ros, side):
    h = Harness(ros)
    n = h.node
    target = 'CONFIRM_ENTRY_TAG' if side == 'entry' else 'CONFIRM_EXIT_TAG'
    approach = 'APPROACH_ENTRY' if side == 'entry' else 'TRAVEL_TO_EXIT'
    # Only the other decision's markers keep arriving, so the expected tag is
    # seen once on the approach and never again inside the decision scope.
    h.tag_sides = ('exit',) if side == 'entry' else ('entry',)
    # The marker is seen while the robot is still driving its approach.
    while n.state != approach:
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    ros.emit('/detected_apriltag', Scalar(n.room.tag(side)))
    assert n.room.tag(side) in n.observed_tags
    while n.state != target:
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    for _ in range(40):
        h.advance()
        assert n.state == target, n.history[-3:]
    # Freshness is scoped: that old observation can never sign the junction.
    n.entered = ros.time - n.params['apriltag_timeout'] - 1.
    h.advance()
    assert n.state == 'FAULT'
    assert 'AprilTag not confirmed' in ros.outputs['/system/fault'].data


def test_authorisation_is_one_shot_and_room_scoped(ros):
    h = Harness(ros)
    n = h.node
    while n.state != 'CONFIRM_ENTRY_TAG':
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    for _ in range(40):
        h.advance()
        if n.state != 'CONFIRM_ENTRY_TAG':
            break
    assert n.state in {'ADVANCE_ENTRY_JUNCTION', 'ALIGN_ENTRY_JUNCTION',
                       'CROSS_ENTRY_JUNCTION'}
    for _ in range(200):
        h.advance()
        if n.state == 'CROSS_ENTRY_JUNCTION':
            break
    assert n.state == 'CROSS_ENTRY_JUNCTION'
    # The signed decision is consumed, so the same observation cannot authorise
    # the return rung, another room, or the opposite decision.
    assert n.scope is None
    assert n.room.tag('entry') in n.observed_tags
    assert not n.tags_authorized(n.room, 'entry')
    assert not n.tags_authorized(n.room, 'exit')
    assert not n.tags_authorized(ROOMS[1], 'entry')
    n.scope = (ROOMS[1].code, 'entry', ros.time)
    assert not n.tags_authorized(ROOMS[1], 'entry')
    assert n.scope is not None


def test_same_tag_observed_in_a_different_decision_state_is_ignored(ros):
    h = Harness(ros)
    n = h.node
    entry_tag = ROOMS[0].tag('entry')
    # The exit decision keeps seeing the entry marker of the very same room:
    # same tag, different FSM state, so it must not sign Track 2 -> Track 1.
    h.tag_override = {'exit': entry_tag}
    while n.state != 'CONFIRM_EXIT_TAG':
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    assert entry_tag in n.observed_tags
    assert not n.tags_authorized(n.room, 'exit')
    for _ in range(40):
        h.advance()
        assert n.state == 'CONFIRM_EXIT_TAG'
    n.entered = ros.time - n.params['apriltag_timeout'] - 1.
    h.advance()
    assert n.state == 'FAULT'
    assert ROOMS[0].tag('exit') in ros.outputs['/system/fault'].data


def test_exit_decision_requires_the_completed_service_cell(ros):
    h = Harness(ros)
    n = h.node
    while n.state != 'CONFIRM_EXIT_TAG':
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    assert n.tags_authorized(n.room, 'exit')
    # The tag alone cannot authorise Track 2 -> Track 1 without the finished
    # process cell holding the wafer at the exit handoff.
    h.node.payload = 'carrier'
    for _ in range(20):
        h.advance()
        assert n.state == 'CONFIRM_EXIT_TAG'
    n.entered = ros.time - n.params['apriltag_timeout'] - 1.
    h.advance()
    assert n.state == 'FAULT'
    assert n.leg == service_edge(n.room).name


@pytest.mark.parametrize('side', ['entry', 'exit'])
def test_missing_decision_tag_faults_before_the_junction(ros, side):
    h = Harness(ros)
    n = h.node
    target = 'CONFIRM_ENTRY_TAG' if side == 'entry' else 'CONFIRM_EXIT_TAG'
    h.tag_sides = () if side == 'entry' else ('entry',)
    while n.state != target:
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    track = ros.outputs['/transport/track'].data
    x, y, _ = h.x, h.y, h.heading
    n.entered = ros.time - n.params['apriltag_timeout'] - 1.
    h.advance()
    assert n.state == 'FAULT'
    assert 'AprilTag not confirmed' in ros.outputs['/system/fault'].data
    assert ros.outputs['/cmd_vel'].linear.x == ros.outputs['/cmd_vel'].angular.z == 0.
    assert not ros.outputs['/wafer_delivered'].data
    assert math.dist((h.x, h.y), (x, y)) < .02
    # The failed decision never crossed the transverse junction.
    assert track == ('TRACK_1' if side == 'entry' else 'TRACK_2')


def test_service_tags_never_pull_the_return_home_off_track_1(ros):
    h = Harness(ros)
    n = h.node
    while n.state != 'ALIGN_MAIN_LANE' or n.index != len(ROOMS)-1:
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    # Flood every service marker while the robot drives home on Track 1.
    h.automatic_tags = False
    for room in ROOMS:
        for side in ('entry', 'exit'):
            ros.emit('/detected_apriltag', Scalar(room.tag(side)))
    while n.state != 'COMPLETE':
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
        assert ros.outputs['/transport/track'].data == 'TRACK_1'
    assert n.scope is None
    assert math.dist((h.x, h.y), START) < .04


def test_junction_states_follow_the_selected_edge_not_the_ir_candidate(ros):
    h = Harness(ros)
    n = h.node
    while n.state != 'CROSS_ENTRY_JUNCTION':
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    assert ros.outputs['/transport/track'].data == 'TRACK_2'
    assert ros.outputs['/transport/route_segment'].data == entry_rung(ROOMS[0]).name
    bogus = Twist()
    bogus.linear.x = .20
    bogus.angular.z = -1.4        # would steer straight off the rung
    h.ir_override = bogus
    while n.state == 'CROSS_ENTRY_JUNCTION':
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    h.ir_override = None
    x, y = h.x, h.y
    assert abs(y-TRACK_2_Y) < .02, (x, y)
    assert abs(x-entry_junction_x(ROOMS[0])) < .02


def test_pose_fallback_steers_the_selected_edge_but_never_authorises_tags(ros):
    h = Harness(ros)
    n = h.node
    h.ir_failure = 'missing'
    h.tag_sides = ()
    while n.state != 'CONFIRM_ENTRY_TAG':
        h.advance()
        assert n.state != 'FAULT', h.history[-6:]
    assert ros.outputs['/transport/steering_mode'].data == 'SIMULATED_IR'
    n.entered = ros.time - n.params['apriltag_timeout'] - 1.
    h.advance()
    # Pose fallback steers the selected edge, but it cannot sign the junction.
    assert n.state == 'FAULT'
    assert n.leg == main_edge(ROOMS[0]).name
    h = Harness(ros)
    h.ir_failure = 'missing'
    assert h.run() == 'COMPLETE'
    assert ros.outputs['/transport/steering_mode'].data == 'SIMULATED_IR'


def test_mission_completes_on_ir_alone_with_the_pose_fallback_disabled(ros):
    h = Harness(ros)
    h.node.params['allow_sensor_fallback'] = False
    assert h.run() == 'COMPLETE'
    # Nothing in the route needed the pretend sensor: the probes read the real
    # lanes and the bounded junction states steered the two rungs.
    assert ros.outputs['/transport/steering_mode'].data == 'IR'
    assert h.node.completed == list(ROOM_CODES)


def test_mission_waits_for_every_process_cell_to_release_the_wafer(ros):
    h = Harness(ros)
    h.cells_detached = False
    for _ in range(40):
        h.advance()
    assert h.node.state == 'WAIT_FOR_SIMULATOR'
    assert not h.node.started
    assert any('process cells' in issue for issue in h.node.readiness_issues())
    h.cells_detached = True
    for _ in range(5):
        h.advance()
    assert h.node.state not in ('WAIT_FOR_SIMULATOR',)
    assert h.node.started is not None
    assert h.node.state != 'FAULT'


def test_internal_robot_processes_entry_stage_and_exit_with_feedback(ros):
    node = ros.process_cell_controller.ProcessCellController()
    room = ROOMS[0]
    entry, stage, exit_handoff = room.handoff('entry'), room.table, room.handoff('exit')

    def feedback(x, z, attachment, wafer):
        ros.emit(f'/process/room_{room.number}/joint_states',
                 NS(name=[room.process_joint('x'), room.process_joint('z')], position=[x, z]))
        ros.emit(f'/attachments/{room.process_attachment()}/state', Scalar(attachment))
        ros.emit('/attachments/vacuum/state', Scalar('detached'))
        ros.emit('/poses/wafer', pose(*wafer))

    def lifted(position):
        return position[0], position[1], PROCESS_TRANSIT_WAFER_Z

    # The cells load attached and park on their own supports before any transfer.
    for other in ROOMS:
        ros.emit(f'/attachments/{other.process_attachment()}/state', Scalar('detached'))
    node.tick()
    assert node.state == 'IDLE'
    assert ros.outputs['/process/cells_detached'].data
    feedback(0., 0., 'detached', entry)
    node.request(Scalar(room.code))
    assert node.state == 'PICK_ENTRY'
    node.tick()
    assert node.state == 'LOWER_ENTRY'
    feedback(0., PROCESS_CONTACT_Z, 'detached', entry)
    node.tick()
    assert f'/attachments/{room.process_attachment()}/attach' in ros.outputs
    feedback(0., PROCESS_CONTACT_Z, 'attached', entry)
    node.tick()
    assert node.state == 'LIFT_ENTRY'
    feedback(0., 0., 'attached', lifted(entry))
    node.tick()
    assert node.state == 'MOVE_PROCESS'
    feedback(PROCESS_TRAVEL/2, 0., 'attached', lifted(stage))
    node.tick()
    assert node.state == 'LOWER_PROCESS'
    feedback(PROCESS_TRAVEL/2, PROCESS_CONTACT_Z, 'attached', stage)
    node.tick()
    feedback(PROCESS_TRAVEL/2, PROCESS_CONTACT_Z, 'detached', stage)
    node.tick()
    assert node.state == 'VERIFY_STAGE'
    assert ros.outputs['/process/ready'].data == ''
    node.tick()
    ros.time += .6
    node.tick()
    assert node.state == 'PROCESSING'
    # Hold the verified dwell while the simulation clock keeps running.
    for _ in range(45):
        ros.time += .05
        feedback(PROCESS_TRAVEL/2, PROCESS_CONTACT_Z, 'detached', stage)
        node.tick()
        if node.state != 'PROCESSING':
            break
    assert node.state == 'PROCESSING'
    assert ros.outputs['/process/ready'].data == ''
    feedback(PROCESS_TRAVEL/2, PROCESS_CONTACT_Z, 'attached', stage)
    node.tick()
    assert node.state == 'LIFT_PROCESS'
    feedback(PROCESS_TRAVEL/2, 0., 'attached', lifted(stage))
    node.tick()
    assert node.state == 'MOVE_EXIT'
    feedback(PROCESS_TRAVEL, 0., 'attached', lifted(exit_handoff))
    node.tick()
    assert node.state == 'LOWER_EXIT'
    feedback(PROCESS_TRAVEL, PROCESS_CONTACT_Z, 'attached', exit_handoff)
    node.tick()
    feedback(PROCESS_TRAVEL, PROCESS_CONTACT_Z, 'detached', exit_handoff)
    node.tick()
    assert node.state == 'VERIFY_EXIT'
    assert ros.outputs['/process/ready'].data == ''
    node.tick()
    ros.time += .6
    node.tick()
    assert node.state == 'RETRACT_EXIT'
    feedback(PROCESS_TRAVEL, 0., 'detached', exit_handoff)
    node.tick()
    assert node.state == 'READY_EXIT'
    assert ros.outputs['/process/ready'].data == room.code
    node.consumed(Scalar(room.code))
    assert node.state == 'RESET'
    feedback(0., 0., 'detached', exit_handoff)
    node.tick()
    assert node.state == 'IDLE'


@pytest.mark.parametrize('failure', ['lost_line', 'stale_ir', 'stale_pose', 'stuck_door',
                                     'failed_pickup'])
def test_four_room_failures_stop_without_delivery(ros, failure):
    h = Harness(ros)
    target = {'stuck_door': 'OPEN_ENTRY', 'failed_pickup': 'ATTACH'}.get(failure, 'TRAVEL_TO_EXIT')
    for _ in range(4000):
        h.advance()
        if h.node.state == target:
            break
        assert h.node.state != 'FAULT', h.history[-6:]
    assert h.node.state == target
    if failure == 'lost_line':
        h.node.params['allow_sensor_fallback'] = False
        ros.emit('/ir/valid', Scalar(False))
    elif failure == 'stale_ir':
        h.node.params['allow_sensor_fallback'] = False
        h.node.received['/ir/healthy'] -= 2.
    elif failure == 'stale_pose':
        h.node.received['/poses/transport_robot'] -= 2.
    else:
        # A timeout is a failure, never a substitute for attachment/door evidence.
        h.node.entered = ros.time - 181.
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
    for _ in range(8000):
        h.advance()
        if h.node.state == 'FAULT':
            break
    assert h.node.state == 'FAULT'
    assert (ROOMS[0].code, blocked) in h.history
    assert not h.node.completed
    assert not ros.outputs['/wafer_delivered'].data
    assert ros.outputs['/cmd_vel'].linear.x == 0.
    assert not any(state == ('PICK_POSITION' if failure == 'door' else 'LIFT')
                   for _, state in h.history)


def test_final_delivery_requires_supported_wafer_and_low_velocity(ros):
    h = Harness(ros)
    n = h.node
    n.index = len(ROOMS)-1
    n.completed = list(ROOM_CODES)
    n.payload = 'carrier'
    h.feedback()
    n.transition('VERIFY_DELIVERY')
    for _ in range(30):
        ros.time += .05
        h.feedback()
        ros.emit('/poses/wafer', pose(0., 0., 0.))
        n.tick()
        assert not ros.outputs['/wafer_delivered'].data
    h.wafer = (h.x, h.y, .156)
    h.applied.linear.x = .02
    for _ in range(30):
        ros.time += .05
        h.feedback()
        n.tick()
        assert not ros.outputs['/wafer_delivered'].data


def test_launch_uses_only_apriltag_mission_nodes():
    import ast
    launch = ast.parse((ROOT/'launch/demo.launch.py').read_text())
    assignment = next(n for n in ast.walk(launch) if isinstance(n, ast.Assign) and
                      any(isinstance(t, ast.Name) and t.id == 'executables' for t in n.targets))
    assert ast.literal_eval(assignment.value) == (
        'four_room_controller', 'ir_line_follower', 'apriltag_detector',
        'process_cell_controller')


def test_ir_steered_entry_requires_apriltag(ros):
    h = Harness(ros)
    for _ in range(4000):
        h.advance()
        if h.node.state == 'TRAVEL_TO_EXIT':
            break
    assert h.node.state == 'TRAVEL_TO_EXIT'
    h.node.tick()
    assert h.node.state == 'TRAVEL_TO_EXIT'
    assert ros.outputs['/cmd_vel'].linear.x > 0.
    assert '/line/cmd_vel' not in ros.subscribers
    assert '/detected_apriltag' in ros.subscribers
    assert ros.outputs['/transport/route_segment'].data == service_edge(ROOMS[0]).name


@pytest.mark.parametrize('failure', ['missing', 'lost', 'zero'])
def test_mid_route_ir_failure_switches_to_simulated_tracking_and_recovers(ros, failure):
    h = Harness(ros)
    for _ in range(4000):
        h.advance()
        if h.node.state == 'TRAVEL_TO_EXIT':
            break
        assert h.node.state != 'FAULT', h.history[-6:]
    assert h.node.state == 'TRAVEL_TO_EXIT'
    h.ir_failure = failure
    if failure == 'missing':
        h.node.received['/ir/cmd_vel'] -= 2.
    h.feedback()
    h.node.tick()
    assert h.node.state == 'TRAVEL_TO_EXIT'
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
    edge = main_edge(n.room)
    n.select(edge)
    n.follow(edge, .40, n.params['route_speed'])
    assert n.command.linear.x == pytest.approx(.20)
    n.command = Twist()
    n.follow(edge, .05, n.params['route_speed'])
    assert .07 <= n.command.linear.x < .20
    n.command = Twist()
    n.follow(edge, .003, n.params['route_speed'])
    assert n.command.linear.x == 0.


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


def test_arrival_window_does_not_stall_before_tag_decision(ros):
    h = Harness(ros)
    h.joints['tray_z'] = .004
    n = h.node
    edge = main_edge(n.room)
    target = entry_decision_stop(n.room)
    h.x, h.y = edge_point(edge, target-.0045)
    n.state = 'APPROACH_ENTRY'
    # A modest drive deadband used to trap the robot 4.5 mm before the target:
    # sqrt(2*.35*(.0045-.004)) == .0187 m/s, below .025 m/s.
    for _ in range(40):
        h.feedback()
        n.command = Twist()
        n.step(.05)
        speed = n.command.linear.x
        if speed >= .025:
            h.x += speed*math.cos(h.heading)*.05
            h.y += speed*math.sin(h.heading)*.05
        ros.time += .05
        if n.state == 'CONFIRM_ENTRY_TAG':
            break
    assert n.state == 'CONFIRM_ENTRY_TAG'
    assert math.dist((h.x, h.y), edge_point(edge, target)) <= .004
    ros.emit('/detected_apriltag', Scalar(n.room.tag('entry')))
    n.step(.05)
    assert n.state == 'ADVANCE_ENTRY_JUNCTION'


def test_decision_diagnostic_exposes_fault_and_tag_wait(ros):
    h = Harness(ros)
    h.feedback()
    n = h.node
    n.transition('CONFIRM_ENTRY_TAG')
    assert 'expected=GLOVEBOX_01_ENTRY' in n.decision_status()
    assert 'authorized=False' in n.decision_status()
    ros.emit('/detected_apriltag', Scalar(n.room.tag('entry')))
    assert 'authorized=True' in n.decision_status()
    ros.emit('/system/fault', Scalar('test failure'))
    n.state = 'FAULT'
    assert n.decision_status() == 'FAULT: test failure'


def test_closed_doors_clear_supports_and_supported_wafer():
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')
    for room in ROOMS:
        wall = room.y-ROOM_DEPTH/2
        structure = world.find(f".//model[@name='room_{room.number}']/link[@name='structure']")
        for name in ('entry_handoff', 'exit_handoff', 'process_stage'):
            collision = structure.find(f"collision[@name='{name}_collision']")
            centre = list(map(float, collision.findtext('pose').split()))
            size = list(map(float, collision.findtext('geometry/box/size').split()))
            # Panel/front wall extends 6 mm inward; require 5 mm clearance.
            assert centre[1]-size[1]/2 > wall+.011
        for side in ('entry', 'exit'):
            assert room.handoff(side)[1]-.05 > wall+.011
            # Mobile wand must still reach the support without exceeding travel.
            assert room.handoff(side)[1]-TRACK_2_Y < .269


@pytest.mark.parametrize('position,closed', [(-.004, True), (-.003, True),
                                           (0., True), (.001, True),
                                           (.003, False), (.02, False),
                                           (-.007, False), (float('nan'), False)])
def test_door_closed_distinguishes_lower_stop_from_opening(ros, position, closed):
    h = Harness(ros)
    name = h.node.room.door_joint('entry')
    h.joints[name] = position
    h.feedback()
    assert h.node.door_closed(name) is closed
    h.node.joint_received[name] -= 2.
    assert not h.node.door_closed(name)
    assert ('stale' in h.node.door_status(name) or 'invalid' in h.node.door_status(name))


def test_real_door_opening_stops_travel_and_reports_measurement(ros):
    h = Harness(ros)
    assert h.run(stop=('APPROACH_ENTRY',)) == 'APPROACH_ENTRY'
    name = h.node.room.door_joint('entry')
    h.joints[name] = .020
    h.feedback()
    h.node.tick()
    assert h.node.state == 'FAULT'
    assert 'position=0.02000m' in ros.outputs['/system/fault'].data
    assert ros.outputs['/cmd_vel'].linear.x == 0.
    assert ros.outputs['/cmd_vel'].angular.z == 0.


def test_downward_door_settling_does_not_interrupt_forward_travel(ros):
    h = Harness(ros)
    assert h.run(stop=('APPROACH_ENTRY',)) == 'APPROACH_ENTRY'
    h.joints[h.node.room.door_joint('entry')] = -.004
    h.feedback()
    h.node.tick()
    assert h.node.state == 'APPROACH_ENTRY'
    assert ros.outputs['/cmd_vel'].linear.x > 0.


@pytest.mark.parametrize('edge_factory', [main_edge, entry_rung, service_edge, exit_rung])
def test_selected_edge_steering_stays_forward_at_the_endpoint(ros, edge_factory):
    h = Harness(ros)
    edge = edge_factory(ROOMS[0])
    end_x, end_y = edge.points[-1]
    heading = math.atan2(end_y-edge.points[-2][1], end_x-edge.points[-2][0])
    # A small overrun and lateral error must not make the lookahead jump behind
    # the robot as it reaches a lane change.
    h.x = end_x + .001*math.cos(heading) - .002*math.sin(heading)
    h.y = end_y + .001*math.sin(heading) + .002*math.cos(heading)
    h.heading = heading
    h.feedback()
    command = h.node.lookahead_command(edge)
    assert command.linear.x > 0.
    assert abs(command.angular.z) < .15


@pytest.mark.parametrize('target', [0., math.pi/2, -math.pi/2, math.pi])
def test_junction_alignment_overcomes_small_turn_deadband(ros, target):
    h = Harness(ros)
    n = h.node
    h.heading = target-.12
    measured_w = 0.
    for _ in range(100):
        h.feedback()
        n.command = Twist()
        done = n.align(target)
        command = n.command.angular.z
        if abs(command) >= .08:
            measured_w += clamp(command-measured_w, -.075, .075)
        else:
            measured_w += clamp(-measured_w, -.075, .075)
        h.heading += measured_w*.05
        h.applied.angular.z = measured_w
        ros.time += .05
        if done:
            break
    assert done
    assert abs(math.atan2(math.sin(target-h.heading), math.cos(target-h.heading))) < .03
