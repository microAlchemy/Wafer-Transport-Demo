"""Execute the real controllers against a small in-memory ROS transport.

These test state logic and watchdogs, not Gazebo physics or DDS behavior.
"""
import importlib
import math
import sys
from types import ModuleType, SimpleNamespace as NS
import pytest


class Scalar:
    def __init__(self, data=None, **kwargs):
        self.data = data
        self.__dict__.update(kwargs)


class Twist:
    def __init__(self):
        self.linear = NS(x=0., y=0., z=0.)
        self.angular = NS(x=0., y=0., z=0.)


class Odometry:
    def __init__(self):
        self.pose = NS(pose=NS(position=NS(x=0., y=0., z=0.),
                               orientation=NS(x=0., y=0., z=0., w=1.)))
        self.twist = NS(twist=Twist())


@pytest.fixture
def ros(monkeypatch):
    bus = NS(time=1., subscribers={}, outputs={})

    def emit(topic, msg):
        bus.outputs[topic] = msg
        for callback in bus.subscribers.get(topic, []):
            callback(msg)
    bus.emit = emit

    class Node:
        def __init__(self, name):
            self.name = name
            self.params = {}

        def get_name(self):
            return self.name

        def get_logger(self):
            return NS(info=lambda _: None, error=lambda _: None)

        def get_clock(self):
            return NS(now=lambda: NS(nanoseconds=int(bus.time * 1e9)))

        def has_parameter(self, name):
            return name in self.params

        def declare_parameter(self, name, value):
            self.params[name] = value

        def get_parameter(self, name):
            return NS(value=self.params[name])

        def create_subscription(self, kind, topic, callback, qos):
            bus.subscribers.setdefault(topic, []).append(callback)

        def create_publisher(self, kind, topic, qos):
            return NS(publish=lambda msg: emit(topic, msg))

        def create_timer(self, *args, **kwargs):
            pass

        def add_on_set_parameters_callback(self, callback):
            pass

    def module(name, **symbols):
        m = ModuleType(name)
        m.__dict__.update(symbols)
        monkeypatch.setitem(sys.modules, name, m)

    module('rclpy')
    module('rclpy.node', Node=Node)
    module('rclpy.qos', QoSProfile=lambda **_: None,
           DurabilityPolicy=NS(TRANSIENT_LOCAL=1), ReliabilityPolicy=NS(RELIABLE=1))
    module('rclpy.clock', Clock=lambda **_: None, ClockType=NS(STEADY_TIME=1))
    for package, symbols in {
        'std_msgs': dict(Bool=Scalar, String=Scalar, Float64=Scalar, Empty=Scalar),
        'geometry_msgs': dict(Twist=Twist, PoseStamped=Scalar),
        'sensor_msgs': dict(JointState=Scalar),
        'nav_msgs': dict(Odometry=Odometry),
        'rcl_interfaces': dict(SetParametersResult=Scalar),
    }.items():
        module(package)
        module(package + '.msg', **symbols)
    for name in ('common', 'wafer_handler', 'transport_controller', 'docking_controller', 'system_manager'):
        key = 'wafer_transport_sim.' + name
        monkeypatch.delitem(sys.modules, key, raising=False)
        mod = importlib.import_module(key)
        monkeypatch.setitem(sys.modules, key, mod)
        setattr(bus, name, mod)
    return bus


def pose(x, y, z, heading=math.pi/2):
    return NS(pose=NS(position=NS(x=x, y=y, z=z),
                      orientation=NS(x=0., y=0., z=math.sin(heading/2), w=math.cos(heading/2))))


def room_feedback(ros, node, x=.65, heading=math.pi/2):
    odom = Odometry()
    odom.pose.pose.orientation.z = math.sin(heading/2)
    odom.pose.pose.orientation.w = math.cos(heading/2)
    ros.emit('/odom', odom)
    ros.emit('/poses/transport_robot', pose(x, .12, 0., heading+math.pi/2))
    ros.emit('/poses/carrier', pose(x, .12, .154))
    ros.emit('/line/valid', Scalar(False))  # No stripe inside the room is expected.
    ros.emit('/line/healthy', Scalar(True))
    ros.emit('/qr/healthy', Scalar(True))
    ros.emit('/attachments/carrier/state', Scalar('attached'))
    for topic, names in [('/robot/joint_states', ('reach_1', 'reach_2', 'wand_x', 'wand_z', 'tray_z')),
                         ('/door/joint_states', ('door_z',))]:
        targets = [ros.outputs.get('/actuators/' + n, Scalar(0.)).data for n in names]
        ros.emit(topic, NS(name=list(names), position=targets))


def set_handler_state(ros, node, state):
    node.sequence = ros.wafer_handler.Sequence(ros.wafer_handler.STEPS,
                    index=ros.wafer_handler.STEPS.index(state), entered=ros.time)
    node.change(state)


def test_wafer_handler_full_sequence_and_readiness(ros):
    node = ros.wafer_handler.WaferHandler()
    ros.emit('/attachments/vacuum/state', Scalar('detached'))
    node.tick()
    assert not ros.outputs['/carrier_ready'].data
    ros.emit('/system/start', Scalar(True))
    node.tick()
    assert node.state == 'HOME'
    history = [node.state]
    entered_room = False
    for _ in range(100):
        ros.time += .05
        if node.state == 'ENTER_ROOM':
            entered_room = True
        elif node.state == 'EXIT_ROOM':
            entered_room = False
        rx = .65 if entered_room else 1.025
        heading = 0. if node.state in ('HOME', 'OPEN_ENTRY', 'TURN_OUT', 'CLOSE_EXIT', 'TRANSFER_COMPLETE') else math.pi/2
        room_feedback(ros, node, rx, heading)
        wx = .4 if node.state in ros.wafer_handler.STEPS[:10] else rx
        wz = .155 + node.joints.get('wand_z', 0.) + .14
        if node.state in ('MOVE_TO_WAFER', 'LOWER_WAND', 'VACUUM_ON', 'VERIFY_VACUUM'):
            wx, wz = .4, .155
        if node.state in ros.wafer_handler.STEPS[12:]:
            wx, wz = rx, .159
        ros.emit('/poses/wafer', pose(wx, .12, wz))
        if node.state in ('VACUUM_ON', 'VERIFY_VACUUM', 'LIFT_PICKUP', 'MOVE_TO_BOX', 'LOWER'):
            ros.emit('/attachments/vacuum/state', Scalar('attached'))
        elif node.state == 'VACUUM_OFF':
            ros.emit('/attachments/vacuum/state', Scalar('detached'))
        node.tick()
        if node.state != history[-1]:
            history.append(node.state)
        if node.state == 'TRANSFER_COMPLETE':
            break
        assert not ros.outputs['/carrier_ready'].data
    assert history == list(ros.wafer_handler.STEPS)
    assert ros.outputs['/carrier_ready'].data
    assert ros.outputs['/door/closed'].data


def test_failed_vacuum_cannot_become_ready(ros):
    node = ros.wafer_handler.WaferHandler()
    set_handler_state(ros, node, 'VACUUM_ON')
    room_feedback(ros, node)
    ros.emit('/robot/joint_states', NS(name=['reach_1', 'reach_2', 'wand_x', 'wand_z'],
                                     position=[.25/3, .25/3, .25/3, -.14]))
    ros.emit('/attachments/vacuum/state', Scalar('detached'))
    ros.emit('/poses/wafer', pose(.4, .12, .155))
    node.tick()
    assert node.state == 'VACUUM_ON'
    ros.time += 31
    node.tick()
    assert node.state == 'FAULT'
    assert not ros.outputs['/carrier_ready'].data


@pytest.mark.parametrize('state', ['CLOSE_ENTRY', 'CLOSE_EXIT'])
def test_door_cannot_close_on_robot(ros, state):
    node = ros.wafer_handler.WaferHandler()
    set_handler_state(ros, node, state)
    room_feedback(ros, node, x=.91)
    ros.emit('/poses/wafer', pose(.91, .12, .159))
    ros.emit('/door/joint_states', NS(name=['door_z'], position=[.52]))
    node.tick()
    assert node.state == 'FAULT'
    assert ros.outputs['/actuators/door_z'].data == .52
    assert ros.outputs['/cleanroom/cmd_vel'].linear.x == 0.


@pytest.mark.parametrize('failure', ['closed_door', 'stale_pose', 'lost_carrier'])
def test_entry_interlocks_stop_motion(ros, failure):
    node = ros.wafer_handler.WaferHandler()
    set_handler_state(ros, node, 'ENTER_ROOM')
    room_feedback(ros, node, x=1.025)
    ros.emit('/poses/wafer', pose(.4, .12, .155))
    ros.emit('/door/joint_states', NS(name=['door_z'], position=[.52]))
    if failure == 'closed_door':
        ros.emit('/door/joint_states', NS(name=['door_z'], position=[.1]))
    elif failure == 'stale_pose':
        ros.time += 1.1
    else:
        ros.emit('/attachments/carrier/state', Scalar('detached'))
    node.tick()
    assert node.state == 'FAULT'
    assert ros.outputs['/cleanroom/cmd_vel'].linear.x == 0.


def test_entry_uses_motion_feedback_and_waits_for_measured_stop(ros):
    node = ros.wafer_handler.WaferHandler()
    set_handler_state(ros, node, 'ENTER_ROOM')
    room_feedback(ros, node, x=1.025)
    ros.emit('/poses/wafer', pose(.4, .12, .155))
    ros.emit('/door/joint_states', NS(name=['door_z'], position=[.52]))
    node.tick()
    assert node.state == 'ENTER_ROOM'
    assert ros.outputs['/cleanroom/cmd_vel'].linear.x > 0.
    room_feedback(ros, node, x=.65)
    ros.emit('/door/joint_states', NS(name=['door_z'], position=[.52]))
    node.values['/odom'].twist.twist.linear.x = .02
    node.tick()
    assert node.state == 'ENTER_ROOM'
    assert ros.outputs['/cleanroom/cmd_vel'].linear.x == 0.
    node.values['/odom'].twist.twist.linear.x = 0.
    node.tick()
    assert node.state == 'CLOSE_ENTRY'


def make_transport(ros):
    node = ros.transport_controller.TransportController()
    node.initialized = True
    ros.emit('/attachments/carrier/state', Scalar('attached'))
    return node


def feedback(ros, node, x=0.):
    odom = Odometry()
    odom.pose.pose.position.x = x
    ros.emit('/odom', odom)
    ros.emit('/poses/carrier', pose(1.025, .12+x, .154))
    ros.emit('/poses/transport_robot', pose(1.025, .12+x, 0.))
    ros.emit('/poses/wafer', pose(1.025, .12+x, .160))
    line = Twist()
    line.linear.x = .035
    ros.emit('/line/cmd_vel', line)
    ros.emit('/line/valid', Scalar(True))
    ros.emit('/qr/healthy', Scalar(True))


def test_robot_waits_for_both_readiness_and_manager_enable(ros):
    node = make_transport(ros)
    feedback(ros, node)
    for ready, enabled in [(False, False), (False, True), (True, False)]:
        ros.emit('/carrier_ready', Scalar(ready))
        ros.emit('/system/transport_enable', Scalar(enabled))
        node.tick()
        assert node.state == 'WAIT_FOR_CARRIER'
        assert ros.outputs['/cmd_vel'].linear.x == 0
    ros.emit('/carrier_ready', Scalar(True))
    ros.emit('/system/transport_enable', Scalar(True))
    node.tick()
    assert node.state == 'VERIFY_CARRIER'


def test_wrong_station_continues_and_target_needs_docking_feedback(ros):
    node = make_transport(ros)
    node.target = 'STATION_C'
    node.change('FOLLOW_PATH')
    feedback(ros, node, .2)
    node.detected(Scalar('STATION_B'))
    for _ in range(3):
        node.tick()
    assert node.state == 'CONTINUE'
    assert ros.outputs['/cmd_vel'].linear.x > 0
    node.tick()
    node.detected(Scalar('STATION_C'))
    for _ in range(3):
        node.tick()
    assert node.state == 'SLOW_APPROACH'
    assert not ros.outputs['/carrier_delivered'].data
    cmd = Twist()
    cmd.linear.x = .018
    ros.emit('/docking/cmd_vel', cmd)
    ros.emit('/docking_distance', Scalar(.2))
    ros.emit('/docking/reached', Scalar(False))
    node.tick()
    assert ros.outputs['/cmd_vel'].linear.x == .018
    ros.emit('/docking_distance', Scalar(.09))
    node.tick()
    assert node.state == 'PRECISION_DOCK'
    node.tick()
    assert node.state == 'PRECISION_DOCK'
    ros.emit('/docking/reached', Scalar(True))
    node.tick()
    assert node.state == 'STOP'
    assert ros.outputs['/cmd_vel'].linear.x == 0


@pytest.mark.parametrize('failure', ['stale', 'lost_line', 'missed_station', 'carrier_lost'])
def test_transport_failures_stop_without_delivery(ros, failure):
    node = make_transport(ros)
    node.target = 'STATION_C'
    node.change('FOLLOW_PATH')
    feedback(ros, node, .2)
    if failure == 'stale':
        ros.time += 1.1
    elif failure == 'lost_line':
        ros.emit('/line/valid', Scalar(False))
    elif failure == 'missed_station':
        feedback(ros, node, .94)
    elif failure == 'carrier_lost':
        ros.emit('/attachments/carrier/state', Scalar('detached'))
    node.tick()
    assert node.state == 'FAULT'
    assert ros.outputs['/cmd_vel'].linear.x == 0
    assert not ros.outputs['/carrier_delivered'].data


def test_drop_requires_release_retraction_and_stable_supported_pose(ros):
    node = make_transport(ros)
    node.target = 'STATION_C'
    node.change('DROP_CARRIER')
    feedback(ros, node, .92)
    for _ in range(60):
        ros.time += .05
        ros.emit('/odom', Odometry())
        ros.emit('/poses/carrier', pose(1.145, 1.04, .134))
        ros.emit('/poses/wafer', pose(1.145, 1.04, .140))
        for name in ('tray_y', 'tray_z'):
            target = ros.outputs.get('/actuators/' + name, Scalar(0.)).data
            ros.emit('/robot/joint_states', NS(name=[name], position=[target]))
        if '/attachments/carrier/detach' in ros.outputs:
            ros.emit('/attachments/carrier/state', Scalar('detached'))
        node.tick()
        if node.state == 'DELIVERY_COMPLETE':
            node.tick()
            break
    assert node.state == 'DELIVERY_COMPLETE'
    assert ros.outputs['/carrier_delivered'].data
    assert node.at_joint('tray_y', 0.)
    assert ros.outputs['/cmd_vel'].linear.x == 0


def test_docking_will_not_complete_with_velocity_or_position_error(ros):
    node = ros.docking_controller.DockingController()
    ros.emit('/docking/target', Scalar('STATION_C'))
    odom = Odometry()
    odom.pose.pose.position.x = .915
    odom.twist.twist.linear.x = .02
    ros.emit('/odom', odom)
    node.tick()
    assert not ros.outputs['/docking/reached'].data
    odom.twist.twist.linear.x = 0.
    ros.emit('/odom', odom)
    node.tick()
    assert ros.outputs['/docking/reached'].data
    odom.pose.pose.position.x = .94
    ros.emit('/odom', odom)
    node.tick()
    assert not ros.outputs['/docking/reached'].data


def test_manager_gates_start_transport_and_completion(ros):
    node = ros.system_manager.SystemManager()
    node.tick()
    assert node.state == 'WAIT_FOR_SIMULATOR'
    assert '/system/start' not in ros.outputs
    for topic in ('/odom', '/door/joint_states', '/robot/joint_states',
                  '/poses/wafer', '/poses/carrier', '/poses/transport_robot'):
        ros.emit(topic, Scalar())
    for topic in ('/line/valid', '/qr/healthy', '/carrier_present', '/door/closed'):
        ros.emit(topic, Scalar(True))
    ros.emit('/attachments/vacuum/state', Scalar('detached'))
    node.tick()
    assert node.state == 'HANDLING_WAFER'
    assert ros.outputs['/system/start'].data
    ros.emit('/carrier_ready', Scalar(True))
    node.tick()
    assert '/system/transport_enable' not in ros.outputs
    ros.emit('/wafer_handler/state', Scalar('TRANSFER_COMPLETE'))
    node.tick()
    assert node.state == 'TRANSPORTING'
    assert ros.outputs['/system/transport_enable'].data
    ros.emit('/carrier_delivered', Scalar(True))
    node.tick()
    assert node.state == 'TRANSPORTING'
    ros.emit('/transport/state', Scalar('DELIVERY_COMPLETE'))
    node.tick()
    assert node.state == 'COMPLETE'


def test_room_command_arbitration_and_heartbeat_stop(ros):
    node = make_transport(ros)
    feedback(ros, node)
    ros.emit('/system/start', Scalar(True))
    ros.emit('/wafer_handler/state', Scalar('ENTER_ROOM'))
    ros.emit('/door/open', Scalar(True))
    candidate = Twist()
    candidate.linear.x = .025
    ros.emit('/cleanroom/cmd_vel', candidate)
    node.tick()
    assert ros.outputs['/cmd_vel'].linear.x == .025
    assert not ros.outputs['/carrier_delivered'].data
    ros.time += 1.1
    feedback(ros, node)
    node.tick()
    assert node.state == 'FAULT'
    assert ros.outputs['/cmd_vel'].linear.x == 0.


def test_pickup_waits_for_closed_door(ros):
    node = ros.wafer_handler.WaferHandler()
    set_handler_state(ros, node, 'MOVE_TO_WAFER')
    room_feedback(ros, node)
    ros.emit('/poses/wafer', pose(.4, .12, .155))
    ros.emit('/door/joint_states', NS(name=['door_z'], position=[.52]))
    node.tick()
    assert node.state == 'FAULT'
    assert '/attachments/vacuum/attach' not in ros.outputs


def test_stuck_door_never_authorizes_entry(ros):
    node = ros.wafer_handler.WaferHandler()
    set_handler_state(ros, node, 'OPEN_ENTRY')
    room_feedback(ros, node, x=1.025, heading=0.)
    ros.emit('/poses/wafer', pose(.4, .12, .155))
    node.tick()
    assert node.state == 'OPEN_ENTRY'
    assert ros.outputs['/actuators/door_z'].data == .52
    assert ros.outputs['/cleanroom/cmd_vel'].linear.x == 0.
    assert ros.outputs['/cleanroom/cmd_vel'].angular.z == 0.
    ros.time += 31
    node.tick()
    assert node.state == 'FAULT'
    assert not ros.outputs['/carrier_ready'].data


def test_extended_wand_prevents_robot_passage(ros):
    node = ros.wafer_handler.WaferHandler()
    set_handler_state(ros, node, 'EXIT_ROOM')
    room_feedback(ros, node)
    ros.emit('/poses/wafer', pose(.65, .12, .159))
    ros.emit('/door/joint_states', NS(name=['door_z'], position=[.52]))
    ros.emit('/robot/joint_states', NS(name=['reach_1'], position=[.08]))
    node.tick()
    assert node.state == 'FAULT'
    assert ros.outputs['/cleanroom/cmd_vel'].linear.x == 0.
