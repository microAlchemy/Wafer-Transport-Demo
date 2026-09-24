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
            return NS(info=lambda _: None, warning=lambda _: None, error=lambda _: None)

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
        'std_msgs': dict(Bool=Scalar, String=Scalar, Float64=Scalar,
                         Float64MultiArray=Scalar, Empty=Scalar),
        'geometry_msgs': dict(Twist=Twist, PoseStamped=Scalar),
        'sensor_msgs': dict(JointState=Scalar),
        'nav_msgs': dict(Odometry=Odometry),
        'rcl_interfaces': dict(SetParametersResult=Scalar),
    }.items():
        module(package)
        module(package + '.msg', **symbols)
    for name in ('common', 'four_room_controller', 'ir_line_follower', 'process_cell_controller'):
        key = 'wafer_transport_sim.' + name
        monkeypatch.delitem(sys.modules, key, raising=False)
        mod = importlib.import_module(key)
        monkeypatch.setitem(sys.modules, key, mod)
        setattr(bus, name, mod)
    return bus


def pose(x, y, z, heading=math.pi/2):
    return NS(pose=NS(position=NS(x=x, y=y, z=z),
                      orientation=NS(x=0., y=0., z=math.sin(heading/2), w=math.cos(heading/2))))
