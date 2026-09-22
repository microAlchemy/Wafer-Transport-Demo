"""Four-probe Raspbot V2 IR reflectance-array line follower.

Gazebo has no stock colour-sensitive IR reflectance sensor. This node samples
the black tape geometry at the four physical probe locations using the live
model pose. Hardware can replace it with the Yahboom IR driver while retaining
the same topics and steering contract.
"""
import math

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Float64MultiArray

from .common import DemoNode, run
from .core import clamp, yaw
from .four_room_layout import project


OFFSETS = (-.030, -.010, .010, .030)  # right to left in base_link metres


def probe_values(x, y, heading, forward=.10684, radius=.025):
    """Return normalized black-tape responses for the four Raspbot probes."""
    c, s = math.cos(heading), math.sin(heading)
    values = []
    for lateral in OFFSETS:
        px = x + forward*c - lateral*s
        py = y + forward*s + lateral*c
        _, distance = project(px, py)
        values.append(clamp(1.-distance/radius, 0., 1.))
    return tuple(values)


def steering(values, speed=.20, kp=25., max_angular=1.4, curve_speed=.09):
    total = sum(values)
    if total <= .05:
        return None
    lateral = sum(offset*value for offset, value in zip(OFFSETS, values))/total
    angular = clamp(kp*lateral, -max_angular, max_angular)
    return (min(speed, curve_speed) if abs(angular) > .20 else speed), angular


class IrLineFollower(DemoNode):
    def __init__(self):
        super().__init__('ir_line_follower')
        for name, default in [('linear_speed', .20), ('kp', 25.),
                              ('max_angular_speed', 1.4), ('curve_speed', .09),
                              ('sensor_forward', .10684), ('detection_radius', .025)]:
            self.param(name, default)
        self.watch('/poses/transport_robot', PoseStamped)
        self.command = self.create_publisher(Twist, '/ir/cmd_vel', 10)
        self.create_timer(.05, self.tick)

    def tick(self):
        healthy = self.fresh('/poses/transport_robot')
        result = None
        values = (0., 0., 0., 0.)
        if healthy:
            msg = self.values['/poses/transport_robot']
            p, q = msg.pose.position, msg.pose.orientation
            get = lambda name: self.get_parameter(name).value
            values = probe_values(p.x, p.y, yaw(q), get('sensor_forward'), get('detection_radius'))
            result = steering(values, get('linear_speed'), get('kp'),
                              get('max_angular_speed'), get('curve_speed'))
        self.send('/ir/healthy', Bool, healthy)
        self.send('/ir/valid', Bool, result is not None)
        self.pub('/ir/values', Float64MultiArray).publish(Float64MultiArray(data=list(values)))
        command = Twist()
        if result is not None:
            command.linear.x, command.angular.z = result
        self.command.publish(command)


def main(args=None):
    run(IrLineFollower, args)
