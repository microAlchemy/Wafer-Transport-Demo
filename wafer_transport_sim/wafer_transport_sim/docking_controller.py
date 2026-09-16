"""Odometry-gated final approach after camera-based target authorization."""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Bool
from .common import DemoNode, LATCHED, run
from .core import STATIONS, docking_speed, clamp, yaw, angle_error


class DockingController(DemoNode):
    def __init__(self):
        super().__init__('docking_controller')
        self.watch('/odom', Odometry)
        self.watch('/docking/target', String, LATCHED)
        self.poses = dict(zip(STATIONS, self.param('station_distances', [0.28, 0.60, 0.92])))
        self.param('position_tolerance', 0.01)
        self.param('approach_speed', 0.018)
        self.param('dock_gain', 0.8)
        self.param('stop_velocity', 0.004)
        self.pub('/docking/cmd_vel', Twist)
        self.create_timer(0.05, self.tick)

    def tick(self):
        cmd = Twist()
        target = self.value('/docking/target')
        reached = False
        if target in self.poses and self.fresh('/odom'):
            odom = self.values['/odom']
            distance = self.poses[target] - odom.pose.pose.position.x
            self.send('/docking_distance', Float64, distance)
            tolerance = self.get_parameter('position_tolerance').value
            cmd.linear.x = docking_speed(distance, tolerance,
                                         self.get_parameter('approach_speed').value,
                                         self.get_parameter('dock_gain').value)
            heading = yaw(odom.pose.pose.orientation)
            lateral = odom.pose.pose.position.y
            cmd.angular.z = clamp(1.5 * angle_error(0.0, heading) - 2 * lateral, -0.25, 0.25)
            if abs(distance) <= tolerance:
                cmd = Twist()
                reached = (abs(odom.twist.twist.linear.x) <
                           self.get_parameter('stop_velocity').value and
                           abs(odom.twist.twist.angular.z) < 0.02 and
                           abs(lateral) < 0.012 and abs(heading) < 0.05)
        self.pub('/docking/cmd_vel', Twist).publish(cmd)
        self.send('/docking/reached', Bool, reached)


def main(args=None):
    run(DockingController, args)
