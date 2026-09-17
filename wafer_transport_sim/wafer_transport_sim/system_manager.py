"""Readiness interlock and overall single-mission coordinator."""
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from rclpy.clock import Clock, ClockType
from .common import DemoNode, LATCHED, run


class SystemManager(DemoNode):
    def __init__(self):
        super().__init__('system_manager')
        for topic in ('/carrier_ready', '/carrier_present', '/carrier_delivered'):
            self.watch(topic, Bool, LATCHED)
        self.watch('/system/fault', String, LATCHED)
        self.watch('/wafer_handler/state', String, LATCHED)
        self.watch('/transport/state', String, LATCHED)
        self.watch('/odom', Odometry)
        self.watch('/door/joint_states', JointState)
        self.watch('/robot/joint_states', JointState)
        self.watch('/line/valid', Bool)
        self.watch('/qr/healthy', Bool)
        self.watch_attachment('vacuum')
        self.watch_pose('wafer')
        self.watch_pose('carrier')
        self.watch_pose('transport_robot')
        self.watch('/door/closed', Bool, LATCHED)
        self.change('WAIT_FOR_SIMULATOR')
        self.wall_started = time.monotonic()
        self.param('startup_timeout_wall', 90.0)
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def tick(self):
        if self.value('/system/fault'):
            self.change('FAULT')
        self.send('/system/state', String, self.state, True)
        if self.state == 'WAIT_FOR_SIMULATOR':
            ready = (self.now() > 0 and self.fresh(
                '/odom', '/door/joint_states', '/robot/joint_states',
                '/line/valid', '/qr/healthy', '/poses/wafer', '/poses/carrier',
                '/poses/transport_robot', '/door/closed') and
                self.value('/line/valid') and self.value('/qr/healthy') and
                self.value('/door/closed') and self.value('/carrier_present') and self.attachment('vacuum') == 'detached')
            if ready:
                self.send('/system/start', Bool, True, True)
                self.change('HANDLING_WAFER')
            elif time.monotonic() - self.wall_started > self.get_parameter('startup_timeout_wall').value:
                self.fault('Simulator readiness timeout; inspect cameras, bridge, joints and attachment states')
        elif self.state == 'HANDLING_WAFER':
            if self.value('/carrier_ready') and self.value('/wafer_handler/state') == 'TRANSFER_COMPLETE':
                self.send('/system/transport_enable', Bool, True, True)
                self.change('TRANSPORTING')
        elif self.state == 'TRANSPORTING':
            if self.value('/carrier_delivered') and self.value('/transport/state') == 'DELIVERY_COMPLETE':
                self.change('COMPLETE', 'TRANSPORT COMPLETE')


def main(args=None):
    run(SystemManager, args)
