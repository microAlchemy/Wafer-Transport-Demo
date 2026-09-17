"""Readiness interlock and overall single-mission coordinator."""
import time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String, Float64
from rclpy.clock import Clock, ClockType
from .common import DemoNode, LATCHED, run


class SystemManager(DemoNode):
    def __init__(self):
        super().__init__('system_manager')
        for topic in ('/carrier_ready', '/carrier_present', '/carrier_delivered', '/cleanroom/exited'):
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
        self.watch('/actuators/door_z', Float64)
        self.change('WAIT_FOR_SIMULATOR')
        self.wall_started = time.monotonic()
        self.last_diagnostic_wall = self.wall_started - 2.0
        self.param('startup_timeout_wall', 90.0)
        self.create_timer(0.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def tick(self):
        if self.value('/system/fault'):
            self.change('FAULT')
        self.send('/system/state', String, self.state, True)
        if self.state == 'WAIT_FOR_SIMULATOR':
            pending = self.readiness_pending()
            ready = not pending
            self.send('/system/readiness', String,
                      'READY' if ready else '; '.join(pending), True)
            if pending and time.monotonic() - self.last_diagnostic_wall >= 2.0:
                self.get_logger().info('[Startup] Waiting for: ' + '; '.join(pending))
                self.last_diagnostic_wall = time.monotonic()
            if ready:
                self.send('/system/start', Bool, True, True)
                self.change('HANDLING_WAFER')
            elif time.monotonic() - self.wall_started > self.get_parameter('startup_timeout_wall').value:
                self.fault('Simulator readiness timeout: ' + '; '.join(pending))
        elif self.state == 'HANDLING_WAFER':
            if time.monotonic() - self.last_diagnostic_wall >= 2.0:
                joints = {}
                for topic in ('/robot/joint_states', '/door/joint_states'):
                    msg = self.values.get(topic)
                    joints.update({n.split('::')[-1]: round(p, 4) for n, p in
                                   zip(getattr(msg, 'name', []), getattr(msg, 'position', []))
                                   if 'wheel' not in n})
                self.get_logger().info(
                    '[Pickup] state=' + str(self.value('/wafer_handler/state')) +
                    ' robot_xyz=' + str(tuple(round(v, 4) for v in self.position('transport_robot'))) +
                    ' joints=' + str(joints) +
                    ' vacuum=' + str(self.attachment('vacuum')) +
                    ' door_target=' + str(self.value('/actuators/door_z')) +
                    ' tape_detected=' + str(self.value('/line/valid')))
                self.last_diagnostic_wall = time.monotonic()
            if (self.value('/carrier_ready') and self.value('/cleanroom/exited') and
                    self.value('/wafer_handler/state') == 'TRANSFER_COMPLETE'):
                self.send('/system/transport_enable', Bool, True, True)
                self.change('TRANSPORTING')
        elif self.state == 'TRANSPORTING':
            if self.value('/carrier_delivered') and self.value('/transport/state') == 'DELIVERY_COMPLETE':
                self.change('COMPLETE', 'TRANSPORT COMPLETE')

    def readiness_pending(self):
        pending = []
        if self.now() <= 0:
            pending.append('/clock: no simulation time (world failed to load or is paused)')
        for topic in ('/odom', '/door/joint_states', '/robot/joint_states',
                      '/line/valid', '/qr/healthy', '/poses/wafer', '/poses/carrier',
                      '/poses/transport_robot', '/door/closed', '/carrier_present'):
            if topic not in self.received:
                pending.append(topic + ': no messages')
            elif not self.fresh(topic):
                age = self.now() - self.received[topic]
                pending.append(f'{topic}: stale ({age:.2f}s simulation time)')
        for topic, reason in (
                ('/line/valid', 'camera has not found the floor stripe'),
                ('/qr/healthy', 'forward camera processing failed'),
                ('/door/closed', 'door joint has not confirmed closure'),
                ('/carrier_present', 'carrier detach/attach handshake incomplete')):
            if self.fresh(topic) and not self.value(topic):
                pending.append(topic + ': ' + reason)
        if self.attachment('vacuum') != 'detached':
            pending.append('/attachments/vacuum/state: waiting for detached acknowledgement')
        return pending


def main(args=None):
    run(SystemManager, args)
