"""Mission arbitration and feedback-verified supported carrier delivery."""
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64
from rcl_interfaces.msg import SetParametersResult
from .common import DemoNode, LATCHED, run
from .core import STATIONS, MOVING, destination_error, select_command, placed


class TransportController(DemoNode):
    def __init__(self):
        super().__init__('transport_controller')
        self.param('destination_station', 'STATION_C')
        self.distances = dict(zip(STATIONS, self.param('station_distances', [0.28, 0.60, 0.92])))
        self.param('slow_distance', 0.10)
        self.param('position_tolerance', 0.01)
        self.param('mission_timeout', 120.0)
        self.watch('/carrier_ready', Bool, LATCHED)
        self.watch('/system/transport_enable', Bool, LATCHED)
        self.watch('/system/fault', String, LATCHED)
        for topic, kind in [('/odom', Odometry), ('/line/cmd_vel', Twist),
                            ('/line/valid', Bool), ('/qr/healthy', Bool),
                            ('/docking/cmd_vel', Twist), ('/docking/reached', Bool),
                            ('/docking_distance', Float64)]:
            self.watch(topic, kind)
        self.watch_attachment('carrier')
        self.watch_joints('/robot/joint_states')
        self.watch_pose('carrier')
        self.watch_pose('wafer')
        self.create_subscription(String, '/detected_station', self.detected, 10)
        self.add_on_set_parameters_callback(self.parameters)
        self.pub('/cmd_vel', Twist)
        self.target = None
        self.pending_qr = None
        self.seen = set()
        self.initialized = False
        self.init_phase = 'DETACH'
        self.drop_phase = 'EXTEND'
        self.stable_since = None
        self.started = None
        self.change('WAIT_FOR_CARRIER', '[Transport] Waiting for wafer carrier')
        self.create_timer(0.05, self.tick)

    def parameters(self, params):
        for p in params:
            if p.name == 'destination_station':
                reason = destination_error(p.value, self.target is not None)
                if reason:
                    return SetParametersResult(successful=False, reason=reason)
        return SetParametersResult(successful=True)

    def detected(self, msg):
        if self.state not in {'FOLLOW_PATH', 'SCAN_QR', 'CONTINUE'}:
            return
        if msg.data in STATIONS and msg.data not in self.seen:
            # Rejecting a station does not suppress a different QR in the frame.
            if self.pending_qr is None or msg.data == self.target:
                self.pending_qr = msg.data

    def carrier_on_robot(self):
        if not self.fresh('/poses/carrier', '/odom'):
            return False
        odom = self.values['/odom']
        p = odom.pose.pose.position
        return placed(self.position('carrier'),
                      (1.025 - p.y, 0.12 + p.x, 0.154), 0.02, 0.012)

    def tick(self):
        self.send('/transport/state', String, self.state, True)
        present = self.initialized and self.attachment('carrier') == 'attached'
        self.send('/carrier_present', Bool, present, True)
        self.send('/carrier_delivered', Bool, self.state == 'DELIVERY_COMPLETE', True)
        if self.value('/system/fault'):
            self.change('FAULT')
        if self.state not in {'WAIT_FOR_CARRIER', 'FAULT', 'DELIVERY_COMPLETE'}:
            if self.now() - self.entered > self.get_parameter('state_timeout').value:
                self.fault('Transport state timed out: ' + self.state)
        if self.started is not None and self.state not in {'FAULT', 'DELIVERY_COMPLETE'}:
            if self.now() - self.started > self.get_parameter('mission_timeout').value:
                self.fault('Mission timeout')
        if self.state in MOVING:
            required = ['/odom', '/line/cmd_vel', '/line/valid', '/qr/healthy',
                        '/poses/carrier', '/poses/wafer']
            if not self.fresh(*required) or not self.value('/line/valid') or not self.value('/qr/healthy'):
                self.fault('Required camera, line, odometry, or pose feedback lost')
            elif (not self.carrier_on_robot() or self.attachment('carrier') != 'attached' or
                  not placed(self.position('wafer'),
                             tuple(v + d for v, d in zip(self.position('carrier'), (0, 0, .006))),
                             .022, .010)):
                self.fault('Carrier retention verification failed')
            elif self.values['/odom'].pose.pose.position.x > self.distances[self.target] + self.get_parameter('position_tolerance').value:
                self.fault('Passed destination without a verified dock')
        self.step()
        if self.state == 'FAULT':
            for name in ('tray_y', 'tray_z'):
                if name in self.joints:
                    self.joint(name, self.joints[name])
        line = self.values.get('/line/cmd_vel', Twist())
        dock = self.values.get('/docking/cmd_vel', Twist())
        fresh = self.fresh('/line/cmd_vel', '/odom')
        if self.state in {'SLOW_APPROACH', 'PRECISION_DOCK'}:
            fresh = fresh and self.fresh('/docking/cmd_vel', '/docking/reached', '/docking_distance')
            if not fresh and self.now() - self.entered > 0.5:
                self.fault('Docking feedback lost')
        v, w = select_command(self.state, (line.linear.x, line.angular.z),
                              (dock.linear.x, dock.angular.z), fresh)
        cmd = Twist()
        cmd.linear.x, cmd.angular.z = v, w
        self.pub('/cmd_vel', Twist).publish(cmd)

    def step(self):
        s = self.state
        if s == 'WAIT_FOR_CARRIER':
            # Re-handshake the stock initially-attached joint. The loading rack
            # supports the carrier while detached; robot tray has no collision.
            if not self.initialized:
                if self.init_phase == 'DETACH':
                    self.attach('carrier', False)
                    if self.attachment('carrier') == 'detached':
                        self.init_phase = 'ATTACH'
                else:
                    self.attach('carrier', True)
                    self.initialized = self.attachment('carrier') == 'attached'
            if (self.initialized and self.value('/carrier_ready') and
                    self.value('/system/transport_enable')):
                self.change('VERIFY_CARRIER', '[Transport] Carrier detected')
        elif s == 'VERIFY_CARRIER':
            # Clear the loading rack before starting wheel motion.
            self.joint('tray_z', 0.004)
            if self.at_joint('tray_z', 0.004) and self.carrier_on_robot():
                self.change('READ_DESTINATION')
        elif s == 'READ_DESTINATION':
            value = self.get_parameter('destination_station').value
            error = destination_error(value, False)
            if error:
                self.fault(error)
                return
            self.target = value
            self.started = self.now()
            self.change('START_TRANSPORT', '[Transport] Destination: ' + value)
        elif s == 'START_TRANSPORT':
            self.change('FOLLOW_PATH', '[Transport] Beginning transport; following corridor')
        elif s == 'FOLLOW_PATH' and self.pending_qr:
            self.change('SCAN_QR', '[Transport] QR detected: ' + self.pending_qr)
        elif s == 'SCAN_QR':
            self.change('IS_TARGET_QR')
        elif s == 'IS_TARGET_QR':
            station = self.pending_qr
            self.seen.add(station)
            self.pending_qr = None
            if station == self.target:
                self.send('/docking/target', String, self.target, True)
                self.change('SLOW_APPROACH', '[Transport] Destination confirmed; slowing for docking')
            else:
                self.change('CONTINUE', '[Transport] Station does not match destination')
        elif s == 'CONTINUE':
            self.change('FOLLOW_PATH', '[Transport] Continuing transport')
        elif s == 'SLOW_APPROACH':
            if self.fresh('/docking_distance') and self.value('/docking_distance') <= self.get_parameter('slow_distance').value:
                self.change('PRECISION_DOCK', '[Transport] Docking')
        elif s == 'PRECISION_DOCK':
            if self.fresh('/docking/reached') and self.value('/docking/reached'):
                self.change('STOP', '[Transport] Position reached')
        elif s == 'STOP':
            if self.fresh('/odom') and abs(self.values['/odom'].twist.twist.linear.x) < 0.004:
                self.change('DROP_CARRIER')
        elif s == 'DROP_CARRIER':
            self.drop()
        elif s == 'VERIFY_DROP':
            y = 0.12 + self.distances[self.target]
            ok = (self.fresh('/poses/carrier', '/poses/wafer') and
                  self.attachment('carrier') == 'detached' and
                  self.at_joint('tray_y', 0.0) and self.at_joint('tray_z', -0.026) and
                  placed(self.position('carrier'), (1.145, y, 0.134)) and
                  placed(self.position('wafer'), (1.145, y, 0.140), 0.022, 0.010))
            if not ok:
                self.stable_since = None
            elif self.stable_since is None:
                self.stable_since = self.now()
            elif self.now() - self.stable_since >= 1.0:
                self.change('DELIVERY_COMPLETE', '[Transport] Carrier delivered; mission complete')

    def drop(self):
        if not self.fresh('/poses/carrier', '/odom'):
            self.fault('Drop feedback stale')
            return
        if abs(self.values['/odom'].twist.twist.linear.x) >= 0.004:
            self.fault('Robot moved during carrier transfer')
            return
        if self.drop_phase == 'EXTEND':
            self.joint('tray_y', -0.12)
            if self.at_joint('tray_y', -0.12):
                self.drop_phase = 'LOWER'
        elif self.drop_phase == 'LOWER':
            self.joint('tray_z', -0.016)
            if self.at_joint('tray_z', -0.016):
                self.drop_phase = 'RELEASE'
        elif self.drop_phase == 'RELEASE':
            target = (1.145, 0.12 + self.distances[self.target], 0.134)
            if placed(self.position('carrier'), target):
                self.attach('carrier', False)
                if self.attachment('carrier') == 'detached':
                    self.drop_phase = 'CLEAR'
        elif self.drop_phase == 'CLEAR':
            self.joint('tray_z', -0.026)
            if self.at_joint('tray_z', -0.026):
                self.drop_phase = 'RETRACT'
        elif self.drop_phase == 'RETRACT':
            self.joint('tray_y', 0.0)
            if self.at_joint('tray_y', 0.0):
                self.change('VERIFY_DROP')


def main(args=None):
    run(TransportController, args)
