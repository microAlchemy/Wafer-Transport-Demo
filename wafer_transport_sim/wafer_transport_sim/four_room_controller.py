'''One command owner for a feedback-gated, four-room wafer mission.

World pose bounds the route and docking stops. Camera pixels provide forward
steering. Doors, vacuum and wand transitions require measured feedback.
'''
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64
from rclpy.clock import Clock, ClockType
from .common import DemoNode, LATCHED, run
from .core import angle_error, clamp, placed, yaw
from .four_room_layout import ROOMS, START, LENGTH, project, room_stop, door_clear

TRANSFER = ('PICK_POSITION', 'PICK_LOWER', 'ATTACH', 'LIFT', 'PLACE_POSITION',
            'PLACE_LOWER', 'RELEASE', 'RETRACT', 'VERIFY_PLACE')
DRIVING = {'APPROACH_ENTRY', 'ENTER_ROOM', 'EXIT_ROOM', 'APPROACH_ENTRY_PICK', 'ENTER_ROOM_PICK', 'EXIT_ROOM_PICK', 'RETURN_HOME'}
TURNING = {'TURN_TO_TABLE_DROP', 'TURN_TO_EXIT_APPROACH', 'TURN_TO_TABLE_PICK', 'TURN_TO_EXIT_APPROACH_PICK'}


class FourRoomController(DemoNode):
    def __init__(self):
        super().__init__('transport_controller')
        for name, default in [('route_speed', .08), ('room_speed', .06),
                              ('turn_speed', .55), ('door_speed', .10),
                              ('mission_timeout', 1200.), ('startup_timeout_wall', 90.),
                              ('intermediate_transfers', True), ('placement_hold', 1.)]:
            self.param(name, default)
        for topic, kind in [('/odom', Odometry), ('/line/healthy', Bool), ('/line/valid', Bool),
                            ('/line/cmd_vel', Twist), ('/qr/healthy', Bool)]:
            self.watch(topic, kind)
        self.watch('/system/fault', String, LATCHED)
        self.watch_joints('/robot/joint_states')
        for room in ROOMS:
            for side in ('entry', 'exit'):
                self.watch_joints(f'/doors/room_{room.number}/{side}/joint_states')
        for name in ('wafer', 'carrier', 'transport_robot'):
            self.watch_pose(name)
        for name in ('vacuum', 'carrier'):
            self.watch_attachment(name)
        self.create_subscription(String, '/detected_station', self.detected, 10)
        self.index = 0
        self.completed = []
        self.confirmed = False
        self.progress = 0.
        self.last_projection = 0.
        self.door_targets = {}
        self.stable_since = None
        self.transfers = []
        self.source = 'carrier'
        self.destination = 'table'
        self.payload = 'carrier'
        self.payload_table = ROOMS[0].table
        self.started = None
        self.last_tick = self.now()
        self.wall_started = time.monotonic()
        self.last_clock_wall = time.monotonic()
        self.last_clock = self.now()
        self.last_report = 0.
        self.command = Twist()
        self.pub('/cmd_vel', Twist)
        self.change('WAIT_FOR_SIMULATOR')
        self.create_timer(.05, self.tick)
        self.create_timer(.25, self.wall_watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    @property
    def room(self):
        return ROOMS[self.index]

    def transition(self, state):
        self.stable_since = None
        self.change(state, f'[FourRooms] {self.room.code}: {state}')

    def detected(self, msg):
        if self.state in {'ENTER_ROOM', 'CONFIRM_ROOM', 'ENTER_ROOM_PICK', 'CONFIRM_ROOM_PICK'} and msg.data == self.room.code:
            self.confirmed = True

    def wall_watchdog(self):
        now = time.monotonic()
        if self.now() != self.last_clock:
            self.last_clock, self.last_clock_wall = self.now(), now
        if self.state == 'WAIT_FOR_SIMULATOR' and now-self.wall_started > self.param('startup_timeout_wall', 90.):
            self.fault('Simulator readiness timed out: ' + '; '.join(self.feedback_issues(*self.required())))
        elif self.started is not None and self.state not in {'FAULT', 'COMPLETE'} and now-self.last_clock_wall > 15.:
            self.fault('Simulation clock stopped for 15 wall seconds')
        if self.state == 'FAULT':
            self.pub('/cmd_vel', Twist).publish(Twist())
            self.send('/system/state', String, 'FAULT', True)
        if now-self.last_report > 3.:
            self.last_report = now
            if self.state == 'WAIT_FOR_SIMULATOR':
                issues = self.feedback_issues(*self.required())
                self.get_logger().info('[Startup] ' + ('; '.join(issues) or 'Waiting for valid tape and cameras'))
            elif self.state not in {'FAULT', 'COMPLETE'}:
                self.get_logger().info(f'[FourRooms] {self.room.code} {self.state}; '
                                       f'robot={self.position("transport_robot")}, vacuum={self.attachment("vacuum")}')

    def required(self):
        return ['/odom', '/robot/joint_states', '/poses/transport_robot', '/poses/wafer', '/poses/carrier',
                *[f'/doors/room_{r.number}/{side}/joint_states' for r in ROOMS for side in ('entry', 'exit')]]

    def heading(self):
        return yaw(self.values['/poses/transport_robot'].pose.orientation)

    def stopped(self):
        if not self.fresh('/odom'):
            return False
        t = self.values['/odom'].twist.twist
        return abs(t.linear.x) < .004 and abs(t.angular.z) < .02

    def stowed(self):
        return all(self.at_joint(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x', 'wand_z'))

    def reach(self, value):
        for name in ('reach_1', 'reach_2', 'wand_x'):
            self.joint(name, value/3)
        return all(self.at_joint(n, value/3, .0007) for n in ('reach_1', 'reach_2', 'wand_x'))

    def clear(self, room, side):
        x, y, _ = self.position('transport_robot')
        if abs(y-room.y) > .30:
            return True
        extension = sum(self.joints.get(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x'))
        return door_clear(x, y, self.heading(), room.door_x(side), extension)

    def door(self, side, opened, dt):
        name = self.room.door_joint(side)
        target = .535 if opened else 0.
        if not opened and (not self.stopped() or not self.clear(self.room, side)):
            self.fault('Door closure blocked: robot/wand has not cleared ' + name)
            return False
        current = self.door_targets.get(name, self.joints.get(name, 0.))
        step = clamp(self.param('door_speed', .40), .01, .50) * dt
        current += clamp(target-current, -step, step)
        self.door_targets[name] = current
        self.joint(name, current)
        return abs(current-target) < 1e-9 and self.at_joint(name, target)

    def align(self, target, tape=False):
        error = angle_error(target, self.heading())
        if abs(error) < .015:
            return self.stopped() and (not tape or (self.fresh('/line/valid') and self.value('/line/valid')))
        self.command.angular.z = clamp(2.5*error, -self.param('turn_speed', .55), self.param('turn_speed', .55))
        return False

    def drive(self, target, inside=False):
        if not self.stowed():
            self.fault('Travel requires a stowed wand')
            return False
        x, y, _ = self.position('transport_robot')
        if inside:
            side_s = 'entry' if self.state in {'ENTER_ROOM', 'ENTER_ROOM_PICK'} else 'exit'
            if not self.at_joint(self.room.door_joint(side_s), .535):
                self.fault('Passage requires fully open ' + side_s + ' door')
                return False
            # Inside, we drive by dead reckoning, not tape
            target_x = self.room.x
            target_y = self.room.y
            dx = target_x - x
            dy = target_y - y
            remaining = math.hypot(dx, dy)
            self.send('/docking_distance', Float64, remaining)
            if remaining <= .004:
                return self.stopped()
            speed = self.param('room_speed', .06)
            angle = math.atan2(dy, dx)
            self.command.linear.x = min(speed, 1.2 * remaining)
            error = angle_error(angle, self.heading())
            self.command.angular.z = clamp(2.5 * error, -.6, .6)
            return False
        else:
            if not self.fresh('/line/healthy', '/line/valid', '/line/cmd_vel') or not self.value('/line/healthy') or not self.value('/line/valid'):
                self.fault('Black tape or downward camera lost')
                return False
            _, deviation = project(x, y)
            if deviation > .08:
                self.fault(f'Robot departed tape corridor ({deviation:.3f} m)')
                return False
            remaining = target-self.progress
            self.send('/docking_distance', Float64, remaining)
            if remaining < -.02:
                self.fault('Passed route stop without verified arrival')
                return False
            if remaining <= .004:
                return self.stopped()
            candidate = self.values['/line/cmd_vel']
            speed = self.param('route_speed', .08)
            self.command.linear.x = min(max(0., candidate.linear.x), speed, 1.2*remaining)
            self.command.angular.z = clamp(candidate.angular.z, -.6, .6)
            return False

    def target_position(self, location):
        if location == 'table':
            return self.room.table
        cx, cy, cz = self.position('carrier')
        return cx, cy, cz+.006

    def stable(self, valid):
        if not valid:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = self.now()
        return self.now()-self.stable_since >= self.param('placement_hold', 1.)

    def transfer_step(self):
        if not self.stopped() or not all(self.at_joint(self.room.door_joint(side), 0.) for side in ('entry', 'exit')):
            self.fault('Wafer transfer requires stopped robot and both room doors closed')
            return
        source_reach = .25 if self.source == 'table' else 0.
        target_reach = .25 if self.destination == 'table' else 0.
        source_z = -.14 if self.source == 'table' else -.136
        target_z = -.14 if self.destination == 'table' else -.136
        verified = False
        s = self.state
        if s == 'PICK_POSITION':
            verified = self.reach(source_reach)
        elif s == 'PICK_LOWER':
            self.joint('wand_z', source_z)
            verified = self.at_joint('wand_z', source_z)
        elif s == 'ATTACH':
            rx, ry, _ = self.position('transport_robot')
            reach = sum(self.joints.get(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x'))
            cup = (rx+reach*math.cos(self.heading()), ry+reach*math.sin(self.heading()),
                   .300+self.joints.get('wand_z', 0.))
            if not placed(self.position('wafer'), self.target_position(self.source), .012, .008):
                self.fault('Wafer missing from ' + self.source)
                return
            if not placed(cup, self.position('wafer'), .012, .009):
                self.fault('Mobile wand is not aligned with wafer')
                return
            self.attach('vacuum', True)
            verified = self.attachment('vacuum') == 'attached'
            if verified:
                self.payload = 'wand'
        elif s == 'LIFT':
            self.joint('wand_z', 0.)
            verified = self.at_joint('wand_z', 0.) and self.position('wafer')[2] > .285
        elif s == 'PLACE_POSITION':
            verified = self.reach(target_reach)
        elif s == 'PLACE_LOWER':
            self.joint('wand_z', target_z)
            verified = self.at_joint('wand_z', target_z)
        elif s == 'RELEASE':
            if placed(self.position('wafer'), self.target_position(self.destination), .012, .008):
                self.attach('vacuum', False)
                verified = self.attachment('vacuum') == 'detached'
                if verified:
                    self.payload = self.destination
                    self.payload_table = self.room.table
        elif s == 'RETRACT':
            self.joint('wand_z', 0.)
            if self.at_joint('wand_z', 0.):
                verified = self.reach(0.)
        elif s == 'VERIFY_PLACE':
            verified = self.stable(self.attachment('vacuum') == 'detached' and self.stowed() and
                                  placed(self.position('wafer'), self.target_position(self.destination), .012, .005))
        if verified:
            if s != TRANSFER[-1]:
                self.transition(TRANSFER[TRANSFER.index(s)+1])
            elif self.transfers:
                self.begin_transfer()
            else:
                self.get_logger().info(f'[FourRooms] {self.room.code}: {self.source}->{self.destination} verified')
                if self.destination == 'table':
                    self.transition('TURN_TO_EXIT_APPROACH')
                else:
                    self.completed.append(self.room.code)
                    self.send('/rooms/completed', String, ','.join(self.completed), True)
                    self.transition('TURN_TO_EXIT_APPROACH_PICK')

    def begin_transfer(self):
        self.source, self.destination = self.transfers.pop(0)
        self.get_logger().info(f'[FourRooms] {self.room.code}: {self.source} -> {self.destination}')
        self.transition('PICK_POSITION')

    def tick(self):
        dt = clamp(self.now()-self.last_tick, 0., .1)
        self.last_tick = self.now()
        self.command = Twist()
        if self.value('/system/fault'):
            self.change('FAULT')
        if self.state not in {'WAIT_FOR_SIMULATOR', 'FAULT', 'COMPLETE'}:
            issues = self.feedback_issues(*self.required())
            if issues:
                self.fault('Required feedback lost: ' + '; '.join(issues))
            elif self.now()-self.entered > self.param('state_timeout', 60.):
                self.fault('State timed out: ' + self.state)
            elif self.started is not None and self.now()-self.started > self.param('mission_timeout', 1200.):
                self.fault('Four-room mission timed out')
            elif self.state not in {'INITIAL_DETACH', 'INITIAL_ATTACH', 'HOME'}:
                rx, ry, _ = self.position('transport_robot')
                if self.attachment('carrier') != 'attached' or not placed(self.position('carrier'), (rx, ry, .154), .02, .012):
                    self.fault('Onboard carrier retention lost')
                if self.payload == 'wand' and self.state != 'RELEASE' and self.attachment('vacuum') != 'attached':
                    self.fault('Vacuum attachment lost during transfer')
                if self.state in DRIVING | TURNING:
                    expected = self.target_position('carrier') if self.payload == 'carrier' else self.payload_table
                    pos = self.position('wafer')
                    if not placed(pos, expected, .022, .010):
                        self.fault('Wafer moved from its verified support')
                for room in ROOMS:
                    for side in ('entry', 'exit'):
                        active = room == self.room and ((side == 'entry' and self.state in
                            {'OPEN_ENTRY', 'ENTER_ROOM', 'CONFIRM_ROOM', 'CLOSE_ENTRY', 'OPEN_ENTRY_PICK', 'ENTER_ROOM_PICK', 'CONFIRM_ROOM_PICK', 'CLOSE_ENTRY_PICK'}) or
                            (side == 'exit' and self.state in {'OPEN_EXIT', 'EXIT_ROOM', 'CLOSE_EXIT', 'OPEN_EXIT_PICK', 'EXIT_ROOM_PICK', 'CLOSE_EXIT_PICK'}))
                        if not active and not self.at_joint(room.door_joint(side), 0.):
                            self.fault('Unexpected open door: ' + room.door_joint(side))
                if self.state in DRIVING | TURNING and (not self.fresh('/line/healthy') or not self.value('/line/healthy')) and not self.state.startswith('ENTER') and not self.state.startswith('EXIT'):
                    self.fault('Downward camera unavailable during motion')
                if self.state in TURNING and not self.stowed():
                    self.fault('Wand must be stowed before turning')
        if self.state not in {'WAIT_FOR_SIMULATOR', 'FAULT', 'COMPLETE'}:
            s, _ = project(*self.position('transport_robot')[:2])
            if self.last_projection != 0.0:
                delta = (s-self.last_projection+LENGTH/2) % LENGTH - LENGTH/2
                if abs(delta) > .25:
                    self.fault('Discontinuous robot pose on loop')
                else:
                    self.progress += delta
            self.last_projection = s
        self.step(dt)
        if self.state in {'FAULT', 'COMPLETE'}:
            self.command = Twist()
        if self.state == 'FAULT':
            for name in ('reach_1', 'reach_2', 'wand_x', 'wand_z', 'tray_z', 'tray_y',
                         *[r.door_joint(side) for r in ROOMS for side in ('entry', 'exit')]):
                if name in self.joints:
                    self.joint(name, self.joints[name])
        self.pub('/cmd_vel', Twist).publish(self.command)
        self.send('/transport/state', String, self.state, True)
        self.send('/system/state', String, self.state, True)
        self.send('/rooms/active', String, self.room.code, True)
        self.send('/carrier_ready', Bool, self.payload == 'carrier' and self.state != 'FAULT', True)
        self.send('/wafer_delivered', Bool, self.state == 'COMPLETE', True)

    def step(self, dt):
        s = self.state
        if s in {'FAULT', 'COMPLETE'}:
            return
        if s == 'WAIT_FOR_SIMULATOR':
            self.attach('vacuum', False)
            if (self.fresh(*self.required(), '/line/healthy', '/line/valid') and
                    self.value('/line/healthy') and self.value('/line/valid') and
                    self.attachment('vacuum') == 'detached' and self.stopped()):
                self.started = self.now()
                self.transition('INITIAL_DETACH')
        elif s == 'INITIAL_DETACH':
            self.attach('carrier', False)
            if self.attachment('carrier') == 'detached':
                self.transition('INITIAL_ATTACH')
        elif s == 'INITIAL_ATTACH':
            self.attach('carrier', True)
            if self.attachment('carrier') == 'attached':
                self.transition('HOME')
        elif s == 'HOME':
            self.joint('tray_z', .004)
            self.joint('tray_y', 0.)
            ready = self.stowed() and self.at_joint('tray_z', .004) and self.at_joint('tray_y', 0.)
            if ready:
                self.transition('APPROACH_ENTRY')
        elif s == 'APPROACH_ENTRY':
            if self.drive(room_stop(self.room, 'entry')):
                self.confirmed = False
                self.transition('OPEN_ENTRY')
        elif s == 'OPEN_ENTRY':
            if not self.stopped():
                return
            if self.door('entry', True, dt):
                self.transition('ENTER_ROOM')
        elif s == 'ENTER_ROOM':
            if self.drive(None, inside=True):
                self.transition('CONFIRM_ROOM')
        elif s == 'CONFIRM_ROOM':
            if self.confirmed and self.stopped() and self.fresh('/qr/healthy') and self.value('/qr/healthy'):
                self.transition('CLOSE_ENTRY')
        elif s == 'CLOSE_ENTRY':
            if self.door('entry', False, dt):
                self.transition('TURN_TO_TABLE_DROP')
        elif s == 'TURN_TO_TABLE_DROP':
            if self.align(self.room.work_heading):
                self.transfers = [('carrier', 'table')]
                self.begin_transfer()
        elif s in TRANSFER:
            self.transfer_step()
        elif s == 'TURN_TO_EXIT_APPROACH':
            if not self.stowed():
                self.fault('Wand must be stowed before turning toward exit')
            elif self.align(self.room.heading):
                self.transition('OPEN_EXIT')
        elif s == 'OPEN_EXIT':
            if self.door('exit', True, dt):
                self.transition('EXIT_ROOM')
        elif s == 'EXIT_ROOM':
            if self.drive(None, inside=True):
                self.transition('CLOSE_EXIT')
        elif s == 'CLOSE_EXIT':
            if self.door('exit', False, dt):
                self.transition('APPROACH_ENTRY_PICK')
        elif s == 'APPROACH_ENTRY_PICK':
            if self.drive(room_stop(self.room, 'entry')):
                self.confirmed = False
                self.transition('OPEN_ENTRY_PICK')
        elif s == 'OPEN_ENTRY_PICK':
            if not self.stopped():
                return
            if self.door('entry', True, dt):
                self.transition('ENTER_ROOM_PICK')
        elif s == 'ENTER_ROOM_PICK':
            if self.drive(None, inside=True):
                self.transition('CONFIRM_ROOM_PICK')
        elif s == 'CONFIRM_ROOM_PICK':
            if self.confirmed and self.stopped():
                self.transition('CLOSE_ENTRY_PICK')
        elif s == 'CLOSE_ENTRY_PICK':
            if self.door('entry', False, dt):
                self.transition('TURN_TO_TABLE_PICK')
        elif s == 'TURN_TO_TABLE_PICK':
            if self.align(self.room.work_heading):
                self.transfers = [('table', 'carrier')]
                self.begin_transfer()
        elif s == 'TURN_TO_EXIT_APPROACH_PICK':
            if not self.stowed():
                self.fault('Wand must be stowed before turning toward exit')
            elif self.align(self.room.heading):
                self.transition('OPEN_EXIT_PICK')
        elif s == 'OPEN_EXIT_PICK':
            if self.door('exit', True, dt):
                self.transition('EXIT_ROOM_PICK')
        elif s == 'EXIT_ROOM_PICK':
            if self.drive(room_stop(self.room, 'exit')):
                self.transition('CLOSE_EXIT_PICK')
        elif s == 'CLOSE_EXIT_PICK':
            if self.door('exit', False, dt):
                if self.index == 3:
                    self.transition('RETURN_HOME')
                else:
                    self.index += 1
                    self.transition('APPROACH_ENTRY')
        elif s == 'RETURN_HOME':
            if self.drive(LENGTH):
                self.transition('VERIFY_DELIVERY')
        elif s == 'VERIFY_DELIVERY':
            supported = (self.stopped() and self.stowed() and self.completed == [r.code for r in ROOMS] and
                         placed(self.position('transport_robot'), (*START, 0.), .02, .02) and
                         placed(self.position('wafer'), ROOMS[-1].table, .012, .005))
            if self.stable(supported):
                self.transition('COMPLETE')
                self.get_logger().info('TRANSPORT COMPLETE')


def main(args=None):
    run(FourRoomController, args)
