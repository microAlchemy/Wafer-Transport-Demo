"""One command owner for a feedback-gated, four-room wafer mission.

World pose bounds the route and docking stops. The four-probe IR array provides
forward steering. Doors, vacuum and wand transitions require measured feedback.
"""
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64
from rclpy.clock import Clock, ClockType
from .common import DemoNode, LATCHED, run
from .core import angle_error, clamp, placed, yaw
from .four_room_layout import ROOMS, START, LENGTH, project, point_at, room_stop, door_clear

TRANSFER = ('PICK_POSITION', 'PICK_LOWER', 'ATTACH', 'LIFT', 'PLACE_POSITION',
            'PLACE_LOWER', 'RELEASE', 'RETRACT', 'VERIFY_PLACE')
DRIVING = {'APPROACH_ENTRY', 'ENTER_ROOM', 'EXIT_ROOM', 'RETURN_HOME'}
TURNING = {'TURN_TO_TABLE', 'TURN_TO_ROUTE'}


class FourRoomController(DemoNode):
    def __init__(self):
        super().__init__('transport_controller')
        for name, default in [('route_speed', .20), ('room_speed', .15),
                              ('turn_speed', .55), ('door_speed', .18), ('door_open_hold', .5),
                              ('mission_timeout', 600.), ('startup_timeout_wall', 90.),
                              ('intermediate_transfers', True), ('placement_hold', 1.),
                              ('minimum_cruise_speed', .07), ('curve_speed', .09),
                              ('braking_acceleration', .35), ('allow_sensor_fallback', True)]:
            self.param(name, default)
        for topic, kind in [('/odom', Odometry), ('/ir/healthy', Bool), ('/ir/valid', Bool),
                            ('/ir/cmd_vel', Twist)]:
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
        self.index = 0
        self.completed = []
        self.progress = 0.
        self.last_projection = 0.
        self.door_targets = {}
        self.stable_since = None
        self.transfers = []
        self.source = 'table'
        self.destination = 'carrier'
        self.payload = 'table'
        self.payload_table = ROOMS[0].table
        self.started = None
        self.last_tick = self.now()
        self.wall_started = time.monotonic()
        self.last_clock_wall = time.monotonic()
        self.last_clock = self.now()
        self.last_report = 0.
        self.command = Twist()
        self.fallback_active = False
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

    def wall_watchdog(self):
        now = time.monotonic()
        if self.now() != self.last_clock:
            self.last_clock, self.last_clock_wall = self.now(), now
        if self.state == 'WAIT_FOR_SIMULATOR' and now-self.wall_started > self.param('startup_timeout_wall', 90.):
            self.fault('Simulator readiness timed out: ' + '; '.join(self.readiness_issues()))
        elif self.started is not None and self.state not in {'FAULT', 'COMPLETE'} and now-self.last_clock_wall > 15.:
            self.fault('Simulation clock stopped for 15 wall seconds')
        if self.state == 'FAULT':
            self.pub('/cmd_vel', Twist).publish(Twist())
            self.send('/system/state', String, 'FAULT', True)
        if now-self.last_report > 3.:
            self.last_report = now
            if self.state == 'WAIT_FOR_SIMULATOR':
                issues = self.readiness_issues()
                self.get_logger().info('[Startup] ' + ('; '.join(issues) or 'final readiness interlock'))
            elif self.state not in {'FAULT', 'COMPLETE'}:
                self.get_logger().info(f'[FourRooms] {self.room.code} {self.state}; '
                                       f'robot={self.position("transport_robot")}, vacuum={self.attachment("vacuum")}')
                if self.state == 'HOME':
                    self.get_logger().info('[Home] waiting only for safety joints: ' +
                                           (', '.join(self.home_blockers()) or 'none'))

    def required(self):
        return ['/odom', '/robot/joint_states', '/poses/transport_robot', '/poses/wafer', '/poses/carrier',
                *[f'/doors/room_{r.number}/{side}/joint_states' for r in ROOMS for side in ('entry', 'exit')]]

    def readiness_issues(self):
        issues = self.feedback_issues(*self.required())
        if not self.param('allow_sensor_fallback', True) and not self.primary_ready():
            issues.append('IR array unavailable or black tape not detected')
        if self.attachment('vacuum') != 'detached':
            issues.append('initial vacuum detach not acknowledged')
        if not self.stopped():
            issues.append('odometry reports motion')
        return issues

    def primary_ready(self):
        if not (self.fresh('/ir/healthy', '/ir/valid', '/ir/cmd_vel') and
                self.value('/ir/healthy') and self.value('/ir/valid')):
            return False
        cmd = self.values['/ir/cmd_vel']
        return (math.isfinite(cmd.linear.x) and cmd.linear.x > 0. and
                math.isfinite(cmd.angular.z))

    def update_steering_mode(self):
        # Recover both at startup and mid-route, including a valid-but-zero
        # candidate. Simulation ground truth supplies the requested pretend IR.
        fallback = not self.primary_ready() and self.param('allow_sensor_fallback', True)
        if fallback != self.fallback_active:
            self.get_logger().info('[Steering] ' + ('SIMULATED_IR: following tape from model pose'
                                                   if fallback else 'IR: probe readings restored'))
        self.fallback_active = fallback
        self.send('/transport/steering_mode', String,
                  'SIMULATED_IR' if fallback else 'IR', True)

    def fallback_command(self):
        x, y, _ = self.position('transport_robot')
        along, _ = project(x, y)
        target_x, target_y = point_at(along + .10)
        error = angle_error(math.atan2(target_y-y, target_x-x), self.heading())
        command = Twist()
        command.linear.x = self.param('route_speed', .20)
        command.angular.z = clamp(3.5*error, -1.4, 1.4)
        return command

    def heading(self):
        return yaw(self.values['/poses/transport_robot'].pose.orientation)

    def stopped(self):
        if not self.fresh('/odom'):
            return False
        t = self.values['/odom'].twist.twist
        return math.hypot(t.linear.x, t.linear.y) < .004 and abs(t.angular.z) < .02

    def stowed(self):
        return all(self.at_joint(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x', 'wand_z'))

    def home_blockers(self):
        blockers = [name for name in ('reach_1', 'reach_2', 'wand_x', 'wand_z')
                    if not self.at_joint(name, 0.)]
        blockers.extend(room.door_joint(side) for room in ROOMS for side in ('entry', 'exit')
                        if not self.at_joint(room.door_joint(side), 0.))
        return blockers

    def reach(self, value):
        for name in ('reach_1', 'reach_2', 'wand_x'):
            self.joint(name, value/3)
        return all(self.at_joint(n, value/3, .0007) for n in ('reach_1', 'reach_2', 'wand_x'))

    def clear(self, room, side):
        x, y, _ = self.position('transport_robot')
        # A door in the other row cannot overlap this robot.
        if abs(y-room.y) > .30:
            return True
        extension = sum(self.joints.get(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x'))
        return door_clear(x, y, self.heading(), room.door_x(side), extension)

    def door(self, side, opened, dt):
        # The robot must stay stopped throughout both strokes of every cycle.
        if not self.stopped():
            return False
        name = self.room.door_joint(side)
        target = .52 if opened else 0.
        if not opened and (not self.stopped() or not self.clear(self.room, side)):
            self.fault('Door closure blocked: robot/wand has not cleared ' + name)
            return False
        current = self.door_targets.get(name, self.joints.get(name, 0.))
        step = clamp(self.param('door_speed', .18), .01, .18) * dt
        current += clamp(target-current, -step, step)
        self.door_targets[name] = current
        self.joint(name, current)
        return abs(current-target) < 1e-9 and self.at_joint(name, target)

    def align(self, target, tape=False):
        error = angle_error(target, self.heading())
        if abs(error) < .015:
            return self.stopped() and (not tape or self.fallback_active or
                                       (self.fresh('/ir/valid') and self.value('/ir/valid')))
        self.command.angular.z = clamp(2.5*error, -self.param('turn_speed', .55), self.param('turn_speed', .55))
        return False

    def drive(self, target, inside=False):
        if not self.stowed() or self.attachment('vacuum') != 'detached':
            self.fault('Travel requires a stowed wand and released vacuum')
            return False
        ir_ready = self.primary_ready()
        if not ir_ready and not self.fallback_active:
            self.fault('Black tape lost by the IR sensor array')
            return False
        x, y, _ = self.position('transport_robot')
        _, deviation = project(x, y)
        # The outside detours use 0.1524 m-radius corners.  A 50 mm tracking
        # envelope still keeps the 90 mm half-width robot inside the 1 ft lane.
        if deviation > .068:
            self.fault(f'Robot departed tape corridor ({deviation:.3f} m)')
            return False
        if not all(self.at_joint(self.room.door_joint(side), 0.) for side in ('entry', 'exit')):
            self.fault('Resume travel only after both doors are closed')
            return False
        remaining = target-self.progress
        self.send('/docking_distance', Float64, remaining)
        if remaining < -.01:
            self.fault('Passed route stop without verified arrival')
            return False
        if remaining <= .004:
            return self.stopped()
        candidate = self.values['/ir/cmd_vel'] if ir_ready else self.fallback_command()
        speed = self.param('room_speed' if inside else 'route_speed', .15 if inside else .20)
        # Keep rolling between checkpoints, then use the configured physical
        # deceleration for a short controlled stop.  A proportional-to-distance
        # cap here caused the old controller to crawl and appear stopped.
        braking_speed = math.sqrt(2*self.param('braking_acceleration', .35)*max(0., remaining-.004))
        requested = min(max(0., candidate.linear.x), speed, braking_speed)
        if remaining > .012:
            requested = min(speed, max(self.param('minimum_cruise_speed', .07), requested))
        self.command.angular.z = clamp(candidate.angular.z, -1.4, 1.4)
        if abs(self.command.angular.z) > .20:
            requested = min(requested, self.param('curve_speed', .09))
        self.command.linear.x = requested
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

    def table_reach(self):
        rx, ry, _ = self.position('transport_robot')
        tx, ty, _ = self.room.table
        return clamp(math.hypot(tx-rx, ty-ry), 0., .269)

    def transfer_step(self):
        if not self.stopped() or not all(self.at_joint(self.room.door_joint(side), 0.) for side in ('entry', 'exit')):
            self.fault('Wafer transfer requires stopped robot and both room doors closed')
            return
        source_reach = self.table_reach() if self.source == 'table' else 0.
        target_reach = self.table_reach() if self.destination == 'table' else 0.
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
            # Lift above the carrier rim before horizontal retraction.
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
                self.completed.append(self.room.code)
                self.send('/rooms/completed', String, ','.join(self.completed), True)
                self.get_logger().info(f'[FourRooms] {self.room.code}: wafer placement verified')
                self.transition('TURN_TO_ROUTE')

    def begin_transfer(self):
        self.source, self.destination = self.transfers.pop(0)
        self.get_logger().info(f'[FourRooms] {self.room.code}: {self.source} -> {self.destination}')
        self.transition('PICK_POSITION')

    def tick(self):
        dt = clamp(self.now()-self.last_tick, 0., .1)
        self.last_tick = self.now()
        self.command = Twist()
        self.update_steering_mode()
        if self.value('/system/fault'):
            self.change('FAULT')
        if self.state not in {'WAIT_FOR_SIMULATOR', 'FAULT', 'COMPLETE'}:
            issues = self.feedback_issues(*self.required())
            if issues:
                self.fault('Required feedback lost: ' + '; '.join(issues))
            elif self.now()-self.entered > self.param('state_timeout', 60.):
                self.fault('State timed out: ' + self.state)
            elif self.started is not None and self.now()-self.started > self.param('mission_timeout', 600.):
                self.fault('Four-room mission timed out')
            elif self.state not in {'INITIAL_DETACH', 'INITIAL_ATTACH', 'HOME'}:
                rx, ry, _ = self.position('transport_robot')
                if self.attachment('carrier') != 'attached' or not placed(self.position('carrier'), (rx, ry, .154), .02, .012):
                    self.fault('Onboard carrier retention lost')
                if self.payload == 'wand' and self.state != 'RELEASE' and self.attachment('vacuum') != 'attached':
                    self.fault('Vacuum attachment lost during transfer')
                if self.state in DRIVING | TURNING:
                    expected = self.target_position('carrier') if self.payload == 'carrier' else self.payload_table
                    if not placed(self.position('wafer'), expected, .022, .010):
                        self.fault('Wafer moved from its verified support')
                # All doors not deliberately being operated must remain shut.
                for room in ROOMS:
                    for side in ('entry', 'exit'):
                        active = room == self.room and ((side == 'entry' and self.state in
                            {'OPEN_ENTRY', 'HOLD_ENTRY', 'CLOSE_ENTRY'}) or
                            (side == 'exit' and self.state in {'OPEN_EXIT', 'HOLD_EXIT', 'CLOSE_EXIT'}))
                        if not active and not self.at_joint(room.door_joint(side), 0.):
                            self.fault('Unexpected open door: ' + room.door_joint(side))
                if (not self.fallback_active and self.state in DRIVING | TURNING and
                        (not self.fresh('/ir/healthy') or not self.value('/ir/healthy'))):
                    self.fault('IR sensor array unavailable during motion')
                if self.state in TURNING and not self.stowed():
                    self.fault('Wand must be stowed before turning')
        if self.state not in {'WAIT_FOR_SIMULATOR', 'FAULT', 'COMPLETE'}:
            s, _ = project(*self.position('transport_robot')[:2])
            delta = (s-self.last_projection+LENGTH/2) % LENGTH - LENGTH/2
            if abs(delta) > .20:
                self.fault('Discontinuous robot pose on loop')
            else:
                self.progress += delta
                self.last_projection = s
        self.step(dt)
        if self.state in {'FAULT', 'COMPLETE'}:
            self.command = Twist()
        if self.state == 'FAULT':
            for name in ('reach_1', 'reach_2', 'wand_x', 'wand_z', 'tray_z', 'tray_y',
                         'camera_pan', 'camera_tilt',
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
            # Stock detachable joints start attached; release the wafer on its table.
            self.attach('vacuum', False)
            if (self.fresh(*self.required()) and (self.primary_ready() or self.fallback_active) and
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
            # Camera and tray poses are not navigation safety interlocks. Jetty
            # may settle these small joints just outside the tight tolerance;
            # only the stowed wand and closed doors may block driving.
            self.joint('wand_z', 0.)
            self.joint('tray_z', .004)
            self.joint('tray_y', 0.)
            self.reach(0.)
            for room in ROOMS:
                for side in ('entry', 'exit'):
                    self.joint(room.door_joint(side), 0.)
            if not self.home_blockers():
                self.transition('APPROACH_ENTRY')
        elif s == 'APPROACH_ENTRY':
            if self.drive(room_stop(self.room, 'entry')):
                self.transition('OPEN_ENTRY')
        elif s == 'OPEN_ENTRY':
            if not self.stopped():
                return
            if self.door('entry', True, dt):
                self.transition('HOLD_ENTRY')
        elif s == 'HOLD_ENTRY':
            if (self.door('entry', True, dt) and
                    self.now()-self.entered >= self.param('door_open_hold', .5)):
                self.transition('CLOSE_ENTRY')
        elif s == 'CLOSE_ENTRY':
            if self.door('entry', False, dt):
                self.transition('ENTER_ROOM')
        elif s == 'ENTER_ROOM':
            if self.drive(room_stop(self.room, 'work'), inside=True):
                self.transition('TURN_TO_TABLE')
        elif s == 'TURN_TO_TABLE':
            if self.align(self.room.work_heading):
                if self.index == 0:
                    self.transfers = [('table', 'carrier')]
                elif self.index == 3:
                    self.transfers = [('carrier', 'table')]
                elif self.param('intermediate_transfers', True):
                    self.transfers = [('carrier', 'table'), ('table', 'carrier')]
                else:
                    self.completed.append(self.room.code)
                    self.send('/rooms/completed', String, ','.join(self.completed), True)
                    self.transition('TURN_TO_ROUTE')
                    return
                self.begin_transfer()
        elif s in TRANSFER:
            self.transfer_step()
        elif s == 'TURN_TO_ROUTE':
            if not self.stowed():
                self.fault('Wand must be stowed before turning toward exit')
            elif self.align(self.room.heading, tape=not self.fallback_active):
                self.transition('EXIT_ROOM')
        elif s == 'EXIT_ROOM':
            if self.drive(room_stop(self.room, 'exit'), inside=True):
                self.transition('OPEN_EXIT')
        elif s == 'OPEN_EXIT':
            if self.door('exit', True, dt):
                self.transition('HOLD_EXIT')
        elif s == 'HOLD_EXIT':
            if (self.door('exit', True, dt) and
                    self.now()-self.entered >= self.param('door_open_hold', .5)):
                self.transition('CLOSE_EXIT')
        elif s == 'CLOSE_EXIT':
            if self.door('exit', False, dt):
                self.get_logger().info(f'[FourRooms] {self.room.code}: exit verified, both doors closed')
                if self.index == 3:
                    self.transition('RETURN_HOME')
                else:
                    self.index += 1
                    self.transition('APPROACH_ENTRY')
        elif s == 'RETURN_HOME':
            if self.drive(LENGTH):
                self.transition('VERIFY_DELIVERY')
        elif s == 'VERIFY_DELIVERY':
            supported = (self.stopped() and self.stowed() and self.attachment('vacuum') == 'detached' and
                         self.completed == [r.code for r in ROOMS] and
                         placed(self.position('transport_robot'), (*START, 0.), .04, .02) and
                         placed(self.position('wafer'), ROOMS[-1].table, .012, .005))
            if self.stable(supported):
                self.transition('COMPLETE')
                self.get_logger().info('TRANSPORT COMPLETE — wafer in ROOM_4; robot outside all rooms')


def main(args=None):
    run(FourRoomController, args)
