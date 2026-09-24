"""One command owner for a feedback-gated, eleven-glovebox wafer mission.

World pose bounds the selected tape edge and the docking stops. The four-probe
IR array supplies primary steering on the straight lanes; the route FSM steers
the bounded junction states onto the edge it has selected, and pose-based
fallback may follow that same selected edge. Doors, vacuum and wand transitions
require measured feedback.

AprilTag observations are scoped to one room and one decision, so a signed
state authorises a single Track 1 <-> Track 2 transition and nothing else.
"""
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float64
from rclpy.clock import Clock, ClockType
from .common import DemoNode, LATCHED, run
from .core import angle_error, clamp, placed, yaw
from .four_room_layout import (ROOMS, START, EDGES, HOME_EDGE, TAIL_EDGE, edge_heading,
                               DOOR_APPROACH_HEADING, edge_point, edge_project,
                               entry_decision_stop, entry_rung, exit_rung,
                               horizontal_door_clear, main_edge, room_stop, service_edge)

TRANSFER = ('PICK_POSITION', 'PICK_LOWER', 'ATTACH', 'LIFT', 'PLACE_POSITION',
            'PLACE_LOWER', 'RELEASE', 'RETRACT', 'VERIFY_PLACE')
DRIVING = {'APPROACH_ENTRY', 'ADVANCE_ENTRY_JUNCTION', 'CROSS_ENTRY_JUNCTION',
           'TRAVEL_TO_ENTRY', 'TRAVEL_TO_EXIT', 'TRAVEL_TO_RETURN_JUNCTION',
           'CROSS_RETURN_JUNCTION', 'RETURN_HOME', 'HOME_RETURN'}
TURNING = {'ALIGN_ENTRY_JUNCTION', 'ALIGN_SERVICE_LANE', 'ALIGN_ENTRY', 'ALIGN_AFTER_ENTRY',
           'ALIGN_EXIT', 'ALIGN_AFTER_EXIT', 'ALIGN_RETURN_JUNCTION', 'ALIGN_MAIN_LANE',
           'TURNAROUND'}
DECISIONS = {'CONFIRM_ENTRY_TAG': 'entry', 'CONFIRM_EXIT_TAG': 'exit'}
# The probes sit 0.10684 m ahead of the chassis, so the last probe length of
# any leg has no tape under its footprint: a stop is exactly where the tape
# legitimately ends.
PROBE_LOOKAHEAD = .11


class FourRoomController(DemoNode):
    def __init__(self):
        super().__init__('transport_controller')
        for name, default in [('route_speed', .20), ('room_speed', .15),
                              ('turn_speed', .55), ('minimum_turn_speed', .12),
                              ('turn_tolerance', .03), ('door_turn_tolerance', .012),
                              ('door_speed', .18), ('door_open_hold', .5),
                              ('mission_timeout', 1800.), ('startup_timeout_wall', 90.),
                              ('apriltag_timeout', 12.), ('apriltag_freshness', 5.),
                              ('tape_tolerance', .068),
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
        self.watch('/apriltag/healthy', Bool)
        self.watch('/process/ready', String, LATCHED)
        self.watch('/process/state', String, LATCHED)
        self.watch('/process/cells_detached', Bool, LATCHED)
        self.create_subscription(String, '/detected_apriltag', self.detected_tag, 10)
        self.index = 0
        self.completed = []
        self.door_targets = {}
        self.stable_since = None
        self.transfers = []
        self.source = 'carrier'
        self.destination = 'carrier'
        self.payload = 'carrier'
        self.payload_table = ROOMS[0].table
        self.transfer_next = None
        # Route selection: one finite edge at a time plus a decision-scoped tag
        # window.  There is deliberately no permanent authorised-tag set.
        self.leg = main_edge(ROOMS[0]).name
        self.leg_distance = None
        self.leg_engaged = False
        self.scope = None
        self.observed_tags = {}
        self.tag_reports = {}
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
        if state in DECISIONS:
            # Open one bounded authorisation window for this room and decision.
            self.scope = (self.room.code, DECISIONS[state], self.now())
        self.change(state, f'[Gloveboxes] {self.room.code}: {state}')

    def detected_tag(self, msg):
        self.observed_tags[msg.data] = self.now()
        # Refresh every detection, but do not bury FSM/fault messages in frame logs.
        if self.now()-self.tag_reports.get(msg.data, -float('inf')) >= 5.:
            self.tag_reports[msg.data] = self.now()
            self.get_logger().info(f'[AprilTag] observed {msg.data}; FSM={self.state}')

    def tags_authorized(self, room, side):
        """True only for a fresh observation inside the open room/decision scope."""
        code, scoped_side, since = self.scope or ('', '', 0.)
        if (code, scoped_side) != (room.code, side):
            return False
        seen = self.observed_tags.get(room.tag(side))
        return (seen is not None and seen >= since and
                self.now()-seen <= self.param('apriltag_freshness', 5.))

    def consume_authorization(self):
        """One-shot: a signed decision cannot authorise a later junction."""
        self.scope = None

    @property
    def edge(self):
        return EDGES[self.leg]

    def select(self, edge):
        """Select a finite route edge; repeated calls in one leg are free."""
        if self.leg != edge.name:
            self.leg = edge.name
            self.leg_distance = None
            self.leg_engaged = False

    def lookahead_command(self, edge, distance=None, lookahead=.10):
        """Pose-based steering toward the selected edge, never across lanes."""
        x, y, _ = self.position('transport_robot')
        if distance is None:
            distance, _ = edge_project(edge, x, y)
        target_distance = distance + lookahead
        target_x, target_y = edge_point(edge, target_distance)
        if target_distance > edge.length:
            # Keep a point ahead along the selected edge's tangent. A target
            # clamped to the endpoint can fall behind the robot in the final
            # millimetres and command a spurious spin before a junction.
            beyond = target_distance - edge.length
            tangent = edge_heading(edge, edge.length)
            target_x += beyond * math.cos(tangent)
            target_y += beyond * math.sin(tangent)
        error = angle_error(math.atan2(target_y-y, target_x-x), self.heading())
        command = Twist()
        command.linear.x = self.param('route_speed', .20)
        command.angular.z = clamp(3.5*error, -1.4, 1.4)
        return command

    def follow(self, edge, target, speed, junction=False):
        """Drive the selected edge to a measured along-edge distance.

        Projection is always taken against the one selected edge, so parallel
        lanes can never capture the robot.  Straight lanes steer on the IR
        candidate; bounded junction states steer on the selected edge itself.
        """
        if not self.stowed() or self.attachment('vacuum') != 'detached':
            self.fault('Travel requires a stowed wand and released vacuum')
            return False
        if not all(self.door_closed(self.room.door_joint(side)) for side in ('entry', 'exit')):
            self.fault('Resume travel only after both doors are closed')
            return False
        x, y, _ = self.position('transport_robot')
        distance, deviation = edge_project(edge, x, y)
        if deviation > self.param('tape_tolerance', .068):
            self.fault(f'Robot departed the selected {edge.name} edge ({deviation:.3f} m)')
            return False
        self.leg_engaged = True
        if self.leg_distance is not None and distance-self.leg_distance < -.05:
            self.fault('Selected-edge projection jumped backwards on ' + edge.name)
            return False
        self.leg_distance = distance if self.leg_distance is None else max(self.leg_distance,
                                                                         distance)
        remaining = target-distance
        self.send('/docking_distance', Float64, remaining)
        if remaining < -.04:
            self.fault('Passed route stop without verified arrival')
            return False
        if remaining <= .004:
            return self.stopped()
        # IR is primary on the straight lanes.  Bounded junction states and the
        # final probe length of a leg steer the selected edge directly.
        ir_ready = self.primary_ready()
        if junction or remaining <= PROBE_LOOKAHEAD:
            candidate = self.lookahead_command(edge, distance)
        elif ir_ready:
            candidate = self.values['/ir/cmd_vel']
        elif not self.fallback_active:
            self.fault('Black tape lost by the IR sensor array')
            return False
        else:
            candidate = self.lookahead_command(edge, distance)
        # Brake toward the target, not the outer edge of the arrival window.
        # Otherwise speed tends to zero just OUTSIDE that window and wheel
        # deadband can prevent the FSM from ever reaching its decision state.
        braking_speed = math.sqrt(2*self.param('braking_acceleration', .35)*max(0., remaining))
        requested = min(max(0., candidate.linear.x), speed, braking_speed)
        if remaining > .012:
            requested = min(speed, max(self.param('minimum_cruise_speed', .07), requested))
        self.command.angular.z = clamp(candidate.angular.z, -1.4, 1.4)
        if abs(self.command.angular.z) > .20:
            requested = min(requested, self.param('curve_speed', .09))
        self.command.linear.x = requested
        return False

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
            self.get_logger().info('[Decision] ' + self.decision_status())
            if self.state == 'WAIT_FOR_SIMULATOR':
                issues = self.readiness_issues()
                self.get_logger().info('[Startup] ' + ('; '.join(issues) or 'final readiness interlock'))
            elif self.state not in {'FAULT', 'COMPLETE'}:
                self.get_logger().info(f'[Gloveboxes] {self.room.code} {self.state}; '
                                       f'robot={self.position("transport_robot")}, vacuum={self.attachment("vacuum")}')
                if self.state == 'HOME':
                    self.get_logger().info('[Home] waiting only for safety joints: ' +
                                           (', '.join(self.home_blockers()) or 'none'))

    def decision_status(self):
        if self.state == 'FAULT':
            return 'FAULT: ' + str(self.value('/system/fault') or 'see preceding error')
        expected = self.room.tag(DECISIONS[self.state]) if self.state in DECISIONS else '-'
        authorized = self.tags_authorized(self.room, DECISIONS[self.state]) if self.state in DECISIONS else False
        motion = self.values.get('/odom')
        velocity = motion.twist.twist if motion is not None else None
        measured = (f'v={math.hypot(velocity.linear.x, velocity.linear.y):.4f} '
                    f'w={velocity.angular.z:.4f}' if velocity else 'odom missing')
        turn_targets = {
            'ALIGN_ENTRY_JUNCTION': math.pi/2, 'ALIGN_SERVICE_LANE': 0.,
            'ALIGN_ENTRY': DOOR_APPROACH_HEADING,
            'ALIGN_AFTER_ENTRY': 0., 'ALIGN_EXIT': DOOR_APPROACH_HEADING,
            'ALIGN_AFTER_EXIT': 0., 'ALIGN_RETURN_JUNCTION': -math.pi/2,
            'ALIGN_MAIN_LANE': 0., 'TURNAROUND': math.pi,
        }
        heading_status = ''
        if self.state in turn_targets and self.fresh('/poses/transport_robot'):
            heading_status = f' heading_error={angle_error(turn_targets[self.state], self.heading()):.3f}'
        return (f'FSM={self.state} edge={self.leg} expected={expected} '
                f'authorized={authorized} stopped={self.stopped()} {measured}{heading_status} '
                f'cmd=({self.command.linear.x:.3f},{self.command.angular.z:.3f})')

    def required(self):
        return ['/odom', '/robot/joint_states', '/poses/transport_robot', '/poses/wafer', '/poses/carrier',
                *[f'/doors/room_{r.number}/{side}/joint_states' for r in ROOMS for side in ('entry', 'exit')]]

    def readiness_issues(self):
        issues = self.feedback_issues(*self.required())
        if not self.param('allow_sensor_fallback', True) and not self.primary_ready():
            issues.append('IR array unavailable or black tape not detected')
        if self.attachment('vacuum') != 'detached':
            issues.append('initial vacuum detach not acknowledged')
        if not self.value('/process/cells_detached'):
            issues.append('internal process cells have not released the wafer')
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
        # Pretend IR follows only the edge the FSM selected; it cannot choose a
        # lane, and it can never substitute for an AprilTag authorisation.
        return self.lookahead_command(self.edge)

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
                        if not self.door_closed(room.door_joint(side)))
        return blockers

    def reach(self, value):
        for name in ('reach_1', 'reach_2', 'wand_x'):
            self.joint(name, value/3)
        return all(self.at_joint(n, value/3, .0007) for n in ('reach_1', 'reach_2', 'wand_x'))

    def clear(self, room, side):
        x, y, _ = self.position('transport_robot')
        # A door in the other row cannot overlap this robot.
        extension = sum(self.joints.get(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x'))
        return horizontal_door_clear(x, y, self.heading(), room.door_position(side)[1], extension)

    def door_status(self, name):
        position = self.joints.get(name)
        age = self.now()-self.joint_received.get(name, -float('inf'))
        if position is None or not math.isfinite(position):
            return f'{name}: missing or invalid position'
        if not 0. <= age < self.param('sensor_timeout', 1.):
            return f'{name}: stale feedback age={age:.3f}s position={position:.5f}m'
        return f'{name}: position={position:.5f}m age={age:.3f}s (closed range -0.006..0.002m)'

    def door_closed(self, name):
        # The prismatic lower limit is -4 mm: settling downward closes the
        # opening further. Keep the positive opening limit at 2 mm.
        position = self.joints.get(name)
        age = self.now()-self.joint_received.get(name, -float('inf'))
        return (position is not None and math.isfinite(position) and
                0. <= age < self.param('sensor_timeout', 1.) and
                -.006 <= position <= .002)

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
        return abs(current-target) < 1e-9 and (self.at_joint(name, target) if opened else self.door_closed(name))

    def align(self, target, tape=False):
        error = angle_error(target, self.heading())
        tolerance = (self.param('door_turn_tolerance', .012)
                     if self.state in {'ALIGN_ENTRY', 'ALIGN_EXIT'}
                     else self.param('turn_tolerance', .03))
        if abs(error) < tolerance:
            return self.stopped() and (not tape or self.fallback_active or
                                       (self.fresh('/ir/valid') and self.value('/ir/valid')))
        limit = self.param('turn_speed', .55)
        minimum = min(self.param('minimum_turn_speed', .12), limit)
        self.command.angular.z = math.copysign(
            min(limit, max(minimum, abs(2.5*error))), error)
        return False

    def target_position(self, location):
        if location in ('entry', 'exit'):
            return self.room.handoff(location)
        if location == 'process':
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

    def support_reach(self, location):
        rx, ry, _ = self.position('transport_robot')
        tx, ty, _ = self.target_position(location)
        return clamp(math.hypot(tx-rx, ty-ry), 0., .269)

    def transfer_step(self):
        handoff = self.destination if self.destination in ('entry', 'exit') else self.source
        door_ok = (handoff in ('entry', 'exit') and self.at_joint(self.room.door_joint(handoff), .52) and
                   self.door_closed(self.room.door_joint('exit' if handoff == 'entry' else 'entry')))
        if not self.stopped() or not door_ok:
            self.fault('Wafer handoff requires stopped robot and the correct open door')
            return
        source_reach = self.support_reach(self.source) if self.source != 'carrier' else 0.
        target_reach = self.support_reach(self.destination) if self.destination != 'carrier' else 0.
        source_z = -.14 if self.source != 'carrier' else -.136
        target_z = -.14 if self.destination != 'carrier' else -.136
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
                    self.payload_table = self.target_position(self.destination)
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
                self.get_logger().info(f'[Handoff] {self.room.code}: {self.destination} placement verified')
                self.transition(self.transfer_next)

    def begin_transfer(self, transfers=None, next_state=None):
        if transfers is not None:
            self.transfers = list(transfers)
            self.transfer_next = next_state
        self.source, self.destination = self.transfers.pop(0)
        self.get_logger().info(f'[Gloveboxes] {self.room.code}: {self.source} -> {self.destination}')
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
                self.fault('Eleven-glovebox mission timed out')
            elif self.state not in {'INITIAL_DETACH', 'INITIAL_ATTACH', 'HOME'}:
                rx, ry, _ = self.position('transport_robot')
                if self.attachment('carrier') != 'attached' or not placed(self.position('carrier'), (rx, ry, .154), .02, .012):
                    self.fault('Onboard carrier retention lost')
                if self.payload == 'wand' and self.state != 'RELEASE' and self.attachment('vacuum') != 'attached':
                    self.fault('Vacuum attachment lost during transfer')
                if self.state in DRIVING | TURNING and self.payload in {'carrier', 'entry', 'exit'}:
                    expected = self.target_position('carrier') if self.payload == 'carrier' else self.payload_table
                    if not placed(self.position('wafer'), expected, .022, .010):
                        self.fault('Wafer moved from its verified support')
                # All doors not deliberately being operated must remain shut.
                for room in ROOMS:
                    for side in ('entry', 'exit'):
                        active = room == self.room and ((side == 'entry' and self.state in
                            {'OPEN_ENTRY', *TRANSFER, 'CLOSE_ENTRY'}) or
                            (side == 'exit' and self.state in {'OPEN_EXIT', *TRANSFER, 'CLOSE_EXIT'}))
                        if not active:
                            name = room.door_joint(side)
                            self.joint(name, 0.)
                            if not self.door_closed(name):
                                self.fault('Door interlock: ' + self.door_status(name))
                if (not self.fallback_active and self.state in DRIVING | TURNING and
                        (not self.fresh('/ir/healthy') or not self.value('/ir/healthy'))):
                    self.fault('IR sensor array unavailable during motion')
                if self.state in TURNING and not self.stowed():
                    self.fault('Wand must be stowed before turning')
        if self.state in DRIVING | TURNING and self.leg_engaged:
            # The selected edge bounds the pose; a parallel lane can never
            # capture the robot because projection is taken against one edge.
            _, deviation = edge_project(self.edge, *self.position('transport_robot')[:2])
            if deviation > self.param('tape_tolerance', .068):
                self.fault(f'Left the selected {self.edge.name} edge ({deviation:.3f} m)')
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
        self.send('/transport/decision', String, self.decision_status(), True)
        self.send('/transport/state', String, self.state, True)
        self.send('/system/state', String, self.state, True)
        self.send('/transport/track', String, self.edge.track, True)
        self.send('/transport/route_segment', String, self.edge.name, True)
        self.send('/rooms/active', String, self.room.code, True)
        self.send('/carrier_ready', Bool, self.payload == 'carrier' and self.state != 'FAULT', True)
        self.send('/wafer_delivered', Bool, self.state == 'COMPLETE', True)

    def step(self, dt):
        s = self.state
        if s in {'FAULT', 'COMPLETE'}:
            return
        room = self.room
        entry_leg, exit_leg = main_edge(room), service_edge(room)
        if s == 'WAIT_FOR_SIMULATOR':
            # The mission starts with the wafer supported in the onboard carrier.
            self.select(entry_leg)
            self.attach('vacuum', False)
            if (self.fresh(*self.required()) and (self.primary_ready() or self.fallback_active) and
                    self.attachment('vacuum') == 'detached' and self.stopped() and
                    self.value('/process/cells_detached')):
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
                self.select(entry_leg)
        elif s == 'APPROACH_ENTRY':
            # Track 1 transit to the pre-junction decision point.
            self.select(entry_leg)
            if self.follow(entry_leg, entry_decision_stop(room), self.param('route_speed', .20)):
                self.transition('CONFIRM_ENTRY_TAG')
        elif s == 'CONFIRM_ENTRY_TAG':
            # A fresh, in-scope entry tag authorises Track 1 -> Track 2.
            if self.tags_authorized(room, 'entry') and self.stopped():
                self.get_logger().info(f'[Decision] {room.tag("entry")}: TRACK_1 -> TRACK_2 authorized')
                self.transition('ADVANCE_ENTRY_JUNCTION')
            elif self.now()-self.entered > self.param('apriltag_timeout', 12.):
                self.fault('Expected AprilTag not confirmed before the Track 1 junction: ' +
                           room.tag('entry'))
        elif s == 'ADVANCE_ENTRY_JUNCTION':
            # Creep to the junction mouth with the authorisation already held.
            self.select(entry_leg)
            if self.follow(entry_leg, entry_leg.length, self.param('room_speed', .15)):
                self.transition('ALIGN_ENTRY_JUNCTION')
        elif s == 'ALIGN_ENTRY_JUNCTION':
            self.select(entry_rung(room))
            if self.align(math.pi/2):
                self.consume_authorization()
                self.transition('CROSS_ENTRY_JUNCTION')
        elif s == 'CROSS_ENTRY_JUNCTION':
            # Explicit selected-edge steering across the transverse rung.
            edge = entry_rung(room)
            if self.follow(edge, edge.length, self.param('room_speed', .15), junction=True):
                self.transition('ALIGN_SERVICE_LANE')
        elif s == 'ALIGN_SERVICE_LANE':
            self.select(exit_leg)
            if self.align(0., tape=True):
                self.transition('TRAVEL_TO_ENTRY')
        elif s == 'TRAVEL_TO_ENTRY':
            if self.follow(exit_leg, room_stop(room, 'entry'), self.param('room_speed', .15)):
                self.transition('ALIGN_ENTRY')
        elif s == 'ALIGN_ENTRY':
            if self.align(DOOR_APPROACH_HEADING):
                self.transition('OPEN_ENTRY')
        elif s == 'OPEN_ENTRY':
            if not self.stopped():
                return
            if self.door('entry', True, dt):
                self.begin_transfer([('carrier', 'entry')], 'CLOSE_ENTRY')
        elif s == 'CLOSE_ENTRY':
            if self.door('entry', False, dt):
                self.payload = 'process'
                self.send('/process/request', String, self.room.code, True)
                self.transition('ALIGN_AFTER_ENTRY')
        elif s == 'ALIGN_AFTER_ENTRY':
            self.select(exit_leg)
            if self.align(edge_heading(exit_leg, room_stop(room, 'entry'))):
                self.transition('TRAVEL_TO_EXIT')
        elif s == 'TRAVEL_TO_EXIT':
            if self.follow(exit_leg, room_stop(room, 'exit'), self.param('room_speed', .15)):
                self.transition('CONFIRM_EXIT_TAG')
        elif s == 'CONFIRM_EXIT_TAG':
            # A fresh, in-scope exit tag plus the completed service cell
            # authorises Track 2 -> Track 1.
            tag_ok = self.tags_authorized(room, 'exit')
            process_ok = self.value('/process/ready') == room.code
            if tag_ok and process_ok and self.stopped():
                self.get_logger().info(f'[Decision] {room.tag("exit")}: collect wafer then TRACK_2 -> TRACK_1')
                self.payload = 'exit'
                self.payload_table = room.handoff('exit')
                self.transition('ALIGN_EXIT')
            elif self.now()-self.entered > self.param('apriltag_timeout', 12.):
                if not tag_ok:
                    self.fault('Expected AprilTag not confirmed before the Track 2 junction: ' +
                               room.tag('exit'))
                elif not process_ok:
                    self.fault('Exit tag observed but ' + room.code +
                               ' has not delivered the processed wafer to its exit handoff')
        elif s == 'ALIGN_EXIT':
            if self.align(DOOR_APPROACH_HEADING):
                self.transition('OPEN_EXIT')
        elif s in TRANSFER:
            self.transfer_step()
        elif s == 'OPEN_EXIT':
            if self.door('exit', True, dt):
                self.begin_transfer([('exit', 'carrier')], 'CLOSE_EXIT')
        elif s == 'CLOSE_EXIT':
            if self.door('exit', False, dt):
                self.send('/process/consumed', String, self.room.code, True)
                self.completed.append(self.room.code)
                self.send('/rooms/completed', String, ','.join(self.completed), True)
                self.get_logger().info(f'[Gloveboxes] {self.room.code}: processed wafer collected')
                self.transition('ALIGN_AFTER_EXIT')
        elif s == 'ALIGN_AFTER_EXIT':
            self.select(exit_leg)
            if self.align(edge_heading(exit_leg, room_stop(room, 'exit'))):
                self.transition('TRAVEL_TO_RETURN_JUNCTION')
        elif s == 'TRAVEL_TO_RETURN_JUNCTION':
            if self.follow(exit_leg, exit_leg.length, self.param('room_speed', .15)):
                self.transition('ALIGN_RETURN_JUNCTION')
        elif s == 'ALIGN_RETURN_JUNCTION':
            self.select(exit_rung(room))
            if self.align(-math.pi/2):
                self.consume_authorization()
                self.transition('CROSS_RETURN_JUNCTION')
        elif s == 'CROSS_RETURN_JUNCTION':
            edge = exit_rung(room)
            if self.follow(edge, edge.length, self.param('room_speed', .15), junction=True):
                self.transition('ALIGN_MAIN_LANE')
        elif s == 'ALIGN_MAIN_LANE':
            if self.index == len(ROOMS)-1:
                self.select(TAIL_EDGE)
                if self.align(edge_heading(TAIL_EDGE, 0.)):
                    self.transition('RETURN_HOME')
            else:
                # Select the next room's Track 1 leg, and advance the room
                # index exactly once, on the tick the turn completes.
                self.select(main_edge(ROOMS[self.index+1]))
                if self.align(0.):
                    self.index += 1
                    self.transition('APPROACH_ENTRY')
        elif s == 'RETURN_HOME':
            self.select(TAIL_EDGE)
            if self.follow(TAIL_EDGE, TAIL_EDGE.length, self.param('route_speed', .20)):
                self.transition('TURNAROUND')
        elif s == 'TURNAROUND':
            # Measured 180 deg turn at the end of Track 1, then home on Track 1.
            self.select(HOME_EDGE)
            if self.align(math.pi) and self.stopped():
                self.transition('HOME_RETURN')
        elif s == 'HOME_RETURN':
            if self.follow(HOME_EDGE, HOME_EDGE.length, self.param('route_speed', .20)):
                self.transition('VERIFY_DELIVERY')
        elif s == 'VERIFY_DELIVERY':
            supported = (self.stopped() and self.stowed() and self.attachment('vacuum') == 'detached' and
                         self.completed == [r.code for r in ROOMS] and
                         placed(self.position('transport_robot'), (*START, 0.), .04, .02) and
                         self.payload == 'carrier' and
                         placed(self.position('wafer'), self.target_position('carrier'), .012, .008))
            if self.stable(supported):
                self.transition('COMPLETE')
                self.get_logger().info('TRANSPORT COMPLETE — wafer processed through all 11 gloveboxes')


def main(args=None):
    run(FourRoomController, args)
