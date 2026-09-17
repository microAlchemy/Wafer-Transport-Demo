"""Robot-mounted vacuum wand, room entry, and guillotine-door interlocks."""
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from .common import DemoNode, LATCHED, run
from .core import Sequence, placed, clamp, yaw, angle_error

STEPS = ('HOME', 'OPEN_ENTRY', 'TURN_IN', 'ENTER_ROOM', 'CLOSE_ENTRY',
         'MOVE_TO_WAFER', 'LOWER_WAND', 'VACUUM_ON', 'VERIFY_VACUUM',
         'LIFT_PICKUP', 'MOVE_TO_BOX', 'LOWER', 'VACUUM_OFF', 'LIFT_RETRACT',
         'OPEN_EXIT', 'ALIGN_EXIT', 'EXIT_ROOM', 'TURN_OUT', 'CLOSE_EXIT',
         'VERIFY_EXIT', 'TRANSFER_COMPLETE')
ROOM_MOTION = {'TURN_IN', 'ENTER_ROOM', 'ALIGN_EXIT', 'EXIT_ROOM', 'TURN_OUT'}
PICKUP = set(STEPS[5:14])


class WaferHandler(DemoNode):
    def __init__(self):
        super().__init__('wafer_handler')
        self.param('room_speed', .06)
        self.param('turn_speed', .55)
        self.param('door_speed', .10)
        self.watch('/system/start', Bool, LATCHED)
        self.watch('/system/fault', String, LATCHED)
        self.watch_attachment('vacuum')
        self.watch_attachment('carrier')
        self.watch_joints('/robot/joint_states')
        self.watch_joints('/door/joint_states')
        self.watch('/odom', Odometry)
        self.watch('/line/healthy', Bool)
        self.watch('/line/valid', Bool)
        self.watch('/line/cmd_vel', Twist)
        for name in ('wafer', 'carrier', 'transport_robot'):
            self.watch_pose(name)
        self.sequence = None
        self.door_setpoint = None
        self.door_command_time = None
        self.command = Twist()
        self.change('IDLE')
        self.create_timer(0.05, self.tick)

    def stopped(self):
        if not self.fresh('/odom'):
            return False
        t = self.values['/odom'].twist.twist
        return abs(t.linear.x) < .004 and abs(t.angular.z) < .02

    def door_clear(self):
        """Oriented chassis/carrier envelope, including forward wand extension."""
        if not self.fresh('/poses/transport_robot', '/robot/joint_states'):
            return False
        p = self.values['/poses/transport_robot'].pose
        heading = yaw(p.orientation)
        reach = max(.10, sum(self.joints.get(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x')) + .014)
        xs = [p.position.x + x * math.cos(heading) - y * math.sin(heading)
              for x in (-.10, reach) for y in (-.08, .08)]
        return max(xs) < .8894 or min(xs) > .9294

    def door(self, opened):
        target = .52 if opened else 0.
        if not opened and (not self.stopped() or not self.door_clear()):
            self.fault('Door closure blocked: robot or wand is not clear')
            return False
        # A ramp bounds commanded travel; actual joint feedback still gates
        # every transition. Never count the profile itself as successful motion.
        now = self.now()
        elapsed = .05 if self.door_command_time is None else clamp(now - self.door_command_time, 0., .1)
        if self.door_setpoint is None:
            self.door_setpoint = self.joints.get('door_z', 0.)
        speed = clamp(self.get_parameter('door_speed').value, .01, .10)
        self.door_setpoint += clamp(target - self.door_setpoint, -speed * elapsed, speed * elapsed)
        self.door_command_time = now
        self.joint('door_z', self.door_setpoint)
        return abs(self.door_setpoint - target) < 1e-9 and self.at_joint('door_z', target)

    def world_heading(self):
        return yaw(self.values['/poses/transport_robot'].pose.orientation)

    def align_heading(self, target=math.pi, require_tape=False):
        error = angle_error(target, self.world_heading())
        if abs(error) < .02:
            return (self.stopped() and (not require_tape or
                    (self.fresh('/line/valid', '/line/cmd_vel') and self.value('/line/valid'))))
        speed = clamp(self.get_parameter('turn_speed').value, .1, .55)
        self.command.angular.z = clamp(2.5 * error, -speed, speed)
        return False

    def exit_heading(self):
        # Face away from a lookahead point on the tape for backward travel.
        # World pose and wheel odometry must not be mixed for heading error.
        _, y, _ = self.position('transport_robot')
        return math.pi - clamp(math.atan2(y - .12, .12), -.25, .25)

    def reverse_exit(self):
        x, y, _ = self.position('transport_robot')
        if not (.58 <= x <= 1.04 and .09 <= y <= .155):
            self.fault(f'Exit outside clearance bounds: x={x:.4f}, y={y:.4f}')
            return False
        remaining = 1.025 - x
        if abs(remaining) < .004:
            if abs(y - .12) > .012:
                self.fault(f'Exit lateral alignment failed: y={y:.4f}, expected .12')
                return False
            return self.stopped()
        if remaining < 0:
            self.fault(f'Exit overshot corridor stop: x={x:.4f}')
            return False
        error = angle_error(self.exit_heading(), self.world_heading())
        # Correct recoverable heading drift while stopped, then reverse.
        self.command.angular.z = clamp(2.5 * error, -.40, .40)
        if abs(error) <= .08:
            speed = clamp(self.get_parameter('room_speed').value, .01, .06)
            self.command.linear.x = -min(speed, 1.2 * remaining)
        return False

    def outside_room(self):
        if not self.fresh('/poses/transport_robot'):
            return False
        x, y, _ = self.position('transport_robot')
        return (abs(x - 1.025) < .010 and abs(y - .12) < .012 and
                abs(angle_error(math.pi/2, self.world_heading())) < .03 and
                self.door_clear() and self.stopped() and self.stowed())

    def follow_room_tape(self):
        issues = self.feedback_issues('/line/healthy', '/line/valid', '/line/cmd_vel')
        if not self.value('/line/healthy'):
            issues.append('/line/healthy: downward image processing failed')
        if not self.value('/line/valid'):
            issues.append('/line/valid: black room tape lost')
        if issues:
            self.fault('Room tape tracking failed: ' + '; '.join(issues))
            return False
        x, y, _ = self.position('transport_robot')
        heading = self.world_heading()
        if abs(y - .12) > .018 or abs(angle_error(math.pi, heading)) > .20:
            self.fault('Room tape approach departed from its clearance corridor')
            return False
        remaining = x - .65
        if remaining < -.004:
            self.fault('Passed wafer pickup stop on room tape')
            return False
        if abs(remaining) < .004:
            return self.stopped()
        line = self.values['/line/cmd_vel']
        # Camera centroid sets steering. Pose is only a stop/clearance interlock.
        speed = clamp(self.get_parameter('room_speed').value, .01, .06)
        self.command.linear.x = clamp(min(line.linear.x, 1.2 * remaining), 0., speed)
        self.command.angular.z = clamp(line.angular.z, -.35, .35)
        return False

    def reach(self, target):
        # Three nested stages share the total horizontal extension.
        for name in ('reach_1', 'reach_2', 'wand_x'):
            self.joint(name, target/3)
        return all(self.at_joint(n, target/3, .0007)
                   for n in ('reach_1', 'reach_2', 'wand_x'))

    def stowed(self):
        return all(self.at_joint(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x', 'wand_z'))

    def tick(self):
        self.command = Twist()
        if self.value('/system/fault'):
            self.change('FAULT')
        self.step()
        if self.state == 'FAULT':
            self.command = Twist()
            # Hold the door and manipulator; retain any attached payload.
            for name in ('reach_1', 'reach_2', 'wand_x', 'wand_z', 'door_z'):
                if name in self.joints:
                    self.joint(name, self.joints[name])
        self.pub('/cleanroom/cmd_vel', Twist).publish(self.command)
        self.send('/wafer_handler/state', String, self.state, True)
        self.send('/carrier_ready', Bool, self.state == 'TRANSFER_COMPLETE', True)
        self.send('/cleanroom/exited', Bool, self.state == 'TRANSFER_COMPLETE', True)
        self.send('/door/open', Bool, self.at_joint('door_z', .52), True)
        self.send('/door/closed', Bool, self.at_joint('door_z', 0.), True)

    def step(self):
        if self.state in {'FAULT', 'TRANSFER_COMPLETE'}:
            return
        if self.state == 'IDLE':
            # Stock detachable joints initially attach. Release while supported.
            if self.attachment('vacuum') != 'detached':
                self.attach('vacuum', False)
            if self.value('/system/start', False) and self.attachment('vacuum') == 'detached':
                self.sequence = Sequence(STEPS, entered=self.now())
                self.change('HOME', '[WaferHandler] Preparing mobile wand and carrier')
            return
        if self.sequence.expired(self.now(), self.get_parameter('state_timeout').value):
            self.fault('Cleanroom state timed out: ' + self.state)
            return
        required = ['/poses/wafer', '/poses/carrier', '/poses/transport_robot',
                    '/odom', '/robot/joint_states', '/door/joint_states']
        # Cameras are required for wheel motion, not for stationary joint work.
        # QR recognition is only required later during corridor transport.
        if self.state in ROOM_MOTION:
            required.append('/line/healthy')
        issues = self.feedback_issues(*required)
        if self.state in ROOM_MOTION and not self.value('/line/healthy'):
            issues.append('/line/healthy: downward image processing failed')
        if issues:
            self.fault('Cleanroom feedback failure: ' + '; '.join(issues))
            return
        if self.attachment('carrier') != 'attached':
            self.fault('Onboard carrier attachment lost')
            return
        rx, ry, _ = self.position('transport_robot')
        if not placed(self.position('carrier'), (rx, ry, .154), .02, .012):
            self.fault('Onboard carrier moved away from robot')
            return
        s = self.state
        if s in ROOM_MOTION and not self.stowed():
            self.fault('Wand must be stowed before robot passage')
            return
        if s in ROOM_MOTION and not self.at_joint('door_z', .52):
            self.fault('Door is not fully open during robot passage')
            return
        if s in PICKUP and (not self.at_joint('door_z', 0.) or not self.stopped()):
            self.fault('Pickup requires a closed door and stationary robot')
            return
        if s in {'LIFT_PICKUP', 'MOVE_TO_BOX', 'LOWER'} and self.attachment('vacuum') != 'attached':
            self.fault('Vacuum attachment lost during wafer transfer')
            return
        verified = False
        if s == 'HOME':
            self.joint('wand_z', 0.)
            retracted = self.reach(0.)
            self.joint('tray_z', .004)
            verified = retracted and all(self.at_joint(n, v) for n, v in
                           [('wand_z', 0.), ('wand_x', 0.), ('tray_z', .004)])
        elif s in {'OPEN_ENTRY', 'OPEN_EXIT'}:
            verified = self.door(True)
        elif s == 'TURN_IN':
            verified = self.align_heading(require_tape=True)
        elif s == 'ENTER_ROOM':
            verified = self.follow_room_tape()
        elif s in {'CLOSE_ENTRY', 'CLOSE_EXIT'}:
            if s == 'CLOSE_EXIT' and not self.outside_room():
                self.fault('Exit must be verified outside before closing the door')
                return
            verified = self.door(False)
        elif s == 'MOVE_TO_WAFER':
            verified = self.reach(.25)
        elif s == 'LOWER_WAND':
            self.joint('wand_z', -.14)
            verified = self.at_joint('wand_z', -.14)
        elif s == 'VACUUM_ON':
            if not placed(self.position('wafer'), (.4, .12, .155), .008):
                self.fault('Wafer is not at the process output')
                return
            # Actual base pose and joint feedback must put the cup at the wafer.
            heading = yaw(self.values['/poses/transport_robot'].pose.orientation)
            extension = sum(self.joints.get(n, 0.) for n in ('reach_1', 'reach_2', 'wand_x'))
            cup = (rx + extension * math.cos(heading), ry + extension * math.sin(heading),
                   .300 + self.joints.get('wand_z', 0.))
            if not placed(cup, self.position('wafer'), .009, .008):
                self.fault('Wand is not aligned with wafer')
                return
            self.attach('vacuum', True)
            verified = self.attachment('vacuum') == 'attached'
        elif s == 'VERIFY_VACUUM':
            verified = self.attachment('vacuum') == 'attached'
        elif s in {'LIFT_PICKUP', 'LIFT_RETRACT'}:
            self.joint('wand_z', 0.)
            verified = self.at_joint('wand_z', 0.)
            if s == 'LIFT_PICKUP':
                verified = verified and self.position('wafer')[2] > .285
        elif s == 'MOVE_TO_BOX':
            verified = self.reach(0.)
        elif s == 'LOWER':
            self.joint('wand_z', -.136)
            verified = self.at_joint('wand_z', -.136)
        elif s == 'VACUUM_OFF':
            cx, cy, cz = self.position('carrier')
            if placed(self.position('wafer'), (cx, cy, cz + .006), .012, .008):
                self.attach('vacuum', False)
                verified = self.attachment('vacuum') == 'detached'
        elif s == 'ALIGN_EXIT':
            verified = self.align_heading(self.exit_heading())
        elif s == 'EXIT_ROOM':
            verified = self.reverse_exit()
        elif s == 'TURN_OUT':
            verified = self.align_heading(math.pi/2)
        elif s == 'VERIFY_EXIT':
            verified = self.outside_room() and self.at_joint('door_z', 0.)
        if s in {'OPEN_EXIT', 'ALIGN_EXIT', 'EXIT_ROOM', 'TURN_OUT', 'CLOSE_EXIT', 'VERIFY_EXIT'}:
            cx, cy, cz = self.position('carrier')
            if not placed(self.position('wafer'), (cx, cy, cz + .006), .022, .010):
                self.fault('Wafer did not remain in onboard carrier')
                return
        if self.sequence.advance(verified, self.now()):
            notes = {'ENTER_ROOM': ' — following black room tape',
                     'ALIGN_EXIT': ' — aligning before reverse',
                     'VERIFY_EXIT': ' — robot outside Class 100 room',
                     'TRANSFER_COMPLETE': ' — exit verified; ready for corridor transport'}
            note = notes.get(self.sequence.state, '')
            self.change(self.sequence.state, '[WaferHandler] ' + self.sequence.state + note)


def main(args=None):
    run(WaferHandler, args)
