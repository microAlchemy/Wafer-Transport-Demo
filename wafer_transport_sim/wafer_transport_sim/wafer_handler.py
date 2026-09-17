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
         'OPEN_EXIT', 'EXIT_ROOM', 'TURN_OUT', 'CLOSE_EXIT', 'TRANSFER_COMPLETE')
ROOM_MOTION = {'TURN_IN', 'ENTER_ROOM', 'EXIT_ROOM', 'TURN_OUT'}
PICKUP = set(STEPS[5:14])


class WaferHandler(DemoNode):
    def __init__(self):
        super().__init__('wafer_handler')
        self.watch('/system/start', Bool, LATCHED)
        self.watch('/system/fault', String, LATCHED)
        self.watch_attachment('vacuum')
        self.watch_attachment('carrier')
        self.watch_joints('/robot/joint_states')
        self.watch_joints('/door/joint_states')
        self.watch('/odom', Odometry)
        self.watch('/line/healthy', Bool)  # Image heartbeat; stripe is absent inside room.
        self.watch('/qr/healthy', Bool)
        for name in ('wafer', 'carrier', 'transport_robot'):
            self.watch_pose(name)
        self.sequence = None
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
        self.joint('door_z', target)
        return self.at_joint('door_z', target)

    def navigate(self, target_x=None, target_heading=math.pi/2):
        odom = self.values['/odom']
        error = angle_error(target_heading, yaw(odom.pose.pose.orientation))
        if target_x is None:
            if abs(error) < .02:
                return self.stopped()
            self.command.angular.z = clamp(1.8 * error, -.35, .35)
        else:
            x, y, _ = self.position('transport_robot')
            if abs(y - .12) > .018 or abs(error) > .10:
                self.fault('Room approach departed from its clearance corridor')
                return False
            distance = x - target_x  # Robot faces -world X, backs out for exit.
            if abs(distance) < .004:
                return self.stopped()
            self.command.linear.x = clamp(.8 * distance, -.035, .035)
            correction = 2.0 * (y - .12) * (1 if distance > 0 else -1)
            self.command.angular.z = clamp(1.8 * error + correction, -.15, .15)
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
        if not self.fresh('/poses/wafer', '/poses/carrier', '/poses/transport_robot',
                          '/odom', '/robot/joint_states', '/door/joint_states',
                          '/line/healthy', '/qr/healthy') or not self.value('/qr/healthy') or not self.value('/line/healthy'):
            self.fault('Cleanroom sensor feedback stale or camera unavailable')
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
            verified = self.navigate()
        elif s == 'ENTER_ROOM':
            verified = self.navigate(.65)
        elif s in {'CLOSE_ENTRY', 'CLOSE_EXIT'}:
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
        elif s == 'EXIT_ROOM':
            verified = self.navigate(1.025)
        elif s == 'TURN_OUT':
            verified = self.navigate(target_heading=0.)
        if s in {'OPEN_EXIT', 'EXIT_ROOM', 'TURN_OUT', 'CLOSE_EXIT'}:
            cx, cy, cz = self.position('carrier')
            if not placed(self.position('wafer'), (cx, cy, cz + .006), .022, .010):
                self.fault('Wafer did not remain in onboard carrier')
                return
        if self.sequence.advance(verified, self.now()):
            self.change(self.sequence.state, '[WaferHandler] ' + self.sequence.state)


def main(args=None):
    run(WaferHandler, args)
