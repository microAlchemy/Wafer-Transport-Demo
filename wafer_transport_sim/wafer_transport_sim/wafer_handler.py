"""Feedback-gated X/Z gantry and vacuum sequence."""
from std_msgs.msg import Bool, String
from .common import DemoNode, LATCHED, run
from .core import Sequence, placed

STEPS = ('HOME', 'MOVE_TO_WAFER', 'LOWER_WAND', 'VACUUM_ON',
         'VERIFY_VACUUM', 'LIFT_PICKUP', 'MOVE_TO_BOX', 'LOWER',
         'VACUUM_OFF', 'LIFT_RETRACT', 'TRANSFER_COMPLETE')
LOGS = ('Homing gantry', 'Moving to wafer', 'Lowering vacuum wand',
        'Vacuum enabled', 'Wafer pickup verified', 'Lifting wafer',
        'Moving to carrier', 'Lowering wafer into carrier', 'Vacuum released',
        'Lifting wafer wand clear of carrier', 'Transfer complete')


class WaferHandler(DemoNode):
    def __init__(self):
        super().__init__('wafer_handler')
        self.watch('/system/start', Bool, LATCHED)
        self.watch('/system/fault', String, LATCHED)
        self.watch_attachment('vacuum')
        self.watch_joints('/gantry/joint_states')
        self.watch_pose('wafer')
        self.watch_pose('carrier')
        self.sequence = None
        self.change('IDLE')
        self.create_timer(0.05, self.tick)

    def tick(self):
        self.send('/wafer_handler/state', String, self.state, True)
        self.send('/carrier_ready', Bool, self.state == 'TRANSFER_COMPLETE', True)
        if self.value('/system/fault'):
            self.change('FAULT')
        if self.state == 'FAULT':
            # Hold measured joint positions, retaining any held wafer.
            for name in ('gantry_x', 'gantry_z'):
                if name in self.joints:
                    self.joint(name, self.joints[name])
            return
        if self.state == 'IDLE':
            # The stock Jetty plugin starts attached. Explicitly release it
            # while the wafer is still supported at the process output.
            if self.attachment('vacuum') != 'detached':
                self.attach('vacuum', False)
            if self.value('/system/start', False):
                self.sequence = Sequence(STEPS, entered=self.now())
                self.change(STEPS[0], '[WaferHandler] ' + LOGS[0])
            return
        if self.state == 'TRANSFER_COMPLETE':
            return
        if self.sequence.expired(self.now(), self.get_parameter('state_timeout').value):
            self.fault('Gantry state timed out: ' + self.state)
            return
        if not self.fresh('/poses/wafer', '/poses/carrier'):
            self.fault('Wafer/carrier pose feedback stale')
            return
        s = self.state
        verified = False
        if s == 'HOME':
            self.joint('gantry_z', 0.20)
            if self.at_joint('gantry_z', 0.20):
                self.joint('gantry_x', 0.10)
                verified = self.at_joint('gantry_x', 0.10)
        elif s == 'MOVE_TO_WAFER':
            self.joint('gantry_x', 0.0)
            verified = self.at_joint('gantry_x', 0.0)
        elif s == 'LOWER_WAND':
            self.joint('gantry_z', 0.0)
            verified = self.at_joint('gantry_z', 0.0)
        elif s == 'VACUUM_ON':
            if not placed(self.position('wafer'), (0.4, 0.12, 0.156), 0.008):
                self.fault('Wafer is not at the pickup point')
                return
            self.attach('vacuum', True)
            verified = self.attachment('vacuum') == 'attached'
        elif s == 'VERIFY_VACUUM':
            verified = self.attachment('vacuum') == 'attached'
        elif s in ('LIFT_PICKUP', 'LIFT_RETRACT'):
            self.joint('gantry_z', 0.20)
            verified = self.at_joint('gantry_z', 0.20)
            if s == 'LIFT_PICKUP':
                verified = verified and self.position('wafer')[2] > 0.34
        elif s == 'MOVE_TO_BOX':
            self.joint('gantry_x', 0.625)
            verified = self.at_joint('gantry_x', 0.625)
        elif s == 'LOWER':
            self.joint('gantry_z', 0.0)
            verified = self.at_joint('gantry_z', 0.0)
        elif s == 'VACUUM_OFF':
            self.attach('vacuum', False)
            cx, cy, cz = self.position('carrier')
            verified = (self.attachment('vacuum') == 'detached' and
                        placed(self.position('wafer'), (cx, cy, cz + 0.006)))
        if self.sequence.advance(verified, self.now()):
            i = self.sequence.index
            self.change(STEPS[i], '[WaferHandler] ' + LOGS[i])


def main(args=None):
    run(WaferHandler, args)
