"""Operate the rail robots inside the ten Class 100 process cells.

Every generated cell parks its vacuum cup at a derived clearance above the
support surfaces, so one transfer lowers the cup onto the supported wafer,
attaches and acknowledges, lifts clear of the shelf, carries the wafer to the
blue stage, lowers and releases it, verifies the supported dwell, picks it up
again and places it on the exit handoff. The cells load attached because the
Gazebo detachable-joint system attaches on its first update, so the controller
parks all ten cups on their own supports before the transport may start.

Active steps fault on stale feedback, on a motion or attachment timeout, when
another owner still holds the wafer and whenever the wafer is no longer on the
support that should hold it. A fault stops commanded motion and keeps whatever
the affected cell is holding.
"""
from std_msgs.msg import Bool, String

from .common import DemoNode, LATCHED, run
from .core import placed
from .four_room_layout import (ROOMS, PROCESS_CONTACT_Z, PROCESS_TRAVEL,
                               PROCESS_TRANSIT_Z, PROCESS_TRANSIT_WAFER_Z)

MOVING = ('PICK_ENTRY', 'LOWER_ENTRY', 'LIFT_ENTRY', 'MOVE_PROCESS', 'LOWER_PROCESS',
          'LIFT_PROCESS', 'MOVE_EXIT', 'LOWER_EXIT', 'RETRACT_EXIT', 'RESET')


class ProcessCellController(DemoNode):
    def __init__(self):
        super().__init__('process_cell_controller')
        self.param('process_hold', 2.0)
        self.param('placement_hold', .5)
        self.param('motion_timeout', 30.0)
        self.param('attachment_timeout', 5.0)
        for room in ROOMS:
            self.watch_joints(f'/process/room_{room.number}/joint_states')
            self.watch_attachment(room.process_attachment())
        self.watch_attachment('vacuum')
        self.watch('/system/fault', String, LATCHED)
        self.watch_pose('wafer')
        self.create_subscription(String, '/process/request', self.request, LATCHED)
        self.create_subscription(String, '/process/consumed', self.consumed, LATCHED)
        self.room = None
        self.stable_since = None
        self.command = None
        self.release_deadline = None
        self.change('RELEASE')
        self.create_timer(.05, self.tick)

    def request(self, msg):
        if self.state != 'IDLE' or self.room:
            return
        self.room = next((r for r in ROOMS if r.code == msg.data), None)
        if self.room:
            self.stable_since = None
            self.command = None
            self.change('PICK_ENTRY', f'[ProcessCell] {self.room.code}: collect entry handoff')

    def consumed(self, msg):
        if self.room and self.state == 'READY_EXIT' and msg.data == self.room.code:
            self.change('RESET')

    def target_x(self, location):
        return {'entry': 0., 'process': PROCESS_TRAVEL/2, 'exit': PROCESS_TRAVEL}[location]

    def move(self, location, lowered=False):
        x = self.target_x(location)
        z = PROCESS_CONTACT_Z if lowered else PROCESS_TRANSIT_Z
        self.joint(self.room.process_joint('x'), x)
        self.joint(self.room.process_joint('z'), z)
        return (self.at_joint(self.room.process_joint('x'), x, .003) and
                self.at_joint(self.room.process_joint('z'), z, .003))

    def support(self, location):
        return self.room.table if location == 'process' else self.room.handoff(location)

    def supported(self, location):
        """The wafer is resting on the support at that location."""
        return placed(self.position('wafer'), self.support(location), .018, .012)

    def carried(self, location):
        """The wafer is on the vacuum cup above the support at that location."""
        x, y, _ = self.support(location)
        return placed(self.position('wafer'), (x, y, PROCESS_TRANSIT_WAFER_Z), .018, .006)

    def stable(self, valid):
        if not valid:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = self.now()
        return self.now()-self.stable_since >= self.param('placement_hold', .5)

    def required(self):
        return ('/poses/wafer', f'/process/room_{self.room.number}/joint_states',
                f'/attachments/{self.room.process_attachment()}/state')

    def cells_detached(self):
        return all(self.attachment(room.process_attachment()) == 'detached'
                   for room in ROOMS)

    def release_cells(self):
        """Run the load-time attachment back onto each cell's own support once."""
        pending = [room for room in ROOMS
                   if self.attachment(room.process_attachment()) != 'detached']
        for room in pending:
            self.attach(room.process_attachment(), False)
        if not pending:
            self.release_deadline = None
            return True
        # Only police the release once the attachment bridge has reported in.
        if self.release_deadline is not None:
            if self.now() > self.release_deadline:
                self.fault('Process cells never reported detached: ' +
                           ', '.join(room.code for room in pending))
        elif any(f'/attachments/{room.process_attachment()}/state' in self.received
                 for room in ROOMS):
            self.release_deadline = self.now() + self.param('attachment_timeout', 5.)
        return False

    def ownership_issues(self):
        """Report every other detachable joint that still holds the wafer."""
        issues = []
        if self.attachment('vacuum') != 'detached':
            issues.append('mobile vacuum still holds the wafer')
        busy = [room.code for room in ROOMS if room != self.room and
                self.attachment(room.process_attachment()) == 'attached']
        if busy:
            issues.append('cells still attached: ' + ', '.join(busy))
        return issues

    def claim(self, attached):
        """Command the active cell's vacuum joint and verify the acknowledgement."""
        name = self.room.process_attachment()
        wanted = 'attached' if attached else 'detached'
        if self.attachment(name) == wanted:
            self.command = None
            return True
        if attached:
            issues = self.ownership_issues()
            if issues:
                self.fault(f'{name} attach refused: ' + '; '.join(issues))
                return False
        if self.command is None or self.command[0] != (name, wanted):
            self.command = ((name, wanted), self.now())
        self.attach(name, attached)
        if self.now() - self.command[1] > self.param('attachment_timeout', 5.):
            self.fault(f'{name} did not acknowledge {wanted}')
        return False

    def hold(self):
        """Stop commanding travel and keep whatever the cell is holding."""
        if not self.room:
            return
        for axis in ('x', 'z'):
            name = self.room.process_joint(axis)
            if name in self.joints:
                self.joint(name, self.joints[name])

    def step(self):
        state = self.state
        if state == 'PICK_ENTRY':
            if self.move('entry'):
                self.change('LOWER_ENTRY')
        elif state == 'LOWER_ENTRY':
            if not self.move('entry', True):
                return
            if not self.supported('entry'):
                self.fault('Wafer missing from the ' + self.room.code + ' entry handoff')
            elif self.claim(True):
                self.change('LIFT_ENTRY')
        elif state == 'LIFT_ENTRY':
            if self.move('entry'):
                if self.carried('entry'):
                    self.change('MOVE_PROCESS')
                else:
                    self.fault('Wafer did not rise with the ' + self.room.code + ' vacuum cup')
        elif state == 'MOVE_PROCESS':
            if self.move('process'):
                if self.carried('process'):
                    self.change('LOWER_PROCESS')
                else:
                    self.fault('Wafer lost between the entry handoff and the blue stage')
        elif state == 'LOWER_PROCESS':
            if not self.move('process', True):
                return
            if not self.supported('process'):
                self.fault('Wafer missing from the ' + self.room.code + ' blue stage')
            elif self.claim(False):
                self.change('VERIFY_STAGE')
        elif state == 'VERIFY_STAGE':
            if not self.supported('process'):
                self.fault('Wafer left the blue stage of ' + self.room.code)
            elif self.stable(True):
                self.change('PROCESSING', f'[ProcessCell] {self.room.code}: '
                                          'wafer processing on the blue stage')
        elif state == 'PROCESSING':
            if not self.supported('process'):
                self.fault('Wafer left the blue stage of ' + self.room.code +
                           ' during processing')
            elif (self.now()-self.entered >= self.param('process_hold', 2.)
                  and self.claim(True)):
                self.change('LIFT_PROCESS')
        elif state == 'LIFT_PROCESS':
            if self.move('process'):
                if self.carried('process'):
                    self.change('MOVE_EXIT')
                else:
                    self.fault('Wafer did not rise from the blue stage of ' + self.room.code)
        elif state == 'MOVE_EXIT':
            if self.move('exit'):
                if self.carried('exit'):
                    self.change('LOWER_EXIT')
                else:
                    self.fault('Wafer lost between the blue stage and the exit handoff')
        elif state == 'LOWER_EXIT':
            if not self.move('exit', True):
                return
            if not self.supported('exit'):
                self.fault('Wafer missing from the ' + self.room.code + ' exit handoff')
            elif self.claim(False):
                self.change('VERIFY_EXIT')
        elif state == 'VERIFY_EXIT':
            if not self.supported('exit'):
                self.fault('Wafer left the ' + self.room.code + ' exit handoff')
            elif self.stable(True):
                self.change('RETRACT_EXIT')
        elif state == 'RETRACT_EXIT':
            if self.move('exit'):
                self.change('READY_EXIT', f'[ProcessCell] {self.room.code}: exit handoff ready')
        elif state == 'READY_EXIT':
            # Ready stays true only while the released wafer is still supported.
            if self.attachment(self.room.process_attachment()) != 'detached':
                self.fault('Vacuum cup still holds the wafer at the ' + self.room.code + ' exit')
            elif not self.supported('exit'):
                self.fault('Wafer lost from the ' + self.room.code + ' exit handoff')
        elif state == 'RESET':
            if self.move('entry'):
                self.room = None
                self.stable_since = None
                self.change('IDLE')

    def active(self):
        if self.value('/system/fault'):
            self.change('FAULT')
            self.hold()
            return
        issues = self.feedback_issues(*self.required())
        if issues:
            self.fault('Required feedback lost: ' + '; '.join(issues))
            self.hold()
            return
        timeout = (self.param('motion_timeout', 30.) if self.state in MOVING
                   else self.param('state_timeout', 180.))
        if self.now()-self.entered > timeout:
            self.fault(f'{self.state} timed out after {timeout:.0f} s')
            self.hold()
            return
        self.step()
        if self.state == 'FAULT':
            self.hold()

    def tick(self):
        if self.state == 'FAULT':
            self.hold()
        elif self.state in ('RELEASE', 'IDLE'):
            if self.release_cells() and self.state == 'RELEASE':
                self.change('IDLE', '[ProcessCell] ten cells released on their supports')
        elif self.room:
            self.active()
        self.send('/process/state', String, self.state, True)
        self.send('/process/ready', String,
                  self.room.code if self.room and self.state == 'READY_EXIT' else '', True)
        self.send('/process/cells_detached', Bool, self.cells_detached(), True)


def main(args=None):
    run(ProcessCellController, args)
