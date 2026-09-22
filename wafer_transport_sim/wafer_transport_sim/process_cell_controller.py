"""Operate the four robots inside the Class 100 process cells."""
from std_msgs.msg import Bool, String

from .common import DemoNode, LATCHED, run
from .core import placed
from .four_room_layout import ROOMS, ROOM_LENGTH


class ProcessCellController(DemoNode):
    def __init__(self):
        super().__init__('process_cell_controller')
        self.param('process_hold', 2.0)
        self.param('placement_hold', .5)
        for room in ROOMS:
            self.watch_joints(f'/process/room_{room.number}/joint_states')
            self.watch_attachment(room.process_attachment())
        self.watch_pose('wafer')
        self.create_subscription(String, '/process/request', self.request, LATCHED)
        self.create_subscription(String, '/process/consumed', self.consumed, LATCHED)
        self.room = None
        self.stable_since = None
        self.change('IDLE')
        self.create_timer(.05, self.tick)

    def request(self, msg):
        if self.state != 'IDLE':
            return
        self.room = next((r for r in ROOMS if r.code == msg.data), None)
        if self.room:
            self.change('PICK_ENTRY', f'[ProcessCell] {self.room.code}: collect entry handoff')

    def consumed(self, msg):
        if self.room and self.state == 'READY_EXIT' and msg.data == self.room.code:
            self.change('RESET')

    def target_x(self, location):
        travel = ROOM_LENGTH-.21
        return {'entry': 0., 'process': travel/2, 'exit': travel}[location]

    def stable(self, valid):
        if not valid:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = self.now()
        return self.now()-self.stable_since >= self.param('placement_hold', .5)

    def move(self, location, lowered=False):
        x = self.target_x(location)
        z = -.105 if lowered else 0.
        self.joint(self.room.process_joint('x'), x)
        self.joint(self.room.process_joint('z'), z)
        return self.at_joint(self.room.process_joint('x'), x, .003) and \
               self.at_joint(self.room.process_joint('z'), z, .003)

    def tick(self):
        self.send('/process/state', String, self.state, True)
        self.send('/process/ready', String,
                  self.room.code if self.room and self.state == 'READY_EXIT' else '', True)
        if self.state == 'IDLE' or not self.room:
            return
        if not self.fresh('/poses/wafer', f'/process/room_{self.room.number}/joint_states',
                          f'/attachments/{self.room.process_attachment()}/state'):
            return
        state = self.state
        attachment_name = self.room.process_attachment()
        if state == 'PICK_ENTRY' and self.move('entry'):
            self.change('LOWER_ENTRY')
        elif state == 'LOWER_ENTRY' and self.move('entry', True):
            if not placed(self.position('wafer'), self.room.handoff('entry'), .018, .012):
                self.fault('Wafer missing at ' + self.room.code + ' entry handoff')
            else:
                self.attach(attachment_name, True)
                if self.attachment(attachment_name) == 'attached':
                    self.change('LIFT_ENTRY')
        elif state == 'LIFT_ENTRY' and self.move('entry'):
            self.change('MOVE_PROCESS')
        elif state == 'MOVE_PROCESS' and self.move('process'):
            self.change('LOWER_PROCESS')
        elif state == 'LOWER_PROCESS' and self.move('process', True):
            self.attach(attachment_name, False)
            if self.attachment(attachment_name) == 'detached':
                self.change('PROCESSING', f'[ProcessCell] {self.room.code}: wafer processing on blue stage')
        elif state == 'PROCESSING':
            if self.now()-self.entered >= self.param('process_hold', 2.) and self.move('process', True):
                self.attach(attachment_name, True)
                if self.attachment(attachment_name) == 'attached':
                    self.change('LIFT_PROCESS')
        elif state == 'LIFT_PROCESS' and self.move('process'):
            self.change('MOVE_EXIT')
        elif state == 'MOVE_EXIT' and self.move('exit'):
            self.change('LOWER_EXIT')
        elif state == 'LOWER_EXIT' and self.move('exit', True):
            self.attach(attachment_name, False)
            if self.attachment(attachment_name) == 'detached':
                self.change('VERIFY_EXIT')
        elif state == 'VERIFY_EXIT':
            if self.stable(placed(self.position('wafer'), self.room.handoff('exit'), .018, .012)):
                self.change('RETRACT_EXIT')
        elif state == 'RETRACT_EXIT' and self.move('exit'):
            self.change('READY_EXIT', f'[ProcessCell] {self.room.code}: exit handoff ready')
        elif state == 'RESET':
            if self.move('entry'):
                self.room = None
                self.change('IDLE')

def main(args=None):
    run(ProcessCellController, args)
