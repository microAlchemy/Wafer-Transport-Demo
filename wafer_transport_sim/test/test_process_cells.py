"""Process-cell reliability: derived geometry, release exclusivity and faults.

These drive the real controller against the in-memory ROS transport from
test_controllers. They check the generated geometry contract and the state
machine; Gazebo contact, detachable joints and the Ubuntu mission still need
the physical run described in README.md.
"""
from pathlib import Path
from types import SimpleNamespace as NS
import ast
import xml.etree.ElementTree as ET

import pytest

from test_controllers import ros, Scalar, pose  # noqa: F401  (fixture and helpers)
from wafer_transport_sim.four_room_layout import (
    ROOMS, ROOM_DEPTH, PROCESS_ARM_CENTER, PROCESS_ARM_LENGTH, PROCESS_ARM_TOP, PROCESS_CONTACT_Z,
    PROCESS_CUP_LENGTH, PROCESS_CUP_OFFSET, PROCESS_CUP_REST_BOTTOM, PROCESS_GRIPPER_Z,
    PROCESS_LIFT_STROKE, PROCESS_RAIL_BOTTOM, PROCESS_TRAVEL, PROCESS_TRANSIT_WAFER_Z,
    SUPPORT_TOP, SUPPORTED_WAFER_Z, WAFER_THICKNESS)

ROOT = Path(__file__).resolve().parents[1]
X, Z = 'x', 'z'


def model(number):
    return ET.parse(ROOT / f'models/room_{number}_process_robot/model.sdf')


def visual(link, name):
    return link.find(f"visual[@name='{name}']")


def span(element, text='geometry/box/size'):
    """Return the vertical centre and extent of a generated box or cylinder."""
    centre = float(element.findtext('pose').split()[2])
    values = [float(v) for v in element.findtext(text).split()]
    return centre, (values[2] if len(values) > 2 else
                    values[1] if len(values) > 1 else values[0])


def test_generated_cells_place_the_cup_on_the_supported_wafer():
    for room in ROOMS:
        cell = model(room.number)
        gripper = cell.find(".//link[@name='gripper']")
        gripper_z = float(gripper.findtext('pose').split()[2])
        cup = visual(gripper, 'vacuum_cup')
        cup_centre, cup_length = span(cup, 'geometry/cylinder/length')
        assert cup_centre == pytest.approx(-PROCESS_CUP_OFFSET)
        assert cup_length == pytest.approx(PROCESS_CUP_LENGTH)
        assert gripper_z == pytest.approx(PROCESS_GRIPPER_Z)
        assert gripper_z - PROCESS_CUP_OFFSET - PROCESS_CUP_LENGTH/2 == pytest.approx(
            PROCESS_CUP_REST_BOTTOM)
        rest_bottom = gripper_z + cup_centre - cup_length/2
        assert rest_bottom == pytest.approx(PROCESS_CUP_REST_BOTTOM)
        joint = cell.find(f".//joint[@name='{room.process_joint(Z)}']")
        lower = float(joint.findtext('axis/limit/lower'))
        upper = float(joint.findtext('axis/limit/upper'))
        # Travel is contact with the wafer top face and one lift stroke clear.
        assert lower == pytest.approx(PROCESS_CONTACT_Z)
        assert upper == pytest.approx(0.)
        assert rest_bottom + lower == pytest.approx(SUPPORT_TOP + WAFER_THICKNESS)
        assert rest_bottom + upper - (SUPPORT_TOP + WAFER_THICKNESS) == pytest.approx(
            PROCESS_LIFT_STROKE)
        assert PROCESS_TRANSIT_WAFER_Z - SUPPORTED_WAFER_Z == pytest.approx(
            PROCESS_LIFT_STROKE)
        assert rest_bottom + lower > SUPPORT_TOP
        # The parked cup reaches the wafer, never the support underneath it.
        assert SUPPORTED_WAFER_Z + (upper - lower) - SUPPORT_TOP > WAFER_THICKNESS
        arm = visual(gripper, 'vertical_arm')
        arm_centre, arm_length = span(arm)
        assert gripper_z + arm_centre + arm_length/2 == pytest.approx(PROCESS_ARM_TOP)
        rail = visual(cell.find(".//link[@name='base']"), 'rail')
        rail_centre, rail_height = span(rail)
        assert rail_centre - rail_height/2 == pytest.approx(PROCESS_RAIL_BOTTOM)
        # The vertical arm has to clear the fixed rail over its whole travel.
        assert PROCESS_ARM_TOP < PROCESS_RAIL_BOTTOM
        slide = cell.find(f".//joint[@name='{room.process_joint(X)}']")
        assert float(slide.findtext('axis/limit/upper')) == pytest.approx(PROCESS_TRAVEL)


def test_support_heights_and_wafer_model_agree():
    wafer = ET.parse(ROOT / 'models/wafer/model.sdf')
    length = float(wafer.findtext(".//visual[@name='silicon']/geometry/cylinder/length"))
    assert length == pytest.approx(WAFER_THICKNESS)
    assert SUPPORTED_WAFER_Z == pytest.approx(SUPPORT_TOP + WAFER_THICKNESS/2)
    world = ET.parse(ROOT / 'worlds/four_rooms.sdf')
    for room in ROOMS:
        room_model = world.find(f".//model[@name='room_{room.number}']")
        for name in ('process_stage', 'entry_handoff', 'exit_handoff'):
            centre, height = span(room_model.find(f".//visual[@name='{name}']"))
            assert centre + height/2 == pytest.approx(SUPPORT_TOP)
        assert room.table[2] == pytest.approx(SUPPORTED_WAFER_Z)
        assert room.handoff('entry')[2] == pytest.approx(SUPPORTED_WAFER_Z)
        assert room.handoff('exit')[2] == pytest.approx(SUPPORTED_WAFER_Z)


def test_ubuntu_check_and_release_marker_cover_the_process_fault():
    source = ROOT / 'wafer_transport_sim' / 'four_room_check.py'
    module = ast.parse(source.read_text())
    scenarios = None
    for node in ast.walk(module):
        if (isinstance(node, ast.Call) and getattr(node.func, 'attr', '') == 'add_argument' and
                node.args and ast.literal_eval(node.args[0]) == '--scenario'):
            scenarios = ast.literal_eval(next(k.value for k in node.keywords
                                              if k.arg == 'choices'))
    assert scenarios, 'four_room_check --scenario choices not found'
    assert 'failed_process' in scenarios
    script = (ROOT / 'scripts/verify_ubuntu.sh').read_text()
    assert 'failed_process' in script
    marker = (ROOT / 'config/release.txt').read_text().strip()
    assert marker and marker in (ROOT.parent / 'README.md').read_text()


class Cell:
    """Kinematic stand-in for one generated cell; commands act within one tick."""

    def __init__(self, ros, room, node=None):
        self.ros = ros
        self.room = room
        self.node = node if node is not None else \
            ros.process_cell_controller.ProcessCellController()
        self.attachment = 'detached'
        self.location = 'entry'
        self.vacuum = 'detached'
        self.other_attached = ()
        self.failed_attach = False
        self.frozen = {}
        self.offsets = (0., 0., 0.)
        self.attach_attempts = 0
        self.detach_attempts = 0

    @property
    def name(self):
        return self.room.process_attachment()

    def joint(self, axis):
        name = self.room.process_joint(axis)
        if name in self.frozen:
            return self.frozen[name]
        return self.ros.outputs.get('/actuators/'+name, Scalar(0.)).data

    def support(self, location):
        return {'entry': self.room.handoff('entry'), 'process': self.room.table,
                'exit': self.room.handoff('exit')}[location]

    def support_at(self, x):
        targets = {'entry': 0., 'process': PROCESS_TRAVEL/2, 'exit': PROCESS_TRAVEL}
        return min(targets, key=lambda location: abs(targets[location]-x))

    def wafer(self):
        if self.attachment == 'attached':
            ex, ey, _ = self.room.handoff('entry')
            base = (*self.room.local_to_world(-PROCESS_TRAVEL/2+self.joint(X), -ROOM_DEPTH/2+.10),
                    SUPPORTED_WAFER_Z + (self.joint(Z) - PROCESS_CONTACT_Z))
        else:
            base = self.support(self.location)
        return tuple(value + offset for value, offset in zip(base, self.offsets))

    def feedback(self):
        self.ros.emit(f'/process/room_{self.room.number}/joint_states',
                      NS(name=[self.room.process_joint(X), self.room.process_joint(Z)],
                         position=[self.joint(X), self.joint(Z)]))
        self.ros.emit(f'/attachments/{self.name}/state', Scalar(self.attachment))
        self.ros.emit('/attachments/vacuum/state', Scalar(self.vacuum))
        for room in ROOMS:
            if room != self.room:
                self.ros.emit(f'/attachments/{room.process_attachment()}/state',
                              Scalar('attached' if room.code in self.other_attached
                                     else 'detached'))
        self.ros.emit('/poses/wafer', pose(*self.wafer()))

    def apply(self):
        for action in ('attach', 'detach'):
            if self.ros.outputs.pop(f'/attachments/{self.name}/{action}', None) is None:
                continue
            if action == 'attach':
                self.attach_attempts += 1
            else:
                self.detach_attempts += 1
            if action == 'attach' and self.failed_attach:
                continue
            self.attachment = 'attached' if action == 'attach' else 'detached'
            if action == 'detach':
                self.location = self.support_at(self.joint(X))

    def step(self, dt=.05):
        self.ros.time += dt
        self.feedback()
        self.node.tick()
        self.apply()

    def released(self, dt=.05):
        """Report every cell parked, then let the controller reach IDLE."""
        self.ros.time += dt
        self.feedback()
        self.node.tick()
        return self.node

    def run_to(self, state, limit=1500):
        for _ in range(limit):
            if self.node.state == state:
                return self
            self.step()
        raise AssertionError(f'{self.node.state} never reached {state}')

    def start(self, state='PICK_ENTRY', limit=1500):
        assert self.node.state == 'IDLE', self.node.state
        self.node.request(Scalar(self.room.code))
        assert self.node.state == 'PICK_ENTRY'
        return self.run_to(state, limit)

    def settle(self, steps=2):
        """Let instant joint travel finish so the next command can be verified."""
        for _ in range(steps):
            self.step()
        return self


def released_cell(ros, room=ROOMS[0]):
    cell = Cell(ros, room)
    cell.released()
    assert cell.node.state == 'IDLE'
    return cell


def test_cells_release_on_startup_before_reporting_ready(ros):
    cell = Cell(ros, ROOMS[0])
    cell.node.tick()
    assert not ros.outputs['/process/cells_detached'].data
    for room in ROOMS:
        ros.emit(f'/attachments/{room.process_attachment()}/state', Scalar('attached'))
    cell.node.tick()
    assert all(f'/attachments/{room.process_attachment()}/detach' in ros.outputs
               for room in ROOMS)
    assert cell.node.state == 'RELEASE'
    assert not ros.outputs['/process/cells_detached'].data
    assert ros.outputs['/process/ready'].data == ''
    # A transfer request may not start while the load-time welds still exist.
    cell.node.request(Scalar(cell.room.code))
    assert cell.node.state == 'RELEASE'
    for room in ROOMS:
        ros.emit(f'/attachments/{room.process_attachment()}/state', Scalar('detached'))
    cell.node.tick()
    assert cell.node.state == 'IDLE'
    assert ros.outputs['/process/cells_detached'].data


def test_cell_that_never_releases_faults_instead_of_reporting_ready(ros):
    cell = Cell(ros, ROOMS[0])
    for room in ROOMS[:-1]:
        ros.emit(f'/attachments/{room.process_attachment()}/state', Scalar('detached'))
    ros.emit(f'/attachments/{ROOMS[-1].process_attachment()}/state', Scalar('attached'))
    cell.node.tick()
    assert cell.node.state == 'RELEASE'
    ros.time += 6.
    cell.node.tick()
    assert cell.node.state == 'FAULT'
    assert ROOMS[-1].code in ros.outputs['/system/fault'].data
    assert not ros.outputs['/process/cells_detached'].data
    assert ros.outputs['/process/ready'].data == ''


def test_active_cell_faults_when_required_feedback_goes_stale(ros):
    cell = released_cell(ros)
    cell.start('LOWER_ENTRY')
    cell.node.received['/poses/wafer'] -= 2.
    cell.node.tick()
    assert cell.node.state == 'FAULT'
    assert 'Required feedback lost' in ros.outputs['/system/fault'].data
    assert ros.outputs['/process/ready'].data == ''
    # A fault stops commanded motion at the measured joint positions.
    assert ros.outputs[f'/actuators/{cell.room.process_joint(Z)}'].data == cell.joint(Z)
    assert ros.outputs[f'/actuators/{cell.room.process_joint(X)}'].data == cell.joint(X)


def test_stuck_cell_faults_on_the_motion_timeout(ros):
    cell = released_cell(ros)
    cell.frozen = {cell.room.process_joint(X): .10, cell.room.process_joint(Z): 0.}
    cell.node.request(Scalar(cell.room.code))
    for _ in range(3):
        cell.step()
    assert cell.node.state == 'PICK_ENTRY'
    cell.node.entered = ros.time - 31.
    cell.node.tick()
    assert cell.node.state == 'FAULT'
    assert 'PICK_ENTRY timed out after 30 s' in ros.outputs['/system/fault'].data
    assert ros.outputs[f'/actuators/{cell.room.process_joint(X)}'].data == .10


def test_unacknowledged_attach_faults_and_retains_the_wafer(ros):
    cell = released_cell(ros)
    cell.failed_attach = True
    cell.start('LOWER_ENTRY')
    cell.settle()
    name = cell.name
    assert cell.attach_attempts >= 1
    assert cell.attachment == 'detached'
    cell.node.command = (cell.node.command[0], ros.time - 6.)
    cell.node.tick()
    assert cell.node.state == 'FAULT'
    assert f'{name} did not acknowledge attached' in ros.outputs['/system/fault'].data
    assert cell.detach_attempts == 0
    assert ros.outputs['/process/ready'].data == ''


def test_attach_is_refused_while_another_owner_holds_the_wafer(ros):
    for owner in ('vacuum', 'cell'):
        cell = released_cell(ros)
        if owner == 'vacuum':
            cell.vacuum = 'attached'
        else:
            cell.other_attached = (ROOMS[4].code,)
        cell.start('LOWER_ENTRY')
        cell.settle()
        assert cell.node.state == 'FAULT'
        reason = ros.outputs['/system/fault'].data
        assert 'attach refused' in reason
        assert ('mobile vacuum' if owner == 'vacuum' else ROOMS[4].code) in reason
        assert f'/attachments/{cell.name}/attach' not in ros.outputs
        assert ros.outputs['/process/ready'].data == ''


def test_blue_stage_placement_is_verified_before_the_dwell(ros):
    cell = released_cell(ros)
    cell.start('LOWER_PROCESS')
    cell.step()
    cell.offsets = (0., 0., .06)
    cell.step()
    assert cell.node.state == 'FAULT'
    assert 'blue stage' in ros.outputs['/system/fault'].data
    assert ros.outputs['/process/ready'].data == ''


def test_wafer_lost_after_pickup_faults_instead_of_carrying_on(ros):
    cell = released_cell(ros)
    cell.start('LIFT_ENTRY')
    cell.step()
    cell.attachment = 'detached'
    cell.offsets = (0., 0., -.10)
    cell.step()
    assert cell.node.state == 'FAULT'
    assert 'vacuum cup' in ros.outputs['/system/fault'].data
    assert ros.outputs['/process/ready'].data == ''


def test_process_dwell_keeps_requiring_the_supported_stage(ros):
    cell = released_cell(ros)
    cell.start('PROCESSING')
    cell.step()
    cell.offsets = (0., 0., .04)
    cell.step()
    assert cell.node.state == 'FAULT'
    assert 'during processing' in ros.outputs['/system/fault'].data
    assert ros.outputs['/process/ready'].data == ''


def test_external_fault_holds_the_cell_and_keeps_its_payload(ros):
    cell = released_cell(ros)
    cell.start('MOVE_PROCESS')
    assert cell.attachment == 'attached'
    ros.emit('/system/fault', Scalar('transport_controller: door closure blocked'))
    cell.step()
    assert cell.node.state == 'FAULT'
    assert cell.attachment == 'attached'
    assert ros.outputs[f'/actuators/{cell.room.process_joint(X)}'].data == cell.joint(X)
    assert f'/attachments/{cell.name}/detach' not in ros.outputs


def test_full_internal_cycle_only_reports_ready_with_a_supported_exit_wafer(ros):
    cell = released_cell(ros)
    cell.start('PICK_ENTRY')
    history = []
    for _ in range(1500):
        history.append((cell.node.state, ros.outputs['/process/ready'].data))
        if cell.node.state == 'READY_EXIT':
            break
        cell.step()
    assert cell.node.state == 'READY_EXIT'
    assert all(ready == '' for _, ready in history[:-1])
    assert ros.outputs['/process/ready'].data == cell.room.code
    assert cell.attachment == 'detached'
    assert cell.wafer() == pytest.approx(cell.room.handoff('exit'))
    cell.node.consumed(Scalar(cell.room.code))
    assert cell.node.state == 'RESET'
    cell.run_to('IDLE')
    assert cell.node.room is None
