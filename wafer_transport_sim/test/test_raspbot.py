"""Static and kinematic checks for the Yahboom Raspbot V2 reconstruction."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from wafer_transport_sim import raspbot_spec as spec
from wafer_transport_sim.four_room_layout import CORRIDOR_WIDTH, PATH

ROOT = Path(__file__).resolve().parents[1]


def model():
    return ET.parse(ROOT/'models/raspbot_v2/model.sdf')


def test_published_stock_envelope_and_mass():
    assert spec.LENGTH == pytest.approx(.20874)
    assert spec.WIDTH == pytest.approx(.16583)
    assert spec.HEIGHT == pytest.approx(.12705)
    assert spec.WHEELBASE == pytest.approx(.11720)
    root = model()
    stock = ['base_link', 'front_left_wheel', 'front_right_wheel',
             'rear_left_wheel', 'rear_right_wheel', 'camera_pan_link', 'camera_tilt_link']
    mass = sum(float(root.findtext(f".//link[@name='{name}']/inertial/mass")) for name in stock)
    assert mass == pytest.approx(spec.STOCK_MASS)
    assert spec.TRACK + spec.WHEEL_WIDTH == pytest.approx(spec.WIDTH)


def test_mecanum_drive_has_four_independent_wheels():
    root = model()
    drive = root.find(".//plugin[@name='gz::sim::systems::MecanumDrive']")
    assert drive is not None
    expected = {
        'front_left_joint': 'front_left_wheel_joint',
        'front_right_joint': 'front_right_wheel_joint',
        'back_left_joint': 'rear_left_wheel_joint',
        'back_right_joint': 'rear_right_wheel_joint',
    }
    assert {name: drive.findtext(name) for name in expected} == expected
    assert root.find(".//plugin[@name='gz::sim::systems::DiffDrive']") is None
    assert len(root.findall(".//collision[@name='mecanum_contact']")) == 4


@pytest.mark.parametrize('command,signs', [
    ((.2, 0., 0.), (1, 1, 1, 1)),
    ((0., .2, 0.), (-1, 1, 1, -1)),
    ((0., 0., .5), (-1, 1, -1, 1)),
])
def test_mecanum_wheel_kinematics(command, signs):
    speeds = spec.wheel_speeds(*command)
    assert tuple(1 if value > 0 else -1 for value in speeds) == signs


def test_camera_actuators_are_bridged_and_lens_is_unobstructed():
    root = model()
    sensor = root.find(".//link[@name='camera_tilt_link']/sensor[@name='forward_camera']")
    assert float(sensor.findtext('pose').split()[0]) > .029
    down = root.find(".//link[@name='base_link']/sensor[@name='down_camera']")
    assert float(down.findtext('pose').split()[2]) < .033
    mappings = yaml.safe_load((ROOT/'config/four_rooms_bridge.yaml').read_text())
    topics = {item['ros_topic_name']: item for item in mappings}
    for name in ('camera_pan', 'camera_tilt'):
        assert topics['/actuators/'+name]['direction'] == 'ROS_TO_GZ'


def test_one_foot_corridor_underlay_is_continuous_and_wide_enough():
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')
    strips = world.findall(".//model[@name='loop_floor']//visual")
    corridor = [v for v in strips if v.get('name', '').startswith('corridor_')]
    tape = [v for v in strips if v.get('name', '').startswith('tape_')]
    assert len(corridor) == len(tape) == sum(math.dist(a, b) > 1e-8 for a, b in zip(PATH, PATH[1:]))
    assert all(float(v.findtext('geometry/box/size').split()[1]) == pytest.approx(CORRIDOR_WIDTH)
               for v in corridor)
    assert CORRIDOR_WIDTH-spec.WIDTH > .13
