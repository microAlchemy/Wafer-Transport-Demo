"""Installed assets and interfaces for the active eleven-glovebox simulation."""
from pathlib import Path
import ast
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_active_world_and_local_resources():
    world = ET.parse(ROOT/'worlds/four_rooms.sdf')
    names = [item.text for item in world.findall('.//include/name')]
    assert len(names) == len(set(names)) == 36  # robot, carrier, wafer, 22 doors, 11 cells
    for path in ROOT.rglob('*.sdf'):
        document = ET.parse(path)
        for uri in document.findall('.//uri'):
            if uri.text.startswith('model://'):
                assert (ROOT/'models'/uri.text[8:]/'model.sdf').is_file()
        for texture in document.findall('.//albedo_map'):
            assert (path.parent/texture.text).resolve().is_file()
        for link in document.findall('.//link'):
            for kind in ('visual', 'collision', 'sensor'):
                names = [item.get('name') for item in link.findall(kind)]
                assert len(names) == len(set(names)), (path, link.get('name'), kind)


def test_only_apriltag_machine_readable_images_are_packaged():
    textures = ROOT/'textures'
    tags = sorted(textures.glob('*_apriltag.png'))
    assert len(tags) == 22
    assert not list(textures.glob('*_qr.png'))
    for room in range(1, 12):
        assert (textures/f'room_{room}_entry_apriltag.png').is_file()
        assert (textures/f'room_{room}_exit_apriltag.png').is_file()


def test_active_entry_points_and_bridge():
    setup = ast.parse((ROOT/'setup.py').read_text())
    assert setup
    source = (ROOT/'setup.py').read_text()
    for name in ('four_room_controller', 'ir_line_follower', 'apriltag_detector',
                 'process_cell_controller', 'four_room_check'):
        assert f"'{name}'" in source
        module = ast.parse((ROOT/'wafer_transport_sim'/f'{name}.py').read_text())
        assert any(isinstance(n, ast.FunctionDef) and n.name == 'main' for n in module.body)
    topics = {item['ros_topic_name']: item for item in
              yaml.safe_load((ROOT/'config/four_rooms_bridge.yaml').read_text())}
    assert topics['/clock']['direction'] == 'GZ_TO_ROS'
    assert topics['/cmd_vel']['direction'] == 'ROS_TO_GZ'
    assert topics['/camera/image_raw']['direction'] == 'GZ_TO_ROS'
    assert '/detected_station' not in topics
    for path in ROOT.rglob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
    assert ET.parse(ROOT/'package.xml').findtext('name') == 'wafer_transport_sim'
