from pathlib import Path
import ast
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_xml_and_local_resources():
    for path in [ROOT / 'package.xml', *ROOT.rglob('*.sdf'), *ROOT.rglob('*.config')]:
        document = ET.parse(path)
        for uri in document.findall('.//uri'):
            if uri.text.startswith('model://'):
                assert (ROOT / 'models' / uri.text[8:] / 'model.sdf').exists()
        for texture in document.findall('.//albedo_map'):
            assert (path.parent / texture.text).resolve().is_file()
    models = ET.parse(ROOT / 'worlds/wafer_transport.sdf').findall('.//include/name')
    assert len(models) == len({m.text for m in models}) == 5


def test_plugin_and_bridge_contracts():
    mappings = yaml.safe_load((ROOT / 'config/bridge.yaml').read_text())
    topics = {m['ros_topic_name']: m for m in mappings}
    assert len(topics) == len(mappings)
    assert topics['/cmd_vel']['direction'] == 'ROS_TO_GZ'
    assert topics['/clock']['direction'] == 'GZ_TO_ROS'
    for camera in ('camera', 'down_camera'):
        assert topics[f'/{camera}/image_raw']['gz_type_name'] == 'gz.msgs.Image'
    for path in ROOT.rglob('*.sdf'):
        for plugin in ET.parse(path).findall('.//plugin'):
            if plugin.get('name', '').startswith('gz::sim::systems::'):
                assert plugin.get('filename').startswith('gz-sim-')
            if plugin.get('name') == 'gz::sim::systems::DetachableJoint':
                assert plugin.findtext('child_link') == 'body'
                assert plugin.find('child_model_link') is None
                for tag in ('attach_topic', 'detach_topic', 'output_topic'):
                    assert plugin.findtext(tag) in topics
                assert topics[plugin.findtext('output_topic')]['gz_type_name'] == 'gz.msgs.StringMsg'


def test_python_sources_parse_and_metadata_matches():
    for path in ROOT.rglob('*.py'):
        ast.parse(path.read_text(), filename=str(path))
    assert ET.parse(ROOT / 'package.xml').findtext('name') == 'wafer_transport_sim'
    assert (ROOT / 'resource/wafer_transport_sim').exists()
    for name in ('wafer_handler', 'transport_controller', 'line_follower',
                 'qr_detector', 'docking_controller', 'system_manager', 'integration_check'):
        module = ast.parse((ROOT / 'wafer_transport_sim' / (name + '.py')).read_text())
        assert any(isinstance(n, ast.FunctionDef) and n.name == 'main' for n in module.body)


def test_dimensions_and_docking_poses_agree():
    world = ET.parse(ROOT / 'worlds/wafer_transport.sdf')
    floor = world.find(".//visual[@name='floor']/geometry/box/size").text
    assert floor == '1.2192 1.2192 0.03'
    cfg = yaml.safe_load((ROOT / 'config/simulation.yaml').read_text())
    a = cfg['transport_controller']['ros__parameters']['station_distances']
    b = cfg['docking_controller']['ros__parameters']['station_distances']
    assert a == b == [0.28, 0.60, 0.92]
    stations = ET.parse(ROOT / 'models/stations/model.sdf')
    for letter, distance in zip('abc', a):
        p = stations.find(f".//visual[@name='{letter}_shelf_-0.054']/pose").text
        xyz = [float(v) for v in p.split()[:3]]
        assert abs(xyz[1] + .054 - (.12 + distance)) < 1e-9


def test_mobile_wand_and_door_geometry_and_bridge():
    robot = ET.parse(ROOT / 'models/transport_robot/model.sdf')
    world = ET.parse(ROOT / 'worlds/wafer_transport.sdf')
    door = ET.parse(ROOT / 'models/guillotine_door/model.sdf')
    assert not (ROOT / 'models/gantry').exists()
    vacuum = robot.find(".//plugin[@name='gz::sim::systems::DetachableJoint'][child_model='wafer']")
    assert vacuum.findtext('parent_link') == 'wand'
    for name in ('wand_x', 'wand_z'):
        assert robot.find(f".//joint[@name='{name}']").get('type') == 'prismatic'
    panel = door.find(".//link[@name='panel']")
    joint = door.find(".//joint[@name='door_z']")
    assert joint.get('type') == 'prismatic'
    travel = float(joint.findtext('axis/limit/upper'))
    assert travel == .52
    # The open panel must clear the stowed robot, and must not slide into header.
    center_z = float(panel.findtext('pose').split()[2])
    thickness, _, height = map(float, panel.findtext('collision/geometry/box/size').split())
    assert center_z - height/2 + travel > .50
    header = world.find(".//collision[@name='loading_header_collision']")
    header_x = float(header.findtext('pose').split()[0])
    header_thickness = float(header.findtext('geometry/box/size').split()[0])
    assert header_x + header_thickness/2 < .9094 - thickness/2
    topics = {m['ros_topic_name'] for m in yaml.safe_load((ROOT / 'config/bridge.yaml').read_text())}
    assert {'/actuators/door_z', '/door/joint_states', '/poses/transport_robot',
            '/actuators/wand_x', '/actuators/wand_z'} <= topics
    assert not any('gantry' in t for t in topics)
