from pathlib import Path
import ast
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_link_visual_collision_and_sensor_names_are_unique():
    # XML parsing accepts duplicate names; Gazebo rejects the entire world.
    for path in ROOT.rglob('*.sdf'):
        for link in ET.parse(path).findall('.//link'):
            for kind in ('visual', 'collision', 'sensor'):
                names = [element.get('name') for element in link.findall(kind)]
                assert len(names) == len(set(names)), (path, link.get('name'), kind, names)


def test_gui_is_enabled_by_default():
    tree = ast.parse((ROOT / 'launch/demo.launch.py').read_text())
    gui = next(node for node in ast.walk(tree) if isinstance(node, ast.Call) and
               isinstance(node.func, ast.Name) and node.func.id == 'DeclareLaunchArgument' and
               node.args and isinstance(node.args[0], ast.Constant) and node.args[0].value == 'gui')
    assert next(ast.literal_eval(k.value) for k in gui.keywords if k.arg == 'default_value') == 'true'


def test_each_room_keeps_one_logo():
    world = ET.parse(ROOT / 'worlds/four_rooms.sdf')
    for number in range(1, 5):
        room = world.find(f".//model[@name='room_{number}']")
        assert len(room.findall(".//visual[@name='microalchemy_logo']")) == 1


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
    assert float(joint.findtext('axis/limit/lower')) < 0. < .52 < travel
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


def test_model_frame_names_and_joint_connections():
    # Links and joints share the SDF frame namespace. XML parsing alone does
    # not catch collisions, which prevent Gazebo from loading the world.
    for path in ROOT.rglob('*.sdf'):
        for model in ET.parse(path).findall('.//model'):
            frames = [child.get('name') for child in model
                      if child.tag in ('link', 'joint', 'frame', 'model')]
            assert len(frames) == len(set(frames)), (path, frames)
            links = {link.get('name') for link in model.findall('link')}
            parents = {}
            for joint in model.findall('joint'):
                parent, child = joint.findtext('parent'), joint.findtext('child')
                assert parent in links | {'world'}, (path, parent)
                assert child in links, (path, child)
                assert parent != child
                assert child not in parents, (path, child)
                parents[child] = parent
            for child in parents:
                visited = set()
                while child in parents:
                    assert child not in visited, (path, child)
                    visited.add(child)
                    child = parents[child]


def test_door_full_stroke_clears_fixed_collision_geometry():
    world = ET.parse(ROOT / 'worlds/wafer_transport.sdf')
    door = ET.parse(ROOT / 'models/guillotine_door/model.sdf')
    origin = [float(v) for v in world.findtext(
        ".//include[name='guillotine_door']/pose").split()[:3]]

    def bounds(link, collision, offset):
        link_pose = [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]
        local = [float(v) for v in collision.findtext('pose', '0 0 0 0 0 0').split()]
        assert link_pose[3:] == local[3:] == [0., 0., 0.]
        size = [float(v) for v in collision.findtext('geometry/box/size').split()]
        center = [offset[i] + link_pose[i] + local[i] for i in range(3)]
        return ([center[i] - size[i]/2 for i in range(3)],
                [center[i] + size[i]/2 for i in range(3)])

    obstacles = []
    room = world.find(".//model[@name='cleanroom']")
    for link in room.findall('link'):
        for collision in link.findall('collision'):
            obstacles.append((collision.get('name'), bounds(link, collision, [0., 0., 0.])))
    frame = door.find(".//link[@name='frame']")
    for collision in frame.findall('collision'):
        obstacles.append((collision.get('name'), bounds(frame, collision, origin)))
    panel = door.find(".//link[@name='panel']")
    travel = float(door.findtext(".//joint[@name='door_z']/axis/limit/upper"))
    lower = float(door.findtext(".//joint[@name='door_z']/axis/limit/lower"))
    for collision in panel.findall('collision'):
        low, high = bounds(panel, collision, origin)
        low[2] += lower
        high[2] += travel
        for name, (fixed_low, fixed_high) in obstacles:
            overlap = all(min(high[i], fixed_high[i]) - max(low[i], fixed_low[i]) > 1e-9
                          for i in range(3))
            assert not overlap, f'{collision.get("name")} hits {name} during door travel'


def test_door_force_drive_contract():
    door = ET.parse(ROOT / 'models/guillotine_door/model.sdf')
    controller = door.find(".//plugin[@name='gz::sim::systems::JointPositionController']")
    assert controller.findtext('joint_name') == 'door_z'
    assert controller.findtext('use_velocity_commands') == 'false'
    mass = float(door.findtext(".//link[@name='panel']/inertial/mass"))
    assert abs(float(controller.findtext('cmd_offset')) - mass * 9.81) < 1e-9
    assert float(controller.findtext('p_gain')) > 0
    assert float(controller.findtext('d_gain')) > 0
    assert float(controller.findtext('cmd_max')) > mass * 9.81
    topics = {m['ros_topic_name']: m for m in yaml.safe_load((ROOT / 'config/bridge.yaml').read_text())}
    assert topics['/actuators/door_z']['direction'] == 'ROS_TO_GZ'


def test_room_tape_connects_corridor_and_pickup_camera():
    world = ET.parse(ROOT / 'worlds/wafer_transport.sdf')
    tape = world.find(".//visual[@name='room_tape']")
    x, y, z, *_ = map(float, tape.findtext('pose').split())
    length, width, _ = map(float, tape.findtext('geometry/box/size').split())
    assert x-length/2 < .65-.13 < .9094 < 1.025 < x+length/2
    assert y == .12 and width == .018 and 0 < z < .001
    assert tape.findtext('material/diffuse') == '0.005 0.005 0.005 1'
    assert world.find(".//collision[@name='room_tape_collision']") is None
