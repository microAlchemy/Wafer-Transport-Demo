#!/usr/bin/env python3
"""Rebuild local SDF geometry and real QR textures; no external model downloads."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import qrcode
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
SILVER = '0.65 0.72 0.78 1'
BLUE = '0.06 0.30 0.48 1'
WHITE = '0.93 0.95 0.97 1'


def el(parent, tag, text=None, **attrs):
    out = ET.SubElement(parent, tag, attrs)
    if text is not None:
        out.text = str(text)
    return out


def pose(parent, values):
    el(parent, 'pose', ' '.join(str(v) for v in values))


def plugin(model, filename, name, **params):
    out = el(model, 'plugin', filename='gz-sim-' + filename + '-system',
             name='gz::sim::systems::' + name)
    for k, v in params.items():
        el(out, k, str(v).lower() if isinstance(v, bool) else v)
    return out


def geometry(parent, shape, size):
    g = el(parent, 'geometry')
    obj = el(g, shape)
    if shape == 'box':
        el(obj, 'size', ' '.join(map(str, size)))
    elif shape == 'cylinder':
        el(obj, 'radius', size[0])
        el(obj, 'length', size[1])
    elif shape == 'sphere':
        el(obj, 'radius', size[0])


def solid(link, name, size, xyz=(0, 0, 0), color=SILVER, shape='box',
          collision=True, rpy=(0, 0, 0), transparency=0):
    visual = el(link, 'visual', name=name)
    pose(visual, (*xyz, *rpy))
    geometry(visual, shape, size)
    material = el(visual, 'material')
    el(material, 'ambient', color)
    el(material, 'diffuse', color)
    if transparency:
        el(visual, 'transparency', transparency)
    if collision:
        c = el(link, 'collision', name=name + '_collision')
        pose(c, (*xyz, *rpy))
        geometry(c, shape, size)
        surface = el(c, 'surface')
        friction = el(el(surface, 'friction'), 'ode')
        el(friction, 'mu', 0.9)
        el(friction, 'mu2', 0.9)
    return visual


def link(model, name, xyz=(0, 0, 0), mass=0.1, size=(0.1, 0.1, 0.1)):
    obj = el(model, 'link', name=name)
    pose(obj, (*xyz, 0, 0, 0))
    inertial = el(obj, 'inertial')
    el(inertial, 'mass', mass)
    inertia = el(inertial, 'inertia')
    x, y, z = size
    for key, value in dict(ixx=mass*(y*y+z*z)/12,
                           iyy=mass*(x*x+z*z)/12,
                           izz=mass*(x*x+y*y)/12,
                           ixy=0, ixz=0, iyz=0).items():
        el(inertia, key, value)
    return obj


def joint(model, name, parent, child, axis=None, limits=None, controller=None):
    j = el(model, 'joint', name=name, type='prismatic' if limits else
           ('revolute' if axis else 'fixed'))
    el(j, 'parent', parent)
    el(j, 'child', child)
    if axis:
        a = el(j, 'axis')
        el(a, 'xyz', axis)
        lim = el(a, 'limit')
        el(lim, 'lower', limits[0] if limits else -1e16)
        el(lim, 'upper', limits[1] if limits else 1e16)
        el(lim, 'effort', 100)
        el(lim, 'velocity', 0.12 if limits else 20)
    if limits:
        controls = dict(use_velocity_commands=True, cmd_max=0.10)
        controls.update(controller or {})
        plugin(model, 'joint-position-controller', 'JointPositionController',
               joint_name=name, topic='/actuators/' + name, **controls)


def attachment(model, name, parent, child):
    plugin(model, 'detachable-joint', 'DetachableJoint', parent_link=parent,
           child_model=child, child_link='body',
           attach_topic='/attachments/' + name + '/attach',
           detach_topic='/attachments/' + name + '/detach',
           output_topic='/attachments/' + name + '/state')


def pose_feedback(model, name):
    plugin(model, 'pose-publisher', 'PosePublisher', publish_link_pose=False,
           publish_model_pose=True, publish_nested_model_pose=False,
           use_pose_vector_msg=False, update_frequency=30,
           topic='/poses/' + name)


def new_model(name, static=False):
    root = ET.Element('sdf', version='1.9')
    model = el(root, 'model', name=name)
    el(model, 'static', str(static).lower())
    el(model, 'self_collide', 'true')
    return root, model


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def save_model(name, root):
    path = ROOT / 'models' / name
    write_xml(path / 'model.sdf', root)
    cfg = ET.Element('model')
    el(cfg, 'name', name)
    el(cfg, 'version', '1.0')
    el(cfg, 'sdf', 'model.sdf', version='1.9')
    el(cfg, 'description', 'Local wafer transport demonstration geometry')
    write_xml(path / 'model.config', cfg)


def textured_plane(link_obj, name, image, width, height, xyz, rpy,
                   texture_prefix='../../textures/'):
    """Orient the XY textured plane with horizontal U, vertical V and +X normal."""
    v = el(link_obj, 'visual', name=name)
    pose(v, (*xyz, math.pi/2, 0, rpy[2] + math.pi/2))
    plane = el(el(v, 'geometry'), 'plane')
    el(plane, 'normal', '0 0 1')
    el(plane, 'size', f'{width} {height}')
    material = el(v, 'material')
    el(material, 'ambient', '1 1 1 1')
    el(material, 'diffuse', '1 1 1 1')
    el(material, 'emissive', '0.35 0.35 0.35 1')
    metal = el(el(material, 'pbr'), 'metal')
    el(metal, 'albedo_map', texture_prefix + image)
    el(metal, 'metalness', 0)
    el(metal, 'roughness', 1)
    el(v, 'cast_shadows', 'false')


def textures():
    dest = ROOT / 'textures'
    dest.mkdir(exist_ok=True)
    for letter, process in [('a', 'LOAD'), ('b', 'SPIN_COAT'), ('c', 'BAKE')]:
        qr = qrcode.QRCode(version=1, error_correction=qrcode.constants.ERROR_CORRECT_M,
                           box_size=12, border=4)
        qr.add_data('STATION_' + letter.upper())
        qr.make(fit=False)
        qr.make_image(fill_color='black', back_color='white').convert('RGB').save(
            dest / f'station_{letter}_qr.png')
        label = Image.new('RGB', (180, 30), '#eff6ff')
        d = ImageDraw.Draw(label)
        d.text((4, 8), f'STATION {letter.upper()} / {process}', fill='#10344a')
        label.resize((720, 120)).save(dest / f'station_{letter}_label.png')
    for name, text in [('enclosure', 'CLASS 100 / 4 x 3 x 4 ft'),
                       ('corridor', 'CORRIDOR 1 x 4 x 4 ft')]:
        image = Image.new('RGB', (210, 30), '#10344a')
        ImageDraw.Draw(image).text((4, 8), text, fill='white')
        image.resize((840, 120)).save(dest / f'{name}_label.png')


def wafer():
    root, model = new_model('wafer')
    body = link(model, 'body', mass=0.02, size=(0.1, 0.1, 0.002))
    solid(body, 'silicon', (0.05, 0.002), shape='cylinder', color='0.18 0.13 0.32 1')
    solid(body, 'orientation_notch', (0.006, 0.004, 0.0002), (0.046, 0, 0.0011),
          color='0.7 0.7 0.8 1', collision=False)
    pose_feedback(model, 'wafer')
    save_model('wafer', root)


def carrier():
    root, model = new_model('carrier')
    body = link(model, 'body', mass=0.09, size=(0.13, 0.13, 0.025))
    solid(body, 'base', (0.13, 0.13, 0.008), color=BLUE)
    for axis in (0, 1):
        for sign in (-1, 1):
            xyz = [0, 0, 0.017]
            xyz[axis] = sign * 0.062
            size = [0.13, 0.13, 0.026]
            size[axis] = 0.006
            solid(body, f'rim_{axis}_{sign}', size, xyz, color=WHITE)
    # An open clamshell lip leaves the wafer and wand visible.
    solid(body, 'hinge', (0.012, 0.13, 0.008), (-0.065, 0, 0.008), color=SILVER)
    pose_feedback(model, 'carrier')
    save_model('carrier', root)


def door():
    root, model = new_model('guillotine_door')
    frame = link(model, 'frame', mass=2)
    for y in (0.008, 0.302):
        solid(frame, 'guide_' + str(y), (0.026, 0.012, 1.05),
              (0, y, 0.525), color=BLUE)
    solid(frame, 'drive_housing', (0.045, 0.31, 0.06), (0, 0.155, 1.08))
    joint(model, 'door_mount', 'world', 'frame')
    panel = link(model, 'panel', (0, 0.155, 0.245), mass=0.4,
                 size=(0.012, 0.278, 0.48))
    solid(panel, 'sliding_panel', (0.012, 0.278, 0.48), color='0.5 0.75 0.85 1',
          transparency=0.35)
    solid(panel, 'bottom_edge', (0.016, 0.278, 0.012), (0, 0, -0.234),
          color='0.9 0.65 0.05 1')
    # Keep both operating positions clear of the mechanical joint limits.
    # Apply force through the physics engine; offset balances the 0.4 kg panel.
    joint(model, 'door_z', 'frame', 'panel', '0 0 1', (-0.004, 0.535),
          controller=dict(use_velocity_commands=False, p_gain=600.0, i_gain=0.0,
                          d_gain=30.0, cmd_max=20.0, cmd_min=-20.0,
                          cmd_offset=0.4 * 9.81))
    plugin(model, 'joint-state-publisher', 'JointStatePublisher', topic='/door/joint_states')
    save_model('guillotine_door', root)


def camera(body, name, xyz, rpy, width, height, fov, topic, rate=20):
    s = el(body, 'sensor', name=name, type='camera')
    pose(s, (*xyz, *rpy))
    el(s, 'always_on', 'true')
    el(s, 'update_rate', rate)
    el(s, 'topic', topic)
    c = el(s, 'camera')
    el(c, 'horizontal_fov', fov)
    im = el(c, 'image')
    el(im, 'width', width)
    el(im, 'height', height)
    el(im, 'format', 'R8G8B8')
    clip = el(c, 'clip')
    el(clip, 'near', 0.01)
    el(clip, 'far', 3)


def robot():
    root, model = new_model('transport_robot')
    body = link(model, 'base_link', (0, 0, 0.066), mass=1.0, size=(0.18, 0.12, 0.05))
    solid(body, 'chassis', (0.18, 0.12, 0.05), color=BLUE)
    for name, y in [('left', 0.07), ('right', -0.07)]:
        wheel = link(model, name + '_wheel', (0, y, 0.035), mass=0.08,
                     size=(0.07, 0.014, 0.07))
        solid(wheel, 'tire', (0.035, 0.014), shape='cylinder',
              rpy=(math.pi/2, 0, 0), color='0.035 0.04 0.045 1')
        joint(model, name + '_wheel_joint', 'base_link', name + '_wheel', '0 1 0')
    for x in (-0.07, 0.07):
        caster = solid(body, 'caster_' + str(x), (0.012,), (x, 0, -0.054),
                       shape='sphere', color=SILVER)
    # Low-friction fixed spherical casters.
    for c in body.findall('collision'):
        if c.get('name', '').startswith('caster'):
            for mu in c.findall('./surface/friction/ode/*'):
                mu.text = '0.001'
    lift = link(model, 'lift', (0, 0, 0.12), mass=0.04)
    solid(lift, 'lift_column', (0.045, 0.04, 0.025), color=SILVER, collision=False)
    tray = link(model, 'tray', (0, 0, 0.14), mass=0.04)
    solid(tray, 'transfer_fork', (0.07, 0.13, 0.008), color=SILVER, collision=False)
    joint(model, 'tray_z', 'base_link', 'lift', '0 0 1', (-0.03, 0.01))
    joint(model, 'tray_y', 'lift', 'tray', '0 1 0', (-0.125, 0.005))
    solid(body, 'camera_mast', (0.012, 0.012, 0.16), (0.077, 0, 0.105), collision=False)
    camera(body, 'forward_camera', (0.095, 0, 0.184), (0, 0, 0),
           960, 720, 1.7, '/camera/image_raw', rate=8)
    # Forward of chassis so it sees the stripe without looking through the body.
    camera(body, 'down_camera', (0.13, 0, 0.015), (0, math.pi/2, 0),
           320, 240, 1.3, '/down_camera/image_raw')
    plugin(model, 'diff-drive', 'DiffDrive', left_joint='left_wheel_joint',
           right_joint='right_wheel_joint', wheel_separation=0.14,
           wheel_radius=0.035, topic='/cmd_vel', odom_topic='/odom',
           tf_topic='/robot/tf', frame_id='odom', child_frame_id='base_link',
           odom_publish_frequency=30, max_linear_acceleration=0.08,
           max_angular_acceleration=0.8)
    plugin(model, 'joint-state-publisher', 'JointStatePublisher', topic='/robot/joint_states')
    # The vacuum manipulator travels with the chassis. Its tip retracts over
    # the carrier; both axes are physical prismatic joints with feedback.
    mast = link(model, 'wand_lift', (-0.075, 0, 0.44), mass=0.025)
    solid(mast, 'mast', (0.016, 0.018, 0.14), (0, 0, -0.05))
    solid(mast, 'fixed_sleeve', (0.12, 0.024, 0.024), (0.045, 0, 0.012),
          collision=False)
    for stage in (1, 2):
        slider = link(model, 'reach_link_' + str(stage), (-0.04, 0, 0.452), mass=0.01)
        width = .024 - stage * .004
        solid(slider, 'telescoping_tube', (.10, width, width), collision=False)
        joint(model, 'reach_' + str(stage), 'wand_lift' if stage == 1 else 'reach_link_1',
              'reach_link_' + str(stage), '1 0 0', (-.003, .09))
    arm = link(model, 'wand', (0, 0, 0.342), mass=0.025)
    solid(arm, 'vacuum_wand', (0.008, 0.08), shape='cylinder')
    solid(arm, 'vacuum_cup', (0.012, 0.004), (0, 0, -0.04),
          shape='cylinder', color='0.05 0.05 0.06 1')
    solid(arm, 'reach_rail', (0.10, 0.012, 0.012), (-0.04, 0, 0.11),
          collision=False)
    joint(model, 'wand_z', 'base_link', 'wand_lift', '0 0 1', (-0.145, 0.01))
    joint(model, 'wand_x', 'reach_link_2', 'wand', '1 0 0', (-.003, .09))
    attachment(model, 'vacuum', 'wand', 'wafer')
    pose_feedback(model, 'transport_robot')
    attachment(model, 'carrier', 'tray', 'carrier')
    save_model('transport_robot', root)


def stations():
    root, model = new_model('stations', True)
    body = link(model, 'structure')
    for letter, y in [('a', 0.4), ('b', 0.72), ('c', 1.04)]:
        # Two rails support the carrier at its longitudinal edges. The robot's
        # narrow fork slides between them, without intersecting the shelf.
        for dy in (-0.054, 0.054):
            solid(body, f'{letter}_shelf_{dy}', (0.12, 0.016, 0.008),
                  (1.145, y + dy, 0.126), color=SILVER)
        solid(body, letter + '_shelf_support', (0.012, 0.12, 0.12),
              (1.201, y, 0.06), color=BLUE)
        solid(body, letter + '_sign_post', (0.006, 0.006, 0.20),
              (1.199, y + 0.09, 0.22), color=SILVER, collision=False)
        # Camera points along +Y. Normal points upstream and toward the route.
        yaw = -2.12
        textured_plane(body, letter + '_qr', f'station_{letter}_qr.png',
                       0.115, 0.115, (1.168, y + 0.055, 0.25), (0, 0, yaw))
        textured_plane(body, letter + '_label', f'station_{letter}_label.png',
                       0.115, 0.028, (1.168, y + 0.055, 0.33), (0, 0, yaw))
        solid(body, letter + '_dock_mark', (0.025, 0.1, 0.001),
              (1.10, y, 0.0007), color='0.1 0.7 0.5 1', collision=False)
    save_model('stations', root)


def world():
    root = ET.Element('sdf', version='1.9')
    world = el(root, 'world', name='wafer_transport')
    physics = el(world, 'physics', name='physics', type='ignored')
    el(physics, 'max_step_size', 0.001)
    el(physics, 'real_time_factor', 1.0)
    el(world, 'gravity', '0 0 -9.81')
    plugin(world, 'physics', 'Physics')
    plugin(world, 'user-commands', 'UserCommands')
    plugin(world, 'scene-broadcaster', 'SceneBroadcaster')
    plugin(world, 'sensors', 'Sensors', render_engine='ogre2')
    scene = el(world, 'scene')
    el(scene, 'ambient', '0.65 0.65 0.65 1')
    el(scene, 'background', '0.78 0.83 0.88 1')
    el(scene, 'shadows', 'false')
    light = el(world, 'light', name='overhead', type='directional')
    pose(light, (0, 0, 3, 0, 0, 0))
    el(light, 'diffuse', '0.9 0.9 0.9 1')
    el(light, 'direction', '-0.2 0.1 -1')
    room = el(world, 'model', name='cleanroom')
    el(room, 'static', 'true')
    body = link(room, 'structure')
    solid(body, 'floor', (1.2192, 1.2192, 0.03), (0.6096, 0.6096, -0.015), color=WHITE)
    solid(body, 'enclosure_back', (0.015, 1.2192, 1.2192), (0.0075, 0.6096, 0.6096))
    solid(body, 'enclosure_far', (0.9144, 0.015, 1.2192), (0.4572, 1.2117, 0.6096))
    # Robot doorway: y=0.014..0.296, with a vertically sliding panel.
    solid(body, 'divider', (0.01, 0.9092, 1.2192), (0.9094, 0.7646, 0.6096),
          color='0.6 0.8 0.9 1', transparency=0.78)
    solid(body, 'loading_header', (0.01, 0.31, 0.6992), (0.8854, 0.155, 0.8696),
          color='0.6 0.8 0.9 1', transparency=0.78)
    solid(body, 'front_glass', (0.9144, 0.01, 1.2192), (0.4572, 0.005, 0.6096),
          color='0.6 0.8 0.9 1', transparency=0.85)
    for x in (0.28, 0.64):
        solid(body, 'glove_port_' + str(x), (0.06, 0.012), (x, -0.003, 0.58),
              shape='cylinder', rpy=(math.pi/2, 0, 0), color=BLUE, collision=False)
        solid(body, 'glove_insert_' + str(x), (0.046, 0.014), (x, -0.006, 0.58),
              shape='cylinder', rpy=(math.pi/2, 0, 0), color=SILVER, collision=False)
    # Keep the corner post behind the door plane, clear of its entire stroke.
    for x in (0.02, 0.87):
        solid(body, 'frame_' + str(x), (0.025, 0.025, 1.2192), (x, 0.0125, 0.6096))
    solid(body, 'top_frame', (0.9144, 0.025, 0.025), (0.4572, 0.025, 1.2067))
    solid(body, 'corridor_boundary', (0.008, 1.2192, 0.025), (1.2152, 0.6096, 0.0125))
    solid(body, 'path', (0.018, 1.19, 0.0005), (1.025, 0.6096, 0.0003),
          color='0.005 0.005 0.005 1', collision=False)
    # Continuous 18 mm tape branch through the doorway to the pickup approach.
    # It extends beyond the downward camera at the stopped pickup pose.
    solid(body, 'room_tape', (0.58, 0.018, 0.0005), (0.79, 0.12, 0.0003),
          color='0.005 0.005 0.005 1', collision=False)
    solid(body, 'process_output', (0.18, 0.16, 0.154), (0.4, 0.12, 0.077), color=BLUE)
    solid(body, 'process_machine', (0.25, 0.3, 0.4), (0.30, 0.55, 0.2), color=SILVER)
    solid(body, 'equipment_screen', (0.12, 0.008, 0.06), (0.30, 0.396, 0.32),
          color='0.05 0.65 0.65 1', collision=False)
    for dy in (-0.054, 0.054):
        solid(body, 'loading_rail_' + str(dy), (0.15, 0.016, 0.008),
              (1.025, 0.12 + dy, 0.142), color=SILVER)
    textured_plane(body, 'enclosure_dimensions', 'enclosure_label.png',
                   0.65, 0.085, (0.4572, -0.001, 1.11), (0, 0, -math.pi/2), texture_prefix='../textures/')
    textured_plane(body, 'corridor_dimensions', 'corridor_label.png',
                   0.50, 0.07, (1.219, 0.62, 0.55), (0, 0, 0), texture_prefix='../textures/')
    for name, xyz, yaw in [
        ('wafer', (0.4, 0.12, 0.156), 0),
        ('carrier', (1.025, 0.12, 0.15), math.pi/2),
        ('guillotine_door', (0.9094, 0, 0), 0),
        ('transport_robot', (1.025, 0.12, 0), math.pi/2),
        ('stations', (0, 0, 0), 0),
    ]:
        inc = el(world, 'include')
        el(inc, 'uri', 'model://' + name)
        el(inc, 'name', name)
        pose(inc, (*xyz, 0, 0, yaw))
    gui = el(world, 'gui', fullscreen='false')
    view = el(gui, 'plugin', filename='MinimalScene', name='3D View')
    el(view, 'engine', 'ogre2')
    el(view, 'scene', 'scene')
    el(view, 'camera_pose', '2.6 -1.6 2.0 0 0.48 2.35')
    el(gui, 'plugin', filename='GzSceneManager', name='Scene Manager')
    el(gui, 'plugin', filename='InteractiveViewControl', name='View Control')
    el(gui, 'plugin', filename='CameraTracking', name='Camera Tracking')
    write_xml(ROOT / 'worlds' / 'wafer_transport.sdf', root)


if __name__ == '__main__':
    textures()
    wafer()
    carrier()
    door()
    robot()
    stations()
    world()
