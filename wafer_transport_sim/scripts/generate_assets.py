#!/usr/bin/env python3
"""Shared SDF model-building helpers for the ten-glovebox generator."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

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
