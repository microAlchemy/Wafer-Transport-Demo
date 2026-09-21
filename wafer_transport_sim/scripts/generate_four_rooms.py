#!/usr/bin/env python3
"""Generate the four-room loop using the proven mobile robot and door assets."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import qrcode
import yaml
from PIL import Image, ImageDraw
from generate_assets import (ROOT, WHITE, BLUE, SILVER, el, pose, plugin, link,
                             solid, textured_plane, save_model, write_xml)

sys.path.insert(0, str(ROOT))
from wafer_transport_sim.four_room_layout import ROOMS, PATH, START


def generate():
    root = ET.Element('sdf', version='1.9')
    world = el(root, 'world', name='wafer_transport')
    physics = el(world, 'physics', name='physics', type='ignored')
    el(physics, 'max_step_size', .001)
    el(physics, 'real_time_factor', 1.)
    el(world, 'gravity', '0 0 -9.81')
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                           ('scene-broadcaster', 'SceneBroadcaster')]:
        plugin(world, filename, name)
    plugin(world, 'sensors', 'Sensors', render_engine='ogre2')
    scene = el(world, 'scene')
    el(scene, 'ambient', '.7 .7 .7 1')
    el(scene, 'background', '.78 .83 .88 1')
    el(scene, 'shadows', 'false')
    light = el(world, 'light', name='overhead', type='directional')
    pose(light, (0, 0, 4, 0, 0, 0))
    el(light, 'diffuse', '.9 .9 .9 1')
    el(light, 'direction', '-.2 .1 -1')
    floor = el(world, 'model', name='loop_floor')
    el(floor, 'static', 'true')
    body = link(floor, 'structure')
    solid(body, 'floor', (4.8, 3.6, .03), (0, 0, -.015), color=WHITE)
    for i, (a, b) in enumerate(zip(PATH, PATH[1:])):
        length = math.dist(a, b)
        solid(body, f'tape_{i}', (length+.001, .018, .0005),
              ((a[0]+b[0])/2, (a[1]+b[1])/2, .0003),
              rpy=(0, 0, math.atan2(b[1]-a[1], b[0]-a[0])),
              color='.005 .005 .005 1', collision=False)
    for dy in (-.054, .054):
        solid(body, 'loading_rail_' + str(dy), (.15, .016, .008),
              (START[0], START[1]+dy, .142), color=SILVER)

    logo_img = Image.new('RGB', (200, 100), '#0c2340')
    d = ImageDraw.Draw(logo_img)
    d.rectangle([5, 5, 195, 95], outline='#38bdf8', width=3)
    d.text((15, 35), 'MICROALCHEMY', fill='#38bdf8')
    logo_img.save(ROOT / 'textures' / 'microalchemy_logo.png')

    logo_img = Image.new('RGB', (200, 100), '#0c2340')
    d = ImageDraw.Draw(logo_img)
    d.rectangle([5, 5, 195, 95], outline='#38bdf8', width=3)
    d.text((15, 35), 'MICROALCHEMY', fill='#38bdf8')
    logo_img.save(ROOT / 'textures' / 'microalchemy_logo.png')

    def include(name, xyz, heading=0.):
        inc = el(world, 'include')
        el(inc, 'uri', 'model://' + name)
        el(inc, 'name', name)
        pose(inc, (*xyz, 0, 0, heading))

    mappings = [m for m in yaml.safe_load((ROOT/'config/bridge.yaml').read_text())
                if m['ros_topic_name'] not in ('/door/joint_states', '/actuators/door_z')]
    for room in ROOMS:
        model = el(world, 'model', name=f'room_{room.number}')
        el(model, 'static', 'true')
        structure = link(model, 'structure')
        ymin, ymax = sorted((room.y-room.direction*.30, room.y+room.direction*.65))
        cy = (ymin+ymax)/2
        # Roof omitted for a clear view of the robot, wand and wafer.
        for y in (ymin, ymax):
            solid(structure, f'glass_{y}', (1., .012, 1.10), (room.x, y, .55),
                  color='.5 .75 .9 1', transparency=.82)
        for side in ('entry', 'exit'):
            x = room.door_x(side)
            for i, (lo, hi) in enumerate(((ymin, room.y-.155), (room.y+.155, ymax))):
                solid(structure, f'{side}_wall_{i}', (.012, hi-lo, 1.10),
                      (x, (lo+hi)/2, .55), color='.5 .75 .9 1', transparency=.78)
            name = f'room_{room.number}_{side}_door'
            door = ET.parse(ROOT/'models/guillotine_door/model.sdf').getroot()
            door.find('model').set('name', name)
            door.find(".//joint[@name='door_z']").set('name', room.door_joint(side))
            ctrl = door.find(".//plugin[@name='gz::sim::systems::JointPositionController']")
            ctrl.find('joint_name').text = room.door_joint(side)
            ctrl.find('topic').text = '/actuators/' + room.door_joint(side)
            feedback = f'/doors/room_{room.number}/{side}/joint_states'
            door.find(".//plugin[@name='gz::sim::systems::JointStatePublisher']/topic").text = feedback
            save_model(name, door)
            include(name, (x, room.y-.155, 0.))
            for topic, ros_type, gz_type, direction in [
                    (ctrl.findtext('topic'), 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'),
                    (feedback, 'sensor_msgs/msg/JointState', 'gz.msgs.Model', 'GZ_TO_ROS')]:
                mappings.append(dict(ros_topic_name=topic, gz_topic_name=topic,
                                     ros_type_name=ros_type, gz_type_name=gz_type, direction=direction))
        tx, ty, _ = room.table
        solid(structure, 'wafer_table', (.16, .16, .154), (tx, ty, .077), color=BLUE)
        solid(structure, 'equipment', (.28, .12, .28),
              (room.x-.27*room.direction, cy+.25*room.direction, .14), color=SILVER)
        qr = qrcode.make(room.code).convert('RGB')
        qr.save(ROOT/f'textures/room_{room.number}_qr.png')
        label = Image.new('RGB', (340, 50), '#cfe3f3')
        action = ('SOURCE', 'TRANSFER', 'TRANSFER', 'DESTINATION')[room.number-1]
        ImageDraw.Draw(label).text((8, 18), f'ROOM {room.number} / {action} / CLASS 100', fill='#173b50')
        label.resize((1020, 150)).save(ROOT/f'textures/room_{room.number}_label.png')
        sign_x, sign_y = room.x + room.direction*.28, room.y + room.direction*.14
        normal = room.heading + math.pi
        textured_plane(structure, 'room_qr', f'room_{room.number}_qr.png', .16, .16,
                       (sign_x, sign_y, .25), (0, 0, normal), texture_prefix='../textures/')
        textured_plane(structure, 'room_label', f'room_{room.number}_label.png', .70, .10,
                       (room.x, ymin-.009, .98), (0, 0, -math.pi/2), texture_prefix='../textures/')
        textured_plane(structure, 'microalchemy_logo', 'microalchemy_logo.png', .30, .15,
                       (room.x, ymax + .01 if room.direction == 1 else ymin - .01, .55),
                       (0, 0, 0 if room.direction == 1 else math.pi), texture_prefix='../textures/')
        textured_plane(structure, 'microalchemy_logo', 'microalchemy_logo.png', .30, .15,
                       (room.x, ymax + .01 if room.direction == 1 else ymin - .01, .55),
                       (0, 0, 0 if room.direction == 1 else math.pi), texture_prefix='../textures/')
    include('transport_robot', (*START, 0.))
    include('carrier', (*START, .15))
    include('wafer', (ROOMS[0].table[0], ROOMS[0].table[1], .156))
    gui = el(world, 'gui', fullscreen='false')
    view = el(gui, 'plugin', filename='MinimalScene', name='3D View')
    el(view, 'engine', 'ogre2')
    el(view, 'scene', 'scene')
    el(view, 'camera_pose', '0 -4.8 5.8 0 .88 1.5708')
    for filename, name in [('GzSceneManager', 'Scene Manager'),
                           ('InteractiveViewControl', 'View Control'), ('CameraTracking', 'Camera Tracking')]:
        el(gui, 'plugin', filename=filename, name=name)
    write_xml(ROOT/'worlds/four_rooms.sdf', root)
    (ROOT/'config/four_rooms_bridge.yaml').write_text(yaml.safe_dump(mappings, sort_keys=False))


if __name__ == '__main__':
    generate()
