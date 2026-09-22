#!/usr/bin/env python3
"""Generate the four-room loop using the proven mobile robot and door assets."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import cv2
import qrcode
import yaml
from PIL import Image, ImageDraw
from generate_assets import (ROOT, WHITE, BLUE, SILVER, el, pose, plugin, link, joint,
                             attachment, new_model, solid, textured_plane, save_model, write_xml)
from generate_raspbot import generate as generate_raspbot

sys.path.insert(0, str(ROOT))
from wafer_transport_sim.four_room_layout import (ROOMS, PATH, START, ROOM_LENGTH,
                                                  ROOM_DEPTH, ROOM_HEIGHT,
                                                  CORRIDOR_WIDTH)


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
    solid(body, 'floor', (5.0, 4.0, .03), (0, 0, -.015), color=WHITE)
    for i, (a, b) in enumerate(zip(PATH, PATH[1:])):
        length = math.dist(a, b)
        if length < 1e-8:
            continue
        center = ((a[0]+b[0])/2, (a[1]+b[1])/2, .00015)
        heading = math.atan2(b[1]-a[1], b[0]-a[0])
        solid(body, f'corridor_{i}', (length+.002, CORRIDOR_WIDTH, .0002), center,
              rpy=(0, 0, heading), color='.72 .75 .77 1', collision=False)
        solid(body, f'tape_{i}', (length+.001, .018, .0005),
              ((a[0]+b[0])/2, (a[1]+b[1])/2, .0003),
              rpy=(0, 0, heading),
              color='.005 .005 .005 1', collision=False)
    for dy in (-.054, .054):
        solid(body, 'loading_rail_' + str(dy), (.15, .016, .008),
              (START[0], START[1]+dy, .142), color=SILVER)

    logo_img = Image.new('RGB', (200, 100), '#0c2340')
    d = ImageDraw.Draw(logo_img)
    d.rectangle([5, 5, 195, 95], outline='#38bdf8', width=3)
    d.text((15, 35), 'MICROALCHEMY', fill='#38bdf8')
    logo_img.save(ROOT / 'textures' / 'microalchemy_logo.png')

    def include(name, xyz, heading=0., resource=None):
        inc = el(world, 'include')
        el(inc, 'uri', 'model://' + (resource or name))
        el(inc, 'name', name)
        pose(inc, (*xyz, 0, 0, heading))

    def process_robot(room):
        """A rail shuttle that carries the wafer through one process cell."""
        name = f'room_{room.number}_process_robot'
        root, model = new_model(name)
        base = link(model, 'base', mass=8., size=(.95, .06, .06))
        solid(base, 'rail', (.95, .055, .055), (0, 0, .37), color=SILVER)
        anchor = el(model, 'joint', name='world_fixed', type='fixed')
        el(anchor, 'parent', 'world')
        el(anchor, 'child', 'base')
        direction = room.direction
        entry = -direction * (ROOM_LENGTH/2-.105)
        carriage = link(model, 'carriage', (entry, 0, .37), mass=.7, size=(.12, .12, .12))
        solid(carriage, 'carriage', (.12, .12, .12), color='.85 .25 .08 1')
        x_name = room.process_joint('x')
        joint(model, x_name, 'base', 'carriage', axis=f'{direction} 0 0',
              limits=(0., ROOM_LENGTH-.21), controller={'cmd_max': .20})
        gripper = link(model, 'gripper', (entry, 0, .28), mass=.25, size=(.08, .08, .08))
        solid(gripper, 'vertical_arm', (.045, .045, .18), (0, 0, .05), color='.12 .18 .24 1')
        solid(gripper, 'vacuum_cup', (.035, .012), (0, 0, -.095), color='.05 .05 .05 1',
              shape='cylinder')
        z_name = room.process_joint('z')
        joint(model, z_name, 'carriage', 'gripper', axis='0 0 1', limits=(-.12, 0.),
              controller={'cmd_max': .10})
        attachment(model, room.process_attachment(), 'gripper', 'wafer')
        state = plugin(model, 'joint-state-publisher', 'JointStatePublisher')
        el(state, 'topic', f'/process/room_{room.number}/joint_states')
        save_model(name, root)
        include(name, (room.x, room.y, 0.))
        for topic, ros_type, gz_type, direction_name in [
                ('/actuators/'+x_name, 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'),
                ('/actuators/'+z_name, 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'),
                (f'/process/room_{room.number}/joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model', 'GZ_TO_ROS'),
                (f'/attachments/{room.process_attachment()}/attach', 'std_msgs/msg/Empty', 'gz.msgs.Empty', 'ROS_TO_GZ'),
                (f'/attachments/{room.process_attachment()}/detach', 'std_msgs/msg/Empty', 'gz.msgs.Empty', 'ROS_TO_GZ'),
                (f'/attachments/{room.process_attachment()}/state', 'std_msgs/msg/String', 'gz.msgs.StringMsg', 'GZ_TO_ROS')]:
            mappings.append(dict(ros_topic_name=topic, gz_topic_name=topic,
                                 ros_type_name=ros_type, gz_type_name=gz_type,
                                 direction=direction_name))

    mappings = [m for m in yaml.safe_load((ROOT/'config/bridge.yaml').read_text())
                if m['ros_topic_name'] not in ('/door/joint_states', '/actuators/door_z')]
    for room in ROOMS:
        model = el(world, 'model', name=f'room_{room.number}')
        el(model, 'static', 'true')
        structure = link(model, 'structure')
        ymin, ymax = room.y-ROOM_DEPTH/2, room.y+ROOM_DEPTH/2
        cy = (ymin+ymax)/2
        # Roof omitted for a clear view of the robot, wand and wafer.
        outer_y = ymax if room.direction == 1 else ymin
        inner_y = ymin if room.direction == 1 else ymax
        solid(structure, 'glass_inner', (ROOM_LENGTH, .012, ROOM_HEIGHT),
              (room.x, inner_y, ROOM_HEIGHT/2),
              color='.5 .75 .9 1', transparency=.82)
        # The chassis stays outside.  Only the wand reaches through this
        # one-foot service opening in the outside-facing wall.
        half_opening = CORRIDOR_WIDTH/2
        for i, (lo, hi) in enumerate(((room.x-ROOM_LENGTH/2, room.x-half_opening),
                                       (room.x+half_opening, room.x+ROOM_LENGTH/2))):
            solid(structure, f'glass_outer_{i}', (hi-lo, .012, ROOM_HEIGHT),
                  ((lo+hi)/2, outer_y, ROOM_HEIGHT/2),
                  color='.5 .75 .9 1', transparency=.82)
        for side in ('entry', 'exit'):
            x = room.door_x(side)
            half_passage = CORRIDOR_WIDTH/2
            for i, (lo, hi) in enumerate(((ymin, room.y-half_passage),
                                           (room.y+half_passage, ymax))):
                solid(structure, f'{side}_wall_{i}', (.012, hi-lo, ROOM_HEIGHT),
                      (x, (lo+hi)/2, ROOM_HEIGHT/2), color='.5 .75 .9 1', transparency=.78)
            name = f'room_{room.number}_{side}_door'
            door = ET.parse(ROOT/'models/guillotine_door/model.sdf').getroot()
            door.find('model').set('name', name)
            door.find(".//joint[@name='door_z']").set('name', room.door_joint(side))
            door.find('.//joint/axis/limit/velocity').text = '0.20'
            ctrl = door.find(".//plugin[@name='gz::sim::systems::JointPositionController']")
            ctrl.find('joint_name').text = room.door_joint(side)
            ctrl.find('topic').text = '/actuators/' + room.door_joint(side)
            feedback = f'/doors/room_{room.number}/{side}/joint_states'
            door.find(".//plugin[@name='gz::sim::systems::JointStatePublisher']/topic").text = feedback
            # Make the clear doorway and the floor corridor exactly 1 ft wide.
            center = CORRIDOR_WIDTH/2 + .015
            frame, panel = door.find(".//link[@name='frame']"), door.find(".//link[@name='panel']")
            for element in frame.findall('visual') + frame.findall('collision'):
                if element.get('name', '').startswith('guide_'):
                    values = list(map(float, element.findtext('pose').split()))
                    if values[1] > .1:
                        values[1] = 2*center-.008
                    element.find('pose').text = ' '.join(map(str, values))
                elif element.get('name', '').startswith('drive_housing'):
                    values = list(map(float, element.findtext('pose').split()))
                    values[1] = center
                    element.find('pose').text = ' '.join(map(str, values))
                    element.find('geometry/box/size').text = f'.045 {2*center} .06'
            values = list(map(float, panel.findtext('pose').split()))
            values[1] = center
            panel.find('pose').text = ' '.join(map(str, values))
            for element in panel.findall('visual') + panel.findall('collision'):
                element.find('geometry/box/size').text = (
                    element.findtext('geometry/box/size').split()[0] +
                    f' {CORRIDOR_WIDTH} ' + element.findtext('geometry/box/size').split()[2])
            save_model(name, door)
            include(name, (x, room.y-center, 0.))
            for topic, ros_type, gz_type, direction in [
                    (ctrl.findtext('topic'), 'std_msgs/msg/Float64', 'gz.msgs.Double', 'ROS_TO_GZ'),
                    (feedback, 'sensor_msgs/msg/JointState', 'gz.msgs.Model', 'GZ_TO_ROS')]:
                mappings.append(dict(ros_topic_name=topic, gz_topic_name=topic,
                                     ros_type_name=ros_type, gz_type_name=gz_type, direction=direction))
        tx, ty, _ = room.table
        solid(structure, 'process_stage', (.22, .22, .154), (tx, ty, .077), color=BLUE)
        for side in ('entry', 'exit'):
            hx, hy, _ = room.handoff(side)
            solid(structure, f'{side}_handoff', (.16, .16, .154), (hx, hy, .077), color=SILVER)
        solid(structure, 'equipment', (.28, .12, .28),
              (room.x-.27*room.direction, cy+.25*room.direction, .14), color=SILVER)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        for side, tag_id in (('entry', 2*(room.number-1)), ('exit', 2*(room.number-1)+1)):
            marker = cv2.aruco.generateImageMarker(dictionary, tag_id, 360)
            marker = cv2.copyMakeBorder(marker, 40, 40, 40, 40, cv2.BORDER_CONSTANT, value=255)
            cv2.imwrite(str(ROOT/f'textures/room_{room.number}_{side}_apriltag.png'), marker)
        label = Image.new('RGB', (340, 50), '#cfe3f3')
        action = ('SOURCE', 'TRANSFER', 'TRANSFER', 'DESTINATION')[room.number-1]
        ImageDraw.Draw(label).text((8, 18), f'ROOM {room.number} / {action} / CLASS 100', fill='#173b50')
        label.resize((1020, 150)).save(ROOT/f'textures/room_{room.number}_label.png')
        for side in ('entry', 'exit'):
            sign_x = room.door_x(side)
            sign_y = room.y - .19
            normal = room.heading + (math.pi if side == 'entry' else 0.)
            textured_plane(structure, f'{side}_apriltag',
                           f'room_{room.number}_{side}_apriltag.png', .20, .20,
                           (sign_x, sign_y, .27), (0, 0, normal), texture_prefix='../textures/')
        textured_plane(structure, 'room_label', f'room_{room.number}_label.png', .70, .10,
                       (room.x, ymin-.009, .98), (0, 0, -math.pi/2), texture_prefix='../textures/')
        textured_plane(structure, 'microalchemy_logo', 'microalchemy_logo.png', .30, .15,
                       (room.x, ymax + .01 if room.direction == 1 else ymin - .01, .55),
                       (0, 0, 0 if room.direction == 1 else math.pi), texture_prefix='../textures/')
        process_robot(room)
    generate_raspbot()
    include('transport_robot', (*START, 0.), resource='raspbot_v2')
    for joint_name in ('camera_pan', 'camera_tilt'):
        topic = '/actuators/' + joint_name
        mappings.append(dict(ros_topic_name=topic, gz_topic_name=topic,
                             ros_type_name='std_msgs/msg/Float64',
                             gz_type_name='gz.msgs.Double', direction='ROS_TO_GZ'))
    include('carrier', (*START, .15))
    include('wafer', (*START, .156))
    gui = el(world, 'gui', fullscreen='false')
    view = el(gui, 'plugin', filename='MinimalScene', name='3D View')
    el(view, 'engine', 'ogre2')
    el(view, 'scene', 'scene')
    el(view, 'camera_pose', '0 -5.2 5.7 0 .83 1.5708')
    for filename, name in [('GzSceneManager', 'Scene Manager'),
                           ('InteractiveViewControl', 'View Control'), ('CameraTracking', 'Camera Tracking')]:
        el(gui, 'plugin', filename=filename, name=name)
    write_xml(ROOT/'worlds/four_rooms.sdf', root)
    (ROOT/'config/four_rooms_bridge.yaml').write_text(yaml.safe_dump(mappings, sort_keys=False))


if __name__ == '__main__':
    generate()
