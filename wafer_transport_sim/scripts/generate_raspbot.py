#!/usr/bin/env python3
"""Dimensioned Raspbot V2 reconstruction with the custom wafer payload module.

No external assets are fetched or executable vendor code run by this generator.
"""
import math
import sys
import xml.etree.ElementTree as ET
from generate_assets import ROOT, el, pose, link, solid, plugin, camera, save_model, geometry

sys.path.insert(0, str(ROOT))
from wafer_transport_sim import raspbot_spec as spec

BLACK = '.055 .06 .065 1'
RUBBER = '.018 .02 .022 1'
YELLOW = '.98 .70 .015 1'
PCB = '.045 .28 .12 1'
SILVER = '.62 .66 .70 1'
GZ = 'http://gazebosim.org/schema'
ET.register_namespace('gz', GZ)


def servo(model, name, parent, child, axis, limit):
    j = el(model, 'joint', name=name, type='revolute')
    el(j, 'parent', parent)
    el(j, 'child', child)
    a = el(j, 'axis')
    el(a, 'xyz', axis)
    limits = el(a, 'limit')
    for key, value in [('lower', -limit), ('upper', limit), ('velocity', 1.), ('effort', .2)]:
        el(limits, key, value)
    plugin(model, 'joint-position-controller', 'JointPositionController',
           joint_name=name, topic='/actuators/'+name, use_velocity_commands=True, cmd_max=1.)


def roller(wheel, name, theta, handed):
    # Eight passive 45-degree rollers, represented visually; anisotropic contact
    # below supplies their idealized free rolling direction without 32 extra joints.
    v = el(wheel, 'visual', name=name)
    axis = (-math.sin(theta)/math.sqrt(2), handed/math.sqrt(2), math.cos(theta)/math.sqrt(2))
    pitch = math.acos(axis[2])
    yaw = math.atan2(axis[1], axis[0])
    pose(v, (.022*math.cos(theta), 0., .022*math.sin(theta), 0., pitch, yaw))
    obj = el(el(v, 'geometry'), 'ellipsoid')
    el(obj, 'radii', '.008 .008 .016')
    material = el(v, 'material')
    el(material, 'ambient', RUBBER)
    el(material, 'diffuse', RUBBER)


def generate():
    # Retain only the proven custom wafer mechanisms and their plugin wiring.
    root = ET.parse(ROOT/'models/transport_robot/model.sdf').getroot()
    model = root.find('model')
    for name in ('base_link', 'left_wheel', 'right_wheel'):
        model.remove(model.find(f"link[@name='{name}']"))
    for name in ('left_wheel_joint', 'right_wheel_joint'):
        model.remove(model.find(f"joint[@name='{name}']"))
    model.remove(model.find("plugin[@name='gz::sim::systems::DiffDrive']"))
    body = link(model, 'base_link', (0, 0, .066), mass=.605, size=(.178, .108, .106))
    model.remove(body)
    model.insert(2, body)  # Keep canonical base_link first, before wand and tray.
    model.set('canonical_link', 'base_link')
    # Chassis, battery enclosure and forward lip reach the published envelope.
    solid(body, 'metal_chassis', (.178, .105, .003), (0, 0, -.014), color=BLACK)
    solid(body, 'battery_box', (.160, .099, .030), (0, 0, -.032), color=BLACK)
    solid(body, 'rear_bumper', (.004, .098, .022), (-.0909, 0, -.027), color=BLACK)
    solid(body, 'front_bumper', (.022, .095, .024), (.084, 0, -.028), color=BLACK)
    # TT motor cans, electronics and standoffs are visual detail in the stock mass.
    for sx in (-1, 1):
        for sy in (-1, 1):
            solid(body, f'tt_motor_{sx}_{sy}', (.037, .018, .022),
                  (sx*spec.WHEELBASE/2, sy*.040, -.032), color=YELLOW, collision=False)
            solid(body, f'standoff_{sx}_{sy}', (.0023, .034),
                  (sx*.027, sy*.021, .013), shape='cylinder', color='.65 .48 .16 1', collision=False)
    solid(body, 'driver_board', (.085, .065, .002), (-.01, 0, -.005), color=PCB, collision=False)
    solid(body, 'raspberry_pi_5', (.085, .056, .002), (-.018, 0, .031), color=PCB, collision=False)
    for y in (-.015, .009):
        solid(body, f'usb_port_{y}', (.016, .014, .011), (-.048, y, .0375), color=SILVER, collision=False)
    solid(body, 'processor', (.013, .013, .003), (-.007, 0, .034), color=SILVER, collision=False)
    # Slotted black upper bracket; open space exposes the board stack.
    solid(body, 'upper_deck', (.080, .070, .002), (-.026, 0, .047), color=BLACK, collision=False)
    for sy in (-1, 1):
        for i in range(6):
            solid(body, f'bracket_rib_{sy}_{i}', (.004, .002, .035),
                  (-.060+i*.013, sy*.035, .028), color=BLACK, collision=False,
                  rpy=(0, (-1 if i%2 else 1)*.25, 0))
    solid(body, 'oled_case', (.025, .029, .010), (-.074, 0, .031), color=BLACK, collision=False)
    solid(body, 'oled_screen', (.021, .025, .0005), (-.074, 0, .0363), color='.03 .32 .42 1', collision=False)
    for i, color in enumerate(('.8 .05 .04 1', '.05 .8 .15 1', '.03 .15 .9 1')):
        solid(body, f'rgb_tail_{i}', (.002, .012, .005), (-.093, (i-1)*.015, -.003), color=color, collision=False)
    # Stock ultrasonic front face and four IR reflectance probes (visual only).
    solid(body, 'ultrasonic_housing', (.032, .052, .025), (.074, 0, .004), color=BLACK)
    for y in (-.014, .014):
        solid(body, f'ultrasonic_ring_{y}', (.008, .004), (.092, y, .005),
              shape='cylinder', rpy=(0, math.pi/2, 0), color=SILVER, collision=False)
        solid(body, f'ultrasonic_face_{y}', (.006, .0045), (.093, y, .005),
              shape='cylinder', rpy=(0, math.pi/2, 0), color=RUBBER, collision=False)
    solid(body, 'ir_tracking_board', (.018, .078, .002), (.10684, 0, -.051), color=PCB, collision=False)
    for i, y in enumerate((-.030, -.010, .010, .030)):
        solid(body, f'ir_probe_{i}', (.007, .006, .005), (.10684, y, -.0545), color=BLACK, collision=False)
    # Four independent mecanum wheel links; no differential-drive casters.
    for name, sx, sy in [('front_left', 1, 1), ('front_right', 1, -1),
                          ('rear_left', -1, 1), ('rear_right', -1, -1)]:
        wheel = link(model, name+'_wheel', (sx*spec.WHEELBASE/2, sy*spec.TRACK/2, spec.WHEEL_RADIUS),
                     mass=.055, size=(.060, .030, .060))
        solid(wheel, 'yellow_hub', (.023, .023), shape='cylinder',
              rpy=(math.pi/2, 0, 0), color=YELLOW, collision=False)
        solid(wheel, 'black_axle', (.009, .030), shape='cylinder',
              rpy=(math.pi/2, 0, 0), color=RUBBER, collision=False)
        handed = -sx*sy
        for i in range(8):
            roller(wheel, f'passive_roller_{i}', i*math.pi/4, handed)
        c = el(wheel, 'collision', name='mecanum_contact')
        pose(c, (0, 0, 0, math.pi/2, 0, 0))
        geometry(c, 'cylinder', (spec.WHEEL_RADIUS, spec.WHEEL_WIDTH))
        ode = el(el(el(c, 'surface'), 'friction'), 'ode')
        el(ode, 'mu', 1.)
        el(ode, 'mu2', 0.)
        el(ode, 'fdir1', f'1 {handed} 0', **{'{'+GZ+'}expressed_in': 'base_link'})
        j = el(model, 'joint', name=name+'_wheel_joint', type='revolute')
        el(j, 'parent', 'base_link')
        el(j, 'child', name+'_wheel')
        a = el(j, 'axis')
        el(a, 'xyz', '0 1 0')
        lim = el(a, 'limit')
        for key, value in [('lower', -1e16), ('upper', 1e16), ('effort', .8), ('velocity', spec.MOTOR_MAX_RAD_S)]:
            el(lim, key, value)
    pan = link(model, 'camera_pan_link', (.058, 0, .074), mass=.025, size=(.028, .030, .012))
    solid(pan, 'pan_servo', (.028, .030, .010), color=BLACK)
    for sy in (-1, 1):
        solid(pan, f'gimbal_side_{sy}', (.028, .003, .038), (.024, sy*.024, .030), color=BLACK, collision=False)
    tilt = link(model, 'camera_tilt_link', spec.CAMERA_PIVOT, mass=.035, size=(.026, .045, .037))
    solid(tilt, 'camera_body', (.024, .045, .034), color=BLACK)
    solid(tilt, 'camera_top', (.031, .05114, .003), (0, 0, .024), color=BLACK, collision=False)
    solid(tilt, 'lens_barrel', (.009, .016), (.020, 0, 0),
          shape='cylinder', rpy=(0, math.pi/2, 0), color=RUBBER, collision=False)
    solid(tilt, 'lens_glass', (.0065, .001), (.0285, 0, 0),
          shape='cylinder', rpy=(0, math.pi/2, 0), color='.06 .14 .19 1', collision=False)
    camera(tilt, 'forward_camera', (spec.CAMERA_OFFSET, 0, 0), (0, 0, 0),
           640, 480, spec.CAMERA_FOV, '/camera/image_raw', rate=8)
    servo(model, 'camera_pan', 'base_link', 'camera_pan_link', '0 0 1', spec.PAN_LIMIT)
    servo(model, 'camera_tilt', 'camera_pan_link', 'camera_tilt_link', '0 1 0', spec.TILT_LIMIT)
    # Custom wafer-module brackets and downward camera are not stock Raspbot parts.
    solid(body, 'custom_wand_mount', (.022, .032, .038), (-.075, 0, .043), color=SILVER, collision=False)
    solid(body, 'custom_wand_upright', (.012, .016, .220), (-.075, 0, .150), color=SILVER, collision=False)
    # Mount below the chassis and behind the IR board so neither stock part
    # occludes the floor image.  The optical centre remains 41 mm above ground.
    solid(body, 'custom_down_camera_bracket', (.020, .014, .006), (.070, 0, -.015), color=SILVER, collision=False)
    solid(body, 'custom_down_camera', (.012, .016, .008), (.070, 0, -.019), color=BLACK, collision=False)
    camera(body, 'down_camera', (.070, 0, -.025), (0, math.pi/2, 0), 320, 240, 1.8, '/down_camera/image_raw')
    plugin(model, 'mecanum-drive', 'MecanumDrive',
           front_left_joint='front_left_wheel_joint', front_right_joint='front_right_wheel_joint',
           back_left_joint='rear_left_wheel_joint', back_right_joint='rear_right_wheel_joint',
           wheel_separation=spec.TRACK, wheelbase=spec.WHEELBASE, wheel_radius=spec.WHEEL_RADIUS,
           topic='/cmd_vel', odom_topic='/odom', tf_topic='/robot/tf', frame_id='odom', child_frame_id='base_link',
           odom_publish_frequency=30, min_velocity=-1.5, max_velocity=1.5,
           min_acceleration=-1.5, max_acceleration=1.5)
    save_model('raspbot_v2', root)
    return root


if __name__ == '__main__':
    generate()
