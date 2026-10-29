#!/usr/bin/env python3
"""Model-based Bode and Nyquist plots for the camera line follower.

These are linearized estimates, not a frequency response measured from a
tracking run. The camera geometry comes from the SDF; the dynamics assume a
flat floor, constant forward speed, and immediate yaw-rate response.
"""

import argparse
import csv
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / 'models' / 'raspbot_v2' / 'model.sdf'
DEFAULTS = dict(kp=0.04, ki=0.0, kd=0.002, speed=0.02, steering_sign=1.0)
WHEEL_SEPARATION = 0.13583
SCAN_FRACTION = 0.75
UPDATE_PERIOD = 0.05
DERIVATIVE_TAU = -UPDATE_PERIOD / math.log(0.8)


def camera_geometry():
    """Approximate camera lookahead and normalized pixels per lateral metre."""
    model = ET.parse(MODEL).getroot()
    base = model.find(".//link[@name='base_link']")
    sensor = base.find("sensor[@name='usb_camera']")
    base_z = float(base.findtext('pose').split()[2])
    sensor_x, _, sensor_z, _, pitch, _ = map(float, sensor.findtext('pose').split())
    camera = sensor.find('camera')
    horizontal_fov = float(camera.findtext('horizontal_fov'))
    width = int(camera.findtext('image/width'))
    height = int(camera.findtext('image/height'))
    vertical_fov = 2 * math.atan(math.tan(horizontal_fov / 2) * height / width)
    down_angle = pitch + math.atan((2 * SCAN_FRACTION - 1) * math.tan(vertical_fov / 2))
    camera_height = base_z + sensor_z
    lookahead = sensor_x + camera_height / math.tan(down_angle)
    slant_distance = camera_height / math.sin(down_angle)
    half_image_width = slant_distance * math.tan(horizontal_fov / 2)
    return lookahead, 1 / half_image_width


def settings_from_csv(path):
    with Path(path).expanduser().open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'No telemetry rows in {path}')
    first = rows[0]
    settings = {name: float(first[name]) for name in ('kp', 'ki', 'kd', 'steering_sign')}
    tracking = next((row for row in rows if row['status'] == 'tracking'), None)
    settings['speed'] = float(tracking['speed_cmd_mps']) if tracking else DEFAULTS['speed']
    return settings


def loop_response(frequency_hz, kp, ki, kd, speed, steering_sign,
                  lookahead, camera_scale):
    """Negative-feedback loop L(s) = -controller(s) * camera_plant(s)."""
    s = 2j * np.pi * frequency_hz
    pid = kp + ki / s + kd * s / (1 + DERIVATIVE_TAU * s)
    # yaw rate -> lateral offset: heading integrates once, position twice.
    camera_plant = camera_scale * (lookahead * s + speed) / (s * s)
    return steering_sign * (2 / WHEEL_SEPARATION) * pid * camera_plant


def make_plots(output_dir, settings, lookahead, camera_scale):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    frequency = np.logspace(math.log10(0.002), math.log10(2.0), 1200)
    loop = loop_response(frequency, lookahead=lookahead,
                         camera_scale=camera_scale, **settings)
    magnitude = np.abs(loop)
    phase = np.rad2deg(np.unwrap(np.angle(loop)))
    if phase[0] > 0:
        phase -= 360

    output_dir = Path(output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    bode = output_dir / 'raspbot_bode_model.png'
    nyquist = output_dir / 'raspbot_nyquist_model.png'
    description = (f"Kp={settings['kp']:g}, Ki={settings['ki']:g}, Kd={settings['kd']:g}, "
                   f"speed={settings['speed']:g} m/s, sign={settings['steering_sign']:g}")
    assumptions = (f'camera lookahead≈{lookahead:.3f} m, image scale≈{camera_scale:.2f}/m; '
                   'ignores delay, saturation, and tape loss')

    figure, (mag_axis, phase_axis) = plt.subplots(2, 1, figsize=(10, 7),
                                                  sharex=True, constrained_layout=True)
    figure.suptitle('Bode: linearized camera/robot loop (model estimate)\n'
                     + description + '\n' + assumptions)
    mag_axis.semilogx(frequency, 20 * np.log10(magnitude))
    mag_axis.axhline(0, color='black', linestyle='--', linewidth=0.8)
    mag_axis.set_ylabel('loop magnitude (dB)')
    phase_axis.semilogx(frequency, phase)
    phase_axis.axhline(-180, color='black', linestyle='--', linewidth=0.8)
    phase_axis.set(xlabel='frequency (Hz)', ylabel='loop phase (degrees)')
    for axis in (mag_axis, phase_axis):
        axis.grid(True, which='both', alpha=0.3)
    figure.savefig(bode, dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(7.5, 7.5), constrained_layout=True)
    figure.suptitle('Nyquist: linearized camera/robot loop (model estimate)\n'
                     + description + '\n' + assumptions)
    axis.plot(loop.real, loop.imag, label='positive frequencies')
    axis.plot(loop.real, -loop.imag, '--', alpha=0.65, label='negative frequencies')
    axis.plot(-1, 0, 'rx', markersize=12, markeredgewidth=2, label='critical point (-1, 0)')
    axis.axhline(0, color='black', linewidth=0.6)
    axis.axvline(0, color='black', linewidth=0.6)
    axis.set(xlim=(-3, 3), ylim=(-3, 3), xlabel='real L(jω)', ylabel='imaginary L(jω)')
    axis.set_aspect('equal')
    axis.grid(True, alpha=0.3)
    axis.legend(loc='upper right')
    figure.savefig(nyquist, dpi=160)
    plt.close(figure)

    crossings = np.where((magnitude[:-1] >= 1) & (magnitude[1:] < 1))[0]
    if len(crossings):
        index = crossings[0]
        print(f'Model gain crossover: {frequency[index]:.3f} Hz; '
              f'phase margin approximately {180 + phase[index]:.1f} degrees')
    else:
        print('No model gain crossover within 0.002–2 Hz')
    print(f'Assumed camera lookahead: {lookahead:.3f} m; scale: {camera_scale:.2f} /m')
    print('Model estimate only: a tracking CSV does not measure open-loop frequency response.')
    print(f'Bode: {bode}\nNyquist: {nyquist}')
    return bode, nyquist


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--csv', help='Read gains and speed from a PID telemetry CSV')
    parser.add_argument('--output-dir', default='~')
    for name in DEFAULTS:
        parser.add_argument('--' + name.replace('_', '-'), type=float)
    parser.add_argument('--lookahead', type=float)
    parser.add_argument('--camera-scale', type=float)
    args = parser.parse_args()
    settings = settings_from_csv(args.csv) if args.csv else DEFAULTS.copy()
    for name in DEFAULTS:
        value = getattr(args, name)
        if value is not None:
            settings[name] = value
    lookahead, camera_scale = camera_geometry()
    make_plots(args.output_dir, settings, args.lookahead or lookahead,
               args.camera_scale or camera_scale)


if __name__ == '__main__':
    main()
