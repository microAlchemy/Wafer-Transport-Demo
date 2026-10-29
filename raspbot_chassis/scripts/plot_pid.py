#!/usr/bin/env python3
"""Plot one camera PID run from the controller's CSV telemetry."""

import argparse
import csv
import math
from pathlib import Path


def plot(input_path, output_path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    with Path(input_path).expanduser().open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f'No telemetry rows in {input_path}')

    def values(name):
        return [float(row[name]) for row in rows]

    times = values('time_s')
    error = values('error')
    p, i, d = (values(name) for name in ('p', 'i', 'd'))
    correction = values('correction_mps')
    yaw_cmd = values('yaw_cmd_rad_s')
    yaw_odom = values('yaw_odom_rad_s')
    x_odom, y_odom = values('x_odom_m'), values('y_odom_m')
    lost_time = [float(row['time_s']) for row in rows if row['status'] == 'lost']

    figure, axes = plt.subplots(4, 1, figsize=(11, 13), constrained_layout=True)
    first = rows[0]
    figure.suptitle('Raspbot camera PID: error, control, response, route\n'
                     f"Kp={first['kp']}  Ki={first['ki']}  Kd={first['kd']}  "
                     f"steering sign={first['steering_sign']}")

    axes[0].plot(times, error, color='tab:blue', label='camera offset')
    axes[0].scatter(lost_time, [0] * len(lost_time), marker='x', s=10,
                    color='tab:red', label='line lost')
    axes[0].axhline(0, color='black', linewidth=0.8)
    axes[0].set(ylabel='normalized offset', ylim=(-1.1, 1.1))
    axes[0].legend(loc='upper right')

    for values_, label in [(p, 'P'), (i, 'I'), (d, 'D'), (correction, 'applied correction')]:
        axes[1].plot(times, values_, label=label)
    axes[1].axhline(0, color='black', linewidth=0.8)
    axes[1].set(ylabel='wheel-side correction (m/s)')
    axes[1].legend(loc='upper right')

    axes[2].plot(times, yaw_cmd, label='commanded yaw')
    axes[2].plot(times, yaw_odom, label='measured yaw (odom)', alpha=0.8)
    axes[2].axhline(0, color='black', linewidth=0.8)
    axes[2].set(ylabel='yaw rate (rad/s)')
    axes[2].legend(loc='upper right')

    first_odom = next(((x, y) for x, y in zip(x_odom, y_odom)
                       if math.isfinite(x) and math.isfinite(y)), None)
    if first_odom:
        axes[3].plot([x - first_odom[0] for x in x_odom],
                     [y - first_odom[1] for y in y_odom], label='robot odometry')
    from generate_path import path_segments
    # The world starts the robot at (-1.55, 0); show tape in that local frame.
    segments = path_segments()
    points = [segments[0][0]] + [end for _, end in segments]
    axes[3].plot([x + 1.55 for x, _ in points], [y for _, y in points],
                 '--', color='black', label='tape center')
    axes[3].axis('equal')
    axes[3].set(xlabel='distance from start (m)', ylabel='lateral position (m)')
    axes[3].legend(loc='upper left')

    for axis in axes[:3]:
        axis.set_xlabel('time (s)')
        axis.grid(True, alpha=0.25)
    axes[3].grid(True, alpha=0.25)
    output = Path(output_path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=150)
    plt.close(figure)
    valid = [abs(value) for value in error if math.isfinite(value)]
    rms = math.sqrt(sum(value * value for value in valid) / len(valid)) if valid else math.nan
    print(f'Plot: {output}')
    print(f'Frames: {len(rows)}, line-lost samples: {len(lost_time)}, RMS offset: {rms:.3f}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', default='~/raspbot_pid.csv')
    parser.add_argument('--output', default='~/raspbot_pid.png')
    args = parser.parse_args()
    plot(args.input, args.output)


if __name__ == '__main__':
    main()
