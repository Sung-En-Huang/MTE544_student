# Plot the sensor data logged for Lab 1.
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from utilities import FileReader


def plot_time(ax, values, columns, labels, ylabel):
    # Split recordings at backward clock jumps. Each segment starts at zero.
    breaks = np.flatnonzero(np.diff(values[:, -1]) < 0) + 1
    for number, segment in enumerate(np.split(values, breaks), 1):
        # convert nanoseconds to elapsed seconds within this recording segment
        time = (segment[:, -1] - segment[0, -1]) / 1e9
        for column, label in zip(columns, labels):
            suffix = f' (segment {number})' if len(breaks) else ''
            style = '-' if column == columns[0] else '--'
            ax.plot(time, segment[:, column], style, label=label + suffix)
    ax.set(xlabel='Time since segment start (s)', ylabel=ylabel)


def plot_errors(filename, output_dir, show=True):
    headers, values = FileReader(filename).read_file()
    values = np.array(values)
    sensor, motion = Path(filename).stem.split('_content_', 1)

    if sensor == 'imu':
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        plot_time(axes[0], values, [0], ['acc_x'], 'X acceleration (m/s²)')
        plot_time(axes[1], values, [1], ['acc_y'], 'Y acceleration (m/s²)')
        plot_time(axes[2], values, [2], ['angular_z'], 'Angular velocity (rad/s)')

    elif sensor == 'odom':
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        breaks = np.flatnonzero(np.diff(values[:, -1]) < 0) + 1
        for number, segment in enumerate(np.split(values, breaks), 1):
            axes[0].plot(segment[:, 0], segment[:, 1], label=f'Path segment {number}')
        axes[0].set(xlabel='x (m)', ylabel='y (m)', aspect='equal')
        plot_time(axes[1], values, [0, 1], ['x', 'y'], 'Position (m)')
        # motions.py already converts quaternion orientation to yaw (th).
        # unwrap +/-pi yaw jumps separately within each recording segment
        yaw_values = np.concatenate([
            np.column_stack((segment[:, :2], np.unwrap(segment[:, 2]), segment[:, -1]))
            for segment in np.split(values, breaks)
        ])
        plot_time(axes[2], yaw_values, [2], ['Yaw'], 'Unwrapped yaw (rad)')

    elif sensor == 'laser':
        fig, ax = plt.subplots(figsize=(6, 5))
        axes = [ax]
        # Plot only one scan: the middle row of the recorded ranges.
        row = values[len(values) // 2]
        ranges = row[[i for i, name in enumerate(headers) if name.startswith('range_')]]
        # reconstruct beam angles from the recorded scan metadata
        angle_min = row[headers.index('angle_min')]
        angles = angle_min + np.arange(len(ranges)) * row[headers.index('angle_increment')]
        # remove invalid ranges while preserving their original beam angles
        valid = np.isfinite(ranges) & (ranges > 0)
        # convert polar measurements to Cartesian points in the sensor frame
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        ax.scatter(x, y, s=8, label='Laser returns')
        ax.scatter(0, 0, marker='^', color='red', label='Sensor origin')
        ax.set(xlabel='Sensor-frame x (m)', ylabel='Sensor-frame y (m)', aspect='equal')

    for ax in axes:
        ax.legend()
        ax.grid(True)
    fig.suptitle(f'{motion.title()} motion — {sensor.upper()}')
    fig.tight_layout()
    fig.savefig(output_dir / f'{Path(filename).stem}.png', dpi=200)
    if show:
        plt.show()
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot Lab 1 sensor CSV files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    parser.add_argument('--output-dir', type=Path, default=Path('plots/filePlotter'))
    parser.add_argument('--save-only', action='store_true', help='Save plots without opening windows')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    print('Plotting the files', args.files)
    for filename in args.files:
        plot_errors(filename, args.output_dir, show=not args.save_only)
