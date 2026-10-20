# Plot the sensor data logged for Lab 1.
import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from utilities import FileReader


def plot_time(ax, values, columns, labels, ylabel):
    # Split recordings at backward clock jumps. Each segment starts at zero.
    breaks = np.flatnonzero(np.diff(values[:, -1]) < 0) + 1
    for number, segment in enumerate(np.split(values, breaks), 1):
        time = (segment[:, -1] - segment[0, -1]) / 1e9
        for column, label in zip(columns, labels):
            suffix = f' (segment {number})' if len(breaks) else ''
            style = '-' if column == columns[0] else '--'
            ax.plot(time, segment[:, column], style, label=label + suffix)
    ax.set(xlabel='Time since segment start (s)', ylabel=ylabel)


def plot_errors(filename, output_dir, show=True, pdf=None):
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
        angle_min = row[headers.index('angle_min')] if 'angle_min' in headers else 0.0
        angles = angle_min + np.arange(len(ranges)) * row[headers.index('angle_increment')]
        valid = np.isfinite(ranges) & (ranges > 0)
        x = ranges[valid] * np.cos(angles[valid])
        y = ranges[valid] * np.sin(angles[valid])
        ax.scatter(x, y, s=8, label='Laser returns')
        ax.scatter(0, 0, marker='^', color='red', label='Sensor origin')
        ax.set(xlabel='Sensor-frame x (m)', ylabel='Sensor-frame y (m)', aspect='equal')
        if 'angle_min' not in headers:
            ax.set_title('angle_min missing: assumed 0 rad')

    else:
        raise ValueError(f'Unknown sensor: {sensor}')

    for ax in axes:
        ax.legend()
        ax.grid(True)
    fig.suptitle(f'{motion.title()} motion — {sensor.upper()}')
    fig.tight_layout()
    for extension in ('png', 'pdf'):
        fig.savefig(output_dir / f'{Path(filename).stem}.{extension}', dpi=200)
    if pdf is not None:
        pdf.savefig(fig)
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
    with PdfPages(args.output_dir / 'all_sensor_plots.pdf') as pdf:
        for filename in args.files:
            plot_errors(filename, args.output_dir, show=not args.save_only, pdf=pdf)
