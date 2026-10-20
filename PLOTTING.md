# Lab 1 plots

`filePlotter.py` uses the provided `FileReader` and `--files` interface.
Install NumPy and Matplotlib if needed: `python -m pip install numpy matplotlib`.

```powershell
python filePlotter.py --files imu_content_circle.csv odom_content_circle.csv laser_content_circle.csv
```

Repeat for line and spiral, or pass all nine CSV filenames. Figures are shown
and saved as PNG/PDF in `plots/filePlotter/`. Use `--save-only` to avoid opening
windows, and `--output-dir` to select another folder.

IMU acceleration and angular velocity have separate axes. Odometry includes
trajectory, position, and unwrapped yaw; quaternion conversion already occurs
in motions.py. Backward timestamp jumps split recordings into independent
segments with time in seconds from each segment start.

Laser plotting uses the middle scan row, excludes NaN/Inf/nonpositive ranges,
and converts polar ranges to Cartesian coordinates. If angle_min is missing,
zero radians is assumed and labeled on the plot.

The previously generated discrepancies.md is retained as a separate note;
the plotting script does not perform audits or generate discrepancy reports.
