Here are the saved debug records, which can be plotted using ./Log/plot.py.

## Pose, velocity and IMU bias results

Set `evo/pose_output_en: true` in the loaded YAML configuration. Set
`evo/seq_name` to the run name. `launch/mapping_mid360.launch` loads
`config/mid360_calibrated.yaml`, which enables this output by default.
Point cloud and image saving are independent of this setting.

Files are written under the source repository's `Log/result/` directory
(`ROOT_DIR` is set at build time). The directory is created if needed.
Each node start overwrites files with the same sequence name:

- `<seq_name>.txt`: TUM trajectory, eight columns, no header:
  `timestamp tx ty tz qx qy qz qw`.
- `<seq_name>_state.txt`: full state, 17 columns, with `#` comment headers:
  `timestamp tx ty tz qx qy qz qw vx vy vz bgx bgy bgz bax bay baz`.

The timestamp is the sensor update time in seconds, not the wall-clock write
time or time since startup. Position is in meters and velocity in m/s, both in
the estimator's world frame. The quaternion rotates IMU/body coordinates into
world coordinates. Gyro bias is in rad/s and accel bias in m/s^2, both in IMU
coordinates. Numeric values use nine decimal places. Each row is flushed to
the file as it is written.

Both files record the same state after each successful LIO/LO update, before
the following VIO update. There is no fixed-rate logging timer or decimation:

- LiDAR/IMU mode: one row per processed LiDAR scan, at the scan end time.
- LiDAR/IMU/camera mode: one row per successful LIO update at an accepted image
  capture time (including the configured time offsets/exposure offset).
  The rate therefore follows accepted image frames, e.g. approximately 30 Hz
  with a 30 Hz camera when synchronization and processing keep up.

Initialization, empty point clouds and skipped images can leave gaps. This is
not IMU-rate output and does not separately log the post-VIO state. The main
loop's `ros::Rate(5000)` does not set the result saving rate.

For example, read the state file with NumPy:

```python
import numpy as np

state = np.loadtxt("Log/result/CBD_Building_01_state.txt", ndmin=2)
timestamp = state[:, 0]
position = state[:, 1:4]
quaternion_xyzw = state[:, 4:8]
velocity = state[:, 8:11]
gyro_bias = state[:, 11:14]
accel_bias = state[:, 14:17]

if len(timestamp) > 1:
    print("Average saving rate (Hz):", (len(timestamp) - 1) / (timestamp[-1] - timestamp[0]))
```
