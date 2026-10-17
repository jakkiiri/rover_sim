# 3-D LiDAR, IMU and RViz

## Sensor

A VLP-16-class spinning LiDAR sits on the instrument mast at the front-centre
of the deck (`urdf/sensors/lidar3d.urdf.xacro`, mount height `mast_height`,
default 0.60 m above the deck, 1.18 m above ground at rest). It is a Gazebo
Classic multi-layer `ray` sensor (or `gpu_ray`) driven by
`libgazebo_ros_ray_sensor.so` publishing `sensor_msgs/PointCloud2` on
`/points_raw` at 10 Hz, 0.5–100 m, Gaussian range noise σ = 2 cm.
`velodyne_gazebo_plugins` is not installed on this machine and cannot be
installed without sudo, so the plain ray sensor plus a post-processing node
is used.

| Profile (`lidar_profile:=`) | Channels | Samples/rev | Vertical FOV | Rays/s |
| --- | --- | --- | --- | --- |
| `lite` (default) | 16 | 450 | ±15° | 72 000 |
| `standard` | 16 | 900 | ±15° | 144 000 |
| `high` | 32 | 1024 | ±22.5° | 327 680 |

`lidar_gpu:=auto|true|false` selects `gpu_ray`; `auto` picks the GPU sensor
only when a display is available, the Gazebo client is open and
`lidar_prefer_gpu:=true`, because the CPU sensor is the safe default on this
Intel-iGPU machine (measured costs below).

The `-15°` beam reaches the ground at `d = h / tan 15° = 1.18 / 0.268 ≈
4.4 m`; the mast height keeps the deck, e-box and wheels out of the field of
view, verified by the self-hit count in the validator.

## Post-processing (`lidar_postprocess`, numpy)

Subscribes `/points_raw`, publishes `/points` with fields
`x y z intensity ring(uint16) time(float32)`, adds range-dependent noise
`σ = 1 cm + 0.3 mm/m · r`, dropout `p = 2 % + 0.2 %/m · r`, an intensity
model when the sensor gives none, and optionally a planar `/scan`
(`scan_2d:=true`) from the ring nearest 0° when `pointcloud_to_laserscan`
is not installed (when it is, that package is launched instead). Each cloud
costs a few numpy operations; measured CPU is in the table below. The raw
topic stays available for comparison.

Not modelled: dust attenuation, multipath, rolling-shutter spin distortion
(per-point `time` is derived from azimuth; points are not displaced by the
rover's motion during the sweep), beam divergence, reflectivity.

## IMU

`libgazebo_ros_imu_sensor.so` on `imu_link` (chassis centre) publishes
`/imu/data` at 100 Hz with Gaussian noise (gyro 0.002 rad/s, accel 0.02
m/s²) and biases (0.0005 rad/s, 0.01 m/s²). The controller uses it for the
tilt limits and the odometry yaw rate.

## RViz (`rviz/rover_mars.rviz`, `rviz:=true`)

Fixed frame `odom` (from the controller's wheel odometry TF). The `/points`
display uses **Boxes of 0.05 m**, AxisColor along Z and a **3 s decay** so
successive sweeps accumulate into a visible map as the rover moves; the raw
cloud, `/scan`, `/ground_truth/odom` and IMU displays are present but off.
TF, RobotModel and the `/odom` arrow trail are on. On a slow machine lower
`Decay Time` to 1 s and `Size (m)` to 0.03, or switch the style to
`Points`.

## Measured cost per profile

LIDAR_TABLE
