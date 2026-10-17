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

Headless flat world, rover at rest, two 10 s windows per profile, 20 sweeps
analysed (`validate_lidar3d.py`, `docs/validation/mars_lidar.json`). The
baseline without any LiDAR is RTF 0.998 / 0.56 cores (Phase A).

| Profile | channels × samples | points per sweep (flat world) | median RTF | gzserver cores | post-processing cores | RTF ≥ 0.9 |
| --- | --- | --- | --- | --- | --- | --- |
| `lite` (CPU `ray`) | 16 × 450 | 3600 | 0.946 | 0.66 | 0.43 | yes |
| `standard` (CPU `ray`) | 16 × 900 | 7200 | 0.900 | 0.68 | 0.44 | yes |
| `high` (CPU `ray`) | 32 × 1024 | 16114 | 0.723 | 0.80 | 0.42 | no |
| `lite` (`gpu_ray`, Intel iGPU, headless with DISPLAY) | 16 × 450 | 3150 | 0.998 | 0.83 | 0.42 | yes, but ranges wrong (see below) |

Only `lite` keeps the real-time factor above 0.9 on this machine, so it is
the default. `standard` sits right at the budget (0.886–0.900 across runs)
and `high` costs a quarter of real time. The CPU ray sensor does not run in
a separate physics-free thread in practice: every doubling of rays shows up
in RTF even though the gzserver process uses well under one core.

`gpu_ray` on the Intel iGPU does work headless (the X display is reachable)
and costs no RTF at all (0.998), **but its geometry is wrong**: the ground
returns have a 1.27 m range bias with 0.57 m scatter, only 7 of 16 channels
return, and 3150 instead of 3600 points arrive. This is the known Gazebo
Classic GPU ray limitation (cube-map projection and vertical interpolation
with few channels), so the launch keeps the CPU sensor unless explicitly
asked for `lidar_gpu:=true`; `auto` never chooses it on its own
(`lidar_prefer_gpu:=true` is required in addition to a display and GUI).

Geometry and noise (CPU profiles): the lowest beam is at −15° (−22.5° for
`high`) with exactly the configured 2° (1.45°) spacing; ground ranges match
the analytic plane within 2 mm per channel after removing the 0.05° body
tilt, with a per-channel residual σ of **2.0 cm** (the configured 2 cm);
the 1 m test box's near face is measured within 3 mm. Self-hits on the
rover's own links: **0 per sweep** in every profile. The processed cloud
carries `ring` and `time` and adds the range-dependent noise and dropout.

The RViz configuration was checked for the expected displays by the unit
tests; an interactive GUI check was not possible in this headless session.

