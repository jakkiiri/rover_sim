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

`lidar_gpu:=auto|true|false` selects `gpu_ray`. Two measured facts decide
the default (details and numbers under **Measured cost per profile**):

1. **The CPU `ray` sensor hangs gzserver on the heightmap world.** With
   `world:=mars` the simulation clock never advances once a ray sensor is
   present, even with a single 450-sample channel, low rock density or a
   30 m range; gzserver ignores SIGINT and has to be killed. This is the ODE
   ray-vs-heightfield collider, not the rock meshes. The CPU sensor is exact
   and cheap on the flat and test worlds.
2. **`gpu_ray` + `gazebo_ros_ray_sensor` labels every point one vertical
   beam step too low** (measured 0.99–1.00 steps on 16- and 32-channel
   configurations): the point labelled −15° carries the range of the −13°
   beam, so the raw ground looks 0.7–45 m too far. The GPU variant therefore
   requests one extra beam below the field of view and `lidar_postprocess`
   (`vertical_shift_steps: 1.0`) re-projects each point one step up and drops
   the top label, which restores the nominal −15°…+15° layout exactly. The
   raw `/points_raw` topic keeps the uncorrected labels for comparison.

`auto` therefore picks `gpu_ray` on `world:=mars` whenever a display is
available (gzserver needs an OpenGL context even headless; Xwayland's `:0`
works on this machine), disables the LiDAR with a warning on the Mars world
without a display, and uses the CPU sensor elsewhere unless
`lidar_prefer_gpu:=true`.

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
| `lite` (`gpu_ray`, corrected labels) | 16 × 450 | 3402 | 0.999 | 0.51 | 0.27 | yes |
| `standard` (`gpu_ray`, corrected labels) | 16 × 900 | 6812 | 0.999 | 0.48 | 0.32 | yes |
| `high` (`gpu_ray`, corrected labels) | 32 × 1024 | 15373 | 0.999 | 0.48 | 0.45 | yes |

Only `lite` of the CPU profiles keeps the real-time factor above 0.9;
`standard` sits at the budget (0.886–0.900 across runs) and `high` costs a
quarter of real time, because the CPU ray sensor's cost lands in the physics
loop even though gzserver uses well under one core. The GPU sensor costs no
RTF at any profile on this Intel iGPU.

On the **Mars heightmap world** (rover at rest, medium rocks) the GPU sensor
measured RTF 0.998 / 0.999 / 0.999 (lite / standard / high) with gzserver at
0.55 / 0.57 / 0.56 cores, against 0.999 / 0.32 without a LiDAR; while
driving the 111 m traverse the RTF stayed at 0.999
(`docs/validation/mars_traverse.md`). The CPU sensor never produced a
simulation clock there. The default profile stays `lite` (16 × 450) so the
CPU fallback on the flat and test worlds also holds the budget;
`standard`/`high` are free on the GPU.

GPU geometry after the one-step label correction (`/points`): lowest beam
−15° (−22.5° for `high`), spacing exactly as configured, ground range bias
+1.9 mm, per-channel σ 2.9 cm (post-processing adds its range-dependent
term to the sensor's own noise), test-box face within 0.5 mm, 0 self-hits per
sweep, 3402 points per sweep (one labelled channel is dropped by the correction and
the dropout model removes a few percent). The uncorrected raw topic showed a
0.7–45 m range bias growing towards the horizon.

Geometry and noise (CPU profiles): the lowest beam is at −15° (−22.5° for
`high`) with exactly the configured 2° (1.45°) spacing; ground ranges match
the analytic plane within 2 mm per channel after removing the 0.05° body
tilt, with a per-channel residual σ of **2.0 cm** (the configured 2 cm);
the 1 m test box's near face is measured within 3 mm. Self-hits on the
rover's own links: **0 per sweep** in every profile. The processed cloud
carries `ring` and `time` and adds the range-dependent noise and dropout.

The RViz configuration was checked for the expected displays by the unit
tests; an interactive GUI check was not possible in this headless session.

