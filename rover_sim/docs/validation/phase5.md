# Phase 5 LiDAR validation

The modular Gazebo Classic LiDAR and optional local RViz configuration are
implemented. The full sensor integration run and matched sensor-disabled
performance baseline **pass**, including their strict watchdog-epoch guards.
The existing motion, keyboard, hardening and shutdown regression suites also
pass with the sensor-enabled implementation. Actual RViz GUI inspection passed:
RobotModel, LaserScan and Global Status all reported OK.

## Implemented interface

| Item | Configuration |
| --- | --- |
| Mount | Fixed `lidar_joint`, `base_link -> lidar_link` |
| Mount XYZ / RPY | `(0, 0, 0.20)` m / `(0, 0, 0)` rad |
| Sensor and plugin | Classic CPU `ray`, `libgazebo_ros_ray_sensor.so` |
| Output | `/scan`, `sensor_msgs/msg/LaserScan`, frame `lidar_link` |
| Cadence | 10 Hz in simulation time |
| Scan | 720 samples, −π to +π inclusive, nominal 0.500695° increment |
| Range | 0.12–30 m; 0.01 m simulator range-resolution setting |
| Noise | Independent Gaussian finite-hit noise, mean 0 m, sigma 0.01 m |
| RViz | Fixed Frame `base_link`, simulation time, RobotModel and current LaserScan |

The description is local to `urdf/sensors/lidar.urdf.xacro`. Standard fixed-joint
reduction transfers the sensor into the Gazebo physical root link `base_root`
at pose `(0, 0, 0.20, 0, 0, 0)`. This matches the original URDF because
`base_root -> base_link` is identity. The dedicated URDF `lidar_link` remains;
robot_state_publisher owns its fixed transform. The sensor plugin publishes
no TF. The 0.15 kg housing has a positive cylinder inertia.

The plugin publishes reliable/volatile/depth-five data; RViz uses compatible
best-effort/volatile reception. Its XML output spelling is
`sensor_msgs/LaserScan`, with `~/out:=/scan` remapping. Message stamps are
simulation time. Classic publishes `scan_time=0` and `time_increment=0`;
each scan is an instantaneous snapshot, without rolling-scan distortion.
No-return beams are positive infinity. Inclusive endpoints duplicate the
rear direction; the closest forward beams are approximately ±0.25035°.

## Tests and measured geometry

All **131 Python unit tests pass**: the previous 117 tests plus 14 new LiDAR
description/RViz tests. The colcon test summary reports **135 passing tests**
when its four CTest wrappers are included. These are unit/configuration tests; the Gazebo and process checks below
provide separate integration evidence.

The current [sensor evidence](phase5_lidar.json) records:

- `/scan` has the required type, frame and 720 samples. The sampled message
  had 222 finite in-range hits, 498 no returns and a simulation age of 0.054 s.
- `base_link -> lidar_link` is `(0, 0, 0.20)` m with identity rotation.
  robot_state_publisher was the only `/tf_static` publisher; no dynamic
  `lidar_link` transform competed with it. Lookup at the scan stamp succeeded.
- All three hills agreed with full-pose analytical ray/sphere intersections.
  Near their center bearings, expected ranges were approximately 8.11 m west,
  9.18 m north and 6.94 m east. The largest selected-beam error was 0.0189 m.
- Temporary boxes placed front and left had nominal nearest faces 5.00 m and
  4.00 m from the spawn origin. The comparison used the measured rover pose
  and actual beam angles, rather than assuming an exact forward beam. The
  largest initial box error was 0.0289 m, below the 0.06 m tolerance. The
  near-forward ray measured 5.0002 m against 5.0000 m expected; a near-left
  ray measured 3.9967 m against 4.0000 m expected.
- Forward motion displaced the body 0.4702 m, with lateral displacement
  −0.0000084 m and yaw change 0.0000041 rad. Mean scan change was 0.8196 m;
  expected front returns shortened to approximately 4.530 m.
- Yaw motion rotated 1.0766 rad with center translation 0.000284 m. Scan
  bearings changed, and measured box returns still matched analytical ranges.
- Temporary validation boxes were removed successfully.

All 12 checks in the full run passed. Its watchdog epoch remained unchanged
through performance measurement, interface/TF queries, hill checks, probe
insertion, forward/yaw motion and cleanup. The controller and actuator
watchdog timeouts were not increased.

## Performance and unresolved host-load observation

Measurements are headless Gazebo physics/sensor measurements, excluding RViz
and Gazebo GUI rendering. The requested sensor work is 7,200 rays per
simulation second. The final [baseline](phase5_baseline.json) and
[sensor run](phase5_lidar.json) each contain three 10-second wall-time windows:

| Metric | LiDAR disabled | LiDAR enabled |
| --- | --- | --- |
| Real-time factors | 0.784748, 0.810042, 0.799550 | 0.800061, 0.758579, 0.805482 |
| Median real-time factor | **0.799550** | **0.800061** |
| Mean real-time factor | 0.798113 | 0.788041 |
| Largest actuator heartbeat gap | 0.0614 s | 0.0622 s |
| Watchdog epoch transitions | None | None |
| Scan rate, simulation time | — | **9.9636–9.9874 Hz** |
| Scan rate, wall time | — | **7.5520–8.0283 Hz** |

The medians are effectively unchanged within run-to-run variation. The
sensor-enabled mean is approximately 1.3% lower; three short windows on a
variable-load host do not isolate an exact sensor CPU cost. The strict
watchdog guards passed both complete final runs, and the 0.3 s actuator
watchdog was not relaxed.

Earlier attempts remain relevant warnings and are retained alongside the
final evidence. One matched sensor-disabled attempt failed: real-time factor fell to
**0.559475**, maximum observed actuator heartbeat gap was **0.3303 s**, and
the actuator watchdog epoch advanced twice. Thus watchdog interruptions have
occurred with LiDAR disabled as well as during earlier sensor-enabled runs.
An earlier concurrent ROS CLI startup
coincided with a larger heartbeat interruption; that timing alone does not
establish a DDS or CLI root cause.

Host load was observed at 5.37 on four CPU cores; `snapd` later consumed about
59% CPU. These observations show variable background contention, but they do
not conclusively explain every interruption. The watchdog correctly inhibits
drive on a wall-time command gap. Its settings have not been increased to
hide failed validation. The final uninterrupted matched measurements passed;
general host-load resilience beyond those conditions is not established.

## Regression and GUI checks

| Check | Status |
| --- | --- |
| Normal workspace build and all unit tests | Passed |
| Sensor interface, fixed TF, hill/probe ranges, scan changes | Passed |
| Complete sensor run with unchanged watchdog epoch | Passed |
| W/S, A/D, direct prohibited lateral commands | 10 motion and 10 actual-terminal keyboard checks passed |
| Reset, pause/resume, slow simulation and watchdog recovery | All 10 hardening checks passed at the default 0.3 s watchdog |
| Repeated SIGINT, final zero and resource cleanup | Both repeated-signal process checks passed; final drive zero, exit 0, one destruction |
| RViz RobotModel and LaserScan visually displayed without TF errors | Passed; Global Status, RobotModel and LaserScan all OK |
| Matched LiDAR-off/on performance comparison | Passed; variation and prior load-related interruptions documented |
| Gazebo/RViz warning audit | Completed; details below |

An installed Xacro warning about private attributes and future Python 3.11
behavior was observed during unit tests; Ubuntu 22.04 uses Python 3.10 and
the tests passed. robot_state_publisher's backwards-time warnings during
intentional simulator time resets are expected. The hardening suite deliberately
triggers the actuator watchdog; additional earlier host-load interruptions are
documented above. No LiDAR plugin or TF errors were observed in the final runs.

The [motion evidence](phase5_motion.json) contains 10 passing cases, including
forward/backward, both yaw directions, combined forward/yaw and prohibited
lateral-only/mixed commands. Lateral-only cases moved less than 0.000008 m
and had zero wheel targets. The [keyboard evidence](phase5_teleop.json)
contains 10 passing actual-terminal checks: W/S, A/D, disabled Q/E, Space,
release timeout, Ctrl+C, and zero lateral velocity in all 1,484 messages.
Space published zero in 0.0042 s, release timeout in 0.6541 s and Ctrl+C in
0.0073 s. All wheels settled, the terminal was restored, and teleop exited 0.

All 10 [hardening checks](phase5_hardening.json) passed: atomic parameter
validation, pause/resume, all three reset services, slow simulation at measured
RTF 0.01998, an independent actuator-watchdog stop, rejection of stale queued
commands on recovery and acceptance of a fresh command. The watchdog retained
its default 0.3 wall-second limit. Both [shutdown checks](phase5_shutdown.json)
passed with repeated SIGINT/SIGTERM at the destruction boundary: final drive
zero when initialized, no fabricated targets before feedback, exit 0 and one
node destruction.

## Actual RViz inspection

The optional `rviz:=true` launch was exercised with the installed configuration.
A direct `/rviz2/get_parameters` request confirmed `use_sim_time=true` after
DDS discovery. The [actual RViz screenshot](phase5_rviz.png) shows the rover,
three hill-return arcs, `base_link` Fixed Frame, zero scan decay, and all three
status indicators OK. A separate live yaw check changed return bearings around
the fixed local rover; common finite beam ranges changed by 1.3345 m on average.
The earlier sensor test also established scan changes after forward translation.

The desktop was locked during validation. A temporary nested Xephyr display
allowed inspection and capture of the actual RViz application without unlocking
the desktop. It was stopped afterward. RViz and Gazebo GUI rendering are excluded
from the matched sensor performance measurements above.

The first RViz attempt inherited Snap VS Code GTK paths and failed with a
`/snap/core20` `libpthread` symbol error. Clearing `GTK_PATH`, `GTK_EXE_PREFIX`,
`GIO_MODULE_DIR` and `GTK_IM_MODULE_FILE` for that process resolved it. The
normal-terminal/one-command workaround is documented in [the LiDAR guide](../lidar.md).
No system or shell-startup changes were made. RViz also logged harmless
Wayland/X11 and unsupported-stereo notices; rendering succeeded with OpenGL 4.5.

## Files and limitations

Modified source files: `urdf/proxy_rover.urdf.xacro`,
`launch/rover_desert.launch.py`, `package.xml`, `CMakeLists.txt`, `README.md`.
Added source files: `urdf/sensors/lidar.urdf.xacro`, `rviz/rover_lidar.rviz`,
`scripts/validate_lidar.py`, `test/test_lidar.py`, `docs/lidar.md`, and this
report. Validation artifacts are `phase5_baseline.json`, `phase5_lidar.json`
and `phase5_motion.json`, `phase5_teleop.json`,
`phase5_hardening.json`, `phase5_shutdown.json`, `phase5_rviz.png`.
Build/install/log directories are generated through the normal workspace
workflow, never edited directly.

RViz remains robot-local: fixed frame `base_link`, zero scan decay, no
world-stable accumulation. No odometry, SLAM, Nav2 or cameras were added.
The scan plane follows body pitch and roll; level flat ground below it is
invisible, while hill surfaces crossing it produce returns. This primitive
CPU-ray sensor does not model dust, realistic reflectivity or a spinning
scanner. Swerve kinematics, W/S forward/backward, A/D yaw and the prohibited
`linear.y` input path are unchanged.

The normal `~/rover_ws` build and test workflow passed. LiDAR sensing, geometry,
fixed TF, local RViz and the existing motion/safety behavior are validated.
All owned simulation, RViz and nested-display processes were stopped afterward.
The documented variable-host-load watchdog limitation remains; no controller
timing was changed to conceal it. Local visualization is ready for Phase 5 use,
with world-stable visualization deliberately deferred until odometry exists.
