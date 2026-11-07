# 3D LiDAR validation — 2026-11-07

Gazebo Classic 11.10.2, ROS 2 Humble, Ubuntu 22.04. Reference source:
`jakkiiri/rover_sim`, commit `4b503e9a0232867b749f1e1f6f7a8958ed70a3a8`.
See the [3D guide](../lidar3d.md) for complete reproduction commands and
[third-party notices](../../THIRD_PARTY_NOTICES.md#3d-lidar-adaptation-2026-11-07)
for local changes. Control, kinematics, teleop, actuator source and control
configuration hashes remain identical to the pre-change versions.

## Delivered configuration

CPU `ray`, `libgazebo_ros_ray_sensor.so`, lite profile: 16 × 450 samples,
360° × 30°, 10 Hz simulation time, 0.5–100 m, Gaussian sensor σ=0.02 m.
Fixed `lidar_link` XYZ `(0, 0, 0.40)` m above `base_link`, RPY zero, exclusively
owned by robot_state_publisher. Raw `/points_raw` and processed `/points` are
PointCloud2 with `lidar_link` frame. Processing adds ring/time, range noise
and dropout. There is no odometry, world TF, scan accumulation or IMU.

## Passing evidence

| Evidence | Passing checks |
| --- | --- |
| [CPU sensor](lidar3d_cpu.json) | 12: two performance windows, cloud interface, simulation timestamps, TF ownership, ground/all hills, known box, forward/yaw and updated box geometry, watchdog guard |
| [Motion](lidar3d_motion.json) | 10: forward/backward, both yaw directions, combined motion, zero and four blocked lateral cases |
| [Keyboard](lidar3d_teleop.json) | 10: W/S/A/D, unsupported keys, Space, release, Ctrl+C and zero lateral output |
| [Hardening](lidar3d_hardening.json) | 10: parameter atomicity, pause, three reset variants, slow simulation, watchdog, stale-command rejection and fresh recovery |
| [Shutdown](lidar3d_shutdown.json) | 2: repeated signals during motion and before feedback |
| [Sensor-disabled baseline](lidar3d_baseline.json) | 3: two performance windows and watchdog guard |

The initial 44 sensor/control checks and three baseline checks pass. A later
loaded repeat failed the watchdog guard as recorded below; this is not an
unqualified all-runs-pass result. Shutdown uses
an isolated synthetic-feedback controller process, while motion/keyboard/
hardening use the actual simulator with 3D LiDAR enabled. Unit tests are
separate: **157 passed**, comprising the original 131 plus 26 new 3D sensor,
point-cloud and repeated-signal post-processor tests. Normal real-workspace
`colcon build`, `colcon test` and `colcon test-result --verbose` pass; colcon
counts **162** results including its five CTest wrappers.

The box has front face X=5 m from the initial rover origin. Across 308 selected
returns its median absolute range error was **0.01393 m**, 95th percentile
**0.03435 m**. All three hills' median errors were **0.01324–0.01457 m**,
95th percentiles **0.03906–0.04664 m**. Ground median error **0.01384 m**,
95th percentile **0.03812 m**. No rover self-hits were counted.
These checks compare actual beam directions and measured rover pose to
analytical collision intersections, not guessed center distances.

Forward motion advanced **0.47095 m**, with lateral drift **−0.0000111 m**
and heading change **0.00000311 rad**. Rotation changed heading **1.03171 rad**
with center drift **0.000264 m**; the same static box's returns remained
correct in the moving sensor frame. These are flat-ground, low-speed checks.

## Performance, RViz and warnings

Two matched five-wall-second headless windows per configuration:

| Measurement | LiDAR off | CPU 3D lite |
| --- | --- | --- |
| RTF windows | 0.80944, 0.78600 | 0.74574, 0.73074 |
| Median RTF | 0.79772 | 0.73824 |
| Simulation publication rate | — | 9.9613–9.9723 Hz |
| Wall publication rate | — | 7.3179–7.3992 Hz |
| Largest observed heartbeat gap | 0.06555 s | 0.06109 s |

Median RTF was about 7.5% lower with CPU 3D LiDAR in this short comparison;
host load varies, so this is not an isolated benchmark or guaranteed cost.
No watchdog epoch change occurred during either measured window or the full
CPU sensor validation. The long CPU launch later logged four watchdog expiry
warnings, including the deliberate SIGSTOP check and other interruptions
outside the no-trip measurement interval. Host-load sensitivity remains;
the existing 0.3 wall-second watchdog was not relaxed. Hardening validated
recovery and slow simulation at measured RTF **0.01992**.

### Loaded final repeat

[Final CPU repeat](lidar3d_cpu_loaded_failure.json) used the final CPU-only
installed launch. Ten sensor/TF/geometry/motion checks passed, but its final
no-watchdog-trip check failed after an epoch change. RTF was **0.53392**, raw
publication **9.8522 Hz** simulation / **5.1947 Hz** wall, and the measured
window's maximum heartbeat gap **0.13643 s**. The trip occurred outside that
short performance window. The known box median error remained **0.01274 m**.

A subsequent host sample found load approximately **5.0 on four CPUs**, with
`apt-extracttemp` using about one CPU core. This is contemporaneous load
evidence, not proof of the precise cause of each interruption. No system
update process or safety setting was changed.

[Loaded sensor-disabled baseline](lidar3d_loaded_baseline.json) passed its
three checks, with RTF **0.80089 / 0.78625** and maximum measured heartbeat
gap **0.13044 s**. Its continuing launch subsequently logged a watchdog
expiry with LiDAR disabled, about 30 wall seconds after spawn. Thus LiDAR
is not required to trigger this host's existing timing sensitivity; the
additional sensor cost may still contribute. A quiet-host run cannot be
claimed from these measurements. The final loaded CPU run is retained as a
failure rather than rerun until it passes or hidden by a larger timeout.

[RViz screenshot](lidar3d_rviz.png): actual RobotModel and height-colored
PointCloud visible; Global Status, RobotModel and PointCloud show OK;
fixed frame `base_link`, `/points`, reliable QoS and decay zero. RViz was
launched with `use_sim_time=true` in a temporary nested X display, which was
closed after inspection. The test did not unlock the desktop or change shell
startup files.

Expected warnings: Xacro's Python 3.11 forward-compatibility warning;
robot_state_publisher's backward-time warning during explicit simulator
reset; RViz Wayland/X11 and unsupported-stereo notices. No fatal CPU sensor,
TF or RViz display error was observed. Snap IDE GTK environment variables
were cleared per process using the documented workaround.

## Failed GPU experiments retained

GPU is **not exposed by the delivered launch or local sensor wrapper**.
The generic vendored macro retains the reference GPU implementation for
provenance, with nominal beam labels instead of the source's workaround.

- [Upstream correction failure](lidar3d_gpu_upstream_correction_failure.json):
  the one-step reprojection produced ground median error **0.8406 m**.
  A separate raw-cloud inspection found local GPU labels already matched
  ground-derived angles: labels −17°, −15°, −13° matched approximately
  −17.01°, −14.99°, −13.00°. Applying the source workaround was incorrect
  on this installed `ros-humble-gazebo-plugins`
  `3.9.0-1jammy.20260907.231520`.
- [Nominal-angle GPU failure](lidar3d_gpu_nominal_failure.json): removing the
  shift improved ground median error to **0.02316 m**, but its 95th percentile
  remained **0.60026 m**, and hill medians **0.0687–0.0995 m** exceeded the
  unchanged accuracy thresholds. That run's RTF was approximately 0.566;
  long GPU launches also logged frequent watchdog interruptions outside the
  measured windows. These results are not passing evidence.

No accuracy tolerance or safety timeout was increased to make GPU pass.
The CPU path is the delivered and validated implementation. Dense/high CPU
profiles are description-tested but are not performance/motion validated;
start with lite. Slope mobility, realistic sand and LiDAR material/dust
physics remain outside this prototype's demonstrated capabilities.
