# Phase 4.5 hardening validation

Validated on 2026-10-10 with Ubuntu 22.04, ROS 2 Humble and Gazebo Classic
11.10.2. No sensors or odometry were added. The original inverse kinematics,
teleop implementation, actuator gains and tire geometry were retained.

## Build and unit tests

The normal workspace build passed:

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim
source ~/rover_ws/install/setup.bash
colcon test --packages-select rover_sim --event-handlers console_direct+
colcon test-result --verbose
```

88 pytest cases passed: 22 existing kinematics, 17 existing teleop, and 49 new
controller regressions. Colcon reports 91 results because it also counts the
three CTest suite wrappers. Zero failures, errors or skipped tests.

The new tests cover complete feedback, complete-to-partial/empty transitions,
duplicates, nonfinite and malformed feedback, invalid timestamps, simulation
feedback timeout, wall command timeout during pause, measured multi-turn
initialization, reset/reinitialization, backward clock jumps, parameter
constraints/atomic rejection, overflow containment and reset acknowledgement.

## Gazebo integration

The package launched and spawned successfully using Gazebo Classic and
gazebo_ros in ROS domain 45 with Gazebo master port 11355. This isolated launch
used `actuator_watchdog:=0.6`; the default remains 0.3 s. The existing user
simulation was not changed. All nine checks in
[phase45.json](phase45.json) passed:

| Check | Observed result |
| --- | --- |
| Atomic invalid runtime parameters | Rejected acceleration -1 with clear reason; timeout and acceleration remained 0.5 and 8.0 |
| Valid runtime parameters | Acceleration 4 accepted, then restored to 8 |
| Pause/resume deadman | Simulation clock stopped; expired command produced zero drive targets during pause; resumed wheel speed fell below 0.05 rad/s |
| Reset all while moving | New reset generation; zero targets and resting wheel speed about 0.0000094 rad/s |
| Reset models while moving | Same safe behavior without relying on a clock rewind |
| Reset time while moving | Same safe behavior through time-rewind detection |
| Slow simulation | Measured RTF 0.01998; 250 wall heartbeats in five seconds; largest heartbeat gap 0.0391 s; all wheel targets remained above 2 rad/s |
| Independent actuator watchdog | Controller process suspended; with 0.6 s timeout, wheel speed fell below 0.05 rad/s after 0.824 wall seconds |
| Controller recovery | Resuming the controller did not restart stale motion; wheel speed about 0.0000097 rad/s |

Each reset case also verified that a fresh command restored motion after
feedback initialization. Watchdog settling time includes physical braking;
it is not a measurement of the exact watchdog trigger instant. Validation
cleanup restored physics update rate, unpaused the test world, resumed the
controller and published zero. The dedicated test launch was stopped afterward.

## Motion regression checks

The unchanged `validate_swerve.py` was run for `forward left rotate` on flat
terrain in the isolated simulation. All three passed; wheel signs and steering
alignment were correct. These are physical simulation checks separate from
the 88 unit tests.

| Case | Translation error | Heading error | Maximum steering error |
| --- | --- | --- | --- |
| Forward | 0.000029 m | 0.000102 rad | 0.000061 rad |
| Strafe left | 0.000202 m | 0.000297 rad | 0.000103 rad |
| Center rotation | 0.000016 m | 0.013668 rad | 0.003777 rad |

Thresholds were 0.15 m translation, 0.08 rad heading and 0.10 rad steering.
These checks do not establish slope parking, slope mobility or high-speed
tracking. The original eight motion types remain covered by the unchanged
kinematics unit tests; only the three listed physical maneuvers were rerun here.

## TF and warnings

Live TF inspection confirmed nine parent-child edges:
`base_root -> base_link -> {front_left,front_right,rear_left,rear_right}_axle
-> corresponding wheel`. Only `base_link` was a static child, and the sole
`/tf_static` publisher was `robot_state_publisher`. No `base_footprint`, odom,
map or world-to-rover ROS transform is provided. Local RViz Fixed Frame is
`base_link`, with `use_sim_time=true`.

No Gazebo actuator/model warnings were observed during this validation.
Robot_state_publisher reported its expected "Moved backwards in time"
warning for reset-all and reset-time and republished transforms. Unit tests
emitted an installed Xacro/Python compatibility warning about private enum
variables in Python 3.11; the tests ran on Python 3.10 and passed.

## Files changed

Modified:

- `rover_sim/controller.py`
- `src/swerve_gazebo_plugin.cpp`
- `config/swerve.yaml`
- `launch/rover_desert.launch.py`
- `urdf/proxy_rover.urdf.xacro`
- `urdf/upstream/wheel_steered.urdf.xacro`
- `urdf/upstream/inertia_tensors.urdf.xacro`
- `CMakeLists.txt`
- `package.xml`
- `THIRD_PARTY_NOTICES.md`
- `README.md`
- `docs/swerve.md`

Added:

- `rover_sim/control_state.py`
- `test/test_controller.py`
- `scripts/validate_hardening.py`
- `LICENSES/UOS-BSD.txt`
- `docs/hardening.md`
- `docs/validation/phase45.json`
- `docs/validation/phase45.md`

Build/install/log artifacts were generated only through normal colcon/ROS
tools in the real workspace; none were edited directly.

## Remaining scope and readiness

The package is ready for a first simulated 2D LiDAR and local RViz display.
Add a fixed `lidar_link` under `base_link`, let robot_state_publisher publish
that transform, and use a Gazebo Classic gazebo_ros laser plugin publishing
`/scan` in `lidar_link`. No sensor implementation is included in this phase.

Odometry/world TF, localization, slope parking, the idealized 2 cm collision
tread, steering alignment pauses, independent wheel acceleration ramps,
command arbitration and broader model-parser support remain deferred. Until
odometry exists, local scans can be viewed in `base_link`; accumulated mapping
and world-fixed navigation require a later odometry/TF phase. The keyboard
deadman still depends on terminal key-repeat behavior, as previously documented.

See [hardening.md](../hardening.md) for exact wall/simulation clock semantics,
parameter limits and reset protocol. See [THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md)
for the preserved AIT pin, prominent modification notices and investigated
UOS BSD provenance.
