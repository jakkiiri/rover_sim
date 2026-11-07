# Swerve control architecture

This guide describes the current prototype, including the no-strafe policy,
controller hardening and the separately integrated LiDAR. The dated
[Phase 3 validation report](validation/phase3.md) preserves the original
phase-specific results.

## Architecture and interfaces

`/cmd_vel` (`geometry_msgs/msg/Twist`, body frame: +X forward, +Y left,
+Z counterclockwise) feeds the Python `swerve_controller` node. Only linear.x
and angular.z command rover motion. Finite linear.y is clamped to zero in
`ControlState.on_command` before storage and inverse kinematics; a pure lateral
command stops drive. Nonfinite values in any of the three planar fields stop
all axes. The expanded model description is passed to
both robot_state_publisher and this node by `rover_desert.launch.py`.

The node reads kingpin origins, drive-joint axes/orientations, wheel collision
radii and velocity limits from that URDF. It rejects layouts outside the
supported wheel-centered, vertical-kingpin geometry rather than guessing.
It publishes eight named targets on `/swerve/joint_commands` using JointState:
steering uses position, wheel drive uses velocity; the other entries are zero.
This is a command interface, not a measured JointState topic.

The local `librover_swerve_actuators.so` model plugin uses gazebo_ros and Gazebo
Classic physics joints. It publishes actual positions, velocities and applied
efforts on `/joint_states` at 50 Hz. This is actuator feedback. The separate
Classic ray sensor publishes `/points_raw` in `lidar_link`, with `/points`
from its separate post-processor. Sensor description, plugin and local RViz
configuration are documented in [lidar3d.md](lidar3d.md). The original
`/scan` sensor is selectable through `lidar_mode:=2d` ([2D guide](lidar.md)).
The actuator plugin discovers joints by the `_kingpin`
and `_wheel_joint` suffixes, checks that eight joints exist, and validates named
commands before accepting the complete command frame.

Steering is effort-controlled PD (Kp=200 Nm/rad, Kd=8 Nm/(rad/s)), limited to
25 Nm by the URDF. Wheel drive uses velocity PI (Kp=8 Nm/(rad/s), Ki=30 Nm/rad), limited to
40 Nm. The integral contribution is capped at 20 Nm, uses saturation
anti-windup, and resets when the wheel target is zero.
Joints are moved by forces through physics, never by setting model poses or
teleporting joints. The validation script alone resets model pose between cases.
Gazebo Classic and gazebo_ros provide simulation integration. Keyboard teleop
is a separate node; feedback/reset handling is described in
[hardening.md](hardening.md). Robot_state_publisher owns the fixed
`base_root -> base_link -> lidar_link` transforms and the measured axle/wheel
transforms. The LiDAR plugin publishes no TF. No ros_gz, gz sim or
ros2_control is used.

## Actual module geometry and joints

| Module | x (m) | y (m) | Steering joint | Drive joint |
| --- | --- | --- | --- | --- |
| Front left | 0.38 | 0.45 | front_left_kingpin | front_left_wheel_joint |
| Front right | 0.38 | -0.45 | front_right_kingpin | front_right_wheel_joint |
| Rear left | -0.38 | 0.45 | rear_left_kingpin | rear_left_wheel_joint |
| Rear right | -0.38 | -0.45 | rear_right_kingpin | rear_right_wheel_joint |

Wheelbase = 0.76 m, track = 0.90 m, radius = 0.18 m.
Visible tires are 0.12 m wide; their centered cylindrical collision band is
0.02 m wide to approximate a rounded tire tread. This avoids excessive ODE
scrub from broad flat cylinder edges during rotation; mass/inertia retain the
full visible width. These values are documented
observations; the controller extracts them from the model at startup. Steering
is continuous, with a 3 rad/s target slew limit. Wheel velocity limit is
20 rad/s in URDF, with an additional configurable 12 rad/s controller cap.

## Inverse kinematics

For chassis twist (vx, vy, wz) and module position (xi, yi):

```text
ui = vx - wz * yi
vi = vy + wz * xi
alpha_i = atan2(vi, ui)
rolling_speed_i = sqrt(ui^2 + vi^2)
```

These are the unchanged general equations. Normal rover operation supplies
`vy=0` because the real rover cannot strafe. Individual module lateral velocity
`vi=wz*xi` during a turn remains necessary for rotation about the chassis center;
it does not command lateral chassis translation.

For each module, evaluate equivalent steering targets `alpha_i + k*pi` near
its previous target and multiply rolling speed by `(-1)^k`. Choose the target
with least steering travel. Continuous joints retain unwrapped angles across
+/-pi and through multiple revolutions. Exact ties mirror left/right choices
to reduce net chassis reaction torque; unequal travel still selects the nearer
target. This bounds each new target change to at most approximately pi/2.

The drive joint's axis is rotated into the zero-steering base frame using its
URDF origin RPY. The rolling direction is `spin_axis cross +Z`. On this model
the axis is -Y, so a positive wheel joint rate rolls toward -X: **forward motion
requires negative wheel joint velocity**. Define `sign_i` as the X component
of this rolling direction (currently -1 for all modules):

```text
wheel_rate_i = sign_i * (-1)^k * rolling_speed_i / radius_i
scale = min(1, min_i(allowed_wheel_rate_i / abs(wheel_rate_i)))
commanded_wheel_rate_i = scale * wheel_rate_i
```

Zero-rate wheels are excluded from the scale division. A common scale retains
the desired chassis twist proportions. Limits combine the URDF wheel velocity
limit and the configuration cap. At zero module velocity, keep its preceding
steering target and command zero wheel speed.

At 50 Hz, steering targets slew using their URDF rate limits. All drive targets
remain zero until every steering joint is within 0.08 rad of its final target.
Wheel rates then ramp at at most 8 rad/s^2 per wheel. This alignment gate may
pause driving during abrupt changes of chassis command. Normalization applies
to steady targets; independent wheel acceleration limits can temporarily change
velocity ratios. A zero command stops wheel targets immediately and holds
steering. Nonfinite commands become a zero command.

Command freshness uses monotonic wall time; feedback freshness uses simulation
time (both timeouts default to 0.5 s). A 50 Hz wall timer supplies actuator
heartbeats while slew and acceleration integrate simulation time. Invalid or
stale feedback inhibits drive. Startup/reset wait for complete feedback and
initialize steering from measurements. The independent plugin watchdog uses
wall time, default 0.3 s, configured by `actuator_watchdog` at launch. See
[hardening.md](hardening.md) for reset acknowledgement and clock semantics.

## Tests and repeatable Gazebo validation

The current complete suite has 322 unit tests: 22 kinematics, 22 teleop,
73 controller, 14 2D LiDAR, 29 3D LiDAR and 162 stereo cases. See the
[current validation report](validation/stereo.md) for the separate 63
recorded stereo integration checks, and the [3D LiDAR guide](lidar3d.md) for
the 3D sensor checks.

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim
source ~/rover_ws/install/setup.bash
colcon test --packages-select rover_sim --event-handlers console_direct+
colcon test-result --verbose
ros2 launch rover_sim rover_desert.launch.py world:=desert gui:=false
```

In another sourced terminal, with no other cmd_vel publisher running:

```bash
ros2 run rover_sim validate_swerve.py --output \
  ~/rover_ws/src/rover_sim/docs/validation/local_motion.json
```

This explicitly invoked script publishes commands continuously during pose
queries, tests forward/backward, left/right rotation, forward+yaw and zero
near the flat origin, and resets only
`proxy_rover` between cases. It compares Gazebo ground-truth displacement and
yaw against the commanded twist integrated over measured simulation time.
Joint feedback checks steering alignment and wheel-rate signs. Its motion
measurements use simulator ground truth rather than LiDAR. The output above
preserves the committed validation evidence. Tolerances are 0.15 m translation
error, 0.08 rad heading error, and 0.10 rad steering error. These checks concern
short, low-speed maneuvers on flat
terrain; they do not establish slope mobility or high-speed tracking.

To explicitly test prohibited lateral Twist injection, select
`blocked_lateral_left`, `blocked_lateral_right`, `blocked_lateral_forward` and
`blocked_lateral_yaw` with `--cases` (alongside any normal cases desired).
Expected motion uses the clamped command, and pure lateral cases must have
zero drive targets, resting wheels and less than 0.01 m displacement. These
intentional invalid-input tests are excluded from the script's default cases.
See [current validation](validation/no_strafe.md); Phase 3/4 reports predate the
no-strafe policy.

The unchanged standalone kinematics unit tests reconstruct each module velocity from steering and signed wheel rate
for forward, backward, both strafes, rotation, forward+yaw, diagonal and zero.
They also cover real URDF extraction, changed geometry, normalization, limits,
reversal optimization, angle wrapping, stationary modules and nonfinite input.
The library's broader mathematical capabilities are not exposed through the
normal rover command path. Controller regressions separately verify the clamp,
including pure lateral stop, mixed forward/yaw requests and nonfinite input.

## Limitations

The low-level actuator bridge is specific to Gazebo Classic. It is not a
hardware driver or a ros2_control controller. There is no odometry, localization,
heading feedback, traction model or command arbitration. Terrain is
rigid, not deformable sand. Wheel contact uses a narrow idealized tread,
not a detailed rubber tire model. Steering under contact friction can produce small
transient chassis motion; the alignment gate reduces scrub but does not control
chassis heading. Passive Phase 2 braking was reduced to enable active drive.
Phase 2 resting measurements therefore describe the earlier configuration.
