# Drive controller

`swerve_controller` turns a body twist on `/cmd_vel` into steering angles
and wheel rates for the four swerve modules, publishes wheel odometry and
diagnostics, and keeps the safety behaviour of the prototype. The core
(`rover_sim/control_state.py`, `kinematics.py`, `odometry.py`) is ROS-free
and unit tested; `rover_sim/controller.py` is the ROS wrapper.

## Interfaces

| Topic | Type | Direction | Notes |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | in | body frame, `linear.x/y`, `angular.z` |
| `/joint_states` | `sensor_msgs/JointState` | in | plugin feedback (12 joints, 50 Hz) |
| `/imu/data` | `sensor_msgs/Imu` | in | tilt limits and odometry yaw rate |
| `/swerve/set_drive_mode` | `std_msgs/String` | in | `ackermann`, `crab`, `point_turn`, `auto` |
| `/swerve/joint_commands` | `sensor_msgs/JointState` | out | steering positions + wheel velocities, 50 Hz wall |
| `/odom`, TF `odom → base_link` | `nav_msgs/Odometry` | out | wheel odometry (gyro yaw) |
| `/swerve/drive_mode` | `std_msgs/String` (latched) | out | current mode |
| `/swerve/diagnostics` | `diagnostic_msgs/DiagnosticArray` | out | 5 Hz: tilt, steer scale, slip, traction, stall |
| `/swerve/reset_epoch`, `/swerve/reset_ack` | `std_msgs/UInt64` | both | reset handshake with the plugin |

The plugin additionally publishes `/ground_truth/odom` (validation only, not
a rover sensor) and `/swerve/actuator_diagnostics` (per-wheel effort,
available torque, slip, parking hold).

## Kinematics

For module *i* at (xᵢ, yᵢ) and twist (v_x, v_y, ω):

```
v_i = (v_x − ω y_i, v_y + ω x_i),   θ_i = atan2(v_iy, v_ix),   φ̇_i = |v_i| / R
```

Steering-flip optimisation picks θ_i or θ_i + π (with reversed wheel rate)
whichever is closer to the current unwrapped angle, so a module never swings
more than ~90°; angles are continuous (multi-turn) and a module whose speed
is zero holds its last angle. All wheel rates are scaled by a common factor
when any would exceed `max_wheel_speed`, preserving the twist direction.

Forward kinematics (`kinematics.forward`) solves the 8 × 3 least-squares
system for the twist from the measured angles and rates; its per-module
residuals and a leave-one-out prediction feed slip detection.

## Control pipeline (`ControlState.step`, 50 Hz)

1. **Freshness**: no `/cmd_vel` within `command_timeout` (wall) or no valid
   `/joint_states` within `feedback_timeout` (sim) → immediate zero twist
   (no deceleration ramp). Nonfinite commands are invalid commands.
2. **Drive mode**: `ackermann` zeroes v_y (default, the teleop default),
   `crab` zeroes ω, `point_turn` zeroes v_x/v_y, `auto` passes everything.
3. **Tilt limits**: with the IMU tilt τ = max(|roll|, |pitch|), speed and
   acceleration limits scale linearly from 1 at `tilt_slowdown_deg` (15°) to
   0 at `tilt_cutoff_deg` (32°, ≈ 0.7 of the 47° longitudinal tipping angle).
4. **Body-level acceleration and jerk limits** on v_x, v_y and ω (not per
   wheel), with a discrete-exact jerk ramp so wheel rates stay mutually
   consistent and paths are smooth.
5. **Inverse kinematics** and steering slew at the URDF rate (3 rad/s).
6. **Steer-error scaling**: each module's drive is scaled by
   `max(0, cos e_i)^p` (`steer_error_power` = 2) where e_i is its steering
   error; with `twist_scaling` the whole twist uses the worst module so the
   rover follows the commanded path instead of stopping. A hard stop applies
   only beyond `steer_error_cutoff` (1.2 rad).
7. **Traction control**: slip_i = (rim speed − predicted speed)/max; above
   `slip_threshold` (0.35) the wheel's command decays at
   `slip_reduction_rate` down to `slip_min_scale`, recovering at
   `slip_recovery_rate`.
8. **Stall detector**: commanded motion while the IMU-fused body speed stays
   below `stall_speed_fraction` of it, or while all four wheels sit at the
   minimum traction scale, for `stall_time` (2 s) raises a WARN diagnostic
   and scales the wheel commands by `stall_scale` (0.6) instead of letting
   the actuator integrators wind up.

The IMU fusion for slip/stall sensing integrates the gravity-compensated
body acceleration minus the change of the wheel-odometry velocity into a
velocity correction that decays with `imu_velocity_tau` (10 s). Pushing
against a wall leaves "wheels report 0.5 m/s, IMU saw the stop", which no
wheel-only estimator can see when all four wheels spin equally. The decay
bounds IMU bias drift, so a stall that lasts longer than a few τ can only be
kept flagged through the traction-control state.

All parameters live in `config/swerve.yaml` and are validated at startup and
on atomic runtime updates; nonsense values abort the node or reject the
update with a reason.

## Odometry

`WheelOdometry` integrates the least-squares twist with the midpoint rule;
with `odometry_use_imu` the gyro yaw rate replaces the wheel yaw estimate.
Published on `/odom` and as `odom → base_link` TF (`publish_odom_tf`). The
plugin's `/ground_truth/odom` is only for validation; measured drift is in
`docs/validation/mars_controller.md`.

## Teleop

`ros2 run rover_sim rover_teleop`: W/S forward/back, A/D rotate, Q/E strafe
(only in `crab`/`auto`), `+`/`-` speed ×1.25 (0.2–2.5×), `M` cycles the
drive mode (publishes `/swerve/set_drive_mode`), `X` toggles a latched
e-stop, Space stops, Ctrl+C quits. A status line prints on every change.
Arrow/function-key escape sequences are consumed so their trailing letters
never become commands; key release is inferred by the 0.7 s deadman.
`ros2 run rover_sim rover_teleop_joy` maps a `sensor_msgs/Joy` gamepad
(deadman LB, A cycles mode, B e-stop, sticks for v_x/v_y/ω).

## Safety behaviour kept from the prototype

Reset epochs with acknowledgement (`docs/hardening.md`), the wall-time
actuator watchdog in the plugin, rejection of malformed, stale or
future-stamped joint feedback, a time-jump reset, and the idempotent shutdown
that publishes a final zero drive command.

## Validation

`scripts/validate_controller.py` (circle and figure-eight cross-track error,
drive-mode switching, stall diagnostics, odometry drift, command watchdog,
reset handshake), `scripts/validate_shutdown.py` and
`scripts/validate_teleop.py`. Results: `docs/validation/mars_controller.md`.
