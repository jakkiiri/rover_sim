# Controller hardening, reset and shutdown

This guide describes current feedback, parameter, watchdog, reset and shutdown
behavior. Historically, Phase 4.5 changed those areas and TF/provenance without
adding sensors or changing kinematics, teleop, tire contact, gains or terrain.
The subsequent reliability fix hardened watchdog recovery and repeated-signal
shutdown. Phase 5 added the separate 2D LiDAR and local RViz; see
[lidar.md](lidar.md). There is still no odometry. Dated validation reports retain
their original phase-specific results.

## Current chassis-command constraint

The subsequent no-strafe control update clamps finite `/cmd_vel.linear.y` to
zero at `ControlState.on_command`, before storage and inverse kinematics.
Forward and yaw commands remain supported; pure lateral input requests zero
drive. Nonfinite input in any planar field still stops all axes. Startup,
feedback validation, timeout, reset and watchdog semantics below are preserved.
See [current control validation](validation/no_strafe.md).

## Steering feedback and initialization

The controller accepts feedback only when all four expected steering joints
are present, names are unique/nonempty, position arrays match names, all
supplied numeric arrays are finite, and optional velocity/effort arrays have
the correct length. Negative, out-of-order and implausibly future timestamps
are rejected. A 0.1 simulation-second allowance accommodates `/clock` delivery
lag relative to the physical feedback timestamp.

An invalid message immediately invalidates drive authorization without
replacing the last complete steering snapshot. The next wall control tick
(nominally within 20 ms) sends zero drive. A new complete valid snapshot is
required before alignment can pass again. Alignment directly indexes all four
measurements; missing positions are never replaced with targets.

At startup, no steering command is fabricated. The plugin holds measured
steering and zero wheel drive while waiting. The first valid snapshot seeds
all steering targets, including continuous multi-turn angles. Reset discards
initialization and the current chassis command; it repeats this initialization
procedure. Commands received after reset may resume motion once initialization
and steering alignment are complete.

## Parameters

`robot_description` is read-only after startup because it defines geometry.
The ROS-managed `use_sim_time` parameter remains boolean. The five numeric
controller parameters are validated at startup and before atomic runtime sets:

| Parameter | Constraint / meaning |
| --- | --- |
| command_timeout | Finite, nonnegative wall seconds; zero inhibits drive |
| feedback_timeout | Finite, nonnegative simulation seconds; zero inhibits drive |
| max_wheel_speed | Finite, positive rad/s; cannot exceed the largest URDF wheel limit (20 here) |
| wheel_acceleration | Finite, strictly positive rad/s² |
| alignment_tolerance | Finite, 0 <= value < pi/2 rad; zero requires exact alignment |

Invalid updates return a reason without committing any value in the atomic
request. Runtime updates are consumed by the next wall tick. An inverse result
that overflows despite finite inputs inhibits drive; the kinematics themselves
are unchanged.

## Clocks, pause and watchdog

The ROS controller sends joint-command heartbeats at 50 Hz **wall time** using
a steady clock. This cadence continues during pauses and slow simulation.
Steering slew and wheel acceleration use **simulation-time deltas**; a pause
does not integrate motion. Feedback age uses its simulation timestamp, so
healthy low-rate simulated feedback is not mistaken for a wall-time failure.
The operator's `/cmd_vel` timeout still uses monotonic wall time, so commands
expire during a pause if the publisher stops.

The actuator watchdog uses monotonic wall time, default 0.3 s, and is configured
through the `actuator_watchdog` launch argument/Xacro argument and plugin SDF
`watchdog_timeout`. It must be finite and strictly positive:

```bash
ros2 launch rover_sim rover_desert.launch.py actuator_watchdog:=0.6
```

If valid joint heartbeats cease, the plugin requests zero wheel velocity,
clears integrators, holds measured steering and advances the reset generation.
Expiry is checked under the same mutex both on physics updates and **before**
a command callback can refresh the heartbeat. A late callback therefore
cannot bypass expiry when physics is paused or its update loses the race.
Recovery requires the current-generation handshake described below; ordinary
heartbeats alone cannot reactivate a timed-out generation. Physics cannot
apply a force while Gazebo is paused; the next physics update publishes the
new generation and applies braking. Wall heartbeats
can deliver a zero target during pause before physics resumes. If a command
publisher deliberately continues sending fresh commands during pause, motion
may resume with that command. Pausing is not an operator emergency stop.

Zero wheel target uses the existing proportional brake with a cleared integral.
Static parking on slopes is intentionally deferred and is not established by
this phase's flat-ground stopping tests.

## Reset handshake

The plugin implements `Reset()` and detects simulation-time rewind. On reset
it invalidates command freshness, clears drive targets/integrators, and reads
steering hold positions on the first update after Gazebo resets the joints.
It publishes a reset generation on `/swerve/reset_epoch` (`std_msgs/msg/UInt64`).
The generation starts from steady-clock nanoseconds to distinguish separate
plugin instances and increments for each reset.

The controller clears initialization, feedback and the chassis command, then
acknowledges on `/swerve/reset_ack`. Both handshake topics use reliable,
transient-local, depth-one QoS so startup discovery cannot lose a one-shot
notification. The plugin inhibits drive until the current generation is
acknowledged. Acknowledgement clears actuator state again and establishes a
simulation-time barrier: joint commands stamped before or at that barrier,
and implausibly future commands, are rejected. This prevents queued pre-reset
joint commands from restarting motion. A backward controller clock jump also
clears its state independently.

On startup the controller waits for the first generation before subscribing
to `/cmd_vel`. Each new generation resets controller state and replaces its
reliable, volatile, depth-one `/cmd_vel` subscription **before** acknowledging.
This discards the old reader's queued Twist samples. Already-taken callbacks
also carry their reader's generation and are ignored after generation changes.
Duplicate acknowledgements are harmless and do not reset current actuator
state. This procedure also applies to watchdog recovery.

The external Twist interface remains unstamped. "Fresh command after recovery"
means input delivered through the new reader, not proof of a physical key
press or an end-to-end age guarantee for arbitrary upstream publishers.
An active publisher can resume with a new publication after discovery. A
publisher that sends only once during recovery may have to send again. Only
one `/cmd_vel` publisher is supported, as before.

## Controller shutdown

The executable owns non-raising SIGINT/SIGTERM handlers through process exit.
Handlers only request stop; the main loop checks at nominal 20 ms intervals.
They do not call ROS APIs or raise KeyboardInterrupt during destruction.
Cleanup inhibits callbacks, cancels the timer, clears the chassis command,
publishes zero wheel drive with measured/initialized steering targets, and
waits up to 0.2 wall seconds for reliable DDS acknowledgement before destroying
the node. DDS acknowledgement is transport delivery, not physical wheel stop.
If steering was never initialized, no steering target is fabricated. Cleanup
is guarded against repeated or reentrant calls; node destruction runs at most
once and context shutdown still runs if cleanup reports another error.
The independent actuator watchdog remains the fallback for transport failure
or forced process termination. SIGKILL cannot run controller cleanup.

## TF ownership and local visualization

```text
base_root                         massless, coincident chassis dummy
└── base_link                      fixed, robot_state_publisher
    ├── lidar_link                 fixed lidar_joint, XYZ (0, 0, 0.20) m
    ├── front_left_axle            front_left_kingpin
    │   └── front_left_wheel       front_left_wheel_joint
    ├── front_right_axle           front_right_kingpin
    │   └── front_right_wheel      front_right_wheel_joint
    ├── rear_left_axle             rear_left_kingpin
    │   └── rear_left_wheel        rear_left_wheel_joint
    └── rear_right_axle            rear_right_kingpin
        └── rear_right_wheel       rear_right_wheel_joint
```

`base_root` replaces the misleading `base_footprint` name without changing
geometry, inertia or spawn coordinates. It does not represent ground contact.
Robot_state_publisher owns all fixed robot transforms and uses measured joint
states for articulated transforms. There is no `odom`, `map` or world-to-robot
ROS transform yet. Local RViz should use `base_link` as Fixed Frame and
`use_sim_time=true`. Robot_state_publisher owns `base_link -> lidar_link`;
the ray sensor plugin publishes `/scan` in `lidar_link` and publishes no TF.
The optional `lidar:=false` launch omits that sensor branch. A future odometry
source must exclusively own the moving transform to the rover root; no static
world-to-base transform is added.

## Tests and integration validation

The current complete suite contains 131 unit tests: 22 kinematics, 22 teleop,
73 controller and 14 LiDAR description/RViz cases. See the
[milestone validation index](validation/phase5.md) for the 44 recorded
integration checks. Deterministic controller tests exercise explicit clocks,
complete/invalid feedback, startup,
timeouts, reset, pause and parameter validation. See
[recorded Phase 4.5 results](validation/phase45.md).
The focused follow-up adds recovery/shutdown regressions; see
[reliability results](validation/reliability.md).

For isolated integration validation, build/source the normal workspace, choose
unused ROS domain/master values, and launch with the current default 0.3 s
watchdog. Older reports retain their historical 0.6 s validation settings.

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim
source ~/rover_ws/install/setup.bash
ROS_DOMAIN_ID=45 GAZEBO_MASTER_URI=http://127.0.0.1:11355 \
  ros2 launch rover_sim rover_desert.launch.py gui:=false actuator_watchdog:=0.3
```

Use the exact `swerve_controller` PID printed by that launch in another sourced
terminal, with the same domain/master:

```bash
ROS_DOMAIN_ID=45 GAZEBO_MASTER_URI=http://127.0.0.1:11355 \
  ros2 run rover_sim validate_hardening.py --controller-pid PID --watchdog 0.3 \
  --output ~/rover_ws/src/rover_sim/docs/validation/local_hardening.json
```

This changes only that isolated world's physics settings, temporarily suspends
the specified controller, then restores both in cleanup. It tests atomic ROS
parameter updates, pause/resume, reset-all/model-only/time-only while moving,
slow simulation at target RTF 0.02, independent actuator expiry, a deliberately
delayed Twist across expiry, and recovery only after fresh commands. Recovery
examines all new wheel-state/target samples instead of accepting a cached zero
snapshot. The existing 0.4 s observation window is unchanged.

Real ROS process shutdown validation uses a different unused domain and no
Gazebo. It injects repeated SIGINT/SIGTERM at the node-destruction boundary,
checks final zero delivery and one destruction, and repeats before feedback:

```bash
ROS_DOMAIN_ID=46 ros2 run rover_sim validate_shutdown.py \
  --output ~/rover_ws/src/rover_sim/docs/validation/local_shutdown.json
```

The `local_*.json` output paths preserve the committed historical reports.
Remaining prototype limitations include slope parking, 2 cm tread realism,
steering alignment pauses and independent wheel ramps, lack of odometry,
single-source command operation and restricted URDF geometry support. See the
[consolidated limitations](../README.md#known-prototype-limitations).
