# No-strafe rover controls and validation

Implemented and validated on 2026-10-11 using Ubuntu 22.04, ROS 2 Humble and
Gazebo Classic 11.10.2 with gazebo_ros. Validation used a dedicated headless
instance, ROS domain 45 and Gazebo master port 11355.

## Controls and command policy

| Key | Command | Default |
| --- | --- | --- |
| W | Forward, positive linear.x | 0.4 m/s |
| S | Backward, negative linear.x | -0.4 m/s |
| A | Turn / rotate left, positive angular.z | 0.5 rad/s |
| D | Turn / rotate right, negative angular.z | -0.5 rad/s |
| Space | Immediate zero command | All axes zero |
| Ctrl+C | Zero commands, restore terminal, exit | Exit code 0 |
| Q/E | Unsupported keys; stop active motion | All axes zero |

Uppercase and lowercase work. `linear_speed`, `angular_speed`, `key_timeout`
and `publish_rate` remain configurable. `lateral_speed` is removed. Deadman
timeout remains 0.7 wall seconds; normal publication remains 20 Hz.

Teleop explicitly publishes `linear.y=0` even if its internal command is
incorrectly populated. The ROS controller delegates every incoming Twist to
`ControlState.on_command`, which clamps finite lateral input to zero before
storage or inverse kinematics. Forward and yaw fields are preserved. Pure
lateral input becomes zero drive while steering targets are held. Nonfinite
input in any planar field still stops all axes. No option enables strafing.
An external publisher's original Twist can still appear on `/cmd_vel`; the
clamp applies to the command the rover controller actually consumes.

The generic inverse kinematics, wheel/steering model and Gazebo actuator plugin
were not changed. Historical Phase 3/4 validation records were preserved and
describe the former controls.

## Build and unit tests

The normal workspace build and tests passed:

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim
source ~/rover_ws/install/setup.bash
colcon test --packages-select rover_sim --event-handlers console_direct+
colcon test-result --verbose
```

103 unit tests passed: 22 unchanged kinematics, 22 teleop and 59 controller.
All 49 previous controller state/reset/parameter regressions are retained.
Colcon reports 106 results because it also counts the three CTest wrappers.
There were zero unit failures, errors or skipped tests.

Teleop mapping/configuration expectations were updated. Seven new teleop
cases cover Q/q/E/e from idle and active motion, every possible input byte
having zero lateral output, publication forcing zero lateral input, and the
removed lateral-speed constructor parameter. Ten new controller cases cover
pure and mixed lateral Twists through the actual ROS callback, stopping an
existing forward command with pure lateral input, and nonfinite lateral input.

## Gazebo and terminal integration

The rover launched and spawned successfully. The installed teleop executable
was exercised through a real pseudo-terminal, including terminal-generated
SIGINT. All 10 checks passed in [no_strafe_teleop.json](no_strafe_teleop.json):

| Check | Observed result |
| --- | --- |
| W | Forward motion; heading error 0.000010 rad |
| S | Backward motion; heading error 0.000104 rad |
| A | Positive center rotation; translation 0.000214 m |
| D | Negative center rotation; translation 0.000243 m |
| Q disabled | Zero command within 0.00454 wall seconds; wheels settled |
| E disabled | Zero command within 0.00226 wall seconds; wheels settled |
| Space | Zero command within 0.00207 wall seconds; wheels settled |
| Release/deadman | Zero after 0.633 wall seconds from the test's release event |
| Ctrl+C | Zero within 0.0344 wall seconds; exit 0; terminal restored |
| All teleop publications | All 1,511 observed Twist messages had linear.y=0 |

The release test event occurs after the last repeated key has finished;
the timeout is measured by the node from that last key, not from physical
key-release notification. Terminals still do not report release events.
Zero-command latency is distinct from physical braking time.

All 10 direct-command cases passed in
[no_strafe_motion.json](no_strafe_motion.json): forward, backward, both
rotations, forward+yaw, zero, positive/negative pure lateral input, lateral
plus forward, and lateral plus yaw. Wheel velocity signs and steering alignment
were correct in every case.

For pure lateral input, all wheel targets were exactly zero and actual wheel
speeds were below 0.05 rad/s. Total measured translation was 0.00000695 m for
positive input and 0.00000684 m for negative input, over approximately 4.4
simulation seconds. Mixed input retained forward or yaw motion without adding
body-frame strafe. These are flat-ground, low-speed checks, not slope tests.

The default motion validation cases now use only forward/yaw commands.
Prohibited lateral injection requires explicitly selecting `blocked_lateral_*`
cases. The full recorded command was:

```bash
ros2 run rover_sim validate_swerve.py --cases \
  forward backward rotate_left rotate_right forward_yaw zero \
  blocked_lateral_left blocked_lateral_right \
  blocked_lateral_forward blocked_lateral_yaw \
  --output ~/rover_ws/src/rover_sim/rover_sim/docs/validation/no_strafe_motion.json
```

## Reset/watchdog regression and an intermittent recovery limitation

**Follow-up:** both issues recorded in this historical section have now been
reproduced and fixed. See [focused reliability results](reliability.md) for
root causes, current recovery semantics and regression evidence.

The unchanged `validate_hardening.py` ran against the dedicated controller
with a 0.6 s actuator watchdog. Its final run passed all nine checks in
[no_strafe_hardening.json](no_strafe_hardening.json): atomic parameter rejection,
valid parameter updates, pause/resume, reset-all, model-only reset, time-only
reset, slow simulation, actuator watchdog and controller recovery. Slow
simulation measured RTF 0.01998. The watchdog stopped wheel speeds below
0.05 rad/s in 0.821 wall seconds, including physical braking.

The first run passed eight checks but failed controller recovery after
SIGCONT: actual wheel speed was 2.332 rad/s after the recovery wait. The same
unmodified suite passed on rerun. Queued unstamped Twist input being treated
as fresh upon callback execution is a suspected cause; this has not been
conclusively isolated or fixed. Do not interpret the passing repeat as proof
that arbitrary OS suspension/resumption cannot briefly replay forward motion.
The no-strafe clamp still applies to every received command. The earlier
recovery result is recorded here because the JSON contains the final run.

This issue concerns the existing command-receipt/watchdog architecture and is
deferred from the requested no-strafe change. Production reset/watchdog code
and its validation script were not modified to make the check pass.

Expected robot_state_publisher backward-time warnings occurred during reset
tests. Unit tests reported the existing installed Xacro/Python compatibility
warning. The dedicated validation launch was stopped after testing. During
shutdown the controller received another SIGINT while destroying subscriptions,
logged KeyboardInterrupt and exited with code -2; Gazebo and
robot_state_publisher exited cleanly. This controller cleanup path was not
changed. The separate keyboard teleop Ctrl+C test exited cleanly with code 0
and restored its terminal. Repeated-SIGINT cleanup remains a separate issue.

## Remaining lateral-motion capability

The normal `/cmd_vel` path cannot request body-frame lateral translation.
The simulation remains a mechanically holonomic four-module proxy: an
explicit low-level publisher on `/swerve/joint_commands` can bypass the chassis
policy, and the standalone mathematics can still calculate strafe. That
internal actuator topic is not a supported user control interface. No joint
constraints or tire physics were changed.

Slip, terrain forces and steering transients can cause physical lateral drift.
Forward+yaw follows a curved world trajectory and naturally changes world Y;
that is not a lateral chassis command. Slope parking, idealized tire contact,
steering alignment pauses, odometry and command arbitration remain deferred.

## Files changed

Modified:

- `rover_sim/teleop.py`
- `rover_sim/control_state.py`
- `config/teleop.yaml`
- `test/test_teleop.py`
- `test/test_controller.py`
- `scripts/validate_teleop.py`
- `scripts/validate_swerve.py`
- `README.md`
- `docs/teleop.md`
- `docs/swerve.md`
- `docs/hardening.md`

Added:

- `docs/validation/no_strafe.md`
- `docs/validation/no_strafe_teleop.json`
- `docs/validation/no_strafe_motion.json`
- `docs/validation/no_strafe_hardening.json`

Existing wrappers continue to call the updated nodes without changes. No
sensors were added. Build/install/log artifacts were produced only through
normal workspace tooling; none were edited directly.
