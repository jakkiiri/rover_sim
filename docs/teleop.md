# Keyboard teleoperation

Launch the Gazebo Classic rover normally. In another interactive terminal:

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 run rover_sim rover_teleop
```

Keep that terminal focused and hold one motion key. No Enter is needed.
Lowercase and uppercase both work. Each key selects one body-frame command;
simultaneous keys do not combine into translating-turn commands.

| Key | Action | Default Twist field/value |
| --- | --- | --- |
| W | Forward | linear.x = 0.4 m/s |
| S | Backward | linear.x = -0.4 m/s |
| A | Turn / rotate left | angular.z = 0.5 rad/s |
| D | Turn / rotate right | angular.z = -0.5 rad/s |
| Space | Stop immediately | All fields zero |
| Ctrl+C | Stop and quit | All fields zero |

The node publishes `geometry_msgs/msg/Twist` on `/cmd_vel` at 20 Hz, with an
additional immediate publication on input changes and deadman expiration.
`linear.y` is explicitly forced to zero on every publication. Other unused
Twist fields remain zero. Q/E are unsupported keys and stop active motion;
they do not rotate. Active commands and stop reasons print when
they change. The executable is installed with the existing ament_cmake Python
package layout; `ros2 run rover_sim rover_teleop` is its console entry point.
Teleop is separate from the Gazebo launch, controller and kinematics.

## Configuration

Parameters are set at startup. Defaults are also recorded in
`config/teleop.yaml`. For example:

```bash
ros2 run rover_sim rover_teleop --ros-args \
  --params-file "$(ros2 pkg prefix --share rover_sim)/config/teleop.yaml" \
  -p linear_speed:=0.25 -p angular_speed:=0.4 \
  -p key_timeout:=0.7 -p publish_rate:=20.0
```

`lateral_speed` has been removed; remove it from old parameter files and
command lines. Speeds must be finite and nonnegative; timeout and rate must be finite and
positive. The publish rate should comfortably exceed 2 Hz to keep the existing
controller's 0.5-second command watchdog satisfied. Only run one `/cmd_vel`
publisher at a time: there is no command multiplexer or arbitration.

## Stopping and terminal behavior

Terminal input uses `select` and cbreak mode, without a blocking input thread.
The loop checks stopping at least every 50 ms, using monotonic wall time even
when simulation is paused. The original terminal settings are restored on
normal exit, Ctrl+C, SIGTERM and handled exceptions. A non-terminal stdin is
rejected with a clear message.

Terminals report key presses and auto-repeat, but **do not report key release**.
Motion therefore stops after 0.7 seconds without another motion-key event.
Choose a timeout longer than the desktop keyboard's initial repeat delay.
Space bypasses that timeout and sends zero immediately; unsupported keys also
stop. Arrow/function-key escape sequences are consumed to prevent their A/D
suffixes from triggering motion. A stop overrides motion buffered in the same
read, and pending terminal input is flushed after a stop.

Ctrl+C and SIGTERM leave ROS alive briefly to publish three zero commands
before destroying the node. If the process is killed abruptly, the existing
swerve controller's 0.5-second command watchdog remains a fallback. Physical
braking still has finite inertia and actuator response; Space requests an
immediate zero velocity, not an instantaneous physical halt. Switching between
translation and rotation also waits for the existing steering alignment logic.

## No-strafe controller policy

Every `/cmd_vel` publisher goes through `ControlState.on_command`, called by
the ROS controller callback. Finite `linear.y` is clamped to zero before it is
stored or reaches inverse kinematics. `linear.x` and `angular.z` retain their
requested values. A pure lateral command becomes a zero command and stops
wheel drive while holding steering. If any of those three input fields is
nonfinite, the complete command becomes zero, as before. There is no parameter
that enables lateral commands.

The generic inverse-kinematics library and four independently steered modules
remain unchanged. Direct publication to the internal `/swerve/joint_commands`
actuator topic can bypass this chassis-command policy; it is not a normal user
control interface. Physical slipping, terrain forces and steering transients
can still cause lateral displacement. Forward travel while turning naturally
changes world Y position; that is not body-frame strafing.

## Validation

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim
source ~/rover_ws/install/setup.bash
colcon test --packages-select rover_sim --event-handlers console_direct+
colcon test-result --verbose
ros2 launch rover_sim rover_desert.launch.py world:=desert gui:=false
```

With the launch running, in a second sourced terminal:

```bash
ros2 run rover_sim validate_teleop.py \
  --output ~/rover_ws/src/rover_sim/docs/validation/no_strafe_teleop.json
```

The validation executable feeds keys into the actual installed teleop through
a pseudo-terminal, measures `/cmd_vel`, actual joint feedback and Gazebo Classic
ground-truth pose, and checks W/S translation, A/D rotation, removed Q/E,
Space, release timeout and real terminal Ctrl+C. Every observed teleop Twist
is checked for zero lateral velocity. It resets only `proxy_rover` between motion cases.
No sensors are added. Do not run another command publisher during validation.
See [the current recorded results](validation/no_strafe.md). The older Phase 3/4
reports describe the previous holonomic interface and remain historical records.
