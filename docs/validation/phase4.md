# Phase 4 keyboard teleop validation

Validated on 2026-10-10 with Ubuntu 22.04, ROS 2 Humble and Gazebo Classic
11.10.2. The existing `rover_desert.launch.py gui:=false` successfully spawned
`proxy_rover`. All nine requested keyboard behaviors passed on the flat area
of the desert world. No sensors, controllers, kinematics, model geometry or
launch files were changed in this phase.

## Build and tests

Executed from `/home/leokarras/rover_ws`:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim
source /home/leokarras/rover_ws/install/setup.bash
colcon test --packages-select rover_sim --event-handlers console_direct+
colcon test-result --verbose
```

Build succeeded. All 17 teleop unit cases and 22 existing swerve kinematics
cases passed. Colcon reported 41 tests including the two CTest wrappers,
with zero errors, failures or skips. Colcon alone generated build/install/log
outputs; no files in those directories were edited directly.

The existing xacro import produced a Python compatibility warning about private
enum attributes becoming normal attributes in Python 3.11. No Gazebo warnings
appeared during spawn or motion. Launch printed its expected interruption
warning when the validation launch was stopped with Ctrl+C, and all remaining
launch processes exited cleanly.

## Motion and stopping

`validate_teleop.py` launched the installed `rover_teleop` executable on its own
controlling pseudo-terminal, injected keyboard bytes, and observed the resulting
ROS `/cmd_vel` messages and actual wheel feedback. Motion was measured from
Gazebo Classic pose packets with their matching simulation timestamps. No Twist
publisher bypassed the keyboard path. The rover was reset to the flat origin
between directional cases; each motion measurement followed steering alignment
and acceleration. These are physics checks, not a manual GUI/desktop-keyboard
test or slope test.

| Input | Measured behavior | Result |
| --- | --- | --- |
| W | Forward 1.2504 m over 3.126 simulation seconds; yaw +0.0145 degrees | Pass |
| S | Backward 1.2660 m over 3.165 seconds; yaw -0.0078 degrees | Pass |
| A | Left 1.1176 m over 3.193 seconds; yaw -0.0135 degrees | Pass |
| D | Right 1.1186 m over 3.196 seconds; yaw -0.0080 degrees | Pass |
| Q | Left rotation 89.576 degrees over 3.140 seconds; center drift 0.0084 mm | Pass |
| E | Right rotation 91.683 degrees over 3.215 seconds; center drift 0.0139 mm | Pass |
| Space | Zero Twist observed within 3.4 ms; wheels settled below 0.000019 rad/s | Pass |
| Input ceases | Zero Twist observed after 0.646 s from the test event; wheels settled below 0.000061 rad/s | Pass |
| Ctrl+C | Real terminal SIGINT; zero Twist within 47.2 ms; exit code 0; terminal restored; wheels below 0.000028 rad/s | Pass |

The input-cessation event was recorded after a 60 ms pause for the key writer
to finish its last iteration, so its measured delay is shorter than the
configured 0.7 seconds since the last received key. Timeout checks run at least
every 50 ms. Stop latencies above measure command publication/receipt; wheel
settling was checked separately, not assumed instantaneous.

All motion comparisons passed 0.15 m translation and 0.08 rad heading-error
limits. Actual translation errors were below 0.13 mm, and rotation errors below
0.43 degrees. All six active command labels, Space, timeout and shutdown
messages were confirmed in the terminal output. Raw measurements are in
[phase4.json](phase4.json).

## Files

Added `rover_sim/teleop.py`, `scripts/rover_teleop`, `config/teleop.yaml`,
`test/test_teleop.py`, `scripts/validate_teleop.py`, `docs/teleop.md`,
`docs/validation/phase4.md` and `docs/validation/phase4.json`.
Modified `CMakeLists.txt` and `README.md`.

Terminal key release cannot be observed directly. Holding a key depends on
desktop auto-repeat; without repeat each tap lasts up to the timeout. Only one
motion key is supported at a time. Steering transitions retain the existing
alignment delay, physical braking takes time, and multiple `/cmd_vel` sources
are not arbitrated. See [teleop usage and configuration](../teleop.md).
