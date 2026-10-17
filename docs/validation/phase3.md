# Phase 3 validation results

Validated on 2026-10-10 with ROS 2 Humble and Gazebo Classic 11.10.2.
The normal workspace build and launch succeeded. Colcon ran 22 kinematics
unit tests with no failures (its aggregate count also includes the CTest wrapper).
All eight scripted Gazebo cases passed. Raw joint feedback, signed targets,
poses and synchronized simulation durations are in [phase3_motion.json](phase3_motion.json).

| Case | Duration (sim s) | Body-frame displacement x, y (m) | Yaw change (deg) | Heading error (deg) | Result |
| --- | --- | --- | --- | --- | --- |
| forward | 4.181 | 1.672400, -0.000050 | 0.0009 | 0.0009 | PASS |
| backward | 4.341 | -1.736400, 0.000017 | -0.0088 | 0.0088 | PASS |
| left | 4.381 | 0.000240, 1.533351 | -0.0185 | 0.0185 | PASS |
| right | 4.399 | -0.000087, -1.539650 | -0.0103 | 0.0103 | PASS |
| rotate | 4.456 | 0.000002, 0.000013 | 126.9930 | 0.6620 | PASS |
| forward_yaw | 4.478 | 0.978235, 0.772718 | 76.8917 | 0.0794 | PASS |
| diagonal | 4.355 | 1.306472, 1.306523 | 0.0062 | 0.0062 | PASS |
| zero | 4.052 | 0.000008, 0.000008 | 0.0000 | 0.0000 | PASS |

All cases had correct measured wheel-speed signs and steering alignment.
Straight translation and strafe hold heading without a heading controller.
Rotation remains approximately centered; the measured center displacement was
0.013 mm during the recorded steady-motion interval. These very small numbers
reflect this idealized simulator contact model, not expected hardware accuracy.
The rotation yaw-rate error is approximately 0.5 percent. Forward+yaw follows
the expected arc. Diagonal translation has equal body X/Y displacement. Zero
holds the steering targets, with approximately 0.011 mm measured translation.

The script measures after initial steering alignment and acceleration. It does
not claim that a chassis has no transient motion while modules steer under load.
Each measurement pairs pose and timestamp from the same Gazebo Pose_V packet;
ROS-clock/query timing is not mixed into the integrated reference.

The eight-case run uses 0.15 m translation, 0.08 rad heading and 0.10 rad steering
error thresholds. The final model keeps visible tires at 0.12 m width but uses
a 0.02 m centered collision band; the initial broad cylindrical collision
produced about 6 percent yaw-rate error during rotation. This is a documented
contact approximation, not a fitted kinematics scale factor.

Visual evidence: [strafe](phase3-strafe.png), [rotation](phase3-rotate.png).
The screenshots hold the orientations after very small velocity commands.

No Gazebo warnings appeared in the final headless launch. The software-rendered
GUI reported ignored XDG_SESSION_TYPE=wayland and Event.cc connection warnings.
The unit test run emitted a warning from the installed Xacro Python module about
Python 3.11 private-variable behavior; tests passed on Ubuntu's Python 3.10.

Limits: low-speed flat-ground tests only; no slope-driving claim, sensors,
odometry, localization, teleop or hardware driver. Steering alignment gates
drive, and per-wheel acceleration limits can alter transient velocity ratios.

Command-timeout check also passed: wheel rates reached 1.111 rad/s
and fell below 0.000009 rad/s after 1.5 wall-clock seconds without cmd_vel.

## Source files changed

Added:
- `rover_sim/__init__.py`, `rover_sim/kinematics.py`, `rover_sim/controller.py`
- `scripts/swerve_controller`, `scripts/validate_swerve.py`
- `src/swerve_gazebo_plugin.cpp`
- `config/swerve.yaml`
- `test/test_kinematics.py`
- `docs/swerve.md`
- `docs/validation/phase3.md`, `phase3_motion.json`, `phase3-strafe.png`, `phase3-rotate.png`

Modified:
- `CMakeLists.txt`, `package.xml`
- `launch/rover_desert.launch.py`
- `urdf/proxy_rover.urdf.xacro`
- `urdf/upstream/wheel_steered.urdf.xacro`
- `THIRD_PARTY_NOTICES.md`, `README.md`

`worlds/desert.world` and the other upstream macros are unchanged in Phase 3.
Build/install/log files were generated only by colcon in ~/rover_ws.
