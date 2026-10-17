# Phase C validation: controller, teleop, hardening, shutdown (2026-10-17)

```bash
ros2 run rover_sim validate_controller.py --output docs/validation/mars_controller.json
ros2 run rover_sim run_hardening_and_teleop.py --output-dir /tmp/hw   # hardening + teleop on one flat-world launch
ROS_DOMAIN_ID=46 ros2 run rover_sim validate_shutdown.py --output docs/validation/mars_shutdown.json
```

## Controller (`validate_controller.py`, flat world, ground truth)

All 7 checks passed ([mars_controller.json](mars_controller.json)); 0.5 m/s, 0.5 rad/s, radius 1 m.

| Check | Result |
| --- | --- |
| circle (1 lap) | cross-track RMS 1.8 mm, max 3.1 mm |
| figure-eight (two opposite loops) | cross-track RMS 1.8 / 0.7 mm, max 3.0 mm |
| wheel odometry vs ground truth (figure-eight, 14.1 m) | position error 1.9 cm = **0.13 %** of distance, yaw error 0.24° (gyro yaw) |
| drive modes | `crab` (0.3 m/s lateral, 4 s): body displacement x 0.010 m, y 1.047 m; `point_turn` spins with < 0.15 m drift; `ackermann` ignores lateral input |
| slip / stall diagnostics (driving into a wall) | first WARN 1.4 s after the impact (9 slip, 0 stall messages in 6 s); per-wheel slip and `stalled` keys present |
| command watchdog (publisher stops) | rover from 0.49 m/s to rest in 0.9 s |
| reset handshake (`/reset_world`) | new epoch 1 step, motion resumed 0.5 s after the reset with the streaming command (0.31 m/s) |

Wheel-only stall sensing cannot see a wall stall when all four wheels spin
equally; the IMU-fused velocity discrepancy (`docs/controller.md`) is what
raises the slip/stall warnings here. With the earlier wheel-only estimate
the same test produced no warning in 6 s.

## Hardening (`validate_hardening.py`, 0.3 s actuator watchdog) — 10/10 passed

| Check | Result |
| --- | --- |
| atomic invalid parameters | rejected: `max_accel must be positive`; values unchanged |
| valid runtime parameters | accepted |
| pause / resume deadman | paused sim advanced 0.00 ms, drive targets zero while paused |
| reset all / models / time while moving | new epoch each time; wheel speed after reset 0.010 / 0.015 / 0.017 rad/s, targets zero |
| slow simulation (RTF 0.020) | 250 wall-time heartbeats, max gap 21 ms < 300 ms, drive maintained |
| actuator watchdog (controller SIGSTOPped) | wheels settled 0.63 s after the stop |
| resume without stale motion | 21 frames after SIGCONT, max wheel speed 0.018 rad/s, new epoch |
| fresh command after watchdog | motion restored |

## Keyboard teleop (`validate_teleop.py`, real pty) — 10/10 passed

| Check | Result |
| --- | --- |
| W / S / A / D | translation error 0.010 m, heading error 0.010 rad (worst of four) |
| Q / E in ackermann mode | stop within 2.0 ms, no lateral command |
| Space / key release / Ctrl+C | zero published after 0.6 ms / 635 ms / 46 ms; residual wheel speed ≤ 0.012 rad/s; terminal restored, exit 0 |
| lateral never published in ackermann | 1011 messages checked |

Before the parking brake, the explicit position hold rocked the rover for
seconds after every stop (residual 0.13–0.2 rad/s) and failed four of these
checks; the ODE velocity-motor brake fixed it.

## Shutdown (`validate_shutdown.py`, no Gazebo) — 2/2 passed

Repeated SIGINT/SIGTERM at the destruction boundary: exit code 0, one
destruction, final drive target 0.0 when initialised; no fabricated
targets before feedback.
