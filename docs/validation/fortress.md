# Fortress backend validation (2026-10-31)

All validators run with `--arg sim:=fortress` on the generated worlds
(`worlds/fortress/*.sdf`, tiled mesh collision, DART physics, GPU LiDAR
through `ros_gz_bridge`). JSON evidence: `docs/validation/fortress/`.

All results below are from the re-run after Fortress became the default
(Bullet collision detector, 2 ms step; reliable point-cloud QoS; 1800/2700
sample LiDAR profiles). Every validator passes. GUI smoke (DISPLAY :0, `world:=flat gui:=true`):
the `Teleop` panel is the only `ignition.msgs.Twist` publisher on gz
`/cmd_vel`, the bridge delivers it on ROS `/cmd_vel`, a 0.4 m/s command
drove the rover 0.91 m, `/gui/follow` is advertised, 0 `[Err]` lines.

| Validator | Result |
| --- | --- |
| `validate_terrain` | passed: deterministic, pad rest drift 0.00 mm / tilt 0.12°, probe median 7.8 mm, RTF 0.987 at rest (0.62 cores), 0.990 with 12 dynamic probe boxes (0.57 cores) |
| `validate_step_climb` | passed: 5/10/15 cm 9/9, 20–40 cm blocked 12/12, 0 tunnelled, 0 flipped |
| `validate_slope` | passed: climbs 5–30° (25° at 0.31 m/s, 30° at 0.11 m/s), 35/40° stall; brake holds ≤ 0.3 mm on 5–25°, 23 mm at 30° |
| `validate_suspension` | passed: 4/4 contact 100 % (bump, settling 0.20 s) and 99.0 % (rock field, never below 3) |
| `validate_lidar3d` (lite/standard/high/dense, Fortress profiles 16×1800 / 16×2700 / 32×1800 / 64×1800) | **passed**: 10.0 Hz on all four, steep-beam σ 4.8 / 3.1 / 3.4 / 3.5 cm, box face ≤ 1 mm, 0 self-hits, RTF 0.988 / 0.987 / 0.989 / 0.991 |
| `validate_controller` | passed 7/7: cross-track RMS 2.0 mm (circle) / 1.5 mm (figure-eight), odometry drift 0.12 %, modes, slip warning 1.3 s, watchdog stop 0.74 s, reset resume |
| Mars RTF at rest (`measure_performance`) | 0.984 without LiDAR (0.65 cores), 0.990 with the dense GPU LiDAR (1.12 cores) |
| `validate_traverse` | passed: 36/36 waypoints, 111 m at 0.57 m/s, max tilt 20.1°, 0 stuck, RTF 0.990 (1.13 cores), odometry drift 2.9 % / 8.6° |

Earlier run (ODE detector, 1 ms, 1024-sample profiles): RTF 0.686 with probe
boxes, LiDAR σ 19/9/6/6 cm and 7.6 Hz received on `dense` (a best-effort
subscriber drop, not the sensor); these were the "errors" fixed by the
settings above.

Side-by-side with Classic: [fortress.md](../fortress.md#side-by-side-results).
