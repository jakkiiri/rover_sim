# Fortress backend validation (2026-10-31)

All validators run with `--arg sim:=fortress` on the generated worlds
(`worlds/fortress/*.sdf`, tiled mesh collision, DART physics, GPU LiDAR
through `ros_gz_bridge`). JSON evidence: `docs/validation/fortress/`.

| Validator | Result |
| --- | --- |
| `validate_terrain` | passed: deterministic (92 files incl. Fortress worlds), pad rest drift 0.00 mm, probe median 7.4 mm, RTF 0.970 at rest (0.89 cores); with 12 dynamic probe boxes 0.686 (informational) |
| `validate_step_climb` | passed: 5/10/15 cm 9/9, 20–40 cm blocked 12/12, 0 tunnelled, 0 flipped |
| `validate_slope` | passed: climbs 5–30° (30° at 0.12 m/s), 35/40° stall; brake holds ≤ 0.3 mm on 5–25°, 25 mm at 30° (the first run slid 3.3 m at 30° because the hold only engaged below 0.5 rad/s; it now engages immediately) |
| `validate_suspension` | passed: 4/4 contact 100 % (bump, settling 0.18 s) and 98.7 % (rock field) |
| `validate_lidar3d` (lite/standard/high/dense) | **not passed on the range-σ criterion**: beam layout exact, box face ≤ 1 mm, 0 self-hits, rates 10/10/9.5/7.6 Hz, but steep-beam σ 19/9/6/6 cm (cube-map resolution follows the horizontal sample count; 1800 samples give 2.5 cm on the −15° beam) |
| `validate_controller` | passed 7/7: cross-track RMS 1.7 mm, odometry drift 0.08 % / 0.08°, modes, slip warning 1.4 s, watchdog stop 0.95 s, reset resume |
| `validate_traverse` | passed: 36/36 waypoints, 111 m at 0.57 m/s, max tilt 20.1°, 0 stuck, RTF 0.969 (1.51 cores), odometry drift 2.6 % / 7.7° |
| Mars RTF at rest | 0.982 without LiDAR (0.89 cores), 0.962 with dense GPU LiDAR (1.46 cores) |

Side-by-side with Classic: [fortress.md](../fortress.md#side-by-side-results).
