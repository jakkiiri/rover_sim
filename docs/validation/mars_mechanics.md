# Phase B validation: rover mechanics (2026-10-17)

Headless Gazebo Classic runs on the generated `step_test`, `slope_test` and
`flat` worlds at Mars gravity, launched and torn down by each validator in an
isolated ROS domain / Gazebo master:

```bash
ros2 run rover_sim validate_step_climb.py --output docs/validation/mars_step_climb.json
ros2 run rover_sim validate_slope.py --output docs/validation/mars_slope.json
ros2 run rover_sim validate_suspension.py --output docs/validation/mars_suspension.json
```

Final parameters: 53 kg, wheels R = 0.20 m (sphere collision), track 1.0 m,
wheelbase 0.8 m, ground clearance 0.30 m, μ_wheel 0.8 / μ_ground 0.65,
stall torque 8 N m, no-load speed 10 rad/s, suspension k = 1500 N/m,
c = 110 N s/m, ±0.10 m, parking brake at zero command.

| Validator | Result |
| --- | --- |
| `validate_step_climb` (21 trials) | **passed**: 5/10/15 cm climbed 9/9; 20/25/30/40 cm blocked 12/12, 0 tunnelled, 0 flipped |
| `validate_slope` (8 ramps + 5 hold tests) | **passed**: climbs 5–25°, stalls at 30/35/40°, holds ≤ 8 mm drift on 5–25° |
| `validate_suspension` (bump + rock field) | **passed**: 4/4 contact 100 % (bump) / 98.8 % (rock field), settling 0.16 s |

The full tables, the calibration history (6.5 N m / 0.5 m/s blocked the
15 cm step; momentum, not torque, is what the rear wheels need) and the
analysis are in [mechanics.md](../mechanics.md#measured-thresholds); the
wheel-shape comparison (sphere vs multisphere vs cylinder vs band, all
stable after the strut-inertia fix) is in
[mechanics.md](../mechanics.md#wheel-collision-shape-comparison); the
suspension design comparison is [suspension.md](../suspension.md).

Not met / caveats:

- 15 cm is reliable only with ≥ 0.8 m/s approach momentum; at 0.5 m/s the
  quasi-static limit of this 4-wheel layout is 10 cm (0.5 R). This is a
  physics limit of rigid 0.20 m wheels at μ = 0.65 with the CG at 0.37 m,
  not a controller rule.
- 25° is climbed at 0.21 m/s instead of the commanded 0.4 m/s (motor
  derating and traction control); 30° stalls rather than slides because
  tan 30° = 0.58 < μ = 0.65.
- Contact retention is measured through the strut compression, not a
  contact sensor.
