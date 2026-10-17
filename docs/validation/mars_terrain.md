# Phase A validation: Mars terrain (2026-10-17)

Command (headless, isolated ROS domain / Gazebo master, run by the script):

```bash
ros2 run rover_sim validate_terrain.py --output docs/validation/mars_terrain.json
```

Result: **all 6 checks passed** ([mars_terrain.json](mars_terrain.json)).

| Check | Result |
| --- | --- |
| determinism | generating seed 7 twice: 46 files, identical SHA-256 (DEM, textures, meshes, worlds) |
| manifest | slope stats present; mean 15.9°, 70 % below 20°, 19.4 % above 25°, 11.3 % above 30°, 6.3 % above 35°, max 61°, relief 15.0 m, pad max slope 0.469° |
| world_loads | `mars_medium.world` (691 rocks) spawned the rover 2.1 s after launch readiness |
| spawn_pad_stable | 10 s at rest: drift 0.26 mm, tilt 0.12°, z change -0.09 mm, base_link 0.407 m above the sampled DEM height |
| height_probes | 12 boxes dropped at random points: median abs error 7.8 mm vs the generator's sampled height, 100 % within 5 cm (confirms DEM orientation and metre scale) |
| real_time_factor | median RTF 0.998, gzserver 0.56 cores (2 × 10 s windows), rover at rest, LiDAR off |

Rock counts placed (requested 300 / 700 / 1500): low 295, medium 691, high 1480.

The resolution/density RTF matrix measured with the previous rover is in
[terrain.md](../terrain.md#performance). Driving-load and LiDAR costs are
reported in the later phase reports.
