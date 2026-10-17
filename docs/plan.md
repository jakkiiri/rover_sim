# Mars overhaul plan memo

Date: 2026-10-17. Branch `feat/mars-overhaul`. Target: Ubuntu 22.04, ROS 2
Humble, Gazebo Classic 11.10.2, ODE. Host: i7-13700H (20 threads), 15 GB RAM,
Intel Iris Xe iGPU only (no NVIDIA), X display `:0` available under a Wayland
session, no `sudo`, no `xvfb`. Optional apt packages
`ros-humble-pointcloud-to-laserscan` and `ros-humble-velodyne-gazebo-plugins`
are *not* installed and cannot be installed from this session (no sudo).

## What exists (baseline, measured 2026-10-17)

| Item | Baseline |
| --- | --- |
| World | `worlds/desert.world`: 100x100 m box ground, three buried spheres, g = 9.81 |
| Rover | `urdf/proxy_rover.urdf.xacro`: 20 kg box chassis, 4 rigid swerve modules at (±0.38, ±0.45), R = 0.18 m, 2 cm tread band, 1 kg wheels |
| Plugin | `src/swerve_gazebo_plugin.cpp`: steering PD, wheel PI, wall watchdog, reset epochs, strict 8-joint check, `/joint_states` 50 Hz |
| Controller | `rover_sim/control_state.py`, `kinematics.py`, `controller.py`: no strafe (vy clamped), hard steer-then-drive gate, per-wheel ramps, no odometry |
| Teleop | W/S/A/D, Space, Ctrl+C, deadman 0.7 s |
| LiDAR | 2D CPU `ray`, 720 samples, `/scan`; RViz fixed frame `base_link`, decay 0 |
| Tests | 131 pytest (22 kinematics, 22 teleop, 73 controller, 14 LiDAR); colcon reports 135 incl. 4 wrappers; all pass |
| Headless RTF | 0.998 (two 10 s windows); gzserver 0.65 cores, 355 MB RSS |

Contradictions with the prompt (all deliberate changes, tests updated as noted):

- Strafing is forbidden by `ControlState.on_command` and by teleop tests;
  the prompt makes `crab` an allowed (default-off) drive mode.
- Drive is gated on steering alignment (`alignment_tolerance`); the prompt
  wants continuous cos-scaling.
- Modules must sit directly on `base_link` (`modules_from_urdf` rejects
  anything else); suspension requires an intermediate link chain.
- The plugin demands exactly eight joints; suspension adds four.
- `test_lidar.py` pins a single-plane 720-sample `/scan` and `base_link`
  fixed frame; Phase D replaces both.
- Gravity, world size and the LiDAR topic/type all change.

## Decisions taken up front

1. **Suspension**: independent sprung/damped vertical travel per module
   (prismatic joint, ODE implicit spring-damper). Rationale in
   `docs/suspension.md`: it keeps every kingpin axis vertical (a rocker
   would camber a steered wheel), keeps module positions constant for the
   swerve kinematics and odometry, needs no gearbox/differential constraint,
   and is the cheapest and most stable option in ODE.
2. **Wheel collision**: candidates `sphere`, `multisphere`, `cylinder`
   selectable by xacro arg; the one that is jitter-free on heightmap and step
   tests wins (measured in Phase B).
3. **LiDAR**: `libgazebo_ros_ray_sensor.so` (PointCloud2) with a numpy
   post-processing node for `ring`, per-point time, dropout and
   range-dependent noise, because `velodyne_gazebo_plugins` is unavailable.
   `gpu_ray` is tried on the Intel iGPU; CPU `ray` is the fallback.
   `/scan` comes from `pointcloud_to_laserscan` when installed, otherwise
   from the post-processing node (documented fallback).
4. **Terrain assets** are generated at build time by CMake from
   `scripts/generate_terrain.py` (deterministic from `seed`), and at launch
   time into a cache directory when a non-default `seed`/`rock_density` is
   requested. Nothing binary is committed.
5. **Ground truth** is published by the actuator plugin itself
   (`/ground_truth/odom`) so slip diagnostics and validation share one source.
6. **Gravity/shadows** are launch-time substitutions into the world file.

## Phases, file ownership, checkpoints

| Phase | Files (owner: lead unless noted) | Checkpoint |
| --- | --- | --- |
| 0 | `rover_sim/validation/*`, `scripts/measure_performance.py`, this memo | baseline numbers above |
| A | `rover_sim/terrain/*`, `scripts/generate_terrain.py`, `worlds/*.world.in`, `worlds/test/*`, `scripts/validate_terrain.py`, `test/test_terrain.py`, `docs/terrain.md` | `validate_terrain` passes, RTF ≥ 0.9 |
| B | `urdf/mars_rover.urdf.xacro`, `urdf/modules/*.xacro`, `src/swerve_gazebo_plugin.cpp`, `scripts/validate_step_climb.py`, `validate_slope.py`, `validate_suspension.py`, `docs/mechanics.md`, `docs/suspension.md` | 15 cm pass / 30 cm fail; 20-25° climb, ≥30-35° fail; 4/4 contact ≥ 95 % |
| D | `urdf/sensors/lidar3d.urdf.xacro`, `imu.urdf.xacro`, `rover_sim/lidar_postprocess.py`, `rviz/rover_mars.rviz`, `scripts/validate_lidar3d.py`, `test/test_lidar.py`, `docs/lidar.md` | profile table with RTF/CPU, self-hits ≈ 0 |
| C | `rover_sim/kinematics.py`, `control_state.py`, `controller.py`, `odometry.py`, `teleop.py`, `teleop_joy.py`, `config/swerve.yaml`, `test/test_*.py`, `scripts/validate_controller.py`, `docs/controller.md` | unit tests green, drift % vs ground truth |
| E | `launch/*`, `package.xml`, `CMakeLists.txt`, `README.md`, `scripts/validate_traverse.py`, `docs/validation/mars_*.md` | clean-checkout build, `colcon test`, traverse |

Shared files (`launch/`, `package.xml`, `CMakeLists.txt`, `README.md`) are
edited only during integration. Subagents, if used, get disjoint file lists
from the table above.

## TODO (kept current)

- [x] Read repo, record baseline, write this memo
- [x] A1 heightmap generator + diagnostics (float32 GeoTIFF DEM after the 16-bit PNG probe failed)
- [x] A2 textures, lighting, gravity
- [x] A3 rock library, scatter, chunked models, density presets
- [x] A4 test worlds (`step_test`, `slope_test`, `boulder_garden`)
- [x] A5 `validate_terrain` (6/6 pass), `docs/terrain.md`, commit d954acc
- [x] B1 chassis + swerve modules + wheel collision candidates (strut inertia fix; sphere/multisphere/cylinder stable)
- [x] B2 suspension + `docs/suspension.md`
- [x] B3 mechanical limits: 5/10 cm climb at any speed, 15 cm needs >= 0.8 m/s + 8 N m, 20-40 cm blocked; slopes climb to 20-25°, stall at 30°+; parking hold added
- [x] B4 plugin: torque-speed model, suspension joints, ground truth, diagnostics, parking hold
- [x] B5 final `validate_step_climb` / `validate_slope` runs, `docs/mechanics.md` measured section, commit ab88041
- [x] D1-D4 LiDAR profiles, IMU, RViz, `validate_lidar3d` 3/3 (CPU), gpu_ray measured and rejected, `docs/lidar.md`, commit
- [x] C1-C8 controller rewrite, odometry, teleop, 274 unit tests, `validate_controller` 7/7, hardening 10/10, teleop 10/10, shutdown 2/2, `docs/controller.md`, commit
- [ ] E1-E3 launch, `validate_traverse`, README, reports, clean-checkout build, final commit


## Paused 2026-10-17 — resume here

State: Phases A–D committed; Phase E in progress on `feat/mars-overhaul`.
Uncommitted-at-pause work is committed as "Phase E (WIP)". All simulations
were stopped cleanly; nothing is running.

What was found right before the pause (see `docs/lidar.md`):
- the CPU `ray` sensor hangs gzserver on the heightmap world (all variants);
- `gpu_ray` works headless with `DISPLAY=:0` but mislabels points one beam
  step low; the correction (`vertical_shift_steps: 1.0`) is implemented and
  unit-tested; launch `auto` now picks `gpu_ray` on `world:=mars`.
- GPU LiDAR validation with the correction had completed `lite` and
  `standard` on the flat world (both passed geometrically; see
  `docs/validation/partial/lidar_gpu_corrected_partial.out`); `high` and the
  traverse had not finished.

Next steps, in order (each command launches its own isolated headless sim):
1. `cd ~/rover_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select rover_sim && source install/setup.bash`
2. `ros2 run rover_sim validate_lidar3d.py --output /tmp/lidar_gpu.json --profiles lite standard high --gpu true`
   → copy to `docs/validation/mars_lidar_gpu.json`, update the GPU rows/table in `docs/lidar.md`
   and `docs/validation/mars_lidar.md` (the current text still describes the uncorrected GPU result).
3. `ros2 run rover_sim validate_traverse.py --output /tmp/traverse.json --snapshots /tmp/snap`
   (uses `world:=mars`, `lidar_gpu:=auto` → GPU lite) → `docs/validation/mars_traverse.json` + `mars_traverse.md`
   (success, max tilt, stuck events, RTF, odometry drift, snapshot PNGs). If RTF < 0.9 while
   driving, try `rock_density:=low` and report both.
4. `ros2 run rover_sim measure_performance.py --launch rover_mars.launch.py --arg world:=mars --arg lidar_profile:=standard`
   and `high` for the Mars-world GPU cost table.
5. README: replace `RESULTS_TABLE` with the measured table (terrain RTF 0.998, steps, slopes,
   suspension, LiDAR per profile, odometry drift 0.13 %, traverse); update the launch-arg table
   (`lidar_gpu auto` semantics, `lidar_channels/samples/vfov_deg/max_range`).
6. Clean-checkout reproduction: `git clone` into a scratch workspace, `colcon build`, `colcon test`,
   one headless smoke run (`ros2 launch rover_sim rover_mars.launch.py gui:=false`), record in `docs/validation/`.
7. Final commit; write the short report (per-phase changes, measured table, test counts
   274→276 unit tests, commands, unmet targets: 15 cm step needs ≥ 0.8 m/s; CPU LiDAR unusable on terrain).
