# rover_sim — Mars rover simulation (ROS 2 Humble, Gazebo Classic 11 / Gazebo Fortress)

A 53 kg four-module swerve rover with passive sprung suspension on a
procedurally generated 120 × 120 m Martian heightmap with rocks, boulders
and craters, a VLP-16-class 3-D LiDAR and IMU, wheel odometry and a drive
controller with drive modes, tilt limits, traction control and stall
detection. Mechanical limits (step and slope) emerge from mass, wheel
radius, friction and motor torque through ODE physics and are measured, not
scripted. Status: see the **Measured results** table and **Limitations**
below; every number there comes from a validator run recorded under
`docs/validation/`.

## Requirements

Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.x (`gazebo_ros_pkgs`) and/or Gazebo Fortress (`ros-humble-ros-gz`, `ignition-fortress`; optional, detected at build time), colcon,
Python 3.10 with numpy, scipy and Pillow (`rosdep` installs them). No
Gazebo Sim / Ignition. Optional: `ros-humble-pointcloud-to-laserscan` for
the 2-D `/scan` (a built-in fallback exists), `ros-humble-joy` for the
gamepad teleop.

```bash
sudo apt install -y git build-essential cmake python3-colcon-common-extensions \
  python3-rosdep python3-pytest ros-humble-desktop ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-dev ros-humble-xacro
mkdir -p ~/rover_ws/src && cd ~/rover_ws/src && git clone <this repository> rover_sim
cd ~/rover_ws && source /opt/ros/humble/setup.bash
rosdep install --from-paths src/rover_sim --ignore-src --rosdistro humble -y
```

## Build and test

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim        # also generates the terrain assets (~7 s)
source install/setup.bash
colcon test --packages-select rover_sim --event-handlers console_direct+
colcon test-result --verbose
```

The build runs `scripts/generate_terrain.py` for the default seed and
installs the heightmap DEM, textures, rock meshes and worlds under
`share/rover_sim/generated/`. The unit suite has **276 pytest tests** (164 controller, 36 kinematics/odometry, 35 teleop, 22 LiDAR/IMU/RViz, 19 terrain, up from 131 before the overhaul); `colcon test` reports **281** including the five CTest wrappers.

## Run

```bash
source /opt/ros/humble/setup.bash && source ~/rover_ws/install/setup.bash
ros2 launch rover_sim rover_mars.launch.py world:=mars rviz:=true      # Gazebo GUI + RViz
ros2 launch rover_sim rover_mars.launch.py world:=mars gui:=false      # headless server
```

In a second terminal:

```bash
source /opt/ros/humble/setup.bash && source ~/rover_ws/install/setup.bash
ros2 run rover_sim rover_teleop            # W/S A/D, Q/E strafe (crab/auto), +/- speed, M mode, X e-stop, Space, Ctrl+C
ros2 run rover_sim rover_teleop_joy        # gamepad via the joy package
ros2 topic pub -1 /swerve/set_drive_mode std_msgs/String "{data: crab}"
```

### Launch arguments (`rover_mars.launch.py`)

| Argument | Default | Meaning |
| --- | --- | --- |
| `sim` | `classic` | `classic` (Gazebo 11, ODE) or `fortress` (gz-sim 6, DART via `ros_gz`); see `docs/fortress.md` |
| `world` | `mars` | `mars`, `flat`, `step_test`, `slope_test`, `boulder_garden`, `desert` (legacy, classic only) |
| `seed` | `7` | terrain seed; other seeds are generated once into `~/.cache/rover_sim` |
| `resolution` | `513` | heightmap pixels per side (`513` or `1025`) |
| `rock_density` | `medium` | `low` (≈300 rocks), `medium` (≈700), `high` (≈1500) |
| `gravity` | `3.71` | m/s² (use `9.81` for Earth tests) |
| `shadows` | `false` | cast shadows (GPU cost) |
| `gui`, `rviz` | `true`, `false` | Gazebo client, RViz (`rviz/rover_mars.rviz`) |
| `lidar`, `lidar_profile` | `true`, `auto` | `auto` = `dense` (64×1024) on `gpu_ray`, `lite` (16×450) on the CPU sensor; also `standard` 16×900, `high` 32×1024; all 10 Hz |
| `lidar_gpu` | `auto` | `auto` = `gpu_ray` on `world:=mars` when a display exists (the CPU ray sensor hangs on the heightmap), CPU `ray` elsewhere unless `lidar_prefer_gpu:=true`; `true`/`false` force it |
| `lidar_channels`, `lidar_samples`, `lidar_vfov_deg`, `lidar_max_range` | profile values, `100.0` | override the profile's beam layout and range |
| `scan_2d` | `false` | also publish a planar `/scan` |
| `drive_mode` | `ackermann` | `ackermann`, `crab`, `point_turn`, `auto` |
| `x`, `y`, `z`, `yaw` | `0 0 auto 0` | spawn pose; `z:=auto` samples the heightmap at the spawn pad |
| `wheel_collision` | `sphere` | `sphere`, `multisphere`, `cylinder`, `band` |
| `wheel_stall_torque`, `wheel_no_load_speed`, `wheel_mu` | `8.0`, `10.0`, `0.8` | motor and tyre parameters |
| `mast_height` | `0.60` | LiDAR mast height above the deck |
| `actuator_watchdog` | `0.3` | plugin wall-clock watchdog |

`rover_desert.launch.py` is kept as a compatibility wrapper (legacy desert
world, Earth gravity).

## Topics

`/cmd_vel` (in), `/joint_states`, `/odom` + TF `odom→base_link`,
`/swerve/diagnostics`, `/swerve/drive_mode`, `/imu/data`, `/points_raw`
(sensor), `/points` (post-processed: ring/time/dropout/noise), `/scan`
(optional), `/ground_truth/odom` (validation only),
`/swerve/actuator_diagnostics`. Details: `docs/controller.md`,
`docs/lidar.md`.

## Measured results (2026-10-17, headless, this machine: i7-13700H, Intel iGPU)

Every number below is from a recorded validator run (`docs/validation/mars_*.json`).

| Area | Measured |
| --- | --- |
| Terrain visual | Gazebo's heightmap shader fails on this Mesa driver (terrain renders black, `docs/validation/terrain_render_heightmap_shader_failure.png`); the textured mesh visual renders correctly (`terrain_render_mesh.png`) |
| Terrain (`validate_terrain`) | deterministic (46 files, identical hashes); 15 m relief, mean slope 15.9°, 70 % below 20°, 6.3 % above 35°; pad drift 0.26 mm/10 s; DEM probe error median 7.8 mm; RTF 0.998 at rest, gzserver 0.56 cores |
| Step climb (`validate_step_climb`, 0.8 m/s, 8 N m, 3 approach angles) | 5 / 10 / 15 cm climbed 9/9; 20 / 25 / 30 / 40 cm blocked 12/12, 0 tunnelled, 0 flipped (30 cm never passes). At 0.5 m/s the limit is 10 cm. |
| Slopes (`validate_slope`, μ = 0.65) | climbs 5–20° at 0.40 m/s, 25° at 0.21 m/s; 30 / 35 / 40° stall at the ramp foot without sliding or flipping; parking brake holds 5–25° with ≤ 9 mm drift in 5 s |
| Suspension (`validate_suspension`) | 12 cm bump: 4/4 wheels in contact 100 %, settling 0.16 s; 5–9 cm rock field: 4/4 contact 98.8 %, mean speed 0.49 m/s |
| Controller (`validate_controller`) | circle / figure-eight cross-track RMS 1.8 / 1.8 mm; crab, point-turn and ackermann modes verified; slip/stall warning 1.4 s after a wall impact; watchdog stop 0.9 s; reset resume 0.5 s |
| Odometry vs ground truth | 0.13 % of distance (1.9 cm over 14 m), yaw error 0.24° (gyro yaw) |
| Hardening / teleop / shutdown | 10/10, 10/10, 2/2 |
| LiDAR CPU `ray` (flat world, at rest) | lite RTF 0.944 (gzserver 0.62 + post-processing 0.41 cores), standard 0.892, high 0.728; range σ 2.0 cm, box face within 1 mm, 0 self-hits. **Unusable on the heightmap world: gzserver hangs.** |
| LiDAR `gpu_ray` (Intel iGPU, headless, corrected labels) | lite / standard / high all pass geometry (σ 2.9 cm, box face within 1 mm, 0 self-hits); RTF 0.999 at every profile on the flat and Mars worlds (gzserver 0.55–0.57 cores). Default on `world:=mars`; `dense` 64×1024 also RTF 0.998 on Mars. |
| Traverse (`validate_traverse`, Mars world, GPU lite LiDAR) | planned 119 m loop: 36/36 waypoints, 111 m in 197 s (mean 0.57 m/s), max tilt 19.8°, 0 stuck events, **RTF 0.999 while driving** (gzserver 0.55 cores), odometry drift 0.60 % / 1.1° |

Baseline before the overhaul: RTF 0.998 at rest with the prototype rover and 2-D scan, 131 unit tests.

## Gazebo Fortress

`sim:=fortress` runs the same rover and validators on Gazebo Fortress (gz-sim 6, DART) through `ros_gz`; measured side by side in `docs/fortress.md`. Classic remains the default backend (identical step outcomes, higher RTF, more accurate GPU LiDAR ranges on this machine).

## Validation scripts

Each launches its own isolated headless simulation and writes JSON:

```bash
ros2 run rover_sim validate_terrain.py --output /tmp/terrain.json
ros2 run rover_sim validate_step_climb.py --output /tmp/step.json
ros2 run rover_sim validate_slope.py --output /tmp/slope.json
ros2 run rover_sim validate_suspension.py --output /tmp/suspension.json
ros2 run rover_sim validate_lidar3d.py --output /tmp/lidar.json --profiles lite standard high
ros2 run rover_sim validate_controller.py --output /tmp/controller.json
ros2 run rover_sim validate_traverse.py --output /tmp/traverse.json
ros2 run rover_sim run_hardening_and_teleop.py --output-dir /tmp/hardening
ROS_DOMAIN_ID=46 ros2 run rover_sim validate_shutdown.py --output /tmp/shutdown.json
ros2 run rover_sim measure_performance.py --launch rover_mars.launch.py --arg world:=mars
```

Reports: `docs/validation/mars_terrain.md`, `mars_mechanics.md`,
`mars_lidar.md`, `mars_controller.md`, `mars_traverse.md`, `mars_clean_checkout.md`.

## Documentation

`docs/terrain.md` (generator, Gazebo heightmap findings, rocks, worlds),
`docs/mechanics.md` (limits derivation and measurements),
`docs/suspension.md`, `docs/controller.md`, `docs/lidar.md`,
`docs/hardening.md` (watchdog/reset/shutdown), `docs/plan.md` (overhaul plan
and TODO), `THIRD_PARTY_NOTICES.md`. Older `docs/validation/phase*.md`
reports describe the prototype this overhaul replaced.

## Limitations

- **Rigid terrain.** ODE cannot model deformable regolith: no sinkage,
  bulldozing or slip-sinkage; traction is a Coulomb friction cone with one μ.
- **Step climbing depends on momentum.** With rigid 0.20 m wheels and
  μ = 0.65 the 15 cm step is climbed only at ≥ 0.8 m/s approach; at 0.5 m/s
  the quasi-static limit of this four-wheel layout is 10 cm. 20 cm and above
  are blocked (30 cm never climbs, tunnels or flips). See `docs/mechanics.md`.
- **Slope limits** come from per-axle load transfer and friction: 25° is
  climbed at reduced speed, 30° stalls (does not slide) because
  tan 30° < μ. On Earth gravity the same motors manage far less.
- **Rock coverage** is far below real landing sites (performance budget);
  rocks under 10 cm have no collision; texture blending is height-only.
- **LiDAR realism** stops at range noise, dropout, ring/time fields and an
  intensity model: no dust, multipath, beam divergence or motion distortion
  within a sweep.
- **Wheel-only stall sensing is impossible when all wheels spin**; the
  IMU-fused discrepancy detects impacts but decays (10 s), so a very long
  stall is kept flagged only through the traction-control state.
- **Single command source**: no `/cmd_vel` multiplexer; teleop key release
  is inferred by a 0.7 s deadman.
- **GPU LiDAR** needs an OpenGL context; on this machine (Intel iGPU) the
  CPU sensor is the default.
- **Wall-clock watchdogs** can trip under heavy host load or suspend
  (documented since the prototype); the actuator watchdog is 0.3 s.
