# rover_sim — Mars rover simulation (ROS 2 Humble, Gazebo Classic 11)

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

Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11.x (`gazebo_ros_pkgs`), colcon,
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
`share/rover_sim/generated/`. UNIT_TEST_COUNT

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
| `world` | `mars` | `mars`, `flat`, `step_test`, `slope_test`, `boulder_garden`, `desert` (legacy) |
| `seed` | `7` | terrain seed; other seeds are generated once into `~/.cache/rover_sim` |
| `resolution` | `513` | heightmap pixels per side (`513` or `1025`) |
| `rock_density` | `medium` | `low` (≈300 rocks), `medium` (≈700), `high` (≈1500) |
| `gravity` | `3.71` | m/s² (use `9.81` for Earth tests) |
| `shadows` | `false` | cast shadows (GPU cost) |
| `gui`, `rviz` | `true`, `false` | Gazebo client, RViz (`rviz/rover_mars.rviz`) |
| `lidar`, `lidar_profile` | `true`, `lite` | `lite` 16×450, `standard` 16×900, `high` 32×1024 at 10 Hz |
| `lidar_gpu` | `auto` | `true` uses `gpu_ray` (needs OpenGL); `auto` stays on the CPU sensor unless `lidar_prefer_gpu:=true` with a display and GUI |
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

RESULTS_TABLE

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
`mars_lidar.md`, `mars_controller.md`, `mars_traverse.md`.

## Documentation

`docs/terrain.md` (generator, Gazebo heightmap findings, rocks, worlds),
`docs/mechanics.md` (limits derivation and measurements),
`docs/suspension.md`, `docs/controller.md`, `docs/lidar.md`,
`docs/hardening.md` (watchdog/reset/shutdown), `docs/plan.md` (overhaul plan
and TODO), `THIRD_PARTY_NOTICES.md`. Older `docs/validation/phase*.md`
reports describe the prototype this overhaul replaced.

## Limitations

LIMITATIONS
