# Original 2D LiDAR and local RViz

This guide covers the retained `lidar_mode:=2d` compatibility mode. The
operator launch now defaults to [3D LiDAR](lidar3d.md). Phase 5 measurement
results and counts below describe the original 2D milestone.

For a clean machine and checkout, complete the
[README setup, dependency and build steps](../README.md) first. From the
normal workspace, launch Gazebo Classic and optional RViz:

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim
source ~/rover_ws/install/setup.bash
ros2 launch rover_sim rover_desert.launch.py lidar_mode:=2d rviz:=true
```

Since the [stereo camera](stereo.md) was added, it is on by default. With it,
`rviz:=true` opens the combined `rviz/rover_sensors.rviz` (the same LiDAR
display plus stereo). Add `stereo:=false` to get exactly the LiDAR-only setup
and RViz view described below, for example to reproduce the Phase 5
measurements. The LiDAR itself is unchanged.

Gazebo Classic remains the simulator. RViz is optional and defaults off;
`gui:=false rviz:=true` runs a headless Gazebo server with RViz. `lidar:=false`
omits the sensor and its frame for a performance comparison. Teleop remains
separate (`ros2 run rover_sim rover_teleop`): W/S forward/backward, A/D rotate,
Space stop and Ctrl+C stop/exit. The controller continues to clamp `linear.y`
to zero. No odometry, SLAM, navigation or cameras are introduced.

## Description and ROS interface

The local macro in `urdf/sensors/lidar.urdf.xacro` attaches `lidar_link` to
`base_link` through the fixed `lidar_joint`: XYZ `(0, 0, 0.20)` m, RPY
`(0, 0, 0)` radians. The housing is a 0.15 kg cylinder with positive inertia;
the mast reaches the chassis top at Z=0.10 m. The scan origin is at the
housing center. On flat ground its world height is approximately 0.53 m;
the exact height and orientation follow rover suspension/contact pose.

Gazebo's standard fixed-joint reduction lumps sensor mass and geometry into
the chassis and preserves the sensor pose. The original URDF still contains
the dedicated frame. Robot_state_publisher exclusively owns the fixed
`base_link -> lidar_link` transform, alongside existing `base_root -> base_link`
and articulated wheel transforms. The sensor plugin publishes no TF.

| Setting | Value |
| --- | --- |
| Classic sensor | CPU `ray`, one horizontal plane |
| Plugin | `libgazebo_ros_ray_sensor.so` from `gazebo_plugins` |
| ROS topic and type | `/scan`, `sensor_msgs/msg/LaserScan` |
| Header frame | `lidar_link` |
| Update rate | 10 Hz in simulation time |
| Samples | 720 |
| Angles | −π to +π radians, inclusive |
| Angular increment | 2π/719 rad = 0.500695° |
| Range bounds | 0.12–30.0 m |
| Range resolution setting | 0.01 m |
| Noise | Gaussian, mean 0 m, standard deviation 0.01 m |

The plugin XML uses `output_type=sensor_msgs/LaserScan` (the plugin's required
spelling), `frame_name=lidar_link`, and `~/out:=/scan` remapping. Its installed
default publisher QoS is reliable, volatile, depth five. RViz uses a compatible
best-effort, volatile subscription. No ROS/Gazebo bridge is needed.

Headers use Gazebo's simulation timestamp. The Classic plugin treats each
scan as an instantaneous snapshot and publishes `time_increment=0` and
`scan_time=0`; determine actual cadence from successive timestamps. Do not
infer spinning-beam timing or motion distortion from those zero fields.
The inclusive endpoints duplicate the rear direction, and an even sample
count places the nearest forward rays at approximately ±0.25035°.

Finite hits lie within range bounds. Positive infinity represents no return,
and negative infinity represents a hit below minimum range. Those are valid
LaserScan conventions, not invalid numeric feedback. The noise model applies
to finite hits; range resolution is a simulator setting, not a guarantee of
1 cm measurement accuracy.

## Visualization and geometry limits

The installed `rviz/rover_lidar.rviz` uses `base_link` as Fixed Frame and the
launch sets `use_sim_time=true`. RobotModel receives the transient-local
`/robot_description`; LaserScan shows bright cyan/green points against a dark
background. Decay Time is zero, so only the current scan is displayed. The
one-meter grid and robot remain local references, not a world coordinate map.

An existing normal simulation can also be viewed separately. In a new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
rviz2 -d "$(ros2 pkg prefix --share rover_sim)/rviz/rover_lidar.rviz" \
  --ros-args -p use_sim_time:=true
```

As the rover translates or rotates, surrounding returns move relative to its
fixed local model. There is deliberately no `odom`, `map` or world-to-robot
ROS transform. World-stable scan accumulation is unavailable until a later
odometry source owns that moving transform. Avoid adding a static world-to-base
transform to imitate odometry.

This sensor measures one plane attached to the body. Flat ground below the
plane is invisible while the rover is level. Hills produce returns where
their collision surface crosses that plane. Body roll and pitch tilt the
plane, so distant hill ranges must be compared using the full measured rover
pose. These are idealized spherical hills and primitive collision shapes,
not a realistic dust/material/reflectivity simulation.

## Validation and performance

See [Phase 5 measured validation](validation/phase5.md). The installed
`validate_lidar.py` checks the ROS interface, TF, analytical hill/probe ranges,
scan changes after forward/yaw movement, publish rate and real-time factor.
It uses simulator ground truth only for testing; this does not add an odometry
node or a rover sensor. Temporary static probe obstacles are removed afterward.
Run validation only in its dedicated ROS domain/Gazebo master with no other
command publisher, as it moves the rover.

### Reproduce the sensor integration checks

Build and run the 322 unit tests using the [README commands](../README.md).
The commands below reproduce the separate 12 LiDAR integration checks,
including three performance windows; the Phase 5 milestone's 44 integration
checks also include motion, keyboard, hardening and shutdown regressions.
Choose unused domain/master values. These examples use ROS domain 45 and
Classic master port 11355; use the same pair in every participating terminal.
They are independent of the normal operator launch's default domain/master.

Terminal 1: start a dedicated, headless simulation with LiDAR enabled. RViz
and Gazebo GUI are excluded from sensor performance measurement. Wait for
`Successfully spawned entity [proxy_rover]` and the actuator bridge to be ready.

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=45
export GAZEBO_MASTER_URI=http://127.0.0.1:11355
ros2 launch rover_sim rover_desert.launch.py \
  world:=desert gui:=false lidar:=true lidar_mode:=2d rviz:=false actuator_watchdog:=0.3
```

Terminal 2: source the same workspace and select the same isolated domain/master.
Do not run teleop, another validator or another `/cmd_vel` publisher concurrently.
Keep the computer awake and avoid intensive background work during measurement;
the safety watchdog uses wall time even if simulation slows or the host suspends.

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=45
export GAZEBO_MASTER_URI=http://127.0.0.1:11355
ros2 run rover_sim validate_lidar.py \
  --duration 10 --windows 3 --watchdog 0.3 \
  --output ~/rover_ws/src/rover_sim/docs/validation/local_lidar.json
```

The `--output` argument is required. The validator creates the JSON file and
its parent directory as needed, reports individual checks and exits 0 when
all checks pass. The `local_lidar.json` name preserves the committed
`phase5_lidar.json` evidence; rerunning this command replaces only your local
result. It temporarily adds known-distance boxes, drives forward and rotates,
then requests stop and removes its boxes in cleanup. Results include scan
metadata/timestamps, fixed TF ownership, hill/probe geometry, motion-dependent
scan changes, simulation/wall publication rates, RTF and watchdog transitions.
The launch and validator above both use the unchanged 0.3 wall-second actuator
watchdog. Do not increase it to mask a failed timing check.

After the validator finishes, RViz can inspect this same simulation. Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=45
export GAZEBO_MASTER_URI=http://127.0.0.1:11355
rviz2 -d "$(ros2 pkg prefix --share rover_sim)/rviz/rover_lidar.rviz" \
  --ros-args -p use_sim_time:=true
```

Confirm Fixed Frame `base_link`, RobotModel and LaserScan status OK, `/scan`
selected, and zero scan decay. World-stable accumulation is intentionally
unavailable. Stop RViz and the simulation with Ctrl+C in their terminals when
finished. Close these isolated terminals, or unset `ROS_DOMAIN_ID` and
`GAZEBO_MASTER_URI` in each before using the normal default-domain workflow.

### Optional matched sensor-disabled baseline

First close RViz and stop Terminal 1's sensor-enabled simulation. In the same
sourced, isolated Terminal 1, launch the sensor-disabled variant:

```bash
ros2 launch rover_sim rover_desert.launch.py \
  world:=desert gui:=false lidar:=false lidar_mode:=2d rviz:=false actuator_watchdog:=0.3
```

After spawn, in the same sourced, isolated Terminal 2:

```bash
ros2 run rover_sim validate_lidar.py --performance-only \
  --duration 10 --windows 3 --watchdog 0.3 \
  --output ~/rover_ws/src/rover_sim/docs/validation/local_lidar_baseline.json
```

The baseline runs four performance/watchdog checks and does not require `/scan`.
Compare it with `local_lidar.json` under similar host load. Stop this launch
with Ctrl+C after measurement.

### GUI/environment workaround

If RViz launched from the Snap version of VS Code fails with a `libpthread`
symbol error mentioning `/snap/core20`, inherited GTK paths are loading the
Snap's libraries into the system ROS application. Launch from a regular Ubuntu
terminal, or clear those paths for the affected command only.

For RViz in the already sourced, isolated Terminal 3, replace its `rviz2`
command with:

```bash
env -u GTK_PATH -u GTK_EXE_PREFIX -u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE \
  rviz2 -d "$(ros2 pkg prefix --share rover_sim)/rviz/rover_lidar.rviz" \
  --ros-args -p use_sim_time:=true
```

For a normal combined launch in a new terminal, use:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
env -u GTK_PATH -u GTK_EXE_PREFIX -u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE \
  ros2 launch rover_sim rover_desert.launch.py lidar_mode:=2d rviz:=true
```

This does not change shell startup files or ROS/simulator configuration.

The default sensor requests approximately 7,200 rays per simulation second.
Use `lidar:=false` for a matched headless comparison; exclude RViz and Gazebo
GUI from sensor-only measurements. Publish Hz in wall time scales with real-time
factor, while the configured cadence is in simulation time. Sensor cost and
GUI rendering are machine-dependent. If needed, reduce samples or update rate
in the modular sensor description before changing unrelated control timing.
