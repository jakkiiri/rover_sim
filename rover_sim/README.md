# rover_sim

Initial rover simulation prototype: a Mars terrain world (default) and a
desert world with three hills, a four-module proxy rover, separate keyboard
teleop, a Gazebo Classic 3D ray LiDAR publishing `/points_raw` and `/points`
(with a selectable 2D `/scan` mode), a stereo camera producing disparity, a
depth image and a 3D point cloud under `/stereo/`, and optional robot-local
RViz visualization. Milestone assessment: **PASS WITH LIMITATIONS**.

## Required environment and dependencies

Use Ubuntu 22.04, ROS 2 Humble, Gazebo **Classic 11.x** and Bash. Validation
used Gazebo Classic 11.10.2. Integration uses `gazebo_ros_pkgs`; modern Gazebo
(`gz sim`) and `ros_gz` are not part of this package.

On a clean machine, first follow the official
[ROS 2 Humble Ubuntu installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
to configure a UTF-8 locale, enable Ubuntu Universe, configure the ROS apt
repository, and apply its Ubuntu update prerequisites. Then install the ROS
desktop, Classic integration, build tools and test dependencies:

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake \
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml python3-numpy \
  ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev \
  ros-humble-xacro
```

The desktop installation includes RViz. The Classic packages provide the
simulator, development libraries and `gazebo_plugins` ray plugin. The rosdep
step below installs the remaining dependencies declared in [package.xml](package.xml).

## Fresh checkout

Obtain this project's Git clone URL from the team repository or maintainer.
Enter that URL at the prompt below. Clone the package repository itself,
rather than its upstream model repository:

```bash
mkdir -p ~/rover_ws/src
read -r -p "rover_sim repository clone URL: " rover_repo_url
git clone "$rover_repo_url" ~/rover_ws/src/rover_sim
test -f ~/rover_ws/src/rover_sim/rover_sim/package.xml
```

The repository holds two packages: this one in `rover_sim/` and the A*
planner in `rover_planner/`. The expected layout is
`~/rover_ws/src/rover_sim/rover_sim/package.xml`, with `urdf/`,
`worlds/`, `launch/`, `rover_sim/` and the other package directories alongside
it. The upstream model components are vendored under `urdf/upstream/`; a
second upstream clone or model download is not required.

Initialize rosdep once per machine, refresh its index as your normal user,
and install this checkout's dependencies:

```bash
source /opt/ros/humble/setup.bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
cd ~/rover_ws
rosdep install --from-paths src/rover_sim --ignore-src --rosdistro humble -y
```

## Build and test

Build from the real workspace, then source its installed environment:

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim
source ~/rover_ws/install/setup.bash
colcon test --packages-select rover_sim --event-handlers console_direct+
colcon test-result --verbose
```

The current suite contains **322 unit tests**: 22 kinematics, 22 teleop,
73 controller, 14 2D LiDAR, 29 3D LiDAR and 162 stereo tests. Unit
tests are separate from simulation validation: the **63 recorded integration
checks** in the [stereo validation report](docs/validation/stereo.md) (the 44
checks of the [Phase 5 report](docs/validation/phase5.md), re-run with stereo
on and the 2D LiDAR, plus 19 stereo checks), and the 3D LiDAR checks in the
[3D LiDAR guide](docs/lidar3d.md).

Colcon generates `build/`, `install/` and `log/` under `~/rover_ws`; do not
edit those directories manually. Rebuild after source changes. Source the
ROS and workspace setup scripts in each new terminal before running ROS commands.

## Run the simulation and RViz

Normal Gazebo Classic launch, with LiDAR enabled and RViz off:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_sim rover_desert.launch.py
```

To start Gazebo and RViz together, use this launch instead of the previous
one; do not start a second simulation for visualization:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_sim rover_desert.launch.py rviz:=true
```

Use `gui:=false` for a headless Gazebo server. RViz uses simulation time,
Fixed Frame `base_link` and only the current LiDAR cloud. The LiDAR is 3D by
default: the `airy` profile, a RoboSense Airy-style hemispherical dome
mounted upside down (48 beams from the horizon to straight down, 900 samples,
10 Hz, CPU) on a mast 0.40 m above the chassis; returns from the rover's own
body are cropped out. It runs the Mars world at about 0.36 × real time;
`lidar_samples:=450` gives about 0.54. `lidar_mode:=2d` restores the original
`/scan` sensor and viewer, `lidar_profile:=lite|standard|high|dense` picks a
16-64 beam spinning LiDAR centred on the horizon (`lite` runs at about 0.68), and `lidar:=false` omits the LiDAR. `stereo:=false` omits the stereo
camera; the LiDAR and stereo switches are independent. With stereo on (the
default), `rviz:=true` also shows the stereo point cloud, left image and depth
image. For profiles, a separate 3D RViz instance and reproducible validation,
see [the 3D LiDAR guide](docs/lidar3d.md); [the 2D guide](docs/lidar.md)
retains the Snap IDE GUI workaround.

Stop the launch with Ctrl+C. For Gazebo rendering problems, launch with
`gui:=false` and open its client in another sourced terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 gzclient --verbose
```

## Worlds: Mars (default) and desert

`rover_desert.launch.py` loads the Mars world by default. `world:=desert`
loads the original flat desert with three hills, and a path to a `.world`
file loads that file. The validation commands in `docs/` use `world:=desert`,
which is the world they were measured in.

The Mars world (`worlds/mars.world`, 64 × 64 m, Earth gravity, ground
friction 0.8 as in the desert) is generated by
`scripts/generate_mars_world.py` with a fixed seed. Running the script again
rebuilds the same terrain mesh, rocks, shrubs and `worlds/mars_layout.json`.
That layout file lists every object and terrain feature, and validation
scripts use it as ground truth. The world has:
- a flat spawn area at the origin;
- a hill at (12, 3), 1.5 m high, with flanks up to 15° (climbable);
- a mesa at (3, 15), 1.4 m high, with 35° flanks (not climbable);
- a crater at (-13, -7), 1.5 m deep, with 32° walls; its only way in is a
  12° gully cut through the east wall;
- dunes south of y = -12;
- 60 pebbles the rover can drive over, plus 45 rocks, 8 boulders and 22
  shrubs; 57 of these objects are too tall to drive over.

The terrain is a COLLADA mesh, not a Gazebo heightmap: an ODE heightmap
together with the 720-ray LiDAR stalled the simulation. Measured with
`rover_planner`'s `slope_test`, the rover climbs and crosses slopes up to
38° and slides at 39°, and turning in place drifts more than 0.3 m above 36°.

## Stereo camera

The stereo camera starts with the normal launch. Its outputs are:
- `/stereo/left/image_raw` and `/stereo/right/image_raw` with their `camera_info`;
- `/stereo/disparity`;
- `/stereo/depth/image_raw` (metres; NaN where there is no depth);
- `/stereo/points2`.

Check that they are flowing, in another sourced terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 topic hz /stereo/points2
```

Camera settings are launch arguments, for example
`ros2 launch rover_sim rover_desert.launch.py stereo_rate:=10 stereo_baseline:=0.2`.
The defaults are 480 × 360 at 15 Hz with a 0.12 m baseline and a 90° field of
view. Depth is usable from about 0.4 to 5 m on textured surfaces. Design,
parameters, the LiDAR keep-out rules and the validation commands are in the
[stereo guide](docs/stereo.md).

## Ground-truth pose and A* path planning

The rover's URDF also loads Gazebo's `libgazebo_ros_p3d.so`, which publishes
the rover's exact pose in Gazebo's world frame as `nav_msgs/Odometry` on
`/ground_truth/odom` (50 Hz). It adds no TF, sensor, mass or collision; turn it
off with the xacro argument `ground_truth:=false`. The separate
[`rover_planner`](../rover_planner/README.md) package uses it, together with
`/stereo/points2`, to plan A* paths around obstacles and, optionally, drive
along them:

```bash
ros2 launch rover_sim rover_desert.launch.py                       # terminal 1
ros2 launch rover_planner astar_planner.launch.py follow:=true     # terminal 2, opens RViz
```

## Keyboard controls

In a separate interactive terminal while the simulation is running:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 run rover_sim rover_teleop
```

Keep this terminal focused. W/S drive forward/backward, A/D rotate left/right,
Space stops, and Ctrl+C stops and exits. Defaults are 0.4 m/s longitudinal
speed and 0.5 rad/s yaw. Motion expires after 0.7 seconds without a key event.
See [teleop configuration and terminal behavior](docs/teleop.md).

`/cmd_vel` uses `geometry_msgs/msg/Twist`: `linear.x` is forward and
`angular.z` is counterclockwise yaw in the rover frame. Teleop always publishes
`linear.y=0`; the controller clamps finite incoming `linear.y` to zero before
kinematics. Pure lateral input stops drive, while nonfinite planar input stops
all axes. Configuration is in [swerve.yaml](config/swerve.yaml) and
[teleop.yaml](config/teleop.yaml). Module geometry and joint signs come from
the expanded URDF.

## Known Prototype Limitations

- Motion validation focuses on flat ground and low speeds; slope mobility and
  slope parking are not established. See [control limits](docs/swerve.md).
- Terrain is rigid, and the centered 2 cm tire collision band approximates a
  rounded tread rather than realistic sand/tire interaction. See
  [model geometry](docs/swerve.md).
- Steering alignment transitions can pause drive; steering and independent
  wheel ramps can cause transient drift. See [swerve behavior](docs/swerve.md).
- There is no estimated odometry or world-stable scan accumulation; the only
  pose source is Gazebo ground truth on `/ground_truth/odom`. This package's
  RViz views remain local to `base_link`. See
  [LiDAR visualization limits](docs/lidar3d.md).
- Only one normal command source is supported. The internal
  `/swerve/joint_commands` actuator interface can bypass the no-strafe policy;
  it is not an operator interface. See [command policy](docs/teleop.md).
- Keyboard operation depends on terminal key-repeat; key release is inferred
  through the deadman timeout. See [terminal behavior](docs/teleop.md).
- Heavy host load or suspension can interrupt wall-time heartbeats and trigger
  a watchdog stop. Timing resilience beyond the measured runs is not established.
  See [clock semantics](docs/hardening.md) and
  [observed performance and interruptions](docs/validation/phase5.md) and
  [3D sensor performance and loaded-run failures](docs/validation/lidar3d.md).
- LiDAR is an instantaneous idealized multi-layer ray scan with simplified
  noise, dropout and intensity. Dust, reflectivity and spinning-beam distortion
  are not modeled; per-point time is synthetic. See [sensor limits](docs/lidar3d.md).
- Stereo depth is reliable only on textured surfaces, at about 0.4–5 m. The
  plain-coloured hills and the sky get (almost) no depth, and small objects lose
  their edges. Frames larger than Fast DDS's 512 KiB shared-memory segment
  (for example 640 × 480) delayed the controller's reset handshake in testing,
  and the launch warns about them. The stereo processing uses OpenCV SGBM in
  Python, not `stereo_image_proc`. See [stereo limitations](docs/stereo.md#limitations).
- GUI behavior depends on the desktop environment. Snap IDE GTK paths may need
  the documented command-local cleanup; Gazebo rendering may need the software
  client command above. See [GUI workaround](docs/lidar.md).

## Architecture, validation and provenance

See [swerve architecture and equations](docs/swerve.md),
[feedback, watchdog, reset and shutdown semantics](docs/hardening.md),
[the stereo camera guide](docs/stereo.md),
[the current validation report](docs/validation/stereo.md),
[3D LiDAR validation](docs/lidar3d.md) and the
[Phase 5 LiDAR validation](docs/validation/phase5.md). The default actuator
watchdog is 0.3 wall seconds, configurable through `actuator_watchdog` at launch.
Robot_state_publisher owns fixed robot transforms, including
`base_link -> lidar_link`; `base_root` is a coincident dummy root.

[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) records the exact upstream
commit, Apache-2.0 model portions, inherited BSD helpers and local modifications.
Older dated [Phase 3](docs/validation/phase3.md),
[Phase 4](docs/validation/phase4.md), [Phase 4.5](docs/validation/phase45.md)
and [reliability](docs/validation/reliability.md) reports preserve historical
behavior and test counts. Historical Phase 2 [flat](docs/validation/flat.png)
and [slope](docs/validation/slope.png) screenshots used stronger passive braking
and do not establish current slope behavior.
