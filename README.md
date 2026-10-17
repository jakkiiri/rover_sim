# rover_sim

Initial rover simulation prototype: a desert world with three hills, a
four-module proxy rover, separate keyboard teleop, a Gazebo Classic 2D ray
LiDAR publishing `/scan`, and optional robot-local RViz visualization.
Milestone assessment: **PASS WITH LIMITATIONS**.

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
  python3-colcon-common-extensions python3-rosdep python3-pytest python3-yaml \
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
test -f ~/rover_ws/src/rover_sim/package.xml
```

The expected layout is `~/rover_ws/src/rover_sim/package.xml`, with `urdf/`,
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

The current suite contains **131 unit tests**: 22 kinematics, 22 teleop,
73 controller and 14 LiDAR description/RViz tests. Colcon reports 135 results
when its four CTest wrappers are included. Unit tests are separate from the
**44 recorded integration checks** linked in the
[current milestone validation report](docs/validation/phase5.md).

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
Fixed Frame `base_link` and only the current scan. `lidar:=false` omits the
sensor for a performance comparison. For a separate RViz instance, isolated
validation commands and the Snap IDE GUI workaround, see
[the LiDAR guide](docs/lidar.md).

Stop the launch with Ctrl+C. For Gazebo rendering problems, launch with
`gui:=false` and open its client in another sourced terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 gzclient --verbose
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
- There is no odometry or world-stable scan accumulation. RViz remains local
  to `base_link`. See [LiDAR visualization limits](docs/lidar.md).
- Only one normal command source is supported. The internal
  `/swerve/joint_commands` actuator interface can bypass the no-strafe policy;
  it is not an operator interface. See [command policy](docs/teleop.md).
- Keyboard operation depends on terminal key-repeat; key release is inferred
  through the deadman timeout. See [terminal behavior](docs/teleop.md).
- Heavy host load or suspension can interrupt wall-time heartbeats and trigger
  a watchdog stop. Timing resilience beyond the measured runs is not established.
  See [clock semantics](docs/hardening.md) and
  [observed performance and interruptions](docs/validation/phase5.md).
- LiDAR is an instantaneous idealized 2D ray scan with simple Gaussian noise;
  dust, reflectivity and spinning-beam distortion are not modeled. See
  [sensor limitations](docs/lidar.md).
- GUI behavior depends on the desktop environment. Snap IDE GTK paths may need
  the documented command-local cleanup; Gazebo rendering may need the software
  client command above. See [GUI workaround](docs/lidar.md).

## Architecture, validation and provenance

See [swerve architecture and equations](docs/swerve.md),
[feedback, watchdog, reset and shutdown semantics](docs/hardening.md), and
[the current validation index](docs/validation/phase5.md). The default actuator
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
