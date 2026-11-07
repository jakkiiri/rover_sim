# rover_sim workspace

Two ROS 2 Humble packages for the rover simulation, cloned into a colcon
workspace as `~/rover_ws/src/rover_sim`:

| Folder | Package | What it is |
| --- | --- | --- |
| [`rover_sim/`](rover_sim/README.md) | `rover_sim` | Gazebo Classic swerve rover, Mars and desert worlds, stereo camera, 3D LiDAR (RoboSense Airy-style dome by default), teleop |
| [`rover_planner/`](rover_planner/README.md) | `rover_planner` | Slope-aware A* planner on a stereo + LiDAR costmap, path follower, terrain validation |

```bash
mkdir -p ~/rover_ws/src
git clone https://github.com/jakkiiri/rover_sim.git ~/rover_ws/src/rover_sim
cd ~/rover_ws
colcon build --base-paths src/rover_sim
source install/setup.bash
ros2 launch rover_sim rover_desert.launch.py rviz:=false
ros2 launch rover_planner astar_planner.launch.py follow:=true   # second terminal
```

See each package's README for details and tests.
