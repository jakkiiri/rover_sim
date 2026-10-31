# Clean-checkout reproduction (2026-10-31)

```bash
mkdir -p /tmp/clean_ws/src && git clone ~/rover_ws/src/rover_sim /tmp/clean_ws/src/rover_sim   # commit a4204fc + Phase E
cd /tmp/clean_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select rover_sim          # 20.7 s wall, includes the terrain generation
source install/setup.bash
colcon test --packages-select rover_sim && colcon test-result --verbose
ros2 launch rover_sim rover_mars.launch.py gui:=false   # headless smoke run
```

| Step | Result |
| --- | --- |
| `colcon build` | finished in 20.7 s; `share/rover_sim/generated/` holds the DEM, textures, 14 rock meshes and the six worlds |
| `colcon test` | **281 tests, 0 errors, 0 failures, 0 skipped** (276 pytest + 5 CTest wrappers); identical result in the development workspace |
| headless smoke run (`world:=mars`, GPU lite LiDAR) | rover spawned on the pad 2.4 s after launch; in 15 s of simulation: 142 `/points` clouds (10 Hz), 757 `/odom`, 1511 `/imu/data`, 756 `/joint_states` messages; the rover drove to (2.30, 1.97) m on a 0.4 m/s / 0.3 rad/s command; RTF 0.999 with gzserver at 0.56 cores; the only error lines in the launch log are the SIGINT shutdown notices |

The validators were executed from the development workspace during the
phases; their JSON evidence is committed under `docs/validation/`.
