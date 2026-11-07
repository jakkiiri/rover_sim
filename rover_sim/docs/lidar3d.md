# 3D LiDAR for Gazebo Classic

The 3D sensor and post-processor are adapted from
[jakkiiri/rover_sim](https://github.com/jakkiiri/rover_sim/tree/4b503e9a0232867b749f1e1f6f7a8958ed70a3a8),
commit `4b503e9a0232867b749f1e1f6f7a8958ed70a3a8` (Apache-2.0).
[Third-party notices](../THIRD_PARTY_NOTICES.md#3d-lidar-adaptation-2026-11-07)
record the imported files and every local change. Only LiDAR functionality
is imported, using Gazebo Classic 11 and `gazebo_ros`.

See the [measured results and retained failures](validation/lidar3d.md).
The initial integration run passed; a later loaded repeat failed its watchdog
guard. The same host also logged a watchdog expiry with LiDAR disabled.
Host-load timing resilience is not established by this implementation.

## Build and run

Complete the [README dependency setup](../README.md) first. The new Python
dependency is `python3-numpy` (also declared for rosdep).

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/rover_sim --ignore-src --rosdistro humble -y
colcon build --packages-select rover_sim
source ~/rover_ws/install/setup.bash
ros2 launch rover_sim rover_desert.launch.py rviz:=true
```

The launch defaults to `lidar_mode:=3d lidar_profile:=airy` (see
[RoboSense Airy-style dome](#robosense-airy-style-dome-airy-profile-default)).
RViz opens with `base_link` fixed frame, RobotModel and a height-colored
`/points` PointCloud2 display. Tilt the view to see the ground rings and
hills. Scan decay is zero: no odometry or world-stable accumulation is added.
Do not copy the upstream `odom`/three-second-decay configuration into this
prototype; it needs transforms that this rover deliberately does not provide.

To view an already running 3D simulation in a new terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
rviz2 -d "$(ros2 pkg prefix --share rover_sim)/rviz/rover_lidar3d.rviz" \
  --ros-args -p use_sim_time:=true
```

Teleop remains separate and unchanged: W/S forward/backward, A/D rotate,
Space stop, Ctrl+C stop/exit. No lateral command path is enabled.

## Sensor and ROS interfaces

The fixed `lidar_joint` mounts `lidar_link` at XYZ `(0, 0, 0.40)` m, RPY zero,
relative to `base_link`. On level ground it is approximately 0.73 m high.
The 0.8 kg housing and inertia come from the reference model; its visual mast
is extended to the local chassis top. The mount clears the chassis and tires
even with the widest downward beam. Robot_state_publisher exclusively owns
`base_root -> base_link -> lidar_link`; the sensor publishes no TF.

| Setting | Default |
| --- | --- |
| Sensor/plugin | Classic CPU `ray` / `libgazebo_ros_ray_sensor.so` |
| Raw ROS output | `/points_raw`, `sensor_msgs/msg/PointCloud2` |
| Processed output | `/points`, `sensor_msgs/msg/PointCloud2` |
| Frame ID | `lidar_link` on both clouds |
| Update rate | 10 Hz in simulation time |
| Horizontal FOV | −π to +π, inclusive (rear direction duplicated) |
| Lite beam layout | 16 vertical channels × 450 horizontal samples |
| Angular spacing | 2° vertical; 360/449 ≈ 0.801782° horizontal |
| Vertical FOV | −15° to +15° |
| Range | 0.5–100 m; simulator resolution setting 0.002 m |
| Sensor noise | Gaussian, zero mean, 0.02 m standard deviation |
| Post-processing noise | Additional σ = 0.01 m + 0.0003 × range |
| Missing returns | Probability min(0.95, 0.02 + 0.002 × range) |

The plugin XML uses `sensor_msgs/PointCloud2` as its `output_type` and
`~/out:=/points_raw`. The vectorized post-processor preserves the simulation
header and adds `ring` (uint16) and `time` (float32) to x/y/z/intensity.
Its reliable, volatile QoS avoids lost large fragmented clouds; RViz uses
matching reliable QoS. Non-finite, too-close and out-of-range raw returns are
discarded. Output is an unorganized cloud of retained finite points.

`time` is synthetic azimuth-derived time within a 0.1 s sweep. Gazebo captures
each sweep instantaneously; these fields do not introduce spinning distortion,
deskewing or a physically correct acquisition timeline. Intensity is a simple
range/albedo proxy where the simulator provides none. Dust, reflectivity,
multi-path and realistic material interactions are not modelled.

| `lidar_profile` | Channels | Horizontal samples | Vertical FOV | CPU rays/s |
| --- | --- | --- | --- | --- |
| `airy` (default) | 48 (27 simulated) | 900 | 0° to −90° (dome, upside down) | 243,000 |
| `lite` | 16 | 450 | 30° | 72,000 |
| `standard` | 16 | 900 | 30° | 144,000 |
| `high` | 32 | 1024 | 45° | 327,680 |
| `dense` | 64 | 1024 | 45° | 655,360 |

Lite uses ten times the original 2D sensor's ray count. Start there and
measure host load before increasing density. The unchanged 0.3 wall-second
actuator watchdog still applies; do not increase it to hide sensor overload.
Processing parameters are validated at startup and read-only thereafter.

The delivered robot/launch uses CPU `ray` only. GPU experiments on this
host failed the terrain-accuracy check, so GPU mode is not exposed. The
reference branch's extra-beam/one-step correction made this host's already
correct beam labels worse; nominal angles removed that large error, but
rasterized GPU ranges still exceeded the accuracy tolerance and watchdog
interruptions were frequent. Both failed results are retained in the
[validation report](validation/lidar3d.md). No heightmap world is imported.
The vendored generic macro and post-processor retain legacy GPU helpers
for provenance/testing, but the local wrapper fixes CPU mode and zero shift.

`lidar_mode:=2d` restores the original dedicated sensor, `/scan` and RViz
config. `lidar:=false` omits either sensor and the processing node. Direct
Xacro expansion defaults to 2D for compatibility; the operator launch
explicitly selects 3D. In 3D mode `/scan` is not published by default. The
post-processor has an optional `publish_scan` parameter, but one inclined
ring is only a projected scan, not the old horizontal sensor (even channel
counts have no exactly horizontal ring).

## Reproduce validation

Use an unused ROS domain/master and no other command publisher. Keep the
host awake. Terminal 1, after building as above:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=55
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
ros2 launch rover_sim rover_desert.launch.py \
  gui:=false rviz:=false lidar_mode:=3d lidar_profile:=lite
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=55
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
ros2 run rover_sim validate_lidar3d.py --profile lite --duration 5 --windows 2 \
  --output ~/rover_ws/src/rover_sim/rover_sim/docs/validation/local_lidar3d.json
```

The required output path is created as needed. The validator tests cloud
fields/limits, timestamp preservation, TF ownership, ground and all three
hills against their actual collision geometry, a box with front face 5 m
ahead, forward/yaw movement, heartbeat gaps, watchdog epochs and measured
RTF/publication rates. It publishes stop and removes its temporary box on
completion or failure. Ground truth is used only for validation.

For a matched baseline, stop Terminal 1, restart it with `lidar:=false`,
then run in the same isolated Terminal 2:

```bash
ros2 run rover_sim validate_lidar3d.py --performance-only --duration 5 --windows 2 \
  --output ~/rover_ws/src/rover_sim/rover_sim/docs/validation/local_lidar3d_baseline.json
```

Measure without RViz/Gazebo GUI. Compare under similar host load; wall-time
publication rate scales with RTF. Unit tests use the normal README colcon
commands. Existing 2D validation commands require `lidar_mode:=2d`.
Close these terminals or unset the domain/master before the normal workflow.

For Snap IDE library errors, use a regular Ubuntu terminal or the existing
[GUI environment workaround](lidar.md#guienvironment-workaround), using
`rover_lidar3d.rviz` for the standalone 3D viewer.

## RoboSense Airy-style dome (`airy` profile, default)

Models a RoboSense Airy-style hemispherical LiDAR mounted upside down at the
top of the 0.40 m mast: 48 beams evenly spaced from the horizon (0°) to
straight down (−90°, 1.91° apart), 360° around, 900 samples per beam
(0.4°), 10 Hz, 0.1 m minimum range. (The real Airy has 96 beams; 48 were
chosen here.) Instead of turning the link upside down, the beam angles are
flipped, so `lidar_link` stays z-up and nothing downstream changes. Ring
numbers count up from the lowest beam: ring 0 is −90°, ring 47 the horizon.
The top beam is simulated 0.1° below the horizon (1.7 cm lower at 10 m):
exactly horizontal rays get false returns 0.2-0.6 m away on the Mars
terrain mesh (a Gazebo ray-versus-mesh fault; the flat desert has none), and
those false returns surrounded the rover with obstacles so it would not move.

Looking down over the rover, the dome sees its own body: in a raw cloud 60 %
of the returns are the chassis, wheels and camera. `lidar_postprocess`
drops returns inside `self_filter_box` (the launch passes the rover's box in
the sensor frame: x −0.62 to 0.64 m, y ±0.66 m, from 0.80 m below the sensor
up to it; steered sideways, a wheel reaches 0.63 m from the centre line, and
its top then read as an obstacle touching the rover). The 21 beams below −50.7° only ever hit that box, so they are not
simulated at all; the published cloud and ring numbers are the same, at 56 %
of the ray cost. The stereo camera's beam-band keep-out check is skipped for
this profile (`lidar_keepout_band:=false`), because a downward dome sees the
camera wherever it is mounted; the housing-clearance check still runs.

It costs more than `lite`. Measured on Mars, headless, nothing else running:

| Setting | Real-time factor | `/points` on screen |
| --- | --- | --- |
| `airy`, 900 samples (default) | 0.36 | 3.6 Hz |
| `airy`, `lidar_samples:=600` | 0.45 | 4.5 Hz |
| `airy`, `lidar_samples:=450` | 0.54 | 5.4 Hz |
| `lite` | 0.68 | 6.9 Hz |

(The 600 and 450 rows were measured two at a time, so they are slightly
pessimistic.) `lidar_samples` overrides any profile's samples per beam. The
GPU sensor is not an option here (see above).

With the planner (`rover_planner`, stereo plus `/points`) on the Mars test
drives, the default `airy` profile reached all three goals: hill 0.25 m off
(14.0 m driven), mesa 0.21 m (28.8 m), crater 0.22 m (17.3 m), with no
contact with any rock or shrub and no look-around turns. One planned path on
the hill run passed over a 0.15 m rock that is below the obstacle threshold;
the rover did not touch it.

## Integration with the stereo camera, Mars world and planner (2026-11-07)

The 3D LiDAR was developed on the original desert-only package, then merged
into the version with the stereo camera, the Mars world and ground-truth
pose. What changed in the merge:

- **Launch.** `rover_desert.launch.py` keeps `world`, `stereo` and the stereo
  arguments, and adds `lidar_mode` (default `3d`) and `lidar_profile`
  (default `lite`). The stereo camera's LiDAR keep-out check receives the 3D
  sensor's mount (0.40 m) and beam band (±15°, or ±22.5° for `high` and
  `dense`). The default camera sits about 40° below the lowest 3D beam.
- **RViz.** One RViz node chooses its view from the two switches: stereo and
  3D → `rover_sensors_3d.rviz` (new: the 3D cloud plus stereo cloud and
  images); stereo and 2D → `rover_sensors.rviz`; 3D only →
  `rover_lidar3d.rviz`; 2D only → `rover_lidar.rviz`.
- **2D-only checks.** `validate_launch_matrix.py`, `validate_stereo.py` and
  `validate_lidar.py` check `/scan`, so their documented commands use
  `lidar_mode:=2d`.

Measured on the Mars world, headless, 30 wall seconds per configuration:

| Configuration | Real-time factor | `/points` | `/scan` | `/stereo/points2` (sim time) |
| --- | --- | --- | --- | --- |
| 3D LiDAR, lite (default) | 0.69 | 10.0 Hz | — | 15.0 Hz |
| 2D LiDAR (`lidar_mode:=2d`) | 1.00 | — | 10.0 Hz | 15.0 Hz |
| No LiDAR (`lidar:=false`) | 1.00 | — | — | 15.0 Hz |

The Mars terrain mesh and its rocks cost more to ray-trace than the flat
desert (about 7.5 % slower there), so the whole simulation runs about 30 %
slower than real time. Sensor rates stay correct in simulation time.

Accuracy on Mars: 30 clouds of `/points` (median 3,480 points each; the
upper beams see only sky) were placed in `odom` with TF and compared with the
generator's true terrain heights, leaving out points near rocks and shrubs.
The median height error was **0.0046 m**, the 95th percentile **0.038 m**,
and 99.3 % of points were within 0.10 m.

The A* planner in `rover_planner` still uses only the stereo cloud. With the
3D LiDAR running it reached the hill (1/1), mesa (1/1) and crater (3/3)
goals; drives took about 30-45 % longer in wall time.
