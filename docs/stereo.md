# Stereo camera: disparity, depth image and point cloud

The rover carries a forward-looking stereo camera that runs alongside the LiDAR
in the same Gazebo Classic world. It publishes left/right images, a disparity
image, a metric depth image and a coloured 3D point cloud, all under `/stereo/`.
It is self-contained: it adds no collision geometry, mass, TF owner or topic
outside `/stereo/`, and it can be switched off independently of the LiDAR.
Measured results: [validation report](validation/stereo.md).

## Quick start

Build and test as in the [README](../README.md). Then, in a terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
ros2 launch rover_sim rover_desert.launch.py rviz:=true
```

Stereo is on by default (`stereo:=true`), as is the LiDAR (`lidar:=true`).
With stereo on, `rviz:=true` opens `rviz/rover_sensors.rviz`: the robot, the
LiDAR scan, the stereo point cloud, and the left-image and depth panels. Drive
with `ros2 run rover_sim rover_teleop` in a second sourced terminal.

If RViz fails from the Snap VS Code terminal with a `/snap/core20` `libpthread`
error, start the launch like this instead:

```bash
env -u GTK_PATH -u GTK_EXE_PREFIX -u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE \
  ros2 launch rover_sim rover_desert.launch.py rviz:=true
```

Other combinations: `stereo:=false`, `lidar:=false`, both, or neither all launch
cleanly. Camera settings are launch arguments, for example
`stereo_rate:=10 stereo_baseline:=0.2`.

## How it works

```text
Gazebo Classic: one "multicamera" sensor (both images share one capture time)
 + libgazebo_ros_camera.so
   ├─ /stereo/left/image_raw, /stereo/left/camera_info ─────────────┐
   └─ /stereo/gazebo/right/image_raw, .../camera_info  (internal)   │
         │                                                          │
   stereo_camera_relay: right frame id + Tx = -fx*B, same stamp     │
         └─ /stereo/right/image_raw, /stereo/right/camera_info ─────┤
                                                                    ▼
   stereo_processor: exact-time sync of all four, OpenCV SGBM
         ├─ /stereo/disparity          stereo_msgs/DisparityImage
         ├─ /stereo/depth/image_raw    32FC1 metres (NaN = no depth)
         ├─ /stereo/depth/camera_info  copy of the left camera_info
         └─ /stereo/points2            PointCloud2 x, y, z, rgb
```

**Why a relay?** In multicamera mode, `libgazebo_ros_camera.so` labels both
images with one frame and gives both camera_infos the same `Tx`. A right camera
needs its own optical frame and `Tx = -fx * baseline`. So the plugin's right
stream goes to an internal topic, and the relay republishes it with the correct
frame and `Tx`, keeping the timestamp. The relay reads the baseline from
`robot_description`, the same URDF the cameras were built from.

**Why a custom processor instead of `stereo_image_proc`?** That package is
available for Humble (`sudo apt install ros-humble-stereo-image-proc`) but was
not installed on the development machine, and installing needed sudo. The custom
node is plain Python + OpenCV (`cv2.StereoSGBM`). It deliberately avoids
`cv_bridge`, because the Python `cv_bridge` breaks when numpy 2 is installed
(as it is in `~/.local` here). Images are read with numpy directly.

### New files, in plain language

| File | What it does |
| --- | --- |
| `urdf/sensors/stereo.urdf.xacro` | Describes the camera: a small visual-only housing, two optical frames, and the Gazebo sensor and plugin. Refuses to build if the camera is inside the LiDAR keep-out region. |
| `rover_sim/stereo_geometry.py` | Camera maths without ROS: focal length from field of view, expected camera_info, baseline from the URDF, keep-out region test. |
| `rover_sim/stereo_matching.py` | Stereo maths without ROS: image checks, SGBM matching, texture and edge filters, disparity → depth → points, point-cloud packing. |
| `rover_sim/stereo_truth.py` | Ground truth for testing: casts camera rays at the world's boxes, cylinders and spheres. |
| `rover_sim/stereo_relay.py` | ROS node `stereo_camera_relay` (the right-camera fix described above). |
| `rover_sim/stereo_processor.py` | ROS node `stereo_processor`: disparity, depth image and point cloud. |
| `rover_sim/stereo_launch.py` | Launch helpers: checks stereo launch arguments, builds the two nodes, warns about oversized frames. |
| `rover_sim/node_runner.py` | Runs a node until Ctrl+C without tracebacks when a second signal arrives during shutdown (same approach as the controller). |
| `scripts/stereo_camera_relay`, `scripts/stereo_processor` | Tiny entry points used by `ros2 run` / the launch file. |
| `config/stereo.yaml` | Matcher settings (validated at startup). |
| `rviz/rover_sensors.rviz` | Combined LiDAR + stereo RViz view. `rover_lidar.rviz` is unchanged. |
| `models/rover_sim_materials/` | Ground and rock materials plus the sand texture. A Gazebo resource folder, not a spawnable model. |
| `scripts/generate_sand_texture.py` | Regenerates `rover_sand.png` byte for byte (fixed seed, numpy only). |
| `scripts/validate_stereo.py` | Live validator: rates, sync, camera_info, TF, depth vs ground truth, non-interference. |
| `scripts/validate_launch_matrix.py` | Starts and stops all four stereo × LiDAR combinations. |
| `test/test_stereo_*.py` | 149 unit tests (no simulator needed). |

## Topics, frames and QoS

| Topic | Type | Publisher | Frame |
| --- | --- | --- | --- |
| `/stereo/left/image_raw` | `sensor_msgs/Image` rgb8 | Gazebo plugin `stereo_camera` | `stereo_left_optical_frame` |
| `/stereo/left/camera_info` | `sensor_msgs/CameraInfo` | Gazebo plugin | `stereo_left_optical_frame` |
| `/stereo/right/image_raw` | `sensor_msgs/Image` rgb8 | `stereo_camera_relay` | `stereo_right_optical_frame` |
| `/stereo/right/camera_info` | `sensor_msgs/CameraInfo` | `stereo_camera_relay` | `stereo_right_optical_frame` |
| `/stereo/disparity` | `stereo_msgs/DisparityImage` | `stereo_processor` | left optical |
| `/stereo/depth/image_raw` | `sensor_msgs/Image` 32FC1 | `stereo_processor` | left optical |
| `/stereo/depth/camera_info` | `sensor_msgs/CameraInfo` | `stereo_processor` | left optical |
| `/stereo/points2` | `sensor_msgs/PointCloud2` | `stereo_processor` | left optical |
| `/stereo/gazebo/right/*` | internal, uncorrected | Gazebo plugin | wrong on purpose; do not use |

QoS: the plugin, relay and processor all publish **reliable**, keep-last,
volatile. The relay and processor also subscribe reliably: best-effort readers
lost about half of the large image frames in testing. RViz displays use **best
effort**, so viewing never slows a sensor publisher. Stamps are simulation time.
Left and right images of a pair carry identical stamps, and every output reuses
the left image's stamp.

TF (all fixed joints, published only by `robot_state_publisher`):

```text
base_link
└── stereo_link                     fixed stereo_joint, xyz (0.52, 0, -0.03), body axes
    ├── stereo_left_optical_frame   (0.03, +B/2, 0), rpy (-pi/2, 0, -pi/2): x right, y down, z forward
    └── stereo_right_optical_frame  (0.03, -B/2, 0), same rotation
```

There are still no `odom`, `map` or `world` frames.

## Parameters

Launch arguments (also xacro arguments of `proxy_rover.urdf.xacro`):

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `stereo` | `true` | Add the camera and its nodes (`enable_stereo` in xacro) |
| `stereo_width`, `stereo_height` | `480`, `360` | Image size in pixels (64–4096) |
| `stereo_rate` | `15` | Hz in simulation time (0–60] |
| `stereo_baseline` | `0.12` | Metres between optical centres (0.01–1.0) |
| `stereo_hfov` | `1.5708` | Horizontal field of view, radians (0.1–2.8) |

Invalid values stop the launch before anything starts. Xacro-only arguments:
`stereo_xyz` (`0.52 0 -0.03`) and `stereo_rpy` (`0 0 0`) set the mount on
`base_link`. The `lidar_keepout_*` arguments are described below.

**Why 480 × 360?** A 640 × 480 RGB frame (921,600 bytes) does not fit Fast
DDS's default 512 KiB shared-memory segment. With 640 × 480, the existing
hardening check `controller_resume_no_stale_motion` failed in every run (6 of 6:
four at 15 Hz, two at 10 Hz). The controller's reset message was delayed long
enough for it to act on a stale command for about 0.3 s. The plugin still
blocked any wheel motion. At 480 × 360 (518,400 bytes) it passed every time
(8 of 8), as it does with stereo off (3 of 3). Rendering alone was not the
cause: with the cameras rendering but nobody subscribed, it also passed. The launch prints a warning if
`width × height × 3` exceeds 512 KiB minus 1 KiB of header room. To go larger,
enlarge the Fast DDS shared-memory segment (an environment-wide change), then
re-run the hardening validator.

Camera model: `fx = fy = width / (2 tan(hfov / 2))` (240 px by default),
`cx = (width - 1) / 2`, `cy = (height - 1) / 2`, no distortion. The plugin's
default principal point `(width + 1) / 2` is one pixel off the centre of
Gazebo's symmetric frustum, so the xacro sets `cx`/`cy` explicitly. The right
camera's `P[3] = Tx = -fx * baseline` (−28.8 by default).

`config/stereo.yaml` (matcher, validated at startup):

| Parameter | Default | Why |
| --- | --- | --- |
| `num_disparities` | 80 | Nearest depth `fx*B/80` = 0.36 m; the closest ground the camera sees is ~0.40 m |
| `block_size` | 5 | SGBM matching window |
| `min_texture` | 4.0 | Grey-level spread a 5 × 5 patch needs. Camera noise alone measured ≤ 2.4; sand ≥ 7.4 |
| `texture_window` | 5 | Same patch as `block_size` |
| `edge_erosion` | 2 | Removes 2 px around every valid region. Block matching smears depth past silhouettes ("edge fattening"), measured up to ~2 px at 480 × 360 |
| `min_valid_disparity` | 2.0 | Disparities below 2 px (beyond 14 m) are marked invalid |
| `uniqueness_ratio`, `speckle_*`, `disp12_max_diff`, `pre_filter_cap` | 10, 100/2, 1, 63 | Standard SGBM filters |
| `cloud_stride` | 2 | `/stereo/points2` uses every 2nd row and column |
| `baseline_tolerance` | 0.002 m | camera_info `Tx` must match the URDF baseline |

**Fail-closed behaviour.** The processor publishes nothing for a pair if any of
these are wrong: stamps that are not identical, image and camera_info sizes,
frames, encodings (rgb8, bgr8 and mono8 are supported), distortion or
rectification (input must already be rectified), a non-zero left `Tx`, a right
`Tx` that is not negative, or a baseline that disagrees with the URDF. It logs
a throttled warning instead. Pixels without a trustworthy match get NaN depth
and no point, never a guess. Every 10 s it logs how many pairs it processed and
rejected.

## Non-interference rules

1. **Separate files.** All stereo logic lives in new files. Shared files only
   gained additions: arguments, one include and one `xacro:if` in
   `proxy_rover.urdf.xacro`; arguments, node creation and a second RViz option in
   `rover_desert.launch.py`; install lines and test targets in `CMakeLists.txt`;
   dependencies and a Gazebo model-path export in `package.xml`; a ground visual
   material and five features in `desert.world`. The LiDAR xacro, `/scan`, its
   plugin and `rover_lidar.rviz` are untouched. A test checks that the LiDAR's
   expanded description is byte-identical with stereo on or off.
2. **No physical footprint.** The stereo links have no `<collision>`, so no ray
   sensor (2D or 3D LiDAR) can hit them. They also have no `<inertial>`, so the
   rover's mass, swerve geometry, joints and actuator plugin are unchanged
   (tested).
3. **LiDAR keep-out region.** This region is configurable for a future 3D LiDAR.
   It covers the cylinder of radius `lidar_keepout_radius` (0.15 m) around the
   LiDAR at `lidar_keepout_origin` (`0 0 0.20`), plus every direction whose
   elevation from that origin lies between `lidar_keepout_vfov_min - margin` and
   `lidar_keepout_vfov_max + margin` (−15° − 2° to +15° + 2°), at any range. The
   xacro **refuses to expand** if the camera's bounding box enters it. A unit test
   also checks sampled points of every stereo visual. The default mount's highest
   point is at −21.4° elevation, 4.4° clear. Example for a LiDAR with ±22.5°
   vertical coverage:
   `xacro proxy_rover.urdf.xacro lidar_keepout_vfov_min:=-22.5 lidar_keepout_vfov_max:=22.5`.
   This rejects the default mount on purpose, so move `stereo_xyz` lower or
   adjust the keep-out.
4. **Names.** Topics are under `/stereo/`, and frames start with `stereo_`.
   Nothing reuses `/scan`, `/points` or `lidar_*`. A live check confirms that the
   stereo nodes publish nothing outside `/stereo/` except the standard
   `/rosout` and `/parameter_events`.
5. **TF.** Only fixed joints; `robot_state_publisher` remains the only
   `/tf_static` publisher. No odometry or world frames.
6. **Independent switches.** `stereo` and `lidar` are separate. All four
   combinations are unit-tested (launch evaluation) and were started and
   stopped live.
7. **Budget.** Measured real-time factor 0.9988 with stereo vs 0.9990 without;
   LiDAR 10.0 Hz either way. Largest controller heartbeat gap 0.024 s against
   the unchanged 0.3 s watchdog. No watchdog trips. See the frame-size note above.

### World additions

The sand ground's **visual** now uses a texture
(`model://rover_sim_materials`, material `RoverSim/Sand`), because stereo
cannot match a flat colour. Its collision, friction, the hills and the spawn
area are unchanged. Five static features were added for stereo testing.
Each is ≤ 0.35 m tall, so below the 2D LiDAR's scan plane (~0.53 m above flat
ground). None is a sphere, because `validate_lidar.py` treats every sphere as a
hill. They sit at least 3.5 m from the spawn point (the existing motion
validators drive the rover centre up to 2.9 m) and clear of the LiDAR
validator's probe boxes and the hill footprints. All of this is unit-tested.

| Model | Shape | Centre x, y, z (m) | Size (m) | Material |
| --- | --- | --- | --- | --- |
| `stereo_feature_crate` | box | 4.5, 0.0, 0.175 | 0.5 × 0.5 × 0.35 | Gazebo/WoodPallet |
| `stereo_feature_rock_slab` | box | 4.5, 1.4, 0.125 | 0.6 × 0.4 × 0.25 | RoverSim/Rock |
| `stereo_feature_brick_block` | box | 3.6, −2.2, 0.15 | 0.4 × 0.4 × 0.3 | Gazebo/Bricks |
| `stereo_feature_rock_post` | cylinder | 5.2, −1.6, 0.15 | r 0.2, length 0.3 | RoverSim/Rock |
| `stereo_feature_stump` | cylinder | 5.4, 2.6, 0.1 | r 0.25, length 0.2 | Gazebo/WoodPallet |

These have collision geometry, so a future 3D LiDAR will see them, as it would
real rocks. The materials are found through `<gazebo_ros
gazebo_model_path="${prefix}/models"/>` in `package.xml`. Do **not** switch
this to `gazebo_media_path`: gazebo_ros would then replace
`GAZEBO_RESOURCE_PATH`, hiding Gazebo's own materials and silently breaking
camera rendering. That happened during development.

## Validation

Unit tests run with the normal `colcon test` command (see the README). For the
live checks, use a dedicated headless simulation in an unused ROS domain and
Gazebo master. Do not run teleop or any other `/cmd_vel` publisher at the same
time; the validators move the rover.

Terminal 1, the stereo-off baseline (needed for the non-interference comparison):

```bash
source /opt/ros/humble/setup.bash
source ~/rover_ws/install/setup.bash
export ROS_DOMAIN_ID=47 GAZEBO_MASTER_URI=http://127.0.0.1:11357
ros2 launch rover_sim rover_desert.launch.py world:=desert gui:=false lidar_mode:=2d stereo:=false actuator_watchdog:=0.3
```

Terminal 2 (same sourcing and exports), once the launch prints
`Eight-joint bridge ready`:

```bash
ros2 run rover_sim validate_stereo.py --performance-only \
  --output ~/rover_ws/src/rover_sim/docs/validation/local_stereo_baseline.json
```

Stop terminal 1 with Ctrl+C. Restart it without `stereo:=false`, then in
terminal 2:

```bash
ros2 run rover_sim validate_stereo.py \
  --compare ~/rover_ws/src/rover_sim/docs/validation/local_stereo_baseline.json \
  --output ~/rover_ws/src/rover_sim/docs/validation/local_stereo.json
```

The launch-combination check starts its own simulations, so nothing else may
be running in that domain:

```bash
ros2 run rover_sim validate_launch_matrix.py \
  --output ~/rover_ws/src/rover_sim/docs/validation/local_launch_matrix.json \
  --log-dir /tmp/rover_launch_matrix
```

Each validator exits 0 when every check passes. The full run checks the
following:
- camera, disparity, depth and cloud rates;
- identical left/right stamps;
- camera_info against the URDF;
- TF ownership;
- depth against analytic ground truth from four rover poses (set with
  `gz model`, as in `validate_swerve.py`), binned by distance;
- that cloud points reproduce the depth image and that ground points lie at
  z ≈ 0;
- real-time factor, heartbeat gaps and watchdog epochs;
- with `--compare`, that the LiDAR rate, the resting LiDAR scan and the
  real-time factor match the stereo-off run.

The pass criteria were fixed before the first measurement. They are in the
validator's docstring and in the [report](validation/stereo.md).

## Limitations

- **Usable depth range: about 0.4–5 m** on textured surfaces (measured). Error
  grows with the square of the distance: median 2 mm at 0.4–1 m, 3.8 cm at
  2–3 m, 17 cm at 4–5 m. Beyond 5 m most pixels are invalid or inaccurate.
- **Plain-coloured surfaces get no depth.** The hills were left unchanged as
  requested; about 2% of hill pixels get a depth, all within ~4 px of an edge,
  with a 0.65 m median error. The sky gets none (0 of 8,746 sampled pixels).
- **Small objects are thin in the depth image.** Edge erosion and the texture
  check remove much of a 0.25–0.35 m object at 4–6 m (1–32% of its pixels get
  depth). Where they do, the median error is 2–12 cm, and up to 44 cm on the
  small stump.
- **The leftmost 81 columns never have depth** (the disparity search margin),
  and depth only covers what both cameras see.
- **Idealised sensor:** pinhole cameras with Gaussian noise; no lens
  distortion, motion blur, exposure, rolling shutter, dust or lighting changes.
  Rigid primitive terrain.
- **Frame size:** frames above Fast DDS's default 512 KiB shared-memory segment
  disturbed the controller's reset handshake (see Parameters).
- **No odometry**, so RViz stays in the rover's frame. The point cloud is a
  per-frame snapshot that is not accumulated.
- **`/stereo/gazebo/right/*` is internal** and carries the plugin's uncorrected
  frame and `Tx`.
