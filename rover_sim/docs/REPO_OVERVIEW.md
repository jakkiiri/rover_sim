# rover_sim — Repository Overview (context file for AI models)

> Purpose: give another model complete working context on this repo without
> reading every file. Written from the code at commit `4905b38` ("First Commit
> (Leo's Code)"), then updated for the stereo camera system (2026-10-17, see
> `docs/stereo.md`). Original author: Leo Karras (package maintainer). When this
> file and the code disagree, the code is authoritative.

## 1. What this is

A single ROS 2 package, `rover_sim`, that simulates a **four-wheel independently
steered ("swerve") proxy rover** in a **Gazebo Classic** desert world with three
hills, plus:

- a Python swerve controller node (`/cmd_vel` → per-joint targets),
- a C++ Gazebo Classic model plugin that actuates the 8 joints with PD/PI
  control through physics (no teleporting),
- a separate keyboard teleop node (W/S/A/D),
- a 2D 360° ray LiDAR publishing `/scan`,
- a stereo camera (Gazebo multicamera + right-camera relay + OpenCV SGBM processor)
  publishing images, disparity, a depth image and a point cloud under `/stereo/` (§11b),
- optional robot-local RViz view,
- unit tests (pytest) and integration validators (scripts that drive a live sim
  and write JSON evidence).

Self-assessed milestone status: **"PASS WITH LIMITATIONS"** (Phase 5 = LiDAR; stereo added after).

**Key policy:** the rover is *not allowed to strafe*. `linear.y` on `/cmd_vel`
is clamped to 0 by the controller, even though the kinematics library is fully
holonomic.

## 2. Hard environment requirements

| Item | Version |
| --- | --- |
| OS | Ubuntu 22.04 |
| ROS | ROS 2 **Humble** |
| Simulator | Gazebo **Classic 11.x** (validated 11.10.2) via `gazebo_ros_pkgs` |
| NOT used | `gz sim` / Ignition, `ros_gz`, `ros2_control`, Nav2, SLAM, odometry |
| Python | 3.10 (system) |
| Build | `ament_cmake` + `ament_cmake_python`, colcon |

Workspace layout expected: `~/rover_ws/src/rover_sim/rover_sim/package.xml`.

```bash
cd ~/rover_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/rover_sim --ignore-src --rosdistro humble -y
colcon build --packages-select rover_sim
source install/setup.bash
colcon test --packages-select rover_sim --event-handlers console_direct+
colcon test-result --verbose
```

## 3. File map

```
CMakeLists.txt              builds the C++ plugin, installs python pkg, scripts, data dirs; 11 pytest targets
package.xml                 deps; licenses Apache-2.0 + BSD-3-Clause; exports gazebo_model_path=${prefix}/models
README.md                   setup/run/limitations (user-facing)
THIRD_PARTY_NOTICES.md      upstream provenance + every local modification to vendored xacro
LICENSES/                   Apache-2.0.txt, UOS-BSD.txt

config/swerve.yaml          controller params (timeouts, wheel speed/accel, alignment tolerance)
config/teleop.yaml          teleop params (speeds, key timeout, publish rate)
config/stereo.yaml          stereo_processor SGBM/filter params (validated at startup)

launch/desert.launch.py         world only (wraps gazebo_ros/gazebo.launch.py); arg: gui
launch/rover_desert.launch.py   MAIN launch: world + xacro → controller + robot_state_publisher + spawn + optional rviz

rover_sim/                  Python package (installed via ament_python_install_package)
  kinematics.py             pure-Python URDF parsing + swerve inverse kinematics (no ROS imports)
  control_state.py          deterministic controller state machine (clocks passed in explicitly; no ROS)
  controller.py             ROS node `swerve_controller` wrapping ControlState; reset handshake; shutdown
  teleop.py                 ROS node `rover_teleop`; raw-terminal keyboard → /cmd_vel
  stereo_geometry.py        (no ROS) intrinsics, camera_info/Tx, baseline from URDF, LiDAR keep-out test
  stereo_matching.py        (no ROS) image checks, SGBM + texture/edge filters, depth, points, cloud packing
  stereo_truth.py           (no ROS) ray casting vs world boxes/cylinders/spheres for validation
  stereo_relay.py           ROS node `stereo_camera_relay`: right image/info with own frame and Tx = -fx*B
  stereo_processor.py       ROS node `stereo_processor`: /stereo/disparity, depth, points2
  stereo_launch.py          launch helpers: stereo arg validation, nodes, oversized-frame warning
  node_runner.py            run a node until SIGINT/SIGTERM without teardown tracebacks

scripts/                    installed to lib/rover_sim (run with `ros2 run rover_sim <name>`)
  swerve_controller         entry → rover_sim.controller.main
  rover_teleop              entry → rover_sim.teleop.main
  validate_swerve.py        motion check vs Gazebo ground truth
  validate_teleop.py        drives real teleop through a pty
  validate_hardening.py     reset/pause/slow-sim/watchdog checks (needs --controller-pid)
  validate_shutdown.py      SIGINT/SIGTERM shutdown regression (no Gazebo)
  validate_lidar.py         scan vs analytic hill/box ranges, TF, rate, RTF
  stereo_camera_relay, stereo_processor   entries for the stereo nodes
  validate_stereo.py        stereo rates, sync, camera_info, TF, depth vs truth, non-interference (--compare)
  validate_launch_matrix.py starts/stops all 4 stereo x lidar combinations
  generate_sand_texture.py  regenerates the ground texture (deterministic)

src/swerve_gazebo_plugin.cpp   Gazebo Classic ModelPlugin → librover_swerve_actuators.so

urdf/proxy_rover.urdf.xacro       top-level robot (xacro args: enable_control, actuator_watchdog, enable_lidar)
urdf/sensors/lidar.urdf.xacro     `rover_lidar` macro (link, fixed joint, ray sensor + gazebo_ros plugin)
urdf/sensors/stereo.urdf.xacro    `rover_stereo` macro (visual-only, massless links; multicamera + plugin; keep-out xacro.fatal)
urdf/upstream/*.urdf.xacro        vendored from Kettenhoax/gazebo_ros_four_wheel_steering @ 9802a27 (modified)

worlds/desert.world         SDF 1.6; 100x100 m ground box (textured visual) + 3 buried spheres as hills
                            + 5 stereo_feature_* boxes/cylinders; ODE 1 kHz
models/rover_sim_materials/ Gazebo resource folder: RoverSim/Sand + RoverSim/Rock materials, rover_sand.png
rviz/rover_lidar.rviz       Fixed Frame base_link, RobotModel + LaserScan (/scan, best effort, decay 0)
rviz/rover_sensors.rviz     same + stereo cloud, left/right/depth images (used when stereo is on)

test/                       pytest unit tests (280 cases incl. parametrized; colcon shows 291 w/ 11 CTest wrappers)
docs/                       swerve.md, hardening.md, teleop.md, lidar.md, stereo.md (design docs), this file
docs/validation/            dated reports (.md), JSON evidence, screenshots; stereo.md is the current index
```

Generated (never edit): `~/rover_ws/build`, `install`, `log`.

## 4. Runtime architecture

```
 rover_teleop (Python)            swerve_controller (Python)                 Gazebo Classic (gzserver)
 ────────────────────            ──────────────────────────                 ─────────────────────────
 keyboard ──► /cmd_vel ────────► ControlState.on_command (clamp vy=0)
              Twist, 20 Hz       50 Hz WALL timer: ControlState.step()
                                 ─► /swerve/joint_commands (JointState) ──► SwerveActuators plugin
                                     steering: position, drive: velocity        PD steer / PI drive → SetForce
                                 ◄─ /joint_states (JointState, 50 Hz sim) ◄──  publishes measured pos/vel/effort
                                 ◄─ /swerve/reset_epoch (UInt64, latched) ◄──  on load / reset / watchdog
                                 ─► /swerve/reset_ack   (UInt64, latched) ──►  unlocks drive for that epoch
                                                                                 ray sensor ─► /scan (LaserScan, 10 Hz sim)
                                                                                 multicamera ─► /stereo/left/*, /stereo/gazebo/right/*
 stereo_camera_relay: /stereo/gazebo/right/* → /stereo/right/* (frame + Tx fixed)
 stereo_processor: left+right (exact stamps) → /stereo/disparity, /stereo/depth/*, /stereo/points2
 robot_state_publisher: robot_description + /joint_states → /tf, /tf_static
 spawn_entity.py: spawns `proxy_rover` from topic robot_description at (0,0,0.36)
 rviz2 (optional, rviz:=true)
```

### Topics

| Topic | Type | Publisher → Subscriber | Notes |
| --- | --- | --- | --- |
| `/cmd_vel` | geometry_msgs/Twist | teleop (or any one source) → controller | unstamped; only `linear.x`, `angular.z` used; depth 1 |
| `/swerve/joint_commands` | sensor_msgs/JointState | controller → plugin | **internal** actuator API; 8 names; steering in `position`, drive in `velocity`; stamped with sim time; bypasses no-strafe policy if published directly |
| `/joint_states` | sensor_msgs/JointState | plugin → controller, RSP | measured feedback, all 8 joints, ~50 Hz sim time |
| `/swerve/reset_epoch` | std_msgs/UInt64 | plugin → controller | reliable, transient_local, depth 1 |
| `/swerve/reset_ack` | std_msgs/UInt64 | controller → plugin | reliable, transient_local, depth 1 |
| `/scan` | sensor_msgs/LaserScan | gazebo_ros ray plugin → RViz | frame `lidar_link`; reliable/volatile/depth 5 |
| `/stereo/{left,right}/image_raw`, `.../camera_info` | Image rgb8 / CameraInfo | plugin (left), relay (right) → processor, RViz | 480x360 @ 15 Hz; identical L/R stamps; reliable |
| `/stereo/disparity`, `/stereo/depth/image_raw` (32FC1 m, NaN invalid), `/stereo/depth/camera_info`, `/stereo/points2` | — | stereo_processor | frame `stereo_left_optical_frame`; reliable |
| `/stereo/gazebo/right/*` | — | plugin → relay | internal, uncorrected frame/Tx; do not use |
| `/robot_description`, `/tf`, `/tf_static` | — | robot_state_publisher | RSP owns *all* robot TF |

### TF tree (no `odom`/`map`/world frame exists — deliberately)

```
base_root (massless dummy, coincident)
└── base_link (chassis)
    ├── lidar_link            fixed, (0,0,0.20)
    ├── stereo_link           fixed, (0.52,0,-0.03), visual only, no collision/inertial
    │   ├── stereo_left_optical_frame    (0.03, +0.06, 0), rpy (-pi/2, 0, -pi/2)
    │   └── stereo_right_optical_frame   (0.03, -0.06, 0)
    ├── {fl,fr,rl,rr}_axle    via <prefix>_kingpin (continuous, Z axis)
    │   └── <prefix>_wheel    via <prefix>_wheel_joint (continuous)
```

Do **not** add a static world→base transform to fake odometry; docs say a future
odometry source must own that moving transform.

## 5. Launch arguments (`rover_desert.launch.py`)

| Arg | Default | Effect |
| --- | --- | --- |
| `gui` | `true` | Gazebo client; `false` = headless gzserver |
| `lidar` | `true` | passed to xacro `enable_lidar` |
| `rviz` | `false` | start rviz2 (sim time): `rover_sensors.rviz` if stereo on, else `rover_lidar.rviz` |
| `stereo` | `true` | stereo camera + relay + processor; independent of `lidar` |
| `stereo_width/height/rate/baseline/hfov` | `480/360/15/0.12/1.5708` | validated before start; warns if a frame > 512 KiB |
| `actuator_watchdog` | `0.3` | wall seconds; validated finite > 0; → plugin `<watchdog_timeout>` |
| `x y z roll pitch yaw` | `0 0 0.36 0 0 0` | spawn pose |

Xacro is processed in Python (`OpaqueFunction`) and the same URDF string goes to
the controller (`robot_description` param, read-only) and RSP. Both use
`use_sim_time: true`.

Usage:
```bash
ros2 launch rover_sim rover_desert.launch.py              # normal
ros2 launch rover_sim rover_desert.launch.py rviz:=true   # with RViz
ros2 launch rover_sim rover_desert.launch.py world:=desert gui:=false   # headless
ros2 run rover_sim rover_teleop                           # separate interactive terminal
```

## 6. Robot model (`urdf/proxy_rover.urdf.xacro`)

| Part | Values |
| --- | --- |
| Chassis `base_link` | box 1.0 × 0.5 × 0.2 m, 20 kg, Gazebo/Orange |
| Modules | kingpins at x = ±0.38, y = ±0.45, z = −0.15 (wheelbase 0.76, track 0.90) |
| Steering link (`_axle`) | sphere r 0.06, 1 kg (overrides upstream 0.1 / 10 kg) |
| Kingpin joint | **continuous**, axis Z, effort 25 Nm, velocity 3 rad/s, friction 0.02, damping 0.1 |
| Wheel | cylinder r 0.18; visual width 0.12; **collision width 0.02** (narrow tread to reduce ODE scrub); 1 kg |
| Wheel joint | continuous, origin rpy (π/2,0,0), axis (0,0,1) → effective spin axis **−Y**; effort 40 Nm, velocity 20 rad/s; damping 0.05, friction 0.02 |
| Wheel contact | mu1 = mu2 = 1.0, kp 1e6, kd 100, minDepth 0.001, maxVel 0.1 |
| LiDAR | `lidar_link` cylinder r 0.055 × 0.04, 0.15 kg, at (0,0,0.20) on base_link |

**Sign gotcha:** because the spin axis is −Y, **forward motion requires
negative wheel joint velocity** (`drive_sign = −1` for all modules).

Module/joint names: `front_left`, `front_right`, `rear_left`, `rear_right` +
`_kingpin` / `_wheel_joint`. The controller (via `kinematics.modules_from_urdf`)
and the plugin both find joints **by these suffixes**; renaming breaks both.

## 7. Kinematics (`rover_sim/kinematics.py`)

`modules_from_urdf(xml) -> list[Module]` (sorted by kingpin name) reads: kingpin
origin x/y, steering limits (±inf if continuous), steering velocity limit, wheel
velocity limit, wheel collision radius, and drive sign (Y component of rotated
spin axis). It **rejects** anything outside the supported layout: kingpin parent
must be `base_link`, zero rpy, axis exactly `"0 0 1"`, wheel centered on kingpin,
spin axis transverse (±Y), centered cylindrical collision, exactly 4 modules.

`inverse(modules, vx, vy, yaw_rate, previous, max_wheel_speed)`:
```
u_i = vx − wz·y_i ;  v_i = vy + wz·x_i
θ_i = atan2(v_i, u_i) ;  |v_i| = hypot(u_i, v_i)
choose angle θ_i + kπ nearest previous target (k ∈ nearest±1 for continuous);
  ties broken to mirror left/right (reduces reaction torque)
wheel_rate_i = (−1)^k · drive_sign_i · |v_i| / r_i
scale = min(1, min_i limit_i/|rate_i|), limit_i = min(URDF limit, max_wheel_speed)
```
Zero-speed modules keep their previous angle with zero rate. Raises `ValueError`
on nonfinite inputs.

## 8. Controller (`control_state.py` + `controller.py`)

### Parameters (`config/swerve.yaml`, validated atomically at startup and on set)

| Param | Default | Constraint |
| --- | --- | --- |
| `command_timeout` | 0.5 | ≥0, **wall** seconds; 0 inhibits drive |
| `feedback_timeout` | 0.5 | ≥0, **sim** seconds; 0 inhibits drive |
| `max_wheel_speed` | 12.0 rad/s | >0, ≤ max URDF wheel limit (20) |
| `wheel_acceleration` | 8.0 rad/s² | >0 |
| `alignment_tolerance` | 0.08 rad | 0 ≤ x < π/2 |
| `robot_description` | — | read-only |

### `ControlState.step(sim_now, wall_now)` logic (called every 20 ms wall)
1. If sim time went backwards → `reset()`.
2. `dt` = sim delta clamped to [0, 0.1] (pause ⇒ no integration).
3. If not initialized (no complete valid feedback yet) → return `None` (publish nothing; never fabricate steering targets).
4. Command used only if command is fresh (wall) **and** feedback is fresh (sim) & valid; else ZERO.
5. Inverse kinematics → targets/speeds; nonfinite/overflow ⇒ hold angles, zero speed.
6. Steering targets slew at the URDF steering velocity limit (3 rad/s).
7. **Alignment gate:** wheel speeds stay 0 until *all* measured steering angles are within `alignment_tolerance` of their targets; then ramp at `wheel_acceleration`. Zero command ⇒ speeds set to 0 immediately.

`on_command` clamps `vy` to 0 if all fields finite, else whole command = ZERO.
`on_feedback` validates names/lengths/finiteness/timestamps (monotonic, not more
than 0.1 s in the future); an invalid message immediately zeroes speeds. First
valid snapshot seeds the steering targets from measured (possibly multi-turn) angles.

### ROS node behavior (`controller.py`)
- Subscribes to `/cmd_vel` **only after** receiving the first reset epoch.
- On each new epoch: `state.reset()`, destroy and recreate the `/cmd_vel`
  subscription (drops queued stale Twists), then ack. Callbacks carry their
  generation and are ignored if stale.
- Custom SIGINT/SIGTERM handlers just set flags; `close()` publishes a final
  zero-drive frame (if initialized), waits ≤0.2 s for DDS ack, destroys the
  node exactly once. `rclpy.init(signal_handler_options=NO)`.

## 9. Gazebo plugin (`src/swerve_gazebo_plugin.cpp`, class `rover_sim::SwerveActuators`)

- Finds joints by suffix; requires exactly 8, else disables itself.
- SDF param `watchdog_timeout` (default 0.3, must be finite > 0).
- Epoch starts at steady-clock ns, increments on `Reset()`, sim-time rewind, or watchdog expiry.
- **Handshake:** on reset, hold measured steering + zero drive, publish epoch;
  drive stays inhibited until the controller acks that exact epoch. The ack sets
  a sim-time barrier; commands stamped ≤ barrier or > sim_now + 0.1 are dropped.
- Command frames are all-or-nothing: wrong sizes, unknown/duplicate names,
  nonfinite values ⇒ ignored. Values clamped to joint limits.
- **Wall-time watchdog:** no valid command for ≥ timeout ⇒ zero drive, clear
  integrators, new epoch (re-handshake required). Checked in both the physics
  update and the command callback (so a late callback can't revive an expired lease).
- Control law every physics step (1 kHz):
  - Steering PD: `τ = 200·(target − pos) − 8·vel`, clamped to effort limit (25 Nm).
  - Drive PI: `τ = 8·err + I`, `I += 30·err·dt` capped at ±0.5·effort (20 Nm), anti-windup, I reset when target = 0. Clamped to 40 Nm.
- Publishes `/joint_states` every ≥0.02 s sim time.

## 10. Teleop (`rover_sim/teleop.py`, node `rover_teleop`)

| Key | Twist |
| --- | --- |
| W / S | linear.x = ±`linear_speed` (0.4) |
| A / D | angular.z = ±`angular_speed` (0.5) |
| Space | zero |
| Ctrl+C | zero ×3 then exit 0 |
| anything else (incl. Q/E, arrows) | stop |

- Deadman: motion stops after `key_timeout` (0.7 s wall) without a repeat (terminals don't report key release).
- Publishes at `publish_rate` (20 Hz) plus immediately on change; always `linear.y = 0`.
- Uses cbreak + `select`; requires a TTY; restores terminal on any exit.
- Keys don't combine (no simultaneous forward+turn from keyboard).
- `KeyboardCommand` class is ROS-free and unit-tested.

## 11. LiDAR

Gazebo Classic CPU `ray` sensor + `libgazebo_ros_ray_sensor.so`. 720 samples,
−π..+π inclusive (increment 2π/719), range 0.12–30 m, Gaussian noise σ = 0.01 m,
10 Hz sim time, frame `lidar_link`, remap `~/out:=/scan`. `scan_time` and
`time_increment` are 0 (instantaneous). +inf = no return. Single horizontal
plane at ~0.53 m world height on flat ground — flat ground is invisible; hills
appear as arcs. Plugin publishes no TF.

## 11b. Stereo camera (details: `docs/stereo.md`)

- One Gazebo `multicamera` sensor on `stereo_link` (both images share one capture
  time) + `libgazebo_ros_camera.so` (`camera_name=stereo`, frame
  `stereo_left_optical_frame`, `hack_baseline=0`, `cx=(w-1)/2`, `cy=(h-1)/2`).
  That plugin uses one frame and one Tx for both cameras, so the right stream is
  remapped to `/stereo/gazebo/right/*` and fixed by `stereo_camera_relay`
  (baseline read from robot_description).
- `stereo_processor`: exact-stamp sync → `check_pair` (fail closed) → OpenCV
  SGBM (80 disparities, block 5) → texture filter (`min_texture` 4 grey levels)
  → 2 px edge erosion → depth `fx*B/d` (NaN invalid) → points (stride 2). No
  cv_bridge (broken with numpy 2 in `~/.local`); large message fields are assigned
  as `array('B')` (assigning `bytes` costs ~70 ms/MB in rclpy Humble).
- Why 480x360: 640x480 frames exceed Fast DDS's default 512 KiB shared-memory
  segment, and delayed the controller's reset handshake (hardening check failed 6/6).
- LiDAR keep-out (xacro args `lidar_keepout_*`): radius 0.15 m around the LiDAR
  + elevation band [-15-2, 15+2] deg, all azimuths. Xacro fails if the stereo
  bounding box enters it; tests check sampled visuals.
- Measured: usable depth 0.4–5 m on textured surfaces (median error 2 mm at
  <1 m, 17 cm at 4–5 m); RTF 0.9988 vs 0.9990 without stereo; LiDAR unchanged.

## 12. World (`worlds/desert.world`)

ODE quick solver, 150 iters, 1 ms step, 1000 Hz. Ground: static 100×100×0.5 m box
(top at z=0), mu 0.8. Hills are buried spheres (static, mu 0.8):

| Name | Center | Radius | Visible height |
| --- | --- | --- | --- |
| hill_west | (−10, 3, −5) | 6 | 1.0 m |
| hill_north | (0, 13, −6.5) | 8 | 1.5 m |
| hill_east | (12, 2, −8) | 10 | 2.0 m |

Origin is flat (spawn location). The ground **visual** uses the textured
`RoverSim/Sand` material (collision unchanged). Five static `stereo_feature_*`
boxes/cylinders (≤ 0.35 m tall, below the 2D LiDAR plane; ≥ 3.5 m from spawn; no
spheres, because `validate_lidar.py` treats every sphere as a hill) are listed
with exact poses in `docs/stereo.md`. Do not export `gazebo_media_path` from
package.xml: gazebo_ros would then replace `GAZEBO_RESOURCE_PATH` and break
camera rendering.

## 13. Clock semantics (important, non-obvious)

| Thing | Clock |
| --- | --- |
| Controller heartbeat timer (50 Hz) | wall (steady) — keeps running during pause |
| `/cmd_vel` freshness | wall monotonic |
| Feedback freshness | sim time |
| Steering slew, wheel acceleration | sim-time deltas |
| Plugin actuator watchdog | wall (steady) |
| Teleop rate / deadman | wall monotonic |
| `/scan`, `/joint_states`, `/stereo/*` stamps | sim time |

Consequence: heavy host load or suspend can starve wall-time heartbeats and trip
the 0.3 s watchdog (documented, observed; intentionally not masked by raising the timeout).

## 14. Tests and validation

**Unit tests** (`test/`, pytest, `PYTHONPATH` = source dir): 280 cases
(22 kinematics, 22 teleop, 73 controller, 14 LiDAR/RViz, 149 stereo in
`test_stereo_*.py`). Controller tests
exercise `ControlState` with explicit clocks and the ROS node with mocks; LiDAR
tests parse the expanded xacro and RViz config.

**Integration validators** (need a running launch; use an isolated
`ROS_DOMAIN_ID` + `GAZEBO_MASTER_URI`, and no other `/cmd_vel` publisher):

```bash
ROS_DOMAIN_ID=45 GAZEBO_MASTER_URI=http://127.0.0.1:11355 \
  ros2 launch rover_sim rover_desert.launch.py world:=desert gui:=false
# other terminal, same env:
ros2 run rover_sim validate_swerve.py   --output .../local_motion.json [--cases ... blocked_lateral_*]
ros2 run rover_sim validate_teleop.py   --output .../local_teleop.json
ros2 run rover_sim validate_hardening.py --controller-pid PID --watchdog 0.3 --output .../local_hardening.json
ros2 run rover_sim validate_lidar.py    --duration 10 --windows 3 --watchdog 0.3 --output .../local_lidar.json
ROS_DOMAIN_ID=46 ros2 run rover_sim validate_shutdown.py --output .../local_shutdown.json   # no Gazebo
# stereo: baseline on a stereo:=false launch, then the full run on a normal launch
ros2 run rover_sim validate_stereo.py --performance-only --output .../local_stereo_baseline.json
ros2 run rover_sim validate_stereo.py --compare .../local_stereo_baseline.json --output .../local_stereo.json
ros2 run rover_sim validate_launch_matrix.py --output .../local_launch_matrix.json --log-dir /tmp/m  # starts its own sims
```

Validators use Gazebo ground truth (model states) for checking only. Use
`local_*.json` names so committed `phase*.json` evidence isn't overwritten.
Current recorded result: 63 integration checks pass (`docs/validation/stereo.md`):
the 44 Phase 5 checks re-run with stereo on + 19 stereo checks.
Motion tolerances: 0.15 m translation, 0.08 rad heading, 0.10 rad steering.

**Project history (phases, from docs):** Phase 2 world + passive rover →
Phase 3 swerve controller + plugin (originally holonomic) → Phase 4 teleop →
Phase 4.5 hardening (reset handshake, watchdog, param validation, TF rename
`base_footprint`→`base_root`) → reliability fix (watchdog recovery, repeated
signals) → no-strafe policy → Phase 5 LiDAR + RViz → stereo camera. Older reports are historical
and may describe superseded behavior (e.g. strafing).

## 15. Known limitations (from README)

- Validated only on flat ground at low speed; slope mobility/parking not established.
- Rigid terrain; narrow 2 cm tread approximates a tire; no sand model.
- Steering alignment gate pauses drive on abrupt command changes; independent wheel ramps can cause transient drift.
- No odometry, localization, SLAM, or world-stable scan accumulation.
- One `/cmd_vel` source only; no mux/arbitration.
- Keyboard relies on terminal key-repeat.
- Wall-time watchdog can trip under host load.
- Idealized LiDAR (no dust, reflectivity, rolling-scan distortion).
- Stereo depth only on textured surfaces, ~0.4–5 m; plain hills/sky get (almost) no depth; small objects lose
  edges; frames > 512 KiB disturbed the controller reset handshake.
- Snap VS Code terminals may break RViz (GTK env) — workaround: `env -u GTK_PATH -u GTK_EXE_PREFIX -u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE rviz2 ...`; Gazebo rendering issues: `LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1 gzclient`.

## 16. Guidance for modifying the code

- **Logic lives in ROS-free modules** (`kinematics.py`, `control_state.py`,
  `KeyboardCommand`). Put new behavior there and unit-test it with explicit
  clocks; keep ROS nodes as thin wrappers.
- Geometry comes from the URDF at runtime — change dimensions in the xacro, not
  in Python. The parser is strict; non-standard layouts need parser changes.
- Keep joint suffixes `_kingpin` / `_wheel_joint` and the 4-module assumption
  (hard-coded in plugin: `actuators_.size() != 8`; controller: `len(result) != 4`).
- Adding a new `/cmd_vel` source (e.g. Nav2) requires a mux; only one publisher is supported.
- Adding odometry: publish `odom → base_root` (or `base_link`) from one owner;
  then RViz Fixed Frame could become `odom`.
- Gain changes are in the C++ plugin (hard-coded constants 200/8 and 8/30); rebuild needed.
- Respect the fail-closed style: invalid input ⇒ zero drive, never guess.
- After source changes: `colcon build --packages-select rover_sim` then re-source.
- Docs carry exact test counts; update README/docs counts if you add tests.
- New sensors: copy the stereo pattern — new xacro file behind its own `enable_*`
  arg, visual-only/massless links outside the LiDAR keep-out, topics under their
  own namespace, frames prefixed, fixed joints only (RSP owns TF), launch arg, and
  re-run the hardening validator (sensor traffic can disturb the reset handshake).
- Licensing: vendored files under `urdf/upstream/` are Apache-2.0 (AIT) with BSD
  inertia helpers; record any change to them in `THIRD_PARTY_NOTICES.md`.
