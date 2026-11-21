# Gazebo Fortress port

Branch `Silvio-Fortress` moves rover_sim from Gazebo Classic 11 to Gazebo
Fortress (gz-sim 6, the Gazebo paired with ROS 2 Humble). It is Fortress
only; the last Classic version is the git tag `classic-final`. The port is
done in steps, each verified headless on the development laptop (one
simulator at a time) before it is committed. Measurements below are from
2026-11-21, Fortress 6.18.

| Step | What | Status |
| --- | --- | --- |
| 1 | Swerve actuators as a gz-sim System, parking hold, ground truth | done |
| 2 | Worlds: DART physics, gz-sim systems, GUI, PBR materials | done |
| 3 | Sensors: gpu_lidar 3D/2D LiDAR, two camera sensors, ros_gz_bridge | done |
| 4 | Launch files (ros_gz_sim, spawn, bridge), validators' Gazebo services | done |
| 5 | Full validation: tests, slope, planner drives, stereo/LiDAR validators | run; 3 open items below |

## 1. Swerve actuators (`src/swerve_gz_system.cpp`)

Same ROS interface as the Classic ModelPlugin: `/swerve/joint_commands` in,
`/joint_states` (8 actuated + 2 rocker joints) out, reset epochs, wall
watchdog, rocker differential. `/ground_truth/odom` now comes from the System
(Classic used `libgazebo_ros_p3d`), same message: `base_root` pose and
world-frame twist.

**Parking hold.** A zero or stale wheel target holds each wheel at the
position where the hold engaged (PD, kp 40, kd 6, limited to the joint's
effort limit), engaging at any speed. Classic only drove the wheel velocity to
zero with a P term, so the rover crept down slopes of 25° and more after
stopping. Real rover on a 30° ramp: 0.0 mm in 24 s (hold gains set to 0: it
rolls off the ramp within 2.5 s).

## 2. Worlds (`rover_sim/fortress_world.py`)

Every world (desert, mars, rover_planner's slope_test ramps) uses one header:
DART at 1 ms with the Bullet collision detector, the gz-sim systems that
Classic loaded implicitly, and a GUI with a Teleop panel on `/cmd_vel`.

- The Mars terrain was already a triangle mesh (not a `<heightmap>`), so
  Fortress's heightmap limits do not apply.
- ogre2 ignores OGRE material scripts: textures are PBR albedo maps. The
  desert ground visual is a mesh plane whose UVs repeat every 8 m, as the
  Classic material's texture scale did.
- Real rover on mars.world: rests at 0.346 m (Classic 0.345), drives 7.36 m
  in 15 s at 0.5 m/s, RTF 1.00 with Bullet or DART's default detector (no
  sensors). Bullet is kept: on feat/mars-overhaul it held RTF 0.98 against
  0.64 with many dynamic bodies on mesh terrain.

## 3. Sensors

| Sensor | Classic | Fortress |
| --- | --- | --- |
| 3D LiDAR | `ray` + `libgazebo_ros_ray_sensor` → `/points_raw` | `gpu_lidar`, gz `/rover/lidar3d/points` → bridged to `/points_raw` |
| 2D LiDAR | `ray` → `/scan` | `gpu_lidar`, gz `/rover/scan` → `/scan` |
| Stereo | one `multicamera` + `libgazebo_ros_camera`, relay fixes right frame/Tx | two `camera` sensors, gz `/stereo/gazebo/{left,right}/…` → relay → `/stereo/{left,right}/…` |

Bridged topics are in `config/ros_gz_bridge.yaml`. `lidar_postprocess`,
`stereo_processor` and RViz are unchanged.

**3D LiDAR range accuracy depends on samples per beam.** ogre2's gpu_lidar
looks ranges up in a cube map whose resolution follows the horizontal sample
count. Real rover in the desert, Airy dome (39 simulated beams), ground height
error of the raw cloud by horizontal distance (sensor noise σ 2 cm included):

| Samples | 1–3 m | 3–6 m | 6–10 m | 10–20 m | 20–50 m |
| --- | --- | --- | --- | --- | --- |
| 360 (Classic setting) | −0 ± 14 mm | −2 ± 17 mm | −3 ± 35 mm | −38 ± 46 mm | 38 ± 65 mm, 27% > 15 cm |
| **900 (default)** | −1 ± 13 mm | −3 ± 12 mm | −6 ± 18 mm | 3 ± 30 mm | −54 ± 43 mm |
| 1800 | −1 ± 13 mm | −3 ± 10 mm | −5 ± 10 mm | −1 ± 16 mm | −9 ± 53 mm |

(median ± standard deviation). Simulation speed on mars.world with the LiDAR
at 5 Hz and stereo at 2 Hz was RTF 1.00 at all three. The Airy profile now
uses 900 samples (Classic used 360 to save CPU ray cost; 900 was its original
value). Use `lidar_samples:=1800` for accurate returns beyond 10 m, at twice
the points per sweep for `lidar_postprocess` and the planner.

**Stereo.** gz-sim renders all due sensors in one pass, so left/right images
carry identical stamps (50/50 pairs) and exact-time sync works without a
multicamera sensor. Fortress's camera_info has the right Tx
(−fx·B = −28.8) but puts the principal point at (w/2, h/2); the relay rewrites
it to ((w−1)/2, (h−1)/2) for both cameras. Depth against the analytic truth
(real rover at rest in the desert, one frame, pixels not covered by the arm):

| Distance | 0.4–1 m | 1–2 m | 2–3 m | 3–4 m | 4–5 m |
| --- | --- | --- | --- | --- | --- |
| Valid (Classic) | 87% (69%) | 74% (75%) | 46% (73%) | 44% (66%) | 37% (67%) |
| Median error (Classic) | 2 mm (2) | 9 mm (9) | 58 mm (38) | 126 mm (92) | 235 mm (171) |

Close range matches Classic; beyond 2 m fewer pixels match and errors are
larger, though within the validator tolerance. Likely cause: Classic's sand
material used 16× anisotropic filtering, which ogre2 PBR materials do not
expose, so the ground texture blurs at grazing angles. To revisit in step 5.
The real rover's stowed arm also covers the right ~25% of the left image
(the Classic stereo results were measured on the proxy rover).

## 4. Launch files and Gazebo services (`rover_sim/sim_services.py`)

`desert.launch.py` runs `ign gazebo` (`gui:=false` adds `-s --headless-rendering`)
with this package's models and plugin library on the IGN_GAZEBO_* paths, and
one `ros_gz_bridge` for the topics in `config/ros_gz_bridge.yaml` plus the
world's services under fixed names, whatever the world is called:

| ROS service | gz service | Classic equivalent |
| --- | --- | --- |
| `/sim/spawn_entity` | `/world/<w>/create` | `/spawn_entity` (gazebo_ros) |
| `/sim/delete_entity` | `/world/<w>/remove` | `/delete_entity` |
| `/sim/set_entity_pose` | `/world/<w>/set_pose` | `/gazebo/set_entity_state`, `gz model -m` |
| `/sim/control` | `/world/<w>/control` | `gz world -p/-r/-t` |

`rover_desert.launch.py` spawns the robot with `ros_gz_sim create` from
`robot_description`. The validators and `slope_test` use the helpers in
`rover_sim/sim_services.py`; rover pose comes from `/ground_truth/odom`
instead of `gz topic -e .../pose/info`. Pause, reset and the slow-simulation
test call `/world/<w>/control` and `set_physics` (real-time factor 0.02 instead
of Classic's update rate 20 Hz) with `ign service`.

Isolation for validation runs is now `ROS_DOMAIN_ID` plus `IGN_PARTITION`
(was `GAZEBO_MASTER_URI`); every validator refuses to start without both.

Differences from Classic found while porting:
- `reset: {model_only: true}` is not supported by gz-sim 6 (it logs a warning
  and does nothing), so validate_hardening checks only full and time-only resets.
- `remove` answers true even for a model that does not exist (the request is
  queued; the error appears only in the Gazebo log).
- `set_pose` keeps the model's velocity; the validators settle after a teleport.

Checked live on desert.world headless: spawn/delete/teleport through the
bridge, all six default `validate_swerve` cases pass (translation and heading
errors under 1 mm / 1 mrad), and the launch shuts every process down cleanly.

## 5. Full validation (2026-11-21)

Every run used one headless simulator, isolated with `ROS_DOMAIN_ID` and
`IGN_PARTITION`; sensor validators on the proxy rover (as their Classic
records), slope and planner drives on the real rover. Unit tests: 374
rover_sim + 71 rover_planner pass.

| Validator | Result |
| --- | --- |
| validate_swerve (6 cases) | pass; translation/heading errors < 1 mm / 1 mrad |
| validate_lidar (2D, baseline + full) | pass (9/9), RTF 1.00 |
| validate_hardening | pass (10/10): pause, full and time-only reset, RTF 0.02, actuator watchdog, controller resume |
| validate_shutdown | pass |
| validate_launch_matrix | 4/4 topics, data, clean exits, no leftovers; 2/4 logged the shutdown race below |
| validate_teleop | 13/14; Space stop: wheel jitter 0.068 rad/s (limit 0.05) |
| validate_lidar3d (lite, default 5 Hz) | pass (12/12) at 450, 900 and 1800 samples, with the gpu_lidar accuracy rule below |
| validate_stereo | rates, sync, camera_info, TF pass; usable depth range 2 m (required 4 m) |
| slope_test | climbs, descends, parks, crosses and turns up to 35°; fails at 40° (Classic: slid at 39°) |
| validate_terrain hill / mesa / crater | goals reached 0.20–0.24 / 0.215 / 0.22 m off (Classic 0.23 / 0.22 / 0.20–0.26) |
| validate_planner | rates (stereo 2 Hz, /points 5 Hz, unchanged with the planner), path and drive pass; grid found 4–5 of 5 obstacles (rock post, see item 2) |

**Parking.** On the 25–35° ramps the parked rover moves 2–3 mm in 5 s
(Classic crept at 25° and above); descending, it stops within 0.04–0.10 m.

**Fixed while validating.** validate_stereo read its /clock history while the
callback appended (Fortress's /clock is 1 kHz), expected Classic's camera node
as the left image's publisher and Classic's right-camera Tx of 0;
validate_launch_matrix waited for Classic's ready message; the bridge
advertised `/scan` and the stereo topics even when those sensors were off
(desert.launch.py now bridges only enabled sensors); slope_test's world had
`--` inside an XML comment, which strict parsers reject.

### 3D LiDAR accuracy rule (`rover_sim/gpu_lidar_check.py`)

Classic's limits (median error < 4 cm, 95th percentile < 9 cm per object)
were set for the exact CPU `ray` sensor; Classic's own GPU sensor failed
them, and Fortress has only the GPU sensor (lite at 450 samples: ground
median 0.10 m, 95th percentile 6.2 m; hills 7-10 cm). Captures at 450, 900
and 1800 samples per beam separated two GPU effects:

| Effect | Measured | Allowance per point |
| --- | --- | --- |
| Beam direction quantised by the cube map | 99th percentile 0.43-0.47 horizontal sample spacings, at all three counts | R x one spacing / tan(grazing angle) |
| Spheres drawn as a mesh inside the surface | hills read long by 0.7-0.8% of the radius, independent of samples (a ~16-segment mesh) | up to r(1 - cos(pi/16)) / sin(incidence), long side only |
| Range noise (URDF) | sigma 0.02 m | 3 sigma |

An object passes when >= 95% of its points are within their allowance and
its median signed error is in [-4 cm, 4 cm + mesh gap]. Points outside it
whose beam, turned by one spacing, lands on another object are edge points
(a box against the ground behind) and are counted but not scored; points on
the world's stereo features are not scored either (before, beams stopped by
the crate counted as probe-box misses). The spacing and noise come from
`/robot_description`, so the rule follows `lidar_samples`.

Checked offline on the three captures with deliberate faults: a mount 3 cm
low or 5 cm high fails on the ground, a 3 deg yaw on the boxes and hills, at
every sample count (a 1 deg yaw, about one spacing at 450 samples, can pass;
the TF check covers the exact mount). Live: 12/12 at 450, 900 and 1800
samples, >= 99.6% of points within for every object, probe-box bias <= 2 mm.

### Open items

1. **Stereo depth beyond 2 m.** 2–3 m is 48% valid (needs 50%) and 4–5 m has
   a 0.378 m median against a 0.372 m tolerance; 0.4–2 m matches Classic
   (2 mm and 7 mm medians). ogre2 exposes no anisotropic filtering.
   In validate_planner the rock post 5.4 m ahead, beyond stereo range, got
   0–1 grid cells from the LiDAR alone (1 cell passes).
2. **Wheel jitter after a stop.** The parking hold leaves 0.01–0.07 rad/s of
   wheel-joint jitter for about 1 s (Classic ~0); the body does not move.
3. **Shutdown race.** Ctrl+C delivers SIGINT twice (process group, then
   launch); robot_state_publisher sometimes exits on the second one (exit −2)
   after its own shutdown. Classic hit the same race (swerve_controller).

**Validator rates follow the launch.** validate_lidar3d and validate_planner
read the expected sensor rates from `/robot_description` (`sim_services.
description_sensors`) instead of fixed 10/15 Hz, which `classic-final`'s
5 Hz LiDAR / 2 Hz stereo defaults had already broken. validate_planner checks
`/points` (3D) or `/scan` (2D), whichever LiDAR the robot has, and subscribes
to it reliably as the planner does: best effort measured 3 Hz of 5 because
one lost fragment drops a whole ~1 MB sweep. lidar_postprocess itself keeps
5 Hz at 360–1800 samples (44–65% of one core).

The hill drive's "gentle cells marked too
steep" check fails at the summit (19–23%, limit 10%); the cells are on the
rim of the large rock east of the summit, which the truth slope map (terrain
mesh only) counts as gentle.

## 6. USGS Utah survey terrain (`worlds/usgs.world`)

Leo's Classic integration (origin/feat/usgs-utah-terrain, 7f2a3b0) is
cherry-picked with its authorship, then ported:

- `scripts/import_usgs_terrain.py` writes the Fortress header from
  `fortress_world.py`; `--world-only` regenerates the world from
  `worlds/usgs.terrain.json` without the LAZ files or conversion venv.
- Each tile link's collision and visual have different names (`collision` /
  `visual`; SDF 1.8 rejects both named `ground`). Meshes, poses and the
  `ground.png` COLLADA texture are unchanged.
- `world:=usgs` is the default again, as on Leo's branch; `z:=auto` (default)
  resolves through `survey_terrain.spawn_height` into the `create` arguments.
- `validate_survey_terrain.py` uses `sim_services` (SpawnEntity/DeleteEntity,
  IGN_PARTITION isolation). Fortress run: 13/13, RTF 1.0
  ([report](validation/usgs_terrain.md)).
