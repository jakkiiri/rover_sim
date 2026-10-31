# Gazebo Fortress backend

`rover_mars.launch.py` runs on Gazebo Fortress (gz-sim 6.16, DART physics)
through `ros_gz_sim` and `ros_gz_bridge`, the Gazebo paired with ROS 2
Humble, by default (`sim:=fortress`); Gazebo Classic (`sim:=classic`)
remains available and is measured side by side below.

## Native teleoperation

The Fortress worlds carry a `<gui>` block with gz-sim's own `Teleop` panel
("Rover teleop"), which publishes `ignition.msgs.Twist` on the gz topic
`/cmd_vel`; the launch bridges it one way into ROS (`/cmd_vel`,
`geometry_msgs/Twist`) so the unchanged `swerve_controller` drives the
rover. Tick **Keyboard** in the panel and hold W/A/S/D or the arrow keys
(forward/back map to `linear.x`, left/right to `angular.z`), or set the
speeds with the sliders and use the buttons. Releasing a key publishes a
zero twist and the controller's 0.5 s command deadman stops the rover if the
GUI stops publishing for any reason. `CameraTracking` follows the rover
(the launch calls `/gui/follow` 8 s after start; the GUI also offers
right-click → Follow on the rover). The terminal teleop
(`rover_teleop`) and gamepad node still work on both backends.

## What is shared and what is backend-specific

| Shared (unchanged) | Classic only | Fortress only |
| --- | --- | --- |
| rover URDF/xacro (geometry, inertia, suspension, friction tags: sdformat's URDF parser serves both) | `librover_swerve_actuators.so` (ModelPlugin), `libgazebo_ros_ray_sensor`/`imu` plugins, `.world` files with `gazebo_ros_state`, heightfield collision | `librover_swerve_gz_system.so` (gz-sim System with an embedded rclcpp node, built as a CMake sub-project in `gz_system/`), `gpu_lidar`/`imu` sensors with `<topic>`, `ros_gz_bridge` for `/clock`, `/points_raw`, `/imu/data`, `.sdf` 1.8 worlds with gz-sim systems, **tiled mesh collision** |
| `swerve_controller`, odometry, teleop, `lidar_postprocess`, RViz config | gpu_ray vertical-label correction | none needed (exact beam labels) |
| validators and the harness (`EntityControl` abstracts teleport/spawn/remove/reset) | `gazebo_ros_state` services | `ign service /world/<name>/{set_pose,create,remove,control}` |

The xacro argument `sim_backend` selects the sensor and plugin blocks; the
launch sets it from `sim`.

## Measured findings that shaped the port (2026-10-31)

- **Heightmaps.** Fortress misreads 16-bit PNG like Classic, cannot open the
  GeoTIFF DEM at all (gz-common 4 has no GDAL path: "unknown image format"),
  and 8-bit PNG quantises the 15 m relief into 6 cm steps. The Fortress
  worlds therefore use the DEM as **36 tiled OBJ collision meshes** (2:1
  decimated, 3 698 triangles each, with per-vertex normals) under the same
  textured visual mesh. Probe boxes land within a median 3 mm (max 17 mm) of
  the DEM, better than Classic's heightfield (7.8 mm).
- **DART requires normals in collision meshes**: hull meshes without `vn`
  lines crash the ODE mesh loader inside DART (segmentation fault at load);
  the generator now writes normals into every collision OBJ.
- **GPU LiDAR** on Fortress (ogre2, `--headless-rendering`) publishes an
  organised cloud with `x y z intensity ring` and exact beam labels, so the
  Classic-only vertical-shift correction is disabled; the post-processor
  still adds `time`, dropout and range-dependent noise.
- The System plugin cannot be built in the same CMake project as the Classic
  plugin (both dependency trees import the `JsonCpp` target); it is built by
  `ExternalProject` from `gz_system/` when `ignition-gazebo6` is installed and
  skipped otherwise.
- gz-sim 6 has no `ISystemReset`; a world reset is detected as simulation time
  moving backwards, which clears the actuator epoch exactly as on Classic.
- **Physics settings** (measured on the Mars world, rover driving with 12
  dynamic probe boxes): DART with its default ODE collision detector at 1 ms
  ran at RTF 0.64 (1.07 cores); the **Bullet collision detector at a 2 ms
  step** runs at 0.98 (0.62 cores) with the same rest height (4.496 m) and
  the same distance driven. Alternatives: Bullet/1 ms 0.92, ODE/2 ms 0.93.
  The Fortress worlds use Bullet/2 ms (`fortress.physics_block`); every
  validator below was re-run with it.
- **Point clouds need reliable QoS.** The 10 Hz `gpu_lidar` sweeps are large
  (230–900 kB) and fragmented; best-effort subscribers received 4.3 Hz
  (`dense`) to 8.1 Hz (`lite`) of the 10.0 Hz published, a reliable
  subscriber all of it with no gaps. `lidar_postprocess`, the LiDAR validator
  and the RViz config now subscribe reliably on both backends (both
  publishers are reliable), which removed the apparent Fortress rate drop.
- **LiDAR range accuracy follows the horizontal sample count** (the ogre2
  lidar looks ranges up in a cube map): 450 samples gave σ 19 cm on the steep
  beams, 1800 samples 2.6 cm on the −15° beam. The Fortress profiles carry
  1800 samples (`standard`: 2700); all four pass the 5 cm criterion at
  10.0 Hz and RTF ≥ 0.987 (`docs/lidar.md`).

## Running

```bash
ros2 launch rover_sim rover_mars.launch.py world:=mars                 # Fortress GUI with the Rover teleop panel
ros2 launch rover_sim rover_mars.launch.py world:=mars rviz:=true
ros2 launch rover_sim rover_mars.launch.py world:=mars gui:=false      # headless (ogre2 headless rendering)
ros2 run rover_sim validate_step_climb.py --output /tmp/step.json     # validators default to Fortress too
ros2 run rover_sim validate_step_climb.py --output /tmp/step.json --arg sim:=classic
```

`sim` defaults to `fortress` in the launch and in the validation harness;
pass `sim:=classic` (launch) or `--arg sim:=classic` (validators) for Gazebo
11. Fortress needs a display for the GUI only; the headless server renders
the GPU LiDAR with `--headless-rendering` (EGL) on this Intel iGPU.

## Side-by-side results

Headless, same seed-7 world, same rover and controller, 2026-10-31. Classic
numbers are the committed Phase A–E results; Fortress numbers come from
`docs/validation/fortress/*.json`, re-run after the switch to Bullet/2 ms
physics, reliable cloud QoS and the 1800-sample LiDAR profiles.

| Measurement | Classic 11 (ODE) | Fortress 6.16 (DART) |
| --- | --- | --- |
| Terrain representation | heightfield (DEM), probe error median 7.8 mm | 36 mesh tiles, probe error median 7.8 mm |
| Spawn-pad rest | drift 0.26 mm, tilt 0.12° | drift 0.00 mm, tilt 0.12° |
| Mars world RTF at rest, no LiDAR / dense GPU LiDAR | 0.999 (0.32 cores) / 0.998 (0.76) | 0.984 (0.65 cores) / 0.990 (1.12) |
| RTF with 12 dynamic probe boxes on the terrain | 0.998 | 0.990 (0.57 cores; Bullet detector, 2 ms step; was 0.686 with the ODE detector at 1 ms) |
| Step climb (0.8 m/s, 8 N m) | 5 cm 3/3, 10 cm 3/3, 15 cm 3/3, 20 cm 0/3, 25 cm 0/3, 30 cm 0/3, 40 cm 0/3 | 5 cm 3/3, 10 cm 3/3, 15 cm 3/3, 20 cm 0/3, 25 cm 0/3, 30 cm 0/3, 40 cm 0/3 |
| Slopes | climbs ≤ 25° (25° at 0.21 m/s), stalls 30°+ | climbs ≤ 30° (25° at 0.31 m/s, 30° at 0.12 m/s), stalls 35°+; holds ≤ 0.3 mm on 5–25°, 25 mm at 30° |
| Suspension: 4/4 contact bump / rock field; settling | 100 % / 98.8 %; 0.16 s | 100 % / 99.0 %; 0.20 s |
| GPU LiDAR lite: rate, ground σ (steep beams), box face, RTF (flat) | 16×450: 10.0 Hz, 2.9 cm, 0.5 mm, 0.999 | 16×1800: 10.0 Hz, 4.8 cm, 0.0 mm, 0.988 |
| GPU LiDAR dense: rate, ground σ, RTF (flat) | 64×1024: 10.0 Hz, 2.6 cm, 0.998 | 64×1800: 10.0 Hz, 3.5 cm, 0.991 (0.45 cores) |
| Self-hits per sweep | 0 | 0 |
| Controller: circle / figure-eight cross-track RMS | 1.8 / 1.8 mm | 2.0 / 1.5 mm |
| Odometry drift (figure-eight) | 0.13 % / 0.24° | 0.12 % |
| Modes, stall warning, watchdog stop, reset | all pass; warning 1.4 s, stop 0.9 s | all pass; warning 1.3 s, stop 0.7 s |
| Traverse 111 m planned loop (dense LiDAR on) | 36/36, 0.57 m/s, tilt 19.8°, 0 stuck, RTF 0.999, odom drift 0.60 % / 1.1° | 36/36, 0.57 m/s, tilt 20.1°, 0 stuck, RTF 0.990 (1.13 cores), odom drift 2.86 % / 8.6° |

**Fortress is the default backend** (user request, 2026-10-31): it reads the
DEM more precisely through mesh tiles, climbs one ramp step more (30°), has
exact LiDAR beam labels, and with Bullet/2 ms physics and the LiDAR/QoS fixes
above its RTF and LiDAR accuracy match Classic within the measurement noise.
Classic remains available with `sim:=classic` and is still validated.
Remaining honest difference: wheel odometry drifts more on Fortress over the
rough 111 m loop (2.9 % / 8.6° against 0.6 % / 1.1°), while the flat-world
figure-eight drift is the same (0.12 %); the wheels slip more on the mesh
tiles under DART's contact model than on the ODE heightfield.
