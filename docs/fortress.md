# Gazebo Fortress backend

`rover_mars.launch.py sim:=fortress` runs the same rover, controller, teleop,
LiDAR post-processing, RViz and validators on Gazebo Fortress (gz-sim 6.16,
DART physics) through `ros_gz_sim` and `ros_gz_bridge`, the Gazebo paired
with ROS 2 Humble. Gazebo Classic (`sim:=classic`) remains available; the
default is set by the measured comparison below.

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

## Running

```bash
ros2 launch rover_sim rover_mars.launch.py sim:=fortress world:=mars rviz:=true
ros2 launch rover_sim rover_mars.launch.py sim:=fortress world:=mars gui:=false   # headless (ogre2 headless rendering)
ros2 run rover_sim validate_step_climb.py --output /tmp/step.json --arg sim:=fortress
```

Every validator accepts `--arg sim:=fortress`. Fortress needs a display for
the GUI only; the headless server renders the GPU LiDAR with
`--headless-rendering` (EGL) on this Intel iGPU.

## Side-by-side results

Headless, same seed-7 world, same rover and controller, 2026-10-31. Classic
numbers are the committed Phase A–E results; Fortress numbers come from
`docs/validation/fortress/*.json` (run with `--arg sim:=fortress`).

| Measurement | Classic 11 (ODE) | Fortress 6.16 (DART) |
| --- | --- | --- |
| Terrain representation | heightfield (DEM), probe error median 7.8 mm | 36 mesh tiles, probe error median 7.4 mm |
| Spawn-pad rest | drift 0.26 mm, tilt 0.12° | drift 0.00 mm, tilt 0.12° |
| Mars world RTF at rest, no LiDAR / dense GPU LiDAR | 0.999 (0.32 cores) / 0.998 (0.76) | 0.982 (0.89 cores) / 0.962 (1.46) |
| RTF with 12 dynamic probe boxes on the terrain | 0.998 | 0.686 (DART box-vs-mesh contacts are expensive) |
| Step climb (0.8 m/s, 8 N m) | 5 cm 3/3, 10 cm 3/3, 15 cm 3/3, 20 cm 0/3, 25 cm 0/3, 30 cm 0/3, 40 cm 0/3 | 5 cm 3/3, 10 cm 3/3, 15 cm 3/3, 20 cm 0/3, 25 cm 0/3, 30 cm 0/3, 40 cm 0/3 |
| Slopes | climbs ≤ 25° (25° at 0.21 m/s), stalls 30°+ | climbs ≤ 30° (25° at 0.31 m/s, 30° at 0.12 m/s), stalls 35°+; holds ≤ 0.3 mm on 5–25°, 25 mm at 30° |
| Suspension: 4/4 contact bump / rock field; settling | 100 % / 98.8 %; 0.16 s | 100 % / 98.7 %; 0.18 s |
| GPU LiDAR lite: rate, ground σ (steep beams), box face | 10.0 Hz, 2.9 cm, 0.5 mm | 10.0 Hz, 19.5 cm, 0.4 mm |
| GPU LiDAR dense: rate, ground σ, RTF (flat) | 10.0 Hz (high), 2.6 cm, 0.999 | 7.6 Hz, 6.0 cm, 0.985 |
| Self-hits per sweep | 0 | 0 |
| Controller: circle / figure-eight cross-track RMS | 1.8 / 1.8 mm | 1.7 / 1.8 mm |
| Odometry drift (figure-eight) | 0.13 % / 0.24° | 0.08 % / 0.08° |
| Modes, stall warning, watchdog stop, reset | all pass; warning 1.4 s, stop 0.9 s | all pass; warning 1.4 s, stop 1.0 s |
| Traverse 111 m planned loop | 36/36, 0.57 m/s, tilt 19.8°, 0 stuck, RTF 0.999, odom drift 0.60 % / 1.1° | 36/36, 0.57 m/s, tilt 20.1°, 0 stuck, RTF 0.969 (1.51 cores), odom drift 2.58 % / 7.7° |

**GPU LiDAR accuracy on Fortress depends on the horizontal sample count**:
the ogre2 lidar renders a cube map whose resolution follows the samples, and
range residuals show a structured 4–10° azimuth pattern that grows towards
grazing beams: at 450 samples σ = 6.6 cm on the −15° beam and 36 cm at −7°;
at 1800 samples 2.5 cm and 7.8 cm. Classic's `gpu_ray` (after the label
correction) gives 2–3 cm on every beam. The flat-plane σ criterion of the
LiDAR validator (5 cm) is therefore **not met on Fortress** at ≤ 1024
samples; beam layout, box faces (≤ 1 mm) and self-hits pass. Use
`lidar_samples:=1800` on Fortress when range accuracy matters (rate drops
further on this iGPU: dense already falls to 7.6 Hz).

**Default backend stays Classic** for now: identical step outcomes, slightly
higher RTF, more accurate LiDAR ranges and tighter odometry on the test loop.
Fortress is fully usable (`sim:=fortress`), climbs one ramp step more (30°)
and reads the DEM more precisely through mesh tiles, at the price of a
heavier server (1.5 cores while driving) and the LiDAR accuracy caveat.

