# Phase D validation: 3-D LiDAR, IMU, RViz (2026-10-17)

```bash
ros2 run rover_sim validate_lidar3d.py --output docs/validation/mars_lidar.json --profiles lite standard high
ros2 run rover_sim validate_lidar3d.py --output docs/validation/mars_lidar_gpu.json --profiles lite --gpu true
```

Flat world, rover at rest, 1 m test cube 6 m ahead, 20 sweeps per profile,
two 10 s performance windows. **All three CPU profiles pass**
([mars_lidar.json](mars_lidar.json)).

| Profile | rate | points/sweep | lowest beam / spacing | ground bias / σ (raw, ≤ −7° channels, tilt removed) | box face bias / σ | processed σ | self-hits raw / processed per sweep | median RTF | gzserver + post-processing cores |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `lite` | 10.00 Hz | 3600 | -15.0° / 2.00° (expected 2.00°) | +1.9 mm / 2.0 cm | +0.1 mm / 2.0 cm (1200 pts) | 2.3 cm | 0 / 0 | 0.944 | 0.62 + 0.41 |
| `standard` | 10.00 Hz | 7200 | -15.0° / 2.00° (expected 2.00°) | +1.6 mm / 2.0 cm | +0.6 mm / 2.0 cm (2400 pts) | 2.3 cm | 0 / 0 | 0.892 | 0.68 + 0.41 |
| `high` | 10.01 Hz | 16114 | -22.5° / 1.50° (expected 1.45°) | +1.4 mm / 2.0 cm | -0.6 mm / 2.0 cm (3435 pts) | 2.3 cm | 0 / 0 | 0.728 | 0.78 + 0.42 |

Upward beams have no return on the flat world, so only the 8 (16) downward
channels appear; their lowest angle and spacing match the configuration.
The raw residual σ equals the configured 2 cm Gaussian; the processed cloud
adds the range-dependent term (σ = 1 cm + 0.3 mm/m) and 2 % + 0.2 %/m
dropout, and carries `ring` and `time`.

`gpu_ray` with the one-step vertical label correction (`validate_lidar3d.py --gpu true`,
[mars_lidar_gpu.json](mars_lidar_gpu.json)): **all three profiles pass**.

| Profile | points/sweep | lowest beam / spacing | ground bias / σ | box face bias | self-hits | RTF (flat) | gzserver + post cores | RTF on Mars world (at rest) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `lite` | 3402 | -15.0° / 2.00° | +1.9 mm / 2.9 cm | +0.5 mm | 0 | 0.999 | 0.51 + 0.27 | 0.998 (0.55 cores) |
| `standard` | 6812 | -15.0° / 2.00° | +1.7 mm / 2.9 cm | +0.6 mm | 0 | 0.999 | 0.48 + 0.32 | 0.999 (0.57 cores) |
| `high` | 15373 | -22.5° / 1.50° | +3.6 mm / 2.6 cm | -1.1 mm | 0 | 0.999 | 0.48 + 0.45 | 0.999 (0.56 cores) |

Before the correction the same sensor reported 0.7–45 m ground range bias
(points labelled one beam step too low; measured 0.99–1.00 steps on 16- and
32-channel layouts). The CPU `ray` sensor hangs gzserver on the heightmap
world in every tested configuration (16×450, 1×450, 16×450 at 30 m, 4×200,
low rock density), so `lidar_gpu:=auto` selects `gpu_ray` on `world:=mars`.

IMU: `/imu/data` at 100 Hz was consumed by the controller validation
(tilt limits, gyro yaw for odometry, stall fusion). RViz: configuration
checked by unit tests (`odom` frame, Boxes 0.05 m, 3 s decay, AxisColor);
no interactive GUI check was possible in this headless session.

Mars-world RTF per GPU profile: `mars_gpu_lidar_rtf_*.json`; the driving load is in [mars_traverse.md](mars_traverse.md).
