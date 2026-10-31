# Phase E validation: Mars traverse (2026-10-31)

```bash
ros2 run rover_sim validate_traverse.py --output docs/validation/mars_traverse.json --snapshots docs/validation/traverse
```

`world:=mars` (seed 7, medium rocks, 3.71 m/s²), GPU `lite` LiDAR
(`lidar_gpu:=auto`), headless. The validator plans a loop over gentle
terrain with A* on the DEM slope map (cells above 18° mean slope and the
surroundings of hull-collision rocks are blocked), then follows the
waypoints with a pure-pursuit heading controller on ground truth at
0.6 m/s. **Passed** ([mars_traverse.json](mars_traverse.json)).

| Metric | Value |
| --- | --- |
| route | 36 waypoints, four quadrant goals, back to the pad |
| waypoints reached | 36 / 36 |
| path length / simulated time | 111.3 m / 197 s (mean 0.57 m/s of 0.6 commanded) |
| max body tilt | 19.8° (tilt slowdown starts at 15°, cutoff 32°) |
| stuck events (no progress for 5 s) | 0 |
| real-time factor while driving (LiDAR + physics + controller) | **0.999**, gzserver 0.55 cores, 859 MB |
| wheel odometry vs ground truth over the loop | 0.67 m = **0.60 %** of distance, yaw error 1.07° |
| elevation range driven | 2.5 – 5.2 m (base_link) |

Snapshots of `/points` (top-down, every 20 s): [traverse_00](traverse/traverse_00.png),
[traverse_02](traverse/traverse_02.png), [traverse_04](traverse/traverse_04.png).

A first attempt with ten blind straight-line waypoints
([partial/traverse_blind_waypoints.out](partial/traverse_blind_waypoints.out))
reached 8/10 in 600 s with 60 stuck events: the route crossed 30° slopes
where the controller's tilt limiter stopped the rover (max tilt 31.9°,
right at the 32° cutoff), and the gyro bias (0.0005 rad/s) integrated to a
19.6° odometry yaw error over 600 s. The RTF was 0.999 there too; the
rover and the budget were fine, the waypoints were not.

Mars-world cost at rest (`measure_performance.py`): no LiDAR RTF
0.999 (0.32 cores); GPU lite / standard / high
0.998 / 0.999 / 0.999 (0.55 / 0.57 / 0.56 cores).
