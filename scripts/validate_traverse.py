#!/usr/bin/env python3
"""Long traverse across the Mars world: success, tilt, stuck events, RTF.

The rover follows a list of waypoints (default: a loop around the spawn pad
over the garden strip and the rolling ground to the north-west) with a
simple pure-pursuit heading controller on ground truth (the point is to load
the physics, not to test navigation). Reports distance, max tilt, stuck
events (no progress for 5 s), mean speed, RTF and gzserver CPU, plus the
odometry drift over the drive and an optional PNG sequence from the
``/points`` cloud if matplotlib is available.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.validation import metrics  # noqa: E402
from rover_sim.validation.harness import Observer, SimulationLaunch, truth_since, write_json  # noqa: E402

DEFAULT_WAYPOINTS = [(10.0, 0.0), (20.0, 8.0), (22.0, 22.0), (8.0, 30.0), (-10.0, 26.0), (-22.0, 14.0),
                     (-20.0, -4.0), (-8.0, -14.0), (6.0, -12.0), (0.0, 0.0)]
ROUTE_TARGETS = [(0.0, 34.0), (-32.0, 0.0), (0.0, -34.0), (0.0, 0.0)]   # loop through four quadrant goals


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def plan_route(terrain_dir, targets=ROUTE_TARGETS, cell=1.0, max_slope_deg=18.0, rock_clearance=1.2,
               spacing=3.0):
    """A* over the DEM slope map (cells steeper than ``max_slope_deg`` or next
    to a large rock are blocked, gentle slopes are cheap) through ``targets``.

    A rover driving a planned route is the realistic use of the terrain: the
    traverse then measures physics, traction and the sensors instead of the
    tilt limiter stopping the rover on a cliff it was pointed at blindly.
    Returns waypoints spaced about ``spacing`` metres apart.
    """
    import heapq
    import re
    import numpy as np
    from rover_sim.terrain.heightmap import diagnostics
    from rover_sim.terrain.pipeline import load_manifest, read_height_field
    manifest = load_manifest(terrain_dir)
    height = read_height_field(terrain_dir)
    size = manifest['size']
    pixel = size / (height.shape[0] - 1)
    slope = diagnostics(height, pixel)[0]
    factor = max(1, int(round(cell / pixel)))
    n = height.shape[0] // factor
    coarse = slope[:n * factor, :n * factor].reshape(n, factor, n, factor).mean(axis=(1, 3))
    cell = pixel * factor
    half = size / 2.0

    def to_cell(x, y):
        return int((half - y) / cell), int((x + half) / cell)          # row (north down), column

    def to_world(row, col):
        return (-half + (col + 0.5) * cell, half - (row + 0.5) * cell)

    blocked = coarse > max_slope_deg
    # Large rocks (hull collisions) from the world file: block their surroundings.
    world = Path(terrain_dir) / 'worlds' / 'mars_medium.world'
    if world.exists():
        text = world.read_text()
        for chunk in re.finditer(r'<model name="rocks_\d+_\d+"><static>true</static><pose>([-\d.]+) ([-\d.]+) ', text):
            cx, cy = float(chunk.group(1)), float(chunk.group(2))
            body = text[chunk.end():]
            body = body[:body.find('</model>')]
            for hull in re.finditer(r'<collision name="c\d+"><pose>([-\d.]+) ([-\d.]+) [^<]*</pose><geometry><mesh>', body):
                rx, ry = cx + float(hull.group(1)), cy + float(hull.group(2))
                r0, c0 = to_cell(rx, ry)
                reach = int(math.ceil(rock_clearance / cell))
                blocked[max(0, r0 - reach):r0 + reach + 1, max(0, c0 - reach):c0 + reach + 1] = True
    margin = int(3.0 / cell)
    blocked[:margin, :] = blocked[-margin:, :] = blocked[:, :margin] = blocked[:, -margin:] = True
    cost = 1.0 + (coarse / 10.0) ** 2

    def nearest_free(node):
        if not blocked[node]:
            return node
        for radius in range(1, n):
            candidates = [(node[0] + dr, node[1] + dc) for dr in range(-radius, radius + 1)
                          for dc in range(-radius, radius + 1)
                          if max(abs(dr), abs(dc)) == radius and 0 <= node[0] + dr < n and 0 <= node[1] + dc < n]
            free = [c for c in candidates if not blocked[c]]
            if free:
                return min(free, key=lambda c: math.hypot(c[0] - node[0], c[1] - node[1]))
        raise RuntimeError('no free terrain cell found')

    def astar(start, goal):
        start, goal = nearest_free(to_cell(*start)), nearest_free(to_cell(*goal))
        frontier = [(0.0, start)]
        came, best = {start: None}, {start: 0.0}
        while frontier:
            _, current = heapq.heappop(frontier)
            if current == goal:
                break
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    if dr == dc == 0:
                        continue
                    nxt = (current[0] + dr, current[1] + dc)
                    if not (0 <= nxt[0] < n and 0 <= nxt[1] < n) or blocked[nxt]:
                        continue
                    step = math.hypot(dr, dc) * cost[nxt]
                    candidate = best[current] + step
                    if candidate < best.get(nxt, math.inf):
                        best[nxt] = candidate
                        came[nxt] = current
                        heapq.heappush(frontier, (candidate + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1]), nxt))
        if goal not in came:
            raise RuntimeError('no route to the goal through terrain below the slope limit')
        path, node = [], goal
        while node is not None:
            path.append(to_world(*node))
            node = came[node]
        return path[::-1]

    # Goals: the farthest cell connected to the pad in each requested direction
    # (within ``goal_radius`` m), so every leg is reachable below the slope limit.
    start = to_cell(0.0, 0.0)
    reachable = {start}
    stack = [start]
    while stack:
        r, c = stack.pop()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                nb = (r + dr, c + dc)
                if 0 <= nb[0] < n and 0 <= nb[1] < n and nb not in reachable and not blocked[nb]:
                    reachable.add(nb)
                    stack.append(nb)
    cells = np.array(sorted(reachable))
    world_xy = np.array([to_world(r, c) for r, c in cells])
    goals = []
    for tx, ty in targets:
        if (tx, ty) == (0.0, 0.0):
            goals.append((0.0, 0.0))
            continue
        direction = np.array([tx, ty]) / math.hypot(tx, ty)
        projection = world_xy @ direction
        lateral = np.abs(world_xy @ np.array([-direction[1], direction[0]]))
        radius = math.hypot(tx, ty)
        candidates = (projection <= radius) & (lateral < 0.5 * radius)
        index = int(np.argmax(np.where(candidates, projection, -np.inf)))
        goals.append(tuple(float(v) for v in world_xy[index]))
    waypoints, previous = [], (0.0, 0.0)
    for target in goals:
        segment = astar(previous, target)
        accumulated = 0.0
        for a, b in zip(segment, segment[1:]):
            accumulated += math.hypot(b[0] - a[0], b[1] - a[1])
            if accumulated >= spacing:
                waypoints.append(b)
                accumulated = 0.0
        waypoints.append(segment[-1])
        previous = target
    return waypoints


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=0.6)
    parser.add_argument('--yaw-gain', type=float, default=1.2)
    parser.add_argument('--lookahead', type=float, default=1.5, help='waypoint reached radius (m)')
    parser.add_argument('--max-sim-seconds', type=float, default=900.0)
    parser.add_argument('--waypoints', type=str, help='x1,y1;x2,y2;... (default: a slope-aware planned loop)')
    parser.add_argument('--terrain-dir', type=Path, help='generated terrain directory (default: installed)')
    parser.add_argument('--max-route-slope', type=float, default=18.0, help='planner slope limit (deg)')
    parser.add_argument('--arg', action='append', default=[], help='extra launch arguments')
    parser.add_argument('--snapshots', type=Path, help='directory for PNG top-down snapshots of /points')
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    if args.waypoints:
        waypoints = [tuple(float(v) for v in pair.split(',')) for pair in args.waypoints.split(';')]
    else:
        terrain_dir = args.terrain_dir
        if terrain_dir is None:
            from ament_index_python.packages import get_package_share_directory
            terrain_dir = Path(get_package_share_directory('rover_sim')) / 'generated'
        waypoints = plan_route(terrain_dir, max_slope_deg=args.max_route_slope)
        print(f'planned {len(waypoints)} waypoints over gentle terrain, route length '
              f'{sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip([(0.0, 0.0)] + waypoints, waypoints)):.0f} m', flush=True)
    launch_args = ['gui:=false', 'world:=mars'] + args.arg
    result = dict(launch_args=launch_args, waypoints=waypoints, speed=args.speed)
    snapshots = []
    with SimulationLaunch('rover_mars.launch.py', launch_args, log_path=args.log) as launch:
        observer = Observer('traverse_validation')
        odom = []
        clouds = []
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from rclpy.qos import qos_profile_sensor_data
        observer.node.create_subscription(Odometry, '/odom', lambda m: odom.append(
            (m.header.stamp.sec + m.header.stamp.nanosec / 1e9, m.pose.pose.position.x, m.pose.pose.position.y,
             2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w))), 50)
        if args.snapshots:
            observer.node.create_subscription(PointCloud2, '/points', lambda m: clouds.append(m) if len(clouds) < 1 else None,
                                              qos_profile_sensor_data)
        try:
            observer.wait_for(lambda: observer.latest_truth() is not None and observer.latest_joints() is not None
                              and bool(odom), timeout=200, what='rover', launch=launch)
            observer.run_sim(3.0, timeout_factor=60.0)
            pids = launch.pids('gzserver')
            sampler = metrics.CpuSampler(pids)
            wall_start = time.monotonic()
            start = observer.sim_time()
            start_truth, start_odom = observer.latest_truth(), odom[-1]
            with observer.lock:
                clock_index = len(observer.clock_samples)
            stuck_events, last_progress_time, last_progress_pos = 0, start, (start_truth[2], start_truth[3])
            reached = []
            index = 0
            max_tilt = 0.0
            while index < len(waypoints) and observer.sim_time() - start < args.max_sim_seconds:
                g = observer.latest_truth()
                wx, wy = waypoints[index]
                dx, dy = wx - g[2], wy - g[3]
                distance = math.hypot(dx, dy)
                if distance < args.lookahead:
                    reached.append(dict(index=index, sim_time=observer.sim_time() - start))
                    index += 1
                    continue
                heading_error = wrap(math.atan2(dy, dx) - g[7])
                yaw_rate = max(-1.0, min(1.0, args.yaw_gain * heading_error))
                speed = args.speed * max(0.0, math.cos(heading_error)) if abs(heading_error) < math.pi / 2 else 0.0
                observer.command = (speed, 0.0, yaw_rate)
                observer.run_sim(0.2, timeout_factor=150.0)
                max_tilt = max(max_tilt, abs(g[5]), abs(g[6]))
                if math.hypot(g[2] - last_progress_pos[0], g[3] - last_progress_pos[1]) > 0.3:
                    last_progress_pos, last_progress_time = (g[2], g[3]), observer.sim_time()
                elif observer.sim_time() - last_progress_time > 5.0:
                    stuck_events += 1
                    last_progress_time = observer.sim_time()
                    # Try to free the rover: reverse briefly, then continue.
                    observer.run_sim(2.0, (-0.4, 0.0, 0.0))
                    last_progress_pos = (observer.latest_truth()[2], observer.latest_truth()[3])
                if args.snapshots and clouds and len(snapshots) < 6 and (observer.sim_time() - start) > 20 * len(snapshots):
                    snapshots.append((observer.sim_time() - start, clouds.pop(), (g[2], g[3], g[7])))
            observer.command = (0.0, 0.0, 0.0)
            observer.run_sim(1.0)
            elapsed_wall, cpu = sampler.finish()
            with observer.lock:
                clock_window = observer.clock_samples[clock_index:]
            samples = truth_since(observer, start)
            path_length = sum(math.hypot(b[2] - a[2], b[3] - a[3]) for a, b in zip(samples, samples[1:]))
            end_truth, end_odom = observer.latest_truth(), odom[-1]
            def body(dx, dy, yaw0):
                return (math.cos(yaw0) * dx + math.sin(yaw0) * dy, -math.sin(yaw0) * dx + math.cos(yaw0) * dy)
            t_body = body(end_truth[2] - start_truth[2], end_truth[3] - start_truth[3], start_truth[7])
            o_body = body(end_odom[1] - start_odom[1], end_odom[2] - start_odom[2], start_odom[3])
            drift = math.hypot(t_body[0] - o_body[0], t_body[1] - o_body[1])
            result.update(
                success=bool(index >= len(waypoints)), waypoints_reached=len(reached), reached=reached,
                sim_seconds=observer.sim_time() - start, wall_seconds=elapsed_wall, path_length_m=path_length,
                mean_speed_m_s=path_length / max(observer.sim_time() - start, 1e-9),
                max_tilt_deg=math.degrees(max_tilt), stuck_events=stuck_events,
                real_time_factor=metrics.real_time_factor(clock_window),
                gzserver_cpu_cores=sum(v for v in cpu.values() if v is not None),
                gzserver_rss_mb={str(p): metrics.memory_mb(p) for p in pids},
                odometry_drift_m=drift, odometry_drift_percent=100.0 * drift / max(path_length, 1e-9),
                odometry_yaw_error_deg=math.degrees(abs(wrap((end_odom[3] - start_odom[3]) - (end_truth[7] - start_truth[7])))),
                min_z=min(g[4] for g in samples), max_z=max(g[4] for g in samples))
        finally:
            observer.close()
    if args.snapshots and snapshots:
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            import numpy as np
            from rover_sim.lidar_postprocess import cloud_to_array
            args.snapshots.mkdir(parents=True, exist_ok=True)
            for k, (t, msg, pose) in enumerate(snapshots):
                pts = cloud_to_array(msg)
                fig, ax = plt.subplots(figsize=(6, 6))
                ax.scatter(pts['x'], pts['y'], c=pts['z'], s=1, cmap='viridis')
                ax.set_aspect('equal')
                ax.set_title(f'/points top-down, t={t:.0f} s, pose=({pose[0]:.1f}, {pose[1]:.1f})')
                ax.set_xlim(-30, 30)
                ax.set_ylim(-30, 30)
                fig.savefig(args.snapshots / f'traverse_{k:02d}.png', dpi=80)
                plt.close(fig)
            result['snapshots'] = [str(args.snapshots / f'traverse_{k:02d}.png') for k in range(len(snapshots))]
        except Exception as error:  # noqa: BLE001
            result['snapshot_error'] = repr(error)
    result['passed'] = bool(result.get('success') and result.get('real_time_factor', 0) >= 0.9
                            and result.get('stuck_events', 99) <= 2)
    write_json(args.output, result)
    print(json.dumps({k: v for k, v in result.items() if k not in ('reached', 'waypoints')}), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
