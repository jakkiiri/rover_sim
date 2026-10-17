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


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=0.6)
    parser.add_argument('--yaw-gain', type=float, default=1.2)
    parser.add_argument('--lookahead', type=float, default=1.5, help='waypoint reached radius (m)')
    parser.add_argument('--max-sim-seconds', type=float, default=600.0)
    parser.add_argument('--waypoints', type=str, help='x1,y1;x2,y2;...')
    parser.add_argument('--arg', action='append', default=[], help='extra launch arguments')
    parser.add_argument('--snapshots', type=Path, help='directory for PNG top-down snapshots of /points')
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    waypoints = DEFAULT_WAYPOINTS if not args.waypoints else [
        tuple(float(v) for v in pair.split(',')) for pair in args.waypoints.split(';')]
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
            observer.run_sim(3.0)
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
                observer.run_sim(0.2)
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
