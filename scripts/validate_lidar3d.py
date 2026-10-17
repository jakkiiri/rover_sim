#!/usr/bin/env python3
"""Validate the 3-D LiDAR: rate, points per sweep, beam angles, ranges against
known geometry, noise, self-hits and the CPU/RTF cost of each profile.

For every requested profile (and GPU setting) the flat world is launched with
the rover at rest, a 1 m test box is spawned 6 m ahead, and ``--sweeps``
clouds are analysed from both the raw sensor (``/points_raw``) and the
post-processed ``/points`` topic.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import threading
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.lidar_postprocess import cloud_to_array  # noqa: E402
from rover_sim.validation import metrics  # noqa: E402
from rover_sim.validation.harness import EntityControl, Observer, SimulationLaunch, write_json  # noqa: E402

PROFILES = {'lite': (16, 450, 30.0), 'standard': (16, 900, 30.0), 'high': (32, 1024, 45.0)}
BOX = dict(x=6.0, size=1.0)      # 1 m cube centred 6 m ahead of base_link, near face at x = 5.5
LIDAR_X = 0.33                   # mast position ahead of base_link (body_length/2 - 0.12)
REST_HEIGHT = 0.41


def analyse(clouds, channels, vfov_deg, sensor_height, min_range):
    """Geometry/noise/self-hit statistics over a list of structured arrays.

    Upward beams have no return on the flat world, so the vertical layout is
    checked through the lowest beam angle and the beam spacing of the
    channels that do return. Range noise is estimated on channels at or
    below -7 deg, where a range error is not amplified by grazing incidence.
    """
    counts, ground_errors, steep_errors, box_errors, self_hits, elevations = [], [], [], [], 0, set()
    half = BOX['size'] / 2
    near_face = BOX['x'] - half - LIDAR_X
    step = math.radians(vfov_deg) / (channels - 1)
    for points in clouds:
        xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float64)
        finite = np.all(np.isfinite(xyz), axis=1)
        xyz = xyz[finite]
        r = np.linalg.norm(xyz, axis=1)
        counts.append(int(len(xyz)))
        azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
        elevation = np.arcsin(np.clip(xyz[:, 2] / np.maximum(r, 1e-9), -1, 1))
        elevations.update(np.round(np.degrees(elevation), 1).tolist())
        on_box = (np.abs(azimuth) < math.atan2(half, near_face) * 0.9) & (xyz[:, 0] > near_face - 1.0) & (xyz[:, 0] < near_face + 1.5)
        down = (elevation < math.radians(-2.0)) & ~on_box & (r > min_range)
        expected = sensor_height / np.sin(-elevation[down])
        ground_errors.extend((r[down] - expected).tolist())
        # Per-channel noise: remove the azimuth-dependent term that a residual
        # body tilt of 0.05 deg produces at grazing incidence, keep the residual.
        for channel in np.unique(np.round(np.degrees(elevation[down]), 1)):
            steep = down & (np.abs(np.degrees(elevation) - channel) < 0.3) & (elevation <= math.radians(-7.0))
            if steep.sum() < 20:
                continue
            err = r[steep] - sensor_height / np.sin(-elevation[steep])
            basis = np.stack([np.cos(azimuth[steep]), np.sin(azimuth[steep]), np.ones(int(steep.sum()))], axis=1)
            coefficients = np.linalg.lstsq(basis, err, rcond=None)[0]
            steep_errors.extend((err - basis @ coefficients + coefficients[2]).tolist())
        face = on_box & (xyz[:, 2] > -sensor_height + 0.05) & (xyz[:, 2] < -sensor_height + BOX['size'] - 0.05) & \
            (np.abs(xyz[:, 0] - near_face) < 0.3)
        expected_box = near_face / (np.cos(azimuth[face]) * np.cos(elevation[face]))
        box_errors.extend((r[face] - expected_box).tolist())
        self_hits += int(np.sum((np.abs(xyz[:, 0] - LIDAR_X) < 0.7) & (np.abs(xyz[:, 1]) < 0.5) & (xyz[:, 2] < -0.15) & (r > min_range)))
    ground_errors, steep_errors, box_errors = np.array(ground_errors), np.array(steep_errors), np.array(box_errors)
    seen = sorted(elevations)
    spacings = np.diff(seen) if len(seen) > 1 else np.array([])
    return dict(points_per_sweep_mean=float(np.mean(counts)), points_per_sweep_min=int(min(counts)),
                points_per_sweep_max=int(max(counts)), vertical_channels_with_returns=len(seen),
                lowest_beam_deg=seen[0] if seen else None,
                beam_spacing_deg=float(np.median(spacings)) if len(spacings) else None,
                expected_spacing_deg=math.degrees(step),
                ground_returns=int(len(ground_errors)), ground_range_bias_m=float(np.mean(ground_errors)) if len(ground_errors) else None,
                steep_ground_sigma_m=float(np.std(steep_errors)) if len(steep_errors) else None,
                steep_ground_bias_m=float(np.mean(steep_errors)) if len(steep_errors) else None,
                box_returns=int(len(box_errors)), box_range_bias_m=float(np.mean(box_errors)) if len(box_errors) else None,
                box_range_sigma_m=float(np.std(box_errors)) if len(box_errors) else None,
                self_hits_total=self_hits, self_hits_per_sweep=self_hits / len(clouds))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profiles', nargs='+', default=['lite', 'standard', 'high'])
    parser.add_argument('--gpu', nargs='+', default=['false'])
    parser.add_argument('--sweeps', type=int, default=20)
    parser.add_argument('--duration', type=float, default=10.0, help='wall seconds per performance window')
    parser.add_argument('--windows', type=int, default=2)
    parser.add_argument('--log-dir', type=Path)
    args = parser.parse_args()
    from sensor_msgs.msg import PointCloud2
    from rclpy.qos import qos_profile_sensor_data
    results = []
    for gpu in args.gpu:
        for profile in args.profiles:
            channels, samples, vfov = PROFILES[profile]
            log = args.log_dir / f'lidar_{profile}_gpu{gpu}.log' if args.log_dir else None
            launch_args = ['gui:=false', 'world:=flat', f'lidar_profile:={profile}', f'lidar_gpu:={gpu}']
            record = dict(profile=profile, gpu=gpu, channels=channels, samples=samples, vertical_fov_deg=vfov)
            with SimulationLaunch('rover_mars.launch.py', launch_args, log_path=log) as launch:
                observer = Observer(f'lidar3d_validation_{profile}')
                raw, processed = [], []
                lock = threading.Lock()

                def on_raw(msg):
                    with lock:
                        raw.append((time.monotonic(), msg))

                def on_processed(msg):
                    with lock:
                        processed.append((time.monotonic(), msg))

                observer.node.create_subscription(PointCloud2, '/points_raw', on_raw, qos_profile_sensor_data)
                observer.node.create_subscription(PointCloud2, '/points', on_processed, qos_profile_sensor_data)
                try:
                    observer.wait_for(lambda: observer.latest_truth() is not None, timeout=150, what='rover', launch=launch)
                    control = EntityControl(observer)
                    control.spawn_box('lidar_test_box', BOX['x'], 0.0, BOX['size'] / 2, BOX['size'], BOX['size'], BOX['size'])
                    try:
                        observer.wait_for(lambda: len(raw) >= 3 and len(processed) >= 3, timeout=60,
                                          what='/points_raw and /points', launch=launch)
                    except RuntimeError as error:
                        record.update(error=str(error), passed=False)
                        results.append(record)
                        print(json.dumps(record), flush=True)
                        continue
                    observer.run_sim(2.0)
                    with lock:
                        raw.clear()
                        processed.clear()
                    observer.wait_for(lambda: len(raw) >= args.sweeps and len(processed) >= args.sweeps,
                                      timeout=max(60.0, args.sweeps * 2.0), what='sweeps', launch=launch)
                    with lock:
                        raw_msgs = [m for _, m in raw[:args.sweeps]]
                        proc_msgs = [m for _, m in processed[:args.sweeps]]
                    stamps = [m.header.stamp.sec + m.header.stamp.nanosec / 1e9 for m in raw_msgs]
                    rate_sim = (len(stamps) - 1) / (stamps[-1] - stamps[0])
                    truth = observer.latest_truth()
                    sensor_height = truth[4] + 0.11 + 0.02 + 0.60 + 0.04   # base_link z + mast geometry
                    record['sensor_height_m'] = sensor_height
                    record['ground_hit_distance_at_min_elevation_m'] = sensor_height / math.tan(math.radians(vfov / 2))
                    record['publish_rate_sim_hz'] = rate_sim
                    record['raw_fields'] = [f.name for f in raw_msgs[0].fields]
                    record['processed_fields'] = [f.name for f in proc_msgs[0].fields]
                    record['frame_id'] = raw_msgs[0].header.frame_id
                    record['raw'] = analyse([cloud_to_array(m) for m in raw_msgs], channels, vfov, sensor_height, 0.5)
                    record['processed'] = analyse([cloud_to_array(m) for m in proc_msgs], channels, vfov, sensor_height, 0.5)
                    control.remove('lidar_test_box')
                    pids = launch.pids('gzserver')
                    post = [pid for pid in metrics.find_processes('lidar_postprocess')] + \
                        [pid for pid in metrics.find_processes('python3') if pid in launch.pids('python3')]
                    windows = []
                    for w in range(args.windows):
                        rec = observer.measure(args.duration, pids + post)
                        rec['window'] = w + 1
                        windows.append(rec)
                    record['windows'] = windows
                    record['median_real_time_factor'] = statistics.median(w['real_time_factor'] for w in windows)
                    record['gzserver_cpu_cores'] = statistics.mean(
                        sum(v for k, v in w['cpu_cores_by_pid'].items() if int(k) in pids and v is not None) for w in windows)
                    record['postprocess_cpu_cores'] = statistics.mean(
                        sum(v for k, v in w['cpu_cores_by_pid'].items() if int(k) not in pids and v is not None) for w in windows)
                    r, p = record['raw'], record['processed']
                    record['passed'] = bool(9.0 <= rate_sim <= 11.0
                                            and r['vertical_channels_with_returns'] >= channels // 2
                                            and abs(r['lowest_beam_deg'] + vfov / 2) < 0.2
                                            and abs(r['beam_spacing_deg'] - r['expected_spacing_deg']) < 0.1
                                            and r['points_per_sweep_mean'] <= channels * samples
                                            and r['points_per_sweep_mean'] > 0.3 * channels * samples
                                            and abs(r['steep_ground_bias_m']) < 0.03 and r['steep_ground_sigma_m'] < 0.05
                                            and r['box_returns'] > 20 and abs(r['box_range_bias_m']) < 0.05
                                            and p['self_hits_per_sweep'] < 1.0 and r['self_hits_per_sweep'] < 1.0
                                            and 'ring' in record['processed_fields'] and 'time' in record['processed_fields'])
                finally:
                    observer.close()
            results.append(record)
            print(json.dumps({k: v for k, v in record.items() if k != 'windows'}), flush=True)
    passed = all(r.get('passed') for r in results)
    write_json(args.output, dict(passed=passed, profiles=results))
    print(json.dumps(dict(passed=passed, summary={f"{r['profile']}/gpu={r['gpu']}": dict(
        rtf=r.get('median_real_time_factor'), gz_cpu=r.get('gzserver_cpu_cores'), rate=r.get('publish_rate_sim_hz'),
        points=r.get('raw', {}).get('points_per_sweep_mean'), passed=r.get('passed')) for r in results})), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
