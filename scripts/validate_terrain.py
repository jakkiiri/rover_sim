#!/usr/bin/env python3
"""Validate the generated Mars terrain: determinism, statistics, stability, RTF.

Checks (JSON + Markdown-friendly output):

1. ``determinism``: generating twice with the same seed yields identical file
   hashes (DEM, textures, meshes, worlds, manifest statistics).
2. ``manifest``: slope statistics exist and lie in the design envelope (most
   of the map below 20 deg, some terrain at or above 30-35 deg, flat pad).
3. ``world_loads``/``spawn_pad_stable``: the headless Mars world launches,
   the rover spawns on the pad and rests without drifting for 10 s.
4. ``height_probes``: boxes dropped at random points settle on the height
   the generator predicts (verifies DEM orientation and metre scale).
5. ``real_time_factor``: RTF and gzserver CPU over measurement windows.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys
import tempfile
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.terrain import heightmap as hm  # noqa: E402
from rover_sim.terrain.pipeline import generate_all, load_manifest, read_height_field  # noqa: E402
from rover_sim.validation.harness import Observer, SimulationLaunch, write_json  # noqa: E402


def check_determinism(seed, log):
    with tempfile.TemporaryDirectory(prefix='rover_sim_terrain_') as a, \
            tempfile.TemporaryDirectory(prefix='rover_sim_terrain_') as b:
        first = generate_all(a, seed=seed, texture_size=256, log=lambda *x: None)
        second = generate_all(b, seed=seed, texture_size=256, log=lambda *x: None)
    same = first['files'] == second['files'] and first['slope_stats'] == second['slope_stats']
    differing = sorted(k for k in set(first['files']) | set(second['files'])
                       if first['files'].get(k) != second['files'].get(k))
    result = dict(case='determinism', passed=bool(same), files_compared=len(first['files']),
                  differing_files=differing[:20], seed=seed)
    log(result)
    return result


def check_manifest(manifest, log):
    stats = manifest['slope_stats']
    required = ['max_slope_deg', 'mean_slope_deg', 'percent_above_15_deg', 'percent_above_25_deg',
                'percent_above_35_deg', 'percent_below_20_deg', 'pad_max_slope_deg']
    present = all(k in stats for k in required)
    envelope = (present and stats['percent_below_20_deg'] >= 50.0 and stats['percent_above_30_deg'] >= 2.0
                and stats['max_slope_deg'] >= 35.0 and stats['pad_max_slope_deg'] < 3.0
                and 8.0 <= stats['relief_m'] <= 30.0)
    rocks_ok = all(v['placed'] > 0 for v in manifest['rocks'].values())
    boulders_ok = [round(b['height'], 2) for b in manifest['boulders']] == [0.1, 0.15, 0.2, 0.25, 0.3, 0.4]
    result = dict(case='manifest', passed=bool(present and envelope and rocks_ok and boulders_ok),
                  slope_stats=stats, rocks={k: v['placed'] for k, v in manifest['rocks'].items()},
                  seed=manifest['seed'], resolution=manifest['resolution'])
    log(result)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--terrain-dir', type=Path, help='generated terrain directory (default: installed)')
    parser.add_argument('--rock-density', default='medium')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--windows', type=int, default=2)
    parser.add_argument('--probes', type=int, default=12)
    parser.add_argument('--skip-sim', action='store_true', help='only run the offline checks')
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()

    results = []

    def log(result):
        print(json.dumps({k: v for k, v in result.items() if k not in ('slope_stats',)}), flush=True)
        results.append(result)

    if args.terrain_dir is None:
        from ament_index_python.packages import get_package_share_directory
        args.terrain_dir = Path(get_package_share_directory('rover_sim')) / 'generated'
    manifest = load_manifest(args.terrain_dir)
    check_determinism(manifest['seed'], log)
    check_manifest(manifest, log)
    if args.skip_sim:
        write_json(args.output, dict(checks=results, passed=all(r['passed'] for r in results)))
        return 0 if all(r['passed'] for r in results) else 1

    height = read_height_field(args.terrain_dir)
    size = manifest['size']
    slope_deg = hm.diagnostics(height, size / (height.shape[0] - 1))[0]
    launch_args = ['gui:=false', 'lidar:=false', 'world:=mars', f'rock_density:={args.rock_density}',
                   f'terrain_dir:={args.terrain_dir}']
    import rclpy  # noqa: F401  (ensures the ROS environment is sourced)
    from gazebo_msgs.srv import GetEntityState, SpawnEntity
    with SimulationLaunch('rover_mars.launch.py', launch_args, log_path=args.log) as launch:
        observer = Observer('terrain_validation')
        try:
            started = time.monotonic()
            observer.wait_for(lambda: observer.latest_joints() is not None, timeout=150,
                              what='rover joint states', launch=launch)
            load_seconds = time.monotonic() - started
            node = observer.node
            state = node.create_client(GetEntityState, '/gazebo/get_entity_state')
            spawn = node.create_client(SpawnEntity, '/spawn_entity')
            observer.wait_for(lambda: state.service_is_ready() and spawn.service_is_ready(), timeout=30,
                              what='gazebo services', launch=launch)

            def entity_pose(name):
                future = state.call_async(GetEntityState.Request(name=name, reference_frame='world'))
                observer.wait_for(future.done, timeout=10, what='entity state')
                pose = future.result().state.pose
                from rover_sim.validation.harness import quaternion_to_euler
                q = pose.orientation
                return (pose.position.x, pose.position.y, pose.position.z, *quaternion_to_euler(q.x, q.y, q.z, q.w))

            log(dict(case='world_loads', passed=True, load_wall_seconds=load_seconds,
                     world_file=str(args.terrain_dir / 'worlds' / f'mars_{args.rock_density}.world')))

            observer.run_sim(3.0)
            first = entity_pose('mars_rover')
            observer.run_sim(10.0)
            second = entity_pose('mars_rover')
            drift = math.hypot(second[0] - first[0], second[1] - first[1])
            tilt = max(abs(second[3]), abs(second[4]))
            expected_ground = float(hm.sample_height(height, size, second[0], second[1]))
            log(dict(case='spawn_pad_stable', passed=bool(drift < 0.02 and tilt < math.radians(1.5)
                                                          and abs(second[2] - first[2]) < 0.01),
                     drift_m=drift, tilt_deg=math.degrees(tilt), z_change_m=second[2] - first[2],
                     base_height_above_sampled_ground=second[2] - expected_ground, pose=list(second)))

            import numpy as np
            rng = np.random.default_rng(manifest['seed'] + 99)
            probes = []
            for i in range(args.probes):
                for _ in range(200):
                    px, py = rng.uniform(-size / 2 + 5, size / 2 - 5, size=2)
                    if float(hm.sample_height(slope_deg, size, px, py)) < 18.0:
                        break
                ground = float(hm.sample_height(height, size, px, py))
                sdf = (f'<sdf version="1.6"><model name="probe_{i}"><link name="l"><inertial><mass>0.5</mass>'
                       '<inertia><ixx>0.003</ixx><iyy>0.003</iyy><izz>0.003</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
                       '<collision name="c"><geometry><box><size>0.2 0.2 0.2</size></box></geometry>'
                       '<surface><friction><ode><mu>5</mu><mu2>5</mu2></ode></friction></surface></collision></link></model></sdf>')
                request = SpawnEntity.Request(name=f'probe_{i}', xml=sdf, reference_frame='world')
                request.initial_pose.position.x, request.initial_pose.position.y = float(px), float(py)
                request.initial_pose.position.z = ground + 0.4
                request.initial_pose.orientation.w = 1.0
                future = spawn.call_async(request)
                observer.wait_for(future.done, timeout=20, what='probe spawn')
                probes.append((i, float(px), float(py), ground))
            observer.run_sim(4.0)
            errors = []
            details = []
            for i, px, py, ground in probes:
                pose = entity_pose(f'probe_{i}')
                landed_ground = float(hm.sample_height(height, size, pose[0], pose[1]))
                error = pose[2] - 0.1 - landed_ground
                errors.append(error)
                details.append(dict(x=px, y=py, predicted_ground=ground, landed=list(pose[:3]),
                                    predicted_at_landing=landed_ground, error_m=error))
            abs_errors = sorted(abs(e) for e in errors)
            median_error = abs_errors[len(abs_errors) // 2]
            within = sum(e < 0.05 for e in abs_errors) / len(abs_errors)
            log(dict(case='height_probes', passed=bool(median_error < 0.03 and within >= 0.75),
                     median_abs_error_m=median_error, fraction_within_5cm=within, probes=details))

            pids = launch.pids('gzserver')
            windows = []
            for w in range(args.windows):
                record = observer.measure(args.duration, pids)
                record['window'] = w + 1
                windows.append(record)
            rtf = statistics.median(x['real_time_factor'] for x in windows)
            cpu = statistics.mean(sum(v for v in x['cpu_cores_by_pid'].values() if v is not None) for x in windows)
            log(dict(case='real_time_factor', passed=bool(rtf >= 0.9), median_real_time_factor=rtf,
                     gzserver_cpu_cores=cpu, windows=windows, rock_density=args.rock_density,
                     resolution=manifest['resolution'], sampling=manifest['sampling']))
        finally:
            observer.close()
    passed = all(r['passed'] for r in results)
    write_json(args.output, dict(terrain_dir=str(args.terrain_dir), seed=manifest['seed'],
                                 checks=results, passed=passed))
    print('PASSED' if passed else 'FAILED', flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
