#!/usr/bin/env python3
"""Step-climb trials on the ``step_test`` world using Gazebo ground truth.

For every step height (5-40 cm) the rover is teleported to the lane start,
approaches at ``--speed`` with several approach yaw offsets and the outcome
is classified as ``climbed``, ``blocked``, ``tunneled``, ``flipped`` or
``timeout``. 30 cm must be ``blocked`` without tunneling or flipping.
"""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.terrain.world import STEP_LENGTH, STEP_X_START, step_lanes  # noqa: E402
from rover_sim.validation.harness import EntityControl, Observer, SimulationLaunch, truth_since, write_json  # noqa: E402

REST_HEIGHT = 0.41
ENTITY = 'mars_rover'


def classify(samples, height, timeout_s, start_sim):
    step_start, step_end = STEP_X_START, STEP_X_START + STEP_LENGTH
    max_x = max(g[2] for g in samples)
    max_tilt = max(max(abs(g[5]), abs(g[6])) for g in samples)
    min_z_on_step = min((g[4] for g in samples if step_start + 0.35 < g[2] < step_end - 0.35), default=None)
    tunneled = min_z_on_step is not None and min_z_on_step < height + REST_HEIGHT - 0.18
    flipped = max_tilt > math.radians(60)
    crossed = max_x > step_end + 0.6
    if flipped:
        outcome = 'flipped'
    elif tunneled:
        outcome = 'tunneled'
    elif crossed:
        outcome = 'climbed'
    else:
        # Blocked: no progress in the last 4 s of simulation while commanded.
        recent = [g for g in samples if g[1] >= samples[-1][1] - 4.0]
        progress = max(g[2] for g in recent) - min(g[2] for g in recent)
        outcome = 'blocked' if progress < 0.08 and samples[-1][1] - start_sim < timeout_s else (
            'blocked' if progress < 0.08 else 'timeout')
    crossing_time = None
    if crossed:
        crossing_time = next(g[1] for g in samples if g[2] > step_end + 0.6) - start_sim
    return dict(outcome=outcome, max_x=max_x, max_tilt_deg=math.degrees(max_tilt), min_z_on_step=min_z_on_step,
                crossing_time_s=crossing_time, max_pitch_deg=math.degrees(max(abs(g[6]) for g in samples)),
                max_roll_deg=math.degrees(max(abs(g[5]) for g in samples)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--speed', type=float, default=0.8, help='approach speed m/s (momentum matters)')
    parser.add_argument('--timeout', type=float, default=25.0, help='simulation seconds per trial')
    parser.add_argument('--heights', type=float, nargs='*', help='subset of step heights in metres')
    parser.add_argument('--yaw-offsets', type=float, nargs='*', default=[0.0, 6.0, -6.0], help='degrees')
    parser.add_argument('--arg', action='append', default=[], help='extra launch arguments')
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    launch_args = ['gui:=false', 'lidar:=false', 'world:=step_test'] + args.arg
    results = []
    with SimulationLaunch('rover_mars.launch.py', launch_args, log_path=args.log) as launch:
        observer = Observer('step_climb_validation')
        try:
            observer.wait_for(lambda: observer.latest_truth() is not None and observer.latest_joints() is not None,
                              timeout=150, what='rover', launch=launch)
            control = EntityControl(observer, launch=launch)
            observer.run_sim(2.0)
            for height, lane in step_lanes():
                if args.heights and not any(abs(height - h) < 1e-6 for h in args.heights):
                    continue
                for trial in range(args.trials):
                    yaw = math.radians(args.yaw_offsets[trial % len(args.yaw_offsets)])
                    observer.command = (0.0, 0.0, 0.0)
                    control.teleport(ENTITY, 0.0, lane, REST_HEIGHT + 0.03, yaw=yaw)
                    observer.run_sim(1.5)
                    start = observer.sim_time()
                    observer.command = (args.speed, 0.0, 0.0)
                    deadline = start + args.timeout
                    step_end = STEP_X_START + STEP_LENGTH
                    while True:
                        observer.run_sim(0.5)
                        samples = truth_since(observer, start)
                        latest = samples[-1]
                        if latest[2] > step_end + 0.8 or max(abs(latest[5]), abs(latest[6])) > math.radians(60):
                            break
                        if observer.sim_time() > deadline:
                            break
                        recent = [g for g in samples if g[1] >= latest[1] - 4.0]
                        if latest[1] - start > 6.0 and max(g[2] for g in recent) - min(g[2] for g in recent) < 0.08:
                            break
                    observer.command = (0.0, 0.0, 0.0)
                    result = dict(height=height, lane=lane, trial=trial, yaw_offset_deg=math.degrees(yaw),
                                  speed=args.speed, **classify(truth_since(observer, start), height, args.timeout, start))
                    results.append(result)
                    print(json.dumps(result), flush=True)
                    observer.run_sim(1.0)
        finally:
            observer.close()
    summary = {}
    for r in results:
        key = f'{r["height"]:.2f}'
        entry = summary.setdefault(key, dict(height=r['height'], trials=0, climbed=0, blocked=0, tunneled=0,
                                             flipped=0, timeout=0))
        entry['trials'] += 1
        entry[r['outcome']] += 1
    for entry in summary.values():
        entry['pass_rate'] = entry['climbed'] / entry['trials']
    required = all(e['tunneled'] == 0 and e['flipped'] == 0 for e in summary.values())
    thirty = summary.get('0.30')
    thirty_fails = thirty is None or thirty['climbed'] == 0
    fifteen = summary.get('0.15')
    fifteen_passes = fifteen is None or fifteen['pass_rate'] >= 0.99
    passed = required and thirty_fails and fifteen_passes
    write_json(args.output, dict(passed=passed, summary=summary, trials=results, launch_args=launch_args))
    print(json.dumps(dict(passed=passed, summary=summary)), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
