#!/usr/bin/env python3
"""Slope trials on the ``slope_test`` world: climb, stall, slide-back, hold.

For each ramp (5-40 deg) the rover is teleported to the run-up and commanded
forward. Outcomes: ``climbed`` (reached the plateau), ``stalled`` (no
progress on the ramp, no slide), ``slid_back`` (lost more than 0.3 m from
its highest point) or ``flipped``. A second pass stops the command mid-ramp
and measures how far the rover drifts in 5 s (hold test).
"""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.terrain.world import RAMP_LENGTH, RAMP_RUNUP, ramp_geometry, ramp_lanes  # noqa: E402
from rover_sim.validation.harness import EntityControl, Observer, SimulationLaunch, truth_since, write_json  # noqa: E402

REST_HEIGHT = 0.41
ENTITY = 'mars_rover'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=0.4)
    parser.add_argument('--timeout', type=float, default=90.0)
    parser.add_argument('--angles', type=float, nargs='*')
    parser.add_argument('--hold-seconds', type=float, default=5.0)
    parser.add_argument('--arg', action='append', default=[])
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    launch_args = ['gui:=false', 'lidar:=false', 'world:=slope_test'] + args.arg
    results = []
    with SimulationLaunch('rover_mars.launch.py', launch_args, log_path=args.log) as launch:
        observer = Observer('slope_validation')
        try:
            observer.wait_for(lambda: observer.latest_truth() is not None and observer.latest_joints() is not None,
                              timeout=150, what='rover', launch=launch)
            control = EntityControl(observer)
            observer.run_sim(2.0)
            ramp_end = RAMP_RUNUP + RAMP_LENGTH
            for angle, lane in ramp_lanes():
                if args.angles and angle not in args.angles:
                    continue
                theta = math.radians(angle)
                _, _, top_height = ramp_geometry(angle)
                # --- climb trial -------------------------------------------------
                observer.command = (0.0, 0.0, 0.0)
                control.teleport(ENTITY, 0.3, lane, REST_HEIGHT + 0.03)
                observer.run_sim(1.5)
                start = observer.sim_time()
                observer.command = (args.speed, 0.0, 0.0)
                outcome = 'timeout'
                while observer.sim_time() - start < args.timeout:
                    observer.run_sim(0.5)
                    samples = truth_since(observer, start)
                    latest = samples[-1]
                    max_x = max(g[2] for g in samples)
                    if max(abs(latest[5]), abs(latest[6])) > math.radians(60):
                        outcome = 'flipped'
                        break
                    if latest[2] > ramp_end + 1.0:
                        outcome = 'climbed'
                        break
                    if latest[2] > RAMP_RUNUP + 0.5 and max_x - latest[2] > 0.3:
                        outcome = 'slid_back'
                        break
                    recent = [g for g in samples if g[1] >= latest[1] - 4.0]
                    if latest[1] - start > 6.0 and max(g[2] for g in recent) - min(g[2] for g in recent) < 0.05:
                        outcome = 'stalled'
                        break
                observer.command = (0.0, 0.0, 0.0)
                samples = truth_since(observer, start)
                max_x = max(g[2] for g in samples)
                if outcome == 'timeout' and max_x > RAMP_RUNUP + 1.0:
                    outcome = 'climbing_slowly'      # still progressing when the window closed
                speeds = [g[8] for g in samples if RAMP_RUNUP + 1.0 < g[2] < ramp_end - 1.0]
                climb_speed = sum(speeds) / len(speeds) if speeds else None
                result = dict(angle=angle, lane=lane, outcome=outcome, max_x=max_x,
                              height_reached=max(g[4] for g in samples) - REST_HEIGHT, top_height=top_height,
                              mean_body_speed_on_ramp=climb_speed, commanded_speed=args.speed,
                              max_pitch_deg=math.degrees(max(abs(g[6]) for g in samples)),
                              max_roll_deg=math.degrees(max(abs(g[5]) for g in samples)))
                # --- hold trial: stop mid-ramp and measure drift ----------------
                if max_x > RAMP_RUNUP + RAMP_LENGTH * 0.5:
                    mid_x = RAMP_RUNUP + RAMP_LENGTH * 0.45
                    z_mid = (mid_x - RAMP_RUNUP) * math.tan(theta) + REST_HEIGHT / math.cos(theta) + 0.03
                    control.teleport(ENTITY, mid_x, lane, z_mid, pitch=-theta)
                    observer.run_sim(2.0)
                    before = observer.latest_truth()
                    observer.run_sim(args.hold_seconds)
                    after = observer.latest_truth()
                    drift = math.hypot(after[2] - before[2], after[3] - before[3]) + abs(after[4] - before[4])
                    result.update(hold_drift_m=drift, holds=bool(drift < 0.05))
                else:
                    result.update(hold_drift_m=None, holds=None)
                results.append(result)
                print(json.dumps(result), flush=True)
                observer.run_sim(1.0)
        finally:
            observer.close()
    climbed = [r['angle'] for r in results if r['outcome'] in ('climbed', 'climbing_slowly')]
    not_climbed = [r['angle'] for r in results if r['outcome'] not in ('climbed', 'climbing_slowly')]
    max_stable = max(climbed) if climbed else None
    no_artifacts = all(r['outcome'] != 'flipped' for r in results)
    passed = (no_artifacts and max_stable is not None and 20 <= max_stable < 35
              and all(a > max_stable for a in not_climbed))
    write_json(args.output, dict(passed=passed, max_stable_angle=max_stable, trials=results, launch_args=launch_args))
    print(json.dumps(dict(passed=passed, max_stable_angle=max_stable,
                          outcomes={r['angle']: r['outcome'] for r in results},
                          holds={r['angle']: r['holds'] for r in results})), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
