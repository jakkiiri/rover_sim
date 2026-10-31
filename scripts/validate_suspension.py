#!/usr/bin/env python3
"""Suspension contact-retention and body-motion metrics over a bump and a rock field.

Runs on the flat world: a 12 cm bump (full width) and a deterministic field
of 5-9 cm rigid blocks are spawned ahead of the rover. Wheels count as "in contact"
when their suspension strut is compressed beyond the free length (an
airborne strut extends to +0.03 m under its own weight). Reports the
fraction of time with 4/4 wheels in contact, body pitch/roll extremes,
vertical acceleration and the settling time after the bump.
"""
import argparse
import json
import math
from pathlib import Path
import statistics
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.validation.harness import EntityControl, Observer, SimulationLaunch, joints_since, truth_since, write_json  # noqa: E402

REST_HEIGHT = 0.41
ENTITY = 'mars_rover'
FREE_LENGTH = -0.0198 - 5.0 * 3.71 / 1500.0    # strut position with no wheel load (Mars): hangs down
CONTACT_THRESHOLD = FREE_LENGTH + 0.006   # in contact when compressed above the free length


def contact_fraction(joints):
    suspension = [[v[0] for n, v in data.items() if n.endswith('_suspension')] for _, _, data in joints]
    counts = [sum(q > CONTACT_THRESHOLD for q in q4) for q4 in suspension]
    return (sum(c == 4 for c in counts) / len(counts) if counts else None,
            sum(c >= 3 for c in counts) / len(counts) if counts else None, min(counts) if counts else None)


def vertical_acceleration(samples):
    if len(samples) < 3:
        return None, None
    t = np.array([g[1] for g in samples])
    vz = np.array([g[10] for g in samples])
    az = np.gradient(vz, t)
    return float(np.max(np.abs(az))), float(np.sqrt(np.mean(az ** 2)))


def settling_time(samples, event_sim, rest_z, tolerance=0.006):
    after = [g for g in samples if g[1] >= event_sim]
    for i, g in enumerate(after):
        if all(abs(h[4] - rest_z) < tolerance for h in after[i:]):
            return g[1] - event_sim
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=0.5)
    parser.add_argument('--bump-height', type=float, default=0.12)
    parser.add_argument('--rocks', type=int, default=24)
    parser.add_argument('--arg', action='append', default=[])
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    launch_args = ['gui:=false', 'lidar:=false', 'world:=flat'] + args.arg
    results = {}
    with SimulationLaunch('rover_mars.launch.py', launch_args, log_path=args.log) as launch:
        observer = Observer('suspension_validation')
        try:
            observer.wait_for(lambda: observer.latest_truth() is not None and observer.latest_joints() is not None,
                              timeout=150, what='rover', launch=launch)
            control = EntityControl(observer, launch=launch)
            observer.run_sim(3.0)
            rest_z = observer.latest_truth()[4]
            rest_joints = observer.latest_joints()
            results['rest'] = dict(base_z=rest_z, suspension={n: v[0] for n, v in rest_joints.items() if n.endswith('_suspension')})

            # --- bump ------------------------------------------------------
            control.spawn_box('bump', 5.0, 0.0, args.bump_height / 2, 0.35, 3.0, args.bump_height)
            control.teleport(ENTITY, 0.0, 0.0, REST_HEIGHT + 0.03)
            observer.run_sim(1.5)
            start = observer.sim_time()
            observer.command = (args.speed, 0.0, 0.0)
            observer.wait_for(lambda: observer.latest_truth()[2] > 9.0 or observer.sim_time() - start > 40,
                              timeout=400, what='bump traverse')
            observer.command = (0.0, 0.0, 0.0)
            observer.run_sim(3.0)
            samples = truth_since(observer, start)
            joints = joints_since(observer, start)
            moving = [g for g in samples if 2.0 < g[2] < 8.5]
            four, three, minimum = contact_fraction([j for j in joints if moving and moving[0][1] <= j[1] <= moving[-1][1]])
            peak_az, rms_az = vertical_acceleration(moving)
            rear_clear = next((g[1] for g in samples if g[2] > 5.0 + 0.35 / 2 + 0.4 + 0.2), None)
            settle = settling_time(samples, rear_clear, rest_z) if rear_clear else None
            results['bump'] = dict(bump_height=args.bump_height, speed=args.speed, contact_4_of_4=four,
                                   contact_at_least_3=three, min_wheels_in_contact=minimum,
                                   max_pitch_deg=math.degrees(max(abs(g[6]) for g in moving)),
                                   max_roll_deg=math.degrees(max(abs(g[5]) for g in moving)),
                                   peak_vertical_accel=peak_az, rms_vertical_accel=rms_az,
                                   settling_time_after_bump_s=settle, crossed=bool(samples[-1][2] > 8.5))
            print(json.dumps(results['bump']), flush=True)
            control.remove('bump')

            # --- rock field --------------------------------------------------
            # Moderately uneven ground: 5-9 cm rigid blocks (below the measured
            # 10 cm step limit) scattered across both wheel tracks.
            rng = np.random.default_rng(11)
            rocks = []
            for i in range(args.rocks):
                size = float(rng.uniform(0.05, 0.09))
                x, y = float(rng.uniform(3.0, 9.0)), float(rng.uniform(-0.9, 0.9))
                control.spawn_box(f'rock_{i}', x, y, size / 2 - 0.01, size, size * 1.3, size, colour='0.3 0.25 0.2')
                rocks.append((x, y, size))
            control.teleport(ENTITY, 0.0, 0.0, REST_HEIGHT + 0.03)
            observer.run_sim(1.5)
            start = observer.sim_time()
            observer.command = (args.speed, 0.0, 0.0)
            observer.wait_for(lambda: observer.latest_truth()[2] > 10.5 or observer.sim_time() - start > 50,
                              timeout=500, what='rock field traverse')
            observer.command = (0.0, 0.0, 0.0)
            observer.run_sim(2.0)
            samples = truth_since(observer, start)
            joints = joints_since(observer, start)
            field = [g for g in samples if 2.5 < g[2] < 9.5]
            four, three, minimum = contact_fraction([j for j in joints if field and field[0][1] <= j[1] <= field[-1][1]])
            peak_az, rms_az = vertical_acceleration(field)
            results['rock_field'] = dict(rocks=len(rocks), speed=args.speed, contact_4_of_4=four,
                                         contact_at_least_3=three, min_wheels_in_contact=minimum,
                                         max_pitch_deg=math.degrees(max(abs(g[6]) for g in field)),
                                         max_roll_deg=math.degrees(max(abs(g[5]) for g in field)),
                                         peak_vertical_accel=peak_az, rms_vertical_accel=rms_az,
                                         crossed=bool(samples[-1][2] > 10.0),
                                         mean_speed=statistics.mean(math.hypot(g[8], g[9]) for g in field))
            print(json.dumps(results['rock_field']), flush=True)
            for i in range(len(rocks)):
                control.remove(f'rock_{i}')
        finally:
            observer.close()
    bump, field = results['bump'], results['rock_field']
    passed = (bump['crossed'] and field['crossed'] and field['contact_4_of_4'] is not None
              and field['contact_4_of_4'] >= 0.95 and bump['contact_4_of_4'] >= 0.95
              and (bump['settling_time_after_bump_s'] is not None and bump['settling_time_after_bump_s'] <= 2.0))
    results['passed'] = passed
    results['launch_args'] = launch_args
    results['contact_threshold_m'] = CONTACT_THRESHOLD
    write_json(args.output, results)
    print(json.dumps(dict(passed=passed)), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
