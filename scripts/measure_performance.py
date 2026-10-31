#!/usr/bin/env python3
"""Launch a headless simulation, then record real-time factor and gzserver CPU.

Example::

    ros2 run rover_sim measure_performance.py --launch rover_desert.launch.py \
        --arg gui:=false --duration 10 --windows 2 --output /tmp/baseline.json
"""
import argparse
import json
from pathlib import Path
import statistics
import time

from rover_sim.validation.harness import Observer, SimulationLaunch, write_json


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', default='rover_mars.launch.py')
    parser.add_argument('--arg', action='append', default=[], help='launch argument key:=value')
    parser.add_argument('--duration', type=float, default=10.0)
    parser.add_argument('--windows', type=int, default=2)
    parser.add_argument('--settle', type=float, default=4.0, help='wall seconds to wait after readiness')
    parser.add_argument('--ready-topic', default='clock', choices=['clock', 'joints', 'truth'])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    launch_args = ['gui:=false'] + [a for a in args.arg if not a.startswith('gui:=')] + \
        [a for a in args.arg if a.startswith('gui:=')]
    result = {'launch': args.launch, 'launch_args': launch_args, 'windows': []}
    with SimulationLaunch(args.launch, launch_args, log_path=args.log) as launch:
        observer = Observer('performance_observer')
        try:
            observer.wait_for(lambda: observer.sim_time() is not None and observer.sim_time() > 0.5,
                              timeout=120, what='/clock', launch=launch)
            if args.ready_topic == 'joints':
                observer.wait_for(lambda: observer.latest_joints() is not None, timeout=90,
                                  what='/joint_states', launch=launch)
            if args.ready_topic == 'truth':
                observer.wait_for(lambda: observer.latest_truth() is not None, timeout=90,
                                  what='/ground_truth/odom', launch=launch)
            time.sleep(args.settle)
            pids = launch.server_pids()
            result['gzserver_pids'] = pids
            for index in range(args.windows):
                record = observer.measure(args.duration, pids)
                record['window'] = index + 1
                result['windows'].append(record)
                print(json.dumps(record), flush=True)
            result['median_real_time_factor'] = statistics.median(
                w['real_time_factor'] for w in result['windows'])
            result['mean_gzserver_cpu_cores'] = statistics.mean(
                sum(v for v in w['cpu_cores_by_pid'].values() if v is not None) for w in result['windows'])
        finally:
            observer.close()
    print(json.dumps({k: v for k, v in result.items() if k != 'windows'}), flush=True)
    if args.output:
        write_json(args.output, result)


if __name__ == '__main__':
    main()
