#!/usr/bin/env python3
"""Launch a headless flat-world simulation and run the hardening and teleop
validators against it (they need the controller PID / an interactive pty)."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.validation.harness import Observer, SimulationLaunch  # noqa: E402


def controller_pid(launch):
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as handle:
                cmdline = handle.read()
            with open(f'/proc/{pid}/environ', 'rb') as handle:
                env = handle.read().split(b'\0')
        except OSError:
            continue
        if b'swerve_controller' in cmdline and b'python' in cmdline and \
                ('ROS_DOMAIN_ID=' + launch.env['ROS_DOMAIN_ID']).encode() in env:
            return int(pid)
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--skip-hardening', action='store_true')
    parser.add_argument('--skip-teleop', action='store_true')
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    scripts = Path(__file__).resolve().parent
    launch_args = ['gui:=false', 'lidar:=false', 'world:=flat']
    codes = {}
    with SimulationLaunch('rover_mars.launch.py', launch_args, log_path=str(args.output_dir / 'sim.log')) as launch:
        observer = Observer('hardening_wrapper')
        try:
            observer.wait_for(lambda: observer.latest_joints() is not None and observer.latest_truth() is not None,
                              timeout=150, what='rover', launch=launch)
            time.sleep(2.0)
        finally:
            observer.close()
        env = dict(os.environ, ROS_DOMAIN_ID=launch.env['ROS_DOMAIN_ID'], GAZEBO_MASTER_URI=launch.env['GAZEBO_MASTER_URI'])
        if not args.skip_hardening:
            pid = controller_pid(launch)
            if pid is None:
                raise RuntimeError('swerve_controller process not found')
            with open(args.output_dir / 'hardening.out', 'w') as out:
                codes['hardening'] = subprocess.call(
                    [sys.executable, str(scripts / 'validate_hardening.py'), '--controller-pid', str(pid),
                     '--watchdog', '0.3', '--world', 'flat', '--output', str(args.output_dir / 'hardening.json')],
                    env=env, stdout=out, stderr=subprocess.STDOUT, timeout=600)
        if not args.skip_teleop:
            with open(args.output_dir / 'teleop.out', 'w') as out:
                codes['teleop'] = subprocess.call(
                    [sys.executable, str(scripts / 'validate_teleop.py'), '--output', str(args.output_dir / 'teleop.json')],
                    env=env, stdout=out, stderr=subprocess.STDOUT, timeout=600)
    print({'exit_codes': codes})
    return 0 if all(code == 0 for code in codes.values()) else 1


if __name__ == '__main__':
    raise SystemExit(main())
