#!/usr/bin/env python3
"""Reset/watchdog validation against a dedicated Gazebo Classic launch.

Requires an isolated ROS_DOMAIN_ID and GAZEBO_MASTER_URI, no other command
publisher, and the exact PID of that launch's swerve controller. Temporarily
SIGSTOPs only that PID to exercise the independent actuator watchdog. All
modified physics settings and SIGSTOP are restored in finally blocks.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt64


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller-pid', type=int, required=True)
    parser.add_argument('--watchdog', type=float, default=.3)
    parser.add_argument('--world', default='flat', help='Gazebo world name of the running launch')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not os.environ.get('GAZEBO_MASTER_URI') or not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('Set an isolated GAZEBO_MASTER_URI and ROS_DOMAIN_ID first')
    process_args = Path(f'/proc/{args.controller_pid}/cmdline').read_bytes().replace(b'\0', b' ')
    process_env = Path(f'/proc/{args.controller_pid}/environ').read_bytes().split(b'\0')
    if b'swerve_controller' not in process_args or (
            ('ROS_DOMAIN_ID=' + os.environ['ROS_DOMAIN_ID']).encode() not in process_env):
        parser.error('PID is not a swerve controller in this validation domain')
    rclpy.init()
    node = Node('hardening_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    state, targets, epochs, command_history, state_history = {}, {}, [], [], []
    command = None
    suspended = False
    results = []

    def on_state(msg):
        state.update({n: (p, v) for n, p, v in zip(msg.name, msg.position, msg.velocity)})
        state_history.append((time.monotonic(), dict(state)))

    def on_targets(msg):
        targets.update({n: (p, v) for n, p, v in zip(msg.name, msg.position, msg.velocity)})
        command_history.append((time.monotonic(), dict(targets)))

    node.create_subscription(JointState, '/joint_states', on_state, 10)
    node.create_subscription(JointState, '/swerve/joint_commands', on_targets, 10)
    node.create_subscription(UInt64, '/swerve/reset_epoch', lambda m: epochs.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def publish():
        values = command
        if values is not None:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = values
            publisher.publish(msg)

    node.create_timer(.02, publish, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
    worker = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    worker.start()

    def wait_for(predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for validation condition')
            time.sleep(.01)

    def gz(*args):
        subprocess.run(['gz', *args], check=True, timeout=10, capture_output=True, text=True)

    def wheel_speed(data):
        return max((abs(v[1]) for n, v in data.items() if n.endswith('_wheel_joint')), default=math.inf)

    def minimum_wheel_speed(data):
        values = [abs(v[1]) for n, v in data.items() if n.endswith('_wheel_joint')]
        return min(values) if len(values) == 4 else 0.0

    def sim_time():
        return node.get_clock().now().nanoseconds / 1e9

    def record(name, passed, **details):
        result = dict(case=name, passed=bool(passed), **details)
        results.append(result)
        print(json.dumps(result), flush=True)
        if not passed:
            raise RuntimeError(f'Failed {name}')

    def drive():
        nonlocal command
        command = (.4, 0., 0.)
        wait_for(lambda: minimum_wheel_speed(state) > 1.5 and minimum_wheel_speed(targets) > 1.5)

    try:
        wait_for(lambda: len(state) == 12 and len(targets) == 8 and epochs)
        setter = node.create_client(SetParametersAtomically, '/swerve_controller/set_parameters_atomically')
        getter = node.create_client(GetParameters, '/swerve_controller/get_parameters')
        wait_for(lambda: setter.service_is_ready() and getter.service_is_ready())

        def set_parameters(parameters):
            return setter.call_async(SetParametersAtomically.Request(
                parameters=[parameter.to_parameter_msg() for parameter in parameters]))

        future = set_parameters([
            Parameter('command_timeout', value=.8), Parameter('max_accel', value=-1.)])
        wait_for(future.done)
        rejected = future.result().result
        future = getter.call_async(GetParameters.Request(names=['command_timeout', 'max_accel']))
        wait_for(future.done)
        actual = [v.double_value for v in future.result().values]
        record('atomic_invalid_parameters', not rejected.successful and actual == [.5, .8],
               reason=rejected.reason, unchanged_parameters=actual)
        future = set_parameters([Parameter('max_accel', value=.4)])
        wait_for(future.done)
        record('valid_runtime_parameters', future.result().result.successful)
        future = set_parameters([Parameter('max_accel', value=.8)])
        wait_for(future.done)
        if not future.result().result.successful:
            raise RuntimeError('Failed to restore max_accel')

        drive()
        command = None
        gz('world', '-w', args.world, '-p', '1')
        time.sleep(.2)
        before = sim_time()
        time.sleep(.7)
        paused_delta = sim_time() - before
        stopped_targets = wheel_speed(targets)
        gz('world', '-w', args.world, '-p', '0')
        wait_for(lambda: wheel_speed(state) < .05)
        record('pause_resume_deadman', paused_delta < .002 and stopped_targets == 0.,
               paused_sim_delta=paused_delta, paused_drive_targets=stopped_targets,
               resumed_wheel_speed=wheel_speed(state))

        for option, name in [('-r', 'reset_all_while_moving'), ('-o', 'reset_models_while_moving'),
                             ('-t', 'reset_time_while_moving')]:
            drive()
            previous_epoch = epochs[-1]
            command = None
            gz('world', '-w', args.world, option)
            wait_for(lambda: epochs[-1] > previous_epoch)
            wait_for(lambda: wheel_speed(targets) == 0. and wheel_speed(state) < .05)
            time.sleep(.4)
            record(name, wheel_speed(targets) == 0. and wheel_speed(state) < .05,
                   previous_epoch=previous_epoch, new_epoch=epochs[-1],
                   wheel_speed=wheel_speed(state), drive_targets=wheel_speed(targets))
            drive()  # a fresh command restores motion after reinitialization

        drive()
        gz('physics', '-w', args.world, '-u', '20')
        start_wall, start_sim = time.monotonic(), sim_time()
        history_start = len(command_history)
        time.sleep(5.)
        duration = time.monotonic() - start_wall
        measured_rtf = (sim_time() - start_sim) / duration
        heartbeat = command_history[history_start:]
        minimum_target = min(minimum_wheel_speed(values) for _, values in heartbeat)
        gaps = [b[0] - a[0] for a, b in zip(heartbeat, heartbeat[1:])]
        record('slow_simulation', 0 < measured_rtf < .067 and minimum_target > 1.5
               and minimum_wheel_speed(state) > 1.3 and len(heartbeat) > 100 and max(gaps) < args.watchdog,
               measured_real_time_factor=measured_rtf, command_frames=len(heartbeat),
               max_wall_command_gap=max(gaps), minimum_drive_target=minimum_target,
               actual_wheel_speed=wheel_speed(state))
        gz('physics', '-w', args.world, '-u', '1000')

        drive()
        command = None
        os.kill(args.controller_pid, signal.SIGSTOP)
        suspended = True
        wait_for(lambda: Path(f'/proc/{args.controller_pid}/stat').read_text().split(') ', 1)[1][0] == 'T')
        previous_epoch = epochs[-1]
        # Publish once while callback execution is impossible. The reliable
        # writer retains the pending sample while the reader's DDS threads are
        # also frozen; do not wait for an impossible acknowledgement here.
        # Its publication predates watchdog expiry regardless of DDS delivery.
        pending = Twist()
        pending.linear.x = .4
        publisher.publish(pending)
        start = time.monotonic()
        wait_for(lambda: wheel_speed(state) < .05, args.watchdog + 6.)
        latency = time.monotonic() - start
        record('actuator_watchdog', latency < args.watchdog + 3.,
               configured_wall_timeout=args.watchdog, wheel_settling_wall_seconds=latency,
               actual_wheel_speed=wheel_speed(state))
        os.kill(args.controller_pid, signal.SIGCONT)
        suspended = False
        resumed_at = time.monotonic()
        # Check every new sample, rather than accepting an old zero snapshot
        # and checking only the eventual final value.
        wait_for(lambda: time.monotonic()-resumed_at >= .4)
        samples = [values for stamp, values in state_history if stamp >= resumed_at]
        frames = [values for stamp, values in command_history if stamp >= resumed_at]
        maximum_speed = max((wheel_speed(values) for values in samples), default=math.inf)
        record('controller_resume_no_stale_motion', bool(frames) and bool(samples)
               and maximum_speed < .05 and wheel_speed(targets) == 0. and epochs[-1] > previous_epoch,
               actual_wheel_speed=wheel_speed(state), maximum_resumed_wheel_speed=maximum_speed,
               command_frames=len(frames), previous_epoch=previous_epoch, new_epoch=epochs[-1],
               nonzero_command_frames=sum(wheel_speed(values) != 0. for values in frames))
        drive()
        record('fresh_command_after_watchdog', minimum_wheel_speed(state) > 1.5)
    finally:
        command = (0., 0., 0.)
        publisher.publish(Twist())
        if suspended:
            os.kill(args.controller_pid, signal.SIGCONT)
        gz('physics', '-w', args.world, '-u', '1000')
        gz('world', '-w', args.world, '-p', '0')
        time.sleep(.1)
        node.executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2) + '\n')


if __name__ == '__main__':
    main()
