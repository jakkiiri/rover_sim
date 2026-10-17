#!/usr/bin/env python3
"""Exercise the installed keyboard executable against a running Classic rover.

Injects actual terminal bytes, observes ROS messages/joints and Gazebo ground
truth. Resets only proxy_rover between cases. Do not run another cmd_vel source.
"""
import argparse
import json
import math
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import termios
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

from validate_swerve import pose

ZERO = (0.0, 0.0, 0.0)
CASES = [('W', (.4, 0., 0.)), ('S', (-.4, 0., 0.)),
         ('A', (0., 0., .5)), ('D', (0., 0., -.5))]


class TerminalProcess:
    def __init__(self):
        self.master, self.slave = pty.openpty()
        self.original = termios.tcgetattr(self.slave)
        executable = str(Path(__file__).resolve().with_name('rover_teleop'))
        # Give the child its own controlling terminal so byte 0x03 is a real
        # terminal SIGINT, just as Ctrl+C in the user's focused terminal.
        bootstrap = ('import os,fcntl,termios,sys; os.setsid(); '
                     'fcntl.ioctl(0,termios.TIOCSCTTY,0); '
                     'os.execv(sys.argv[1],sys.argv[1:])')
        self.process = subprocess.Popen(
            [sys.executable, '-c', bootstrap, executable],
            stdin=self.slave, stdout=self.slave, stderr=self.slave)
        self.key = None
        self.output = ''
        self.done = threading.Event()

        def input_output():
            while not self.done.is_set():
                if self.key is not None:
                    os.write(self.master, self.key.encode())
                if select.select([self.master], [], [], .05)[0]:
                    self.output += os.read(self.master, 4096).decode(errors='replace')

        self.worker = threading.Thread(target=input_output, daemon=True)
        self.worker.start()
        deadline = time.monotonic()+15
        while 'STOP ready:' not in self.output:
            if self.process.poll() is not None or time.monotonic() > deadline:
                self.close()
                raise RuntimeError('Teleop failed to start: '+self.output)
            time.sleep(.02)

    def send(self, key):
        os.write(self.master, key)

    def close(self):
        self.key = None
        if self.process.poll() is None:
            self.send(b'\x03')
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.restored = termios.tcgetattr(self.slave) == self.original
        time.sleep(.1)
        self.done.set()
        self.worker.join(timeout=2)
        os.close(self.master)
        os.close(self.slave)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node('keyboard_teleop_validation')
    messages, state = [], {}
    node.create_subscription(Twist, '/cmd_vel', lambda m: messages.append(
        (time.monotonic(), (m.linear.x, m.linear.y, m.angular.z))), 10)
    node.create_subscription(JointState, '/joint_states', lambda m: state.update(
        {n: (p, v) for n, p, v in zip(m.name, m.position, m.velocity)}), 10)
    worker = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    worker.start()
    terminal = None
    results = []

    def wait_for(predicate, seconds=10):
        deadline = time.monotonic()+seconds
        while not predicate():
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for validation condition')
            time.sleep(.01)

    def wheel_speed():
        return max((abs(v[1]) for n, v in state.items()
                    if n.endswith('_wheel_joint')), default=math.inf)

    def record(result):
        results.append(result)
        print(json.dumps(result), flush=True)

    try:
        wait_for(lambda: len(state) >= 8, 25)
        terminal = TerminalProcess()
        wait_for(lambda: messages and messages[-1][1] == ZERO)
        for key, command in CASES:
            terminal.key = None
            terminal.send(b' ')
            wait_for(lambda: messages[-1][1] == ZERO and wheel_speed() < .05)
            subprocess.run(['gz', 'model', '-m', 'proxy_rover', '-x', '0', '-y', '0',
                            '-z', '0.34', '-R', '0', '-P', '0', '-Y', '0'],
                           check=True, timeout=10)
            terminal.key = key
            wait_for(lambda: messages[-1][1] == command)
            time.sleep(2.5)  # steering and acceleration transient
            a, t0 = pose()
            time.sleep(2.0)
            b, t1 = pose()
            delta = (b[0]-a[0], b[1]-a[1])
            bx = math.cos(a[5])*delta[0]+math.sin(a[5])*delta[1]
            by = -math.sin(a[5])*delta[0]+math.cos(a[5])*delta[1]
            yaw = math.atan2(math.sin(b[5]-a[5]), math.cos(b[5]-a[5]))
            duration = t1-t0
            translation_error = math.hypot(bx-command[0]*duration, by-command[1]*duration)
            heading_error = abs(yaw-command[2]*duration)
            record(dict(case=key, command=list(command), duration=duration,
                        body_displacement=[bx, by], yaw_change=yaw,
                        translation_error=translation_error, heading_error=heading_error,
                        passed=translation_error < .15 and heading_error < .08))

        # Removed controls are unsupported keys: they immediately stop existing
        # motion and never introduce a rotation or lateral command.
        for key in ('Q', 'E'):
            terminal.key = 'w'
            wait_for(lambda: messages[-1][1] == (.4, 0., 0.) and wheel_speed() > 1.)
            terminal.key = None
            time.sleep(.06)
            event_at = time.monotonic()
            terminal.send(key.encode())
            wait_for(lambda: any(t >= event_at and cmd == ZERO for t, cmd in messages), 3)
            first_zero = next(t for t, cmd in messages if t >= event_at and cmd == ZERO)
            wait_for(lambda: wheel_speed() < .05, 5)
            time.sleep(.2)
            observed = [cmd for t, cmd in messages if t >= first_zero]
            record(dict(case=key+'_disabled', zero_latency_wall_seconds=first_zero-event_at,
                        max_wheel_speed_after_stop=wheel_speed(),
                        passed=first_zero-event_at < .2 and bool(observed)
                        and all(cmd == ZERO for cmd in observed) and wheel_speed() < .05))

        # Stop tests begin from measured forward motion; record the first zero
        # received after each event, then verify all four actual wheels settle.
        for name in ('Space', 'release', 'Ctrl+C'):
            terminal.key = 'w'
            wait_for(lambda: messages[-1][1] == (.4, 0., 0.) and wheel_speed() > 1.0)
            time.sleep(.5)
            terminal.key = None
            time.sleep(.06)  # allow any in-flight input writer iteration to finish
            event_at = time.monotonic()
            if name == 'Space':
                terminal.send(b' ')
            elif name == 'Ctrl+C':
                terminal.send(b'\x03')
            wait_for(lambda: any(t >= event_at and cmd == ZERO for t, cmd in messages), 3)
            first_zero = next(t for t, cmd in messages if t >= event_at and cmd == ZERO)
            wait_for(lambda: wheel_speed() < .05, 5)
            time.sleep(.2)
            latency = first_zero-event_at
            result = dict(case=name, zero_latency_wall_seconds=latency,
                          max_wheel_speed_after_stop=wheel_speed(),
                          passed=latency < (.8 if name == 'release' else .2)
                          and messages[-1][1] == ZERO and wheel_speed() < .05)
            if name == 'Ctrl+C':
                terminal.process.wait(timeout=5)
                terminal.close()
                result.update(exit_code=terminal.process.returncode,
                              terminal_restored=terminal.restored,
                              active_command_printed=all(label in terminal.output for label in (
                                  'W forward:', 'S backward:', 'A rotate left:',
                                  'D rotate right:',
                                  'STOP Space:', 'STOP key timeout:', 'STOP shutdown:')))
                result['passed'] &= (result['exit_code'] == 0 and
                                     result['terminal_restored'] and result['active_command_printed'])
                terminal = None
            record(result)
        record(dict(case='teleop_never_publishes_lateral', messages_checked=len(messages),
                    passed=bool(messages) and all(cmd[1] == 0. for _, cmd in messages)))
    finally:
        if terminal is not None:
            terminal.close()
        node.executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2)+'\n')
    if len(results) != 10 or not all(r['passed'] for r in results):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
