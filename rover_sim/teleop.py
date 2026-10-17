"""Standalone terminal teleop for the rover's body-frame Twist interface."""
import math
import os
import select
import signal
import sys
import termios
import time
import tty
from contextlib import contextmanager

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

ZERO = (0.0, 0.0, 0.0)
DEFAULTS = dict(linear_speed=0.4, angular_speed=0.5,
                key_timeout=0.7, publish_rate=20.0)


class KeyboardCommand:
    """Input state, with an explicit monotonic time supplied by the caller."""

    def __init__(self, linear_speed=0.4, angular_speed=0.5,
                 key_timeout=0.7, publish_rate=20.0):
        for name, value in locals().copy().items():
            if name == 'self':
                continue
            if not math.isfinite(value) or value < 0 or (
                    name in ('key_timeout', 'publish_rate') and value == 0):
                raise ValueError(f'{name} must be finite and ' +
                                 ('positive' if name in ('key_timeout', 'publish_rate')
                                  else 'nonnegative'))
        self.mapping = {
            'w': ((linear_speed, 0.0, 0.0), 'W forward'),
            's': ((-linear_speed, 0.0, 0.0), 'S backward'),
            'a': ((0.0, 0.0, angular_speed), 'A rotate left'),
            'd': ((0.0, 0.0, -angular_speed), 'D rotate right'),
        }
        self.timeout = key_timeout
        self.command, self.label = ZERO, 'STOP ready'
        self.last_key = -math.inf
        self.escape = ''

    def stop(self, reason):
        self.command, self.label = ZERO, 'STOP ' + reason

    def feed(self, data, now):
        """Return True for Ctrl+C. A stop takes precedence over queued motion."""
        if b'\x03' in data:
            self.stop('Ctrl+C')
            return True
        if b' ' in data:
            self.escape = ''
            self.stop('Space')
            return False
        for char in data.decode('ascii', errors='replace'):
            # Consume full arrow/function-key sequences, including split reads.
            # Their trailing A/D must never become a turn command.
            if self.escape:
                if self.escape == 'start' and char in '[O':
                    self.escape = 'sequence'
                elif self.escape == 'start' or '@' <= char <= '~':
                    self.escape = ''
                continue
            if char == '\x1b':
                self.escape = 'start'
                self.stop('unsupported key')
            elif char.lower() in self.mapping:
                self.command, self.label = self.mapping[char.lower()]
                self.last_key = now
            else:
                self.stop('unsupported key')
        return False

    def expire(self, now):
        if self.command != ZERO and now - self.last_key >= self.timeout:
            self.stop('key timeout')


@contextmanager
def keyboard_terminal(stream):
    """Cbreak keeps terminal Ctrl+C signals; select makes input nonblocking."""
    fd = stream.fileno()
    if not os.isatty(fd):
        raise RuntimeError('rover_teleop needs an interactive terminal for stdin')
    original = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd, termios.TCSANOW)
        termios.tcflush(fd, termios.TCIFLUSH)
        yield fd
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, original)


class RoverTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop')
        values = {name: self.declare_parameter(name, value).value
                  for name, value in DEFAULTS.items()}
        self.input = KeyboardCommand(**values)
        self.period = 1.0 / values['publish_rate']
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.displayed = None

    def publish_command(self):
        msg = Twist()
        vx, _, wz = self.input.command
        # Enforce the rover interface at publication as well as in key mappings.
        msg.linear.x, msg.angular.z = vx, wz
        msg.linear.y = 0.0
        self.publisher.publish(msg)
        display = (self.input.label, (vx, 0.0, wz))
        if display != self.displayed:
            print(f'{self.input.label}: vx={vx:+.3f} m/s '
                  f'yaw={wz:+.3f} rad/s', flush=True)
            self.displayed = display


def main(args=None):
    # Keep the ROS context alive until the final zero commands have been sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    stopping = False
    previous = {}

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, request_stop)
        node = RoverTeleop()
        with keyboard_terminal(sys.stdin) as fd:
            print('W/S forward/backward | A/D rotate left/right | '
                  'Space stop | Ctrl+C quit\n'
                  f'Hold one key; release stops after {node.input.timeout:g}s. '
                  'Keep this terminal focused.', flush=True)
            node.publish_command()
            next_publish = time.monotonic() + node.period
            while rclpy.ok() and not stopping:
                ready, _, _ = select.select([fd], [], [],
                                             min(0.05, max(0.0, next_publish-time.monotonic())))
                if stopping:
                    break
                if ready:
                    data = os.read(fd, 1024)
                    if not data:
                        break
                    stopping = node.input.feed(data, time.monotonic())
                    if node.input.command == ZERO:
                        termios.tcflush(fd, termios.TCIFLUSH)
                    # Publish input changes immediately, without waiting for a tick.
                    node.publish_command()
                before = node.input.command
                node.input.expire(time.monotonic())
                if before != node.input.command:
                    node.publish_command()
                if time.monotonic() >= next_publish:
                    node.publish_command()
                    next_publish = time.monotonic() + node.period
                rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError, OSError) as error:
        print(f'rover_teleop: {error}', file=sys.stderr, flush=True)
        raise SystemExit(1)
    finally:
        if node is not None and rclpy.ok():
            node.input.stop('shutdown')
            # Repeat briefly so reliable DDS can deliver the stop before teardown.
            for _ in range(3):
                node.publish_command()
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
