"""Standalone terminal teleop for the rover's body-frame Twist interface.

Keys: W/S forward/backward, A/D rotate, Q/E strafe (crab/auto modes only),
+/- speed up/down, M cycle drive mode, X e-stop toggle, Space stop,
Ctrl+C quit. A status line prints whenever the state changes.
"""
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
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

ZERO = (0.0, 0.0, 0.0)
DRIVE_MODES = ('ackermann', 'crab', 'point_turn', 'auto')
STRAFE_MODES = ('crab', 'auto')
DEFAULTS = dict(linear_speed=0.4, lateral_speed=0.3, angular_speed=0.5,
                key_timeout=0.7, publish_rate=20.0, speed_step=1.25,
                min_scale=0.2, max_scale=2.5)


class KeyboardCommand:
    """Input state, with an explicit monotonic time supplied by the caller."""

    def __init__(self, linear_speed=0.4, lateral_speed=0.3, angular_speed=0.5,
                 key_timeout=0.7, publish_rate=20.0, speed_step=1.25,
                 min_scale=0.2, max_scale=2.5, drive_mode='ackermann'):
        for name, value in dict(linear_speed=linear_speed, lateral_speed=lateral_speed,
                                angular_speed=angular_speed, key_timeout=key_timeout,
                                publish_rate=publish_rate, speed_step=speed_step,
                                min_scale=min_scale, max_scale=max_scale).items():
            if not math.isfinite(value) or value < 0 or (
                    name in ('key_timeout', 'publish_rate', 'min_scale', 'max_scale') and value == 0):
                raise ValueError(f'{name} must be finite and ' +
                                 ('positive' if name in ('key_timeout', 'publish_rate', 'min_scale', 'max_scale')
                                  else 'nonnegative'))
        if speed_step <= 1.0:
            raise ValueError('speed_step must exceed 1')
        if min_scale > max_scale:
            raise ValueError('min_scale must not exceed max_scale')
        if drive_mode not in DRIVE_MODES:
            raise ValueError(f'drive_mode must be one of {DRIVE_MODES}')
        self.speeds = dict(linear=linear_speed, lateral=lateral_speed, angular=angular_speed)
        self.scale, self.speed_step, self.min_scale, self.max_scale = 1.0, speed_step, min_scale, max_scale
        self.mode = drive_mode
        self.mode_changed = False
        self.estop = False
        self.timeout = key_timeout
        self.command, self.label = ZERO, 'STOP ready'
        self.last_key = -math.inf
        self.escape = ''

    def _mapping(self):
        lin, lat, ang = (self.speeds['linear'] * self.scale, self.speeds['lateral'] * self.scale,
                         self.speeds['angular'] * self.scale)
        mapping = {
            'w': ((lin, 0.0, 0.0), 'W forward'),
            's': ((-lin, 0.0, 0.0), 'S backward'),
            'a': ((0.0, 0.0, ang), 'A rotate left'),
            'd': ((0.0, 0.0, -ang), 'D rotate right'),
        }
        if self.mode in STRAFE_MODES:
            mapping['q'] = ((0.0, lat, 0.0), 'Q strafe left')
            mapping['e'] = ((0.0, -lat, 0.0), 'E strafe right')
        return mapping

    def stop(self, reason):
        self.command, self.label = ZERO, 'STOP ' + reason

    def status(self):
        return (f'mode={self.mode} speed x{self.scale:.2f} '
                f'({self.speeds["linear"] * self.scale:.2f} m/s, {self.speeds["angular"] * self.scale:.2f} rad/s) '
                f'{"E-STOP " if self.estop else ""}{self.label}')

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
            key = char.lower()
            mapping = self._mapping()
            if char == '\x1b':
                self.escape = 'start'
                self.stop('unsupported key')
            elif key == 'x':
                self.estop = not self.estop
                self.stop('e-stop engaged' if self.estop else 'e-stop released')
            elif key == 'm':
                self.mode = DRIVE_MODES[(DRIVE_MODES.index(self.mode) + 1) % len(DRIVE_MODES)]
                self.mode_changed = True
                self.stop('mode ' + self.mode)
            elif key in ('+', '='):
                self.scale = min(self.max_scale, self.scale * self.speed_step)
                self.label = f'speed x{self.scale:.2f}'
            elif key in ('-', '_'):
                self.scale = max(self.min_scale, self.scale / self.speed_step)
                self.label = f'speed x{self.scale:.2f}'
            elif key in mapping and not self.estop:
                self.command, self.label = mapping[key]
                self.last_key = now
            else:
                self.stop('e-stop' if self.estop and key in mapping else 'unsupported key')
        if self.estop:
            self.command = ZERO
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
        drive_mode = self.declare_parameter('drive_mode', 'ackermann').value
        self.input = KeyboardCommand(drive_mode=drive_mode, **values)
        self.period = 1.0 / values['publish_rate']
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.mode_publisher = self.create_publisher(
            String, '/swerve/set_drive_mode', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.displayed = None

    def publish_command(self):
        msg = Twist()
        vx, vy, wz = self.input.command
        if self.input.estop or self.input.mode not in STRAFE_MODES:
            vy = 0.0
        if self.input.estop:
            vx = wz = 0.0
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.publisher.publish(msg)
        if getattr(self.input, 'mode_changed', False):
            self.mode_publisher.publish(String(data=self.input.mode))
            self.input.mode_changed = False
        display = (self.input.status(), (vx, vy, wz))
        if display != self.displayed:
            print(f'{self.input.label}: vx={vx:+.3f} vy={vy:+.3f} m/s yaw={wz:+.3f} rad/s | '
                  f'{self.input.status()}', flush=True)
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
            print('W/S forward/backward | A/D rotate | Q/E strafe (crab/auto) | +/- speed | '
                  'M drive mode | X e-stop | Space stop | Ctrl+C quit\n'
                  f'Hold one key; release stops after {node.input.timeout:g}s. '
                  'Keep this terminal focused.', flush=True)
            node.mode_publisher.publish(String(data=node.input.mode))
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
