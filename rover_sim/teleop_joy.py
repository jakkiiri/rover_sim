"""Gamepad teleop: sensor_msgs/Joy -> /cmd_vel with a deadman button.

Default mapping (Xbox-style via the ``joy`` package): left stick forward/back
= linear.x, left stick left/right = linear.y (crab/auto modes only), right
stick left/right = angular.z, LB (button 4) deadman, A (button 0) cycles the
drive mode, B (button 1) toggles the e-stop. Publishing stops (zero twist)
when the deadman is released or no Joy message arrives for ``joy_timeout``.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from rover_sim.teleop import DRIVE_MODES, STRAFE_MODES, ZERO


class JoyCommand:
    """ROS-free mapping from joystick axes/buttons to a body twist."""

    def __init__(self, linear_speed=0.6, lateral_speed=0.4, angular_speed=0.8, deadzone=0.1,
                 axis_linear=1, axis_lateral=0, axis_angular=3, button_deadman=4, button_mode=0,
                 button_estop=1, joy_timeout=0.5, drive_mode='ackermann'):
        for name, value in dict(linear_speed=linear_speed, lateral_speed=lateral_speed,
                                angular_speed=angular_speed).items():
            if not math.isfinite(value) or value < 0:
                raise ValueError(f'{name} must be finite and nonnegative')
        if not 0 <= deadzone < 1 or not math.isfinite(joy_timeout) or joy_timeout <= 0:
            raise ValueError('deadzone must lie in [0, 1) and joy_timeout must be positive')
        if drive_mode not in DRIVE_MODES:
            raise ValueError(f'drive_mode must be one of {DRIVE_MODES}')
        self.speeds = (linear_speed, lateral_speed, angular_speed)
        self.deadzone = deadzone
        self.axes = (axis_linear, axis_lateral, axis_angular)
        self.buttons = (button_deadman, button_mode, button_estop)
        self.timeout = joy_timeout
        self.mode = drive_mode
        self.estop = False
        self.command = ZERO
        self.last_message = -math.inf
        self._previous_buttons = (0, 0, 0)
        self.mode_changed = False

    def _axis(self, axes, index):
        if index < 0 or index >= len(axes):
            return 0.0
        value = axes[index]
        if not math.isfinite(value) or abs(value) < self.deadzone:
            return 0.0
        return max(-1.0, min(1.0, (abs(value) - self.deadzone) / (1 - self.deadzone))) * (1 if value > 0 else -1)

    def feed(self, axes, buttons, now):
        self.last_message = now
        pressed = tuple(int(buttons[i]) if 0 <= i < len(buttons) else 0 for i in self.buttons)
        deadman, mode_button, estop_button = pressed
        if estop_button and not self._previous_buttons[2]:
            self.estop = not self.estop
        if mode_button and not self._previous_buttons[1]:
            self.mode = DRIVE_MODES[(DRIVE_MODES.index(self.mode) + 1) % len(DRIVE_MODES)]
            self.mode_changed = True
        self._previous_buttons = pressed
        if not deadman or self.estop:
            self.command = ZERO
            return self.command
        vx = self._axis(axes, self.axes[0]) * self.speeds[0]
        vy = self._axis(axes, self.axes[1]) * self.speeds[1] if self.mode in STRAFE_MODES else 0.0
        wz = self._axis(axes, self.axes[2]) * self.speeds[2]
        self.command = (vx, vy, wz)
        return self.command

    def expire(self, now):
        if now - self.last_message >= self.timeout:
            self.command = ZERO
        return self.command


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('rover_teleop_joy')
        values = {name: self.declare_parameter(name, default).value for name, default in dict(
            linear_speed=0.6, lateral_speed=0.4, angular_speed=0.8, deadzone=0.1, axis_linear=1,
            axis_lateral=0, axis_angular=3, button_deadman=4, button_mode=0, button_estop=1,
            joy_timeout=0.5, publish_rate=20.0, drive_mode='ackermann').items()}
        rate = values.pop('publish_rate')
        self.input = JoyCommand(**values)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.mode_publisher = self.create_publisher(
            String, '/swerve/set_drive_mode', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(1.0 / rate, self.publish)
        self.mode_publisher.publish(String(data=self.input.mode))

    def on_joy(self, msg):
        self.input.feed(list(msg.axes), list(msg.buttons), time.monotonic())
        if self.input.mode_changed:
            self.mode_publisher.publish(String(data=self.input.mode))
            self.input.mode_changed = False

    def publish(self):
        vx, vy, wz = self.input.expire(time.monotonic())
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
