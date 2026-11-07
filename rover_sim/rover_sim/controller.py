"""ROS 2 Twist controller with validated state and explicit reset handshake."""
import signal
import time

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt64
from rover_sim.control_state import ControlState, DEFAULTS, ZERO, validate_parameters
from rover_sim.kinematics import modules_from_urdf


class SwerveController(Node):
    def __init__(self):
        super().__init__('swerve_controller')
        self._closing = False
        self._closed = False
        self.command_sub = None
        description = self.declare_parameter('robot_description', '',
                                             ParameterDescriptor(read_only=True)).value
        modules = modules_from_urdf(description)
        parameters = {name: self.declare_parameter(name, default).value for name, default in DEFAULTS.items()}
        self.state = ControlState(modules, parameters)
        self.add_on_set_parameters_callback(self.validate_update)
        self.publisher = self.create_publisher(JointState, '/swerve/joint_commands', 1)
        self.reset_ack = self.create_publisher(UInt64, '/swerve/reset_ack',
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.epoch = None
        self.create_subscription(UInt64, '/swerve/reset_epoch', self.on_reset,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.update, clock=self.wall_clock)
        self.get_logger().info('Waiting for complete steering feedback; modules: ' +
                               str([(m.steering, m.x, m.y, m.radius, m.drive_sign) for m in modules]))

    def validate_update(self, parameters):
        candidate = {name: self.get_parameter(name).value for name in DEFAULTS}
        candidate.update({p.name: p.value for p in parameters if p.name in DEFAULTS})
        try:
            validate_parameters(candidate, self.state.modules)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        return SetParametersResult(successful=True)

    def on_command(self, msg, generation=None):
        if getattr(self, '_closing', False) or (generation is not None and generation != self.epoch):
            return
        self.state.on_command((msg.linear.x, msg.linear.y, msg.angular.z), time.monotonic())

    def renew_command_subscription(self):
        # Humble's Python executor discards DDS MessageInfo. A new VOLATILE
        # reader drops queued old samples without guessing their publication age.
        if self.command_sub is not None:
            self.destroy_subscription(self.command_sub)
        generation = self.epoch
        self.command_sub = self.create_subscription(
            Twist, '/cmd_vel', lambda msg: self.on_command(msg, generation), 1)

    def on_reset(self, msg):
        if getattr(self, '_closing', False):
            return
        if msg.data != self.epoch:
            self.epoch = msg.data
            self.state.reset()
            self.renew_command_subscription()
            self.get_logger().info(f'Actuator epoch {self.epoch}: cleared commands; waiting for steering feedback')
        self.reset_ack.publish(UInt64(data=msg.data))

    def on_state(self, msg):
        if self._closing:
            return
        if not 0 <= msg.header.stamp.nanosec < 1000000000:
            self.state.invalidate_feedback()
            return
        self.state.on_feedback(msg.name, msg.position,
                               msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                               self.get_clock().now().nanoseconds / 1e9,
                               msg.velocity, msg.effort)

    def update(self):
        if self._closing:
            return
        self.state.configure({name: self.get_parameter(name).value for name in DEFAULTS})
        targets = self.state.step(self.get_clock().now().nanoseconds / 1e9, time.monotonic())
        if targets is None:
            return
        angles, speeds = targets
        self.publish_targets(angles, speeds)

    def publish_targets(self, angles, speeds):
        if self._closing:
            speeds = [0.0]*4
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [m.steering for m in self.state.modules] + [m.drive for m in self.state.modules]
        msg.position = angles + [0.0]*4
        msg.velocity = [0.0]*4 + speeds
        self.publisher.publish(msg)

    def close(self):
        """Stop and destroy once; repeated signals never interrupt this path."""
        if self._closed:
            return
        self._closed = True
        self._closing = True
        try:
            self.timer.cancel()
            self.state.on_command(ZERO, time.monotonic())
            self.state.speeds = [0.0]*4
            # Without complete feedback there are no fabricated steering
            # targets to publish; the plugin already inhibits wheel drive.
            if self.state.initialized and self.context.ok():
                self.publish_targets(self.state.angles, [0.0]*4)
                if not self.publisher.wait_for_all_acked(Duration(seconds=.2)):
                    self.get_logger().warning('Stop acknowledgement timed out; actuator watchdog remains active')
        finally:
            Node.destroy_node(self)


def main(args=None):
    node = None
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True
        if node is not None:
            node._closing = True

    # This console process owns these handlers through teardown and process
    # exit. They only set flags; neither ROS shutdown nor KeyboardInterrupt is
    # triggered from an asynchronous signal handler.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = SwerveController()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.02)
    finally:
        try:
            if node is not None:
                node.close()
        finally:
            if rclpy.ok():
                rclpy.try_shutdown()
