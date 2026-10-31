"""ROS 2 swerve controller: Twist in, joint targets, odometry and diagnostics out.

Keeps the reset-epoch handshake, wall-time command watchdog, simulation-time
feedback guard and the idempotent shutdown of the prototype, and adds drive
modes, IMU tilt limits, traction/stall diagnostics and wheel odometry.
"""
import math
import signal
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String, UInt64
from tf2_msgs.msg import TFMessage
from rover_sim.control_state import ControlState, DEFAULTS, DRIVE_MODES, ZERO, validate_parameters
from rover_sim.kinematics import modules_from_urdf


def quaternion_to_roll_pitch(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


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
        drive_mode = self.declare_parameter('drive_mode', 'ackermann').value
        self.publish_tf = self.declare_parameter('publish_odom_tf', True).value
        self.odom_frame = self.declare_parameter('odom_frame', 'odom').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        try:
            validate_parameters(parameters, modules)
            self.state = ControlState(modules, parameters, drive_mode)
        except ValueError as error:
            # Fail loudly: an invalid configuration must not start a controller.
            raise SystemExit(f'swerve_controller configuration rejected: {error}')
        self.add_on_set_parameters_callback(self.validate_update)
        self.publisher = self.create_publisher(JointState, '/swerve/joint_commands', 1)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/swerve/diagnostics', 10)
        self.mode_pub = self.create_publisher(String, '/swerve/drive_mode',
                                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.reset_ack = self.create_publisher(UInt64, '/swerve/reset_ack',
                                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.epoch = None
        self.create_subscription(UInt64, '/swerve/reset_epoch', self.on_reset,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(String, '/swerve/set_drive_mode', self.on_drive_mode, 1)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.02, self.update, clock=self.wall_clock)
        self.diag_timer = self.create_timer(0.2, self.publish_diagnostics, clock=self.wall_clock)
        self.odom_seq = 0
        self.mode_pub.publish(String(data=self.state.drive_mode))
        self.get_logger().info(f'Drive mode {self.state.drive_mode}; waiting for complete steering feedback; modules: ' +
                               str([(m.steering, m.x, m.y, m.radius, m.drive_sign) for m in modules]))

    # -- parameters and mode -------------------------------------------------
    def validate_update(self, parameters):
        candidate = {name: self.get_parameter(name).value for name in DEFAULTS}
        candidate.update({p.name: p.value for p in parameters if p.name in DEFAULTS})
        mode = None
        for p in parameters:
            if p.name == 'drive_mode':
                if p.value not in DRIVE_MODES:
                    return SetParametersResult(successful=False, reason=f'drive_mode must be one of {DRIVE_MODES}')
                mode = p.value
        try:
            validate_parameters(candidate, self.state.modules)
        except (ValueError, TypeError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        if mode is not None and mode != self.state.drive_mode:
            self.apply_drive_mode(mode)
        return SetParametersResult(successful=True)

    def apply_drive_mode(self, mode):
        self.state.set_drive_mode(mode)
        self.mode_pub.publish(String(data=mode))
        self.get_logger().info(f'Drive mode -> {mode}')

    def on_drive_mode(self, msg):
        if self._closing:
            return
        mode = msg.data.strip().lower()
        if mode not in DRIVE_MODES:
            self.get_logger().warning(f'Ignoring unknown drive mode {msg.data!r}; valid: {DRIVE_MODES}')
            return
        if mode != self.state.drive_mode:
            self.apply_drive_mode(mode)

    # -- inputs ---------------------------------------------------------------
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
        accepted = self.state.on_feedback(msg.name, msg.position,
                                          msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                                          self.get_clock().now().nanoseconds / 1e9,
                                          msg.velocity, msg.effort)
        # Odometry is integrated on feedback, so publish it with the feedback's
        # precise simulation stamp. The node clock only advances with /clock
        # (10 Hz by default), which would give runs of identical stamps and
        # make RViz jump between poses.
        if accepted and self.state.initialized:
            self.publish_odometry(rclpy.time.Time.from_msg(msg.header.stamp))

    def on_imu(self, msg):
        if self._closing:
            return
        q, a = msg.orientation, msg.linear_acceleration
        roll, pitch = quaternion_to_roll_pitch(q.x, q.y, q.z, q.w)
        self.state.on_imu(roll, pitch, msg.angular_velocity.z, msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                          (a.x, a.y, a.z))

    # -- control tick ---------------------------------------------------------
    def update(self):
        if self._closing:
            return
        changes = {name: self.get_parameter(name).value for name in DEFAULTS}
        self.state.configure(changes)
        now = self.get_clock().now()
        targets = self.state.step(now.nanoseconds / 1e9, time.monotonic())
        if targets is None:
            return
        angles, speeds = targets
        self.publish_targets(angles, speeds)

    def publish_targets(self, angles, speeds):
        if self._closing:
            speeds = [0.0] * len(speeds)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [m.steering for m in self.state.modules] + [m.drive for m in self.state.modules]
        msg.position = list(angles) + [0.0] * len(speeds)
        msg.velocity = [0.0] * len(angles) + list(speeds)
        self.publisher.publish(msg)

    def publish_odometry(self, now):
        x, y, yaw = self.state.odometry.pose()
        vx, vy, wz = self.state.odometry.twist
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = vx, vy, wz
        self.odom_pub.publish(odom)
        if self.publish_tf:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x, transform.transform.translation.y = x, y
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf_pub.publish(TFMessage(transforms=[transform]))

    def publish_diagnostics(self):
        if self._closing:
            return
        d = self.state.diagnostics
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(name='swerve_controller', hardware_id='rover_sim')
        if d.get('stalled'):
            status.level, status.message = DiagnosticStatus.WARN, 'stall: commanded motion without measured motion'
        elif any(s > self.state.parameters['slip_threshold'] for s in d.get('slip', [])):
            status.level, status.message = DiagnosticStatus.WARN, 'wheel slip: traction control active'
        elif d.get('tilt_scale', 1.0) < 1.0:
            status.level, status.message = DiagnosticStatus.WARN, 'tilt limit active'
        elif not self.state.initialized:
            status.level, status.message = DiagnosticStatus.WARN, 'waiting for steering feedback'
        else:
            status.level, status.message = DiagnosticStatus.OK, 'ok'
        values = dict(drive_mode=d.get('mode'), tilt_deg=d.get('tilt_deg', 0.0), tilt_scale=d.get('tilt_scale'),
                      steer_scale=d.get('steer_scale'), stalled=d.get('stalled'), command_fresh=d.get('command_fresh'),
                      feedback_fresh=d.get('feedback_fresh'), twist=d.get('twist'), measured_twist=d.get('measured_twist'),
                      odom_pose=self.state.odometry.pose())
        for i, m in enumerate(self.state.modules):
            values[f'slip_{m.drive}'] = d.get('slip', [0.0] * 4)[i]
            values[f'traction_scale_{m.drive}'] = d.get('traction_scale', [1.0] * 4)[i]
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        array.status.append(status)
        self.diag_pub.publish(array)

    # -- shutdown -------------------------------------------------------------
    def close(self):
        """Stop and destroy once; repeated signals never interrupt this path."""
        if self._closed:
            return
        self._closed = True
        self._closing = True
        try:
            self.timer.cancel()
            self.state.on_command(ZERO, time.monotonic())
            self.state.speeds = [0.0] * len(self.state.modules)
            # Without complete feedback there are no fabricated steering
            # targets to publish; the plugin already inhibits wheel drive.
            if self.state.initialized and self.context.ok():
                self.publish_targets(self.state.angles, [0.0] * len(self.state.modules))
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
