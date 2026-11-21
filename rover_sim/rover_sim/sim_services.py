"""Gazebo Fortress world services as seen from ROS (replaces gazebo_msgs).

launch/desert.launch.py bridges the world's gz services to fixed ROS names
(ros_gz_interfaces types), so tools work in any world:

    /sim/spawn_entity     SpawnEntity    (gz /world/<name>/create)
    /sim/delete_entity    DeleteEntity   (gz /world/<name>/remove)
    /sim/set_entity_pose  SetEntityPose  (gz /world/<name>/set_pose)
    /sim/control          ControlWorld   (gz /world/<name>/control)

The request builders below are plain functions so tests can check them
without a simulator. Isolation: Classic tools required GAZEBO_MASTER_URI;
Fortress transport is separated by IGN_PARTITION (plus ROS_DOMAIN_ID).
"""
import math
import os
import subprocess

SPAWN = '/sim/spawn_entity'
DELETE = '/sim/delete_entity'
SET_POSE = '/sim/set_entity_pose'
CONTROL = '/sim/control'
ROVER = 'proxy_rover'          # entity name of the rover in every launch (both robots)


def require_isolation(parser):
    """argparse error unless this run has its own ROS domain and gz partition."""
    if not os.environ.get('ROS_DOMAIN_ID') or not os.environ.get('IGN_PARTITION'):
        parser.error('Set an isolated ROS_DOMAIN_ID and IGN_PARTITION first '
                     '(e.g. export ROS_DOMAIN_ID=73 IGN_PARTITION=rover_validation)')


def spawn_request(name, sdf, position, orientation=(1.0, 0.0, 0.0, 0.0)):
    """SpawnEntity request for an SDF string at a world pose; orientation is (w, x, y, z)."""
    from ros_gz_interfaces.srv import SpawnEntity
    request = SpawnEntity.Request()
    factory = request.entity_factory
    factory.name = name
    factory.allow_renaming = False
    factory.sdf = sdf
    _set_pose(factory.pose, position, orientation)
    return request


def delete_request(name):
    """DeleteEntity request for a model by name."""
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import DeleteEntity
    request = DeleteEntity.Request()
    request.entity.name = name
    request.entity.type = Entity.MODEL
    return request


def set_pose_request(name, position, orientation):
    """SetEntityPose request for a model by name; orientation is (w, x, y, z)."""
    from ros_gz_interfaces.msg import Entity
    from ros_gz_interfaces.srv import SetEntityPose
    request = SetEntityPose.Request()
    request.entity.name = name
    request.entity.type = Entity.MODEL
    _set_pose(request.pose, position, orientation)
    return request


def succeeded(response):
    """True for a ros_gz_interfaces response that reports success."""
    return response is not None and bool(getattr(response, 'success', False))


def yaw_quaternion(roll, pitch, yaw):
    """(w, x, y, z) for fixed-axis roll, pitch, yaw (radians)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def odometry_pose(msg):
    """([x, y, z, roll, pitch, yaw], sim stamp seconds) from a nav_msgs/Odometry."""
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [p.x, p.y, p.z, roll, pitch, yaw], msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9


def _one_shot(work, timeout):
    """Run work(node, executor) on a private node and executor (callers may be spinning their own)."""
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    node = rclpy.create_node(f'sim_services_{os.getpid()}_{id(work)}')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        return work(node, executor, timeout)
    finally:
        executor.remove_node(node)
        node.destroy_node()


def ground_truth_pose(timeout=10.0, after=None):
    """Latest rover pose from /ground_truth/odom (the swerve System's ground truth).

    Returns ([x, y, z, roll, pitch, yaw], sim stamp). ``after``: wait for a
    message stamped later than this sim time. Replaces Classic's
    `gz topic -e /gazebo/<world>/pose/info` reader.
    """
    from nav_msgs.msg import Odometry

    def work(node, executor, timeout):
        latest = []
        node.create_subscription(Odometry, '/ground_truth/odom', latest.append, 10)
        import time
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if latest:
                pose, stamp = odometry_pose(latest[-1])
                if after is None or stamp > after:
                    return pose, stamp
        raise RuntimeError('No /ground_truth/odom; is the rover launch running?')
    return _one_shot(work, timeout)


def teleport(position, rpy=(0.0, 0.0, 0.0), name=ROVER, timeout=10.0):
    """Move a model to a world pose through /sim/set_entity_pose (replaces `gz model -m`)."""
    from ros_gz_interfaces.srv import SetEntityPose

    def work(node, executor, timeout):
        client = node.create_client(SetEntityPose, SET_POSE)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{SET_POSE} is not available')
        future = client.call_async(set_pose_request(name, position, yaw_quaternion(*rpy)))
        executor.spin_until_future_complete(future, timeout_sec=timeout)
        if not succeeded(future.result()):
            raise RuntimeError(f'{SET_POSE} failed for {name}')
    _one_shot(work, timeout)


def robot_description(timeout=10.0):
    """The running robot's URDF from /robot_description (latched by robot_state_publisher)."""
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    def work(node, executor, timeout):
        import time
        got = []
        node.create_subscription(String, '/robot_description', lambda m: got.append(m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = time.monotonic() + timeout
        while not got and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if not got:
            raise RuntimeError('No /robot_description; is the rover launch running?')
        return got[0]
    return _one_shot(work, timeout)


def description_sensors(urdf):
    """Gazebo sensors in a URDF: {name: {'type', 'topic', 'rate', 'frame'}}.

    Validators take expected sensor rates from here, so they follow the launch
    arguments (lidar_rate, stereo_rate) instead of hard-coded numbers.
    """
    import xml.etree.ElementTree as ET
    sensors = {}
    for sensor in ET.fromstring(urdf).iter('sensor'):
        rate = sensor.findtext('update_rate')
        sensors[sensor.get('name')] = {
            'type': sensor.get('type'), 'topic': (sensor.findtext('topic') or '').strip(),
            'rate': float(rate) if rate else None,
            'frame': (sensor.findtext('ignition_frame_id') or '').strip()}
    return sensors


def gz_world_service(world, service, request_type, request, timeout_ms=5000):
    """Call a gz world service with the `ign service` CLI (respects IGN_PARTITION).

    For services the ROS bridge does not carry, e.g. set_physics. Raises on
    failure or a false reply.
    """
    result = subprocess.run(
        ['ign', 'service', '-s', f'/world/{world}/{service}', '--reqtype', f'ignition.msgs.{request_type}',
         '--reptype', 'ignition.msgs.Boolean', '--timeout', str(timeout_ms), '--req', request],
        capture_output=True, text=True, timeout=timeout_ms / 1000 + 5)
    if result.returncode != 0 or 'data: true' not in result.stdout:
        raise RuntimeError(f'/world/{world}/{service} {request!r} failed: {result.stdout} {result.stderr}')


def world_control(world, request):
    """WorldControl request text, e.g. 'pause: true' or 'reset: {all: true}'."""
    gz_world_service(world, 'control', 'WorldControl', request)


def set_real_time_factor(world, factor, step=0.001):
    """Target real-time factor (Classic's `gz physics -u` changed the update rate instead)."""
    gz_world_service(world, 'set_physics', 'Physics', f'max_step_size: {step:g}, real_time_factor: {factor:g}')


def _set_pose(pose, position, orientation):
    pose.position.x, pose.position.y, pose.position.z = (float(v) for v in position)
    w, x, y, z = (float(v) for v in orientation)
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = w, x, y, z
