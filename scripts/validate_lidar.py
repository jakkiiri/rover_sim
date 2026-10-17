#!/usr/bin/env python3
"""Validate Classic LiDAR against analytic terrain and temporary measured boxes.

Requires a dedicated rover launch, an isolated ROS_DOMAIN_ID and
GAZEBO_MASTER_URI, and no other /cmd_vel publisher. Uses Gazebo ground truth
only for validation; it does not add odometry or a world TF broadcaster. The
temporary obstacle entities are deleted in finally, including failed checks.
Use --performance-only for matched sensor-disabled performance measurements.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import statistics
import threading
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import UInt64
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from validate_swerve import pose


def rotation(roll, pitch, yaw):
    """Active ROS fixed-axis RPY rotation, base coordinates into world."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                             math.cos(pitch), math.sin(pitch),
                             math.cos(yaw), math.sin(yaw))
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def rotate(matrix, vector):
    return tuple(sum(row[i]*vector[i] for i in range(3)) for row in matrix)


def ray_sphere(origin, direction, center, radius):
    """Nearest positive ray–sphere distance; direction must have unit length."""
    delta = [origin[i]-center[i] for i in range(3)]
    b = sum(delta[i]*direction[i] for i in range(3))
    discriminant = b*b-sum(v*v for v in delta)+radius*radius
    if discriminant < 0:
        return math.inf
    root = math.sqrt(discriminant)
    hits = [t for t in (-b-root, -b+root) if t >= 0]
    return min(hits, default=math.inf)


def ray_box(origin, direction, center, size):
    """Nearest positive ray–axis-aligned-box distance, using slab intervals."""
    lower, upper = -math.inf, math.inf
    for p, d, c, length in zip(origin, direction, center, size):
        a, b = c-length/2, c+length/2
        if abs(d) < 1e-12:
            if p < a or p > b:
                return math.inf
            continue
        t0, t1 = sorted(((a-p)/d, (b-p)/d))
        lower, upper = max(lower, t0), min(upper, t1)
        if lower > upper:
            return math.inf
    if upper < 0:
        return math.inf
    return lower if lower >= 0 else upper


def world_spheres(world_path):
    """Read this world's sphere collisions; no guessed hill geometry."""
    spheres = []
    for model in ET.parse(world_path).getroot().findall('world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                radius = collision.findtext('geometry/sphere/radius')
                if radius is None:
                    continue
                # Current terrain is unrotated with centered collision links.
                # Refuse changed assumptions rather than silently guessing.
                values = [float(v) for v in model.findtext('pose', '0 0 0 0 0 0').split()]
                if any(abs(v) > 1e-12 for v in values[3:]):
                    raise ValueError('Validation needs an explicit rotated sphere model transform')
                for element in (link, collision):
                    local = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
                    if any(abs(v) > 1e-12 for v in local):
                        raise ValueError('Validation needs an explicit sphere link/collision transform')
                spheres.append((model.attrib['name'], tuple(values[:3]), float(radius)))
    if len(spheres) < 3:
        raise ValueError('Expected at least three analytic hill sphere collisions')
    return spheres


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def sensor_geometry(base_pose, mount):
    matrix = rotation(*base_pose[3:])
    offset = rotate(matrix, mount)
    return tuple(base_pose[i]+offset[i] for i in range(3)), matrix


def scan_change(before, after):
    """Mean bounded range change; an infinite no-return is the maximum range."""
    if len(before.ranges) != len(after.ranges):
        raise ValueError('Scan length changed during movement')
    maximum = min(before.range_max, after.range_max)
    values = [abs((a if math.isfinite(a) else maximum)-
                  (b if math.isfinite(b) else maximum))
              for a, b in zip(before.ranges, after.ranges)]
    return sum(values)/len(values)


def json_safe(value):
    """Keep diagnostics valid JSON even when a malformed message contains NaN."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--performance-only', action='store_true')
    parser.add_argument('--duration', type=float, default=10., help='Wall seconds per performance window')
    parser.add_argument('--windows', type=int, default=3)
    parser.add_argument('--watchdog', type=float, default=.3, help='Configured actuator wall timeout')
    args = parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID') or not os.environ.get('GAZEBO_MASTER_URI'):
        parser.error('Set an isolated ROS_DOMAIN_ID and GAZEBO_MASTER_URI first')
    if not math.isfinite(args.duration) or args.duration < 2 or args.windows < 1:
        parser.error('Performance duration must be finite and at least 2 seconds; windows must be positive')
    if not math.isfinite(args.watchdog) or args.watchdog <= 0:
        parser.error('Watchdog must be finite and positive')

    rclpy.init()
    node = Node('lidar_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    command_pub = node.create_publisher(Twist, '/cmd_vel', 1)
    scans, clocks, epochs, heartbeats = deque(maxlen=10000), deque(maxlen=200000), [], []
    history_lock = threading.Lock()
    feedback, fixed_transforms, dynamic_lidar_transforms = {}, [], []
    command = (0., 0.)
    errors = []
    stage = 'startup'
    stage_details = {}
    epoch_events = []
    stage_events = [{'stage': stage, 'monotonic_wall_time': time.monotonic(), 'simulation_time': 0.}]
    results = {'performance_only': args.performance_only, 'performance': [], 'checks': [],
               'epoch_events': epoch_events, 'stage_events': stage_events}
    entities = []
    listener = None

    def on_scan(msg):
        with history_lock:
            scans.append((time.monotonic(), msg))

    def on_clock(msg):
        with history_lock:
            clocks.append((time.monotonic(), stamp_seconds(msg.clock)))

    def on_epoch(msg):
        wall = time.monotonic()
        epochs.append((wall, msg.data))
        epoch_events.append({'epoch': msg.data, 'monotonic_wall_time': wall,
                             'simulation_time': node.get_clock().now().nanoseconds/1e9,
                             'stage': stage, 'stage_details': dict(stage_details),
                             'command': {'linear_x': command[0], 'linear_y': 0., 'angular_z': command[1]},
                             'last_clock_wall_age': wall-clocks[-1][0] if clocks else None,
                             'last_scan_wall_age': wall-scans[-1][0] if scans else None,
                             'last_heartbeat_wall_age': wall-heartbeats[-1] if heartbeats else None})

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    node.create_subscription(ClockMessage, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', lambda msg: feedback.update(
        {name: velocity for name, velocity in zip(msg.name, msg.velocity)}), 10)
    node.create_subscription(JointState, '/swerve/joint_commands',
                             lambda msg: heartbeats.append(time.monotonic()), 10)
    node.create_subscription(UInt64, '/swerve/reset_epoch',
                             on_epoch,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, '/tf_static', lambda msg: fixed_transforms.extend(
        t for t in msg.transforms if t.child_frame_id == 'lidar_link'),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, '/tf', lambda msg: dynamic_lidar_transforms.extend(
        t for t in msg.transforms if t.child_frame_id == 'lidar_link'), qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def publish():
        msg = Twist()
        msg.linear.x, msg.angular.z = command
        msg.linear.y = 0.0
        command_pub.publish(msg)

    node.create_timer(.02, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def spin():
        try:
            rclpy.spin(node)
        except Exception as error:
            errors.append(repr(error))

    worker = threading.Thread(target=spin, daemon=True)
    worker.start()

    def set_stage(value, **details):
        nonlocal stage, stage_details
        stage, stage_details = value, details
        stage_events.append({'stage': value, 'stage_details': dict(details),
                             'monotonic_wall_time': time.monotonic(),
                             'simulation_time': node.get_clock().now().nanoseconds/1e9})

    def wait_for(predicate, timeout=20.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if errors:
                raise RuntimeError('Validation executor failed: '+errors[-1])
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for LiDAR validation condition')
            time.sleep(.01)

    def record(name, passed, **details):
        result = {'case': name, 'passed': bool(passed), **details}
        results['checks'].append(result)
        print(json.dumps(json_safe(result), allow_nan=False), flush=True)
        if not passed:
            raise RuntimeError('Failed '+name)

    def epoch_values(since):
        previous = [value for stamp, value in epochs if stamp < since]
        values = ([previous[-1]] if previous else [])+[value for stamp, value in epochs if stamp >= since]
        return list(dict.fromkeys(values))

    def call(client, request):
        wait_for(client.service_is_ready)
        future = client.call_async(request)
        wait_for(future.done)
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError('Gazebo entity service failed: '+str(result))
        return result

    def run_sim(seconds, velocity=(0., 0.)):
        nonlocal command
        command = velocity
        start = node.get_clock().now().nanoseconds/1e9
        wait_for(lambda: node.get_clock().now().nanoseconds/1e9-start >= seconds,
                 timeout=max(30., seconds*30.))

    def stopped_scan():
        nonlocal command
        command = (0., 0.)
        wait_for(lambda: len(feedback) >= 8 and max(
            abs(value) for name, value in feedback.items() if name.endswith('_wheel_joint')) < .05)
        run_sim(.4)
        measured_pose, measured_at = pose()
        wait_for(lambda: scans and stamp_seconds(scans[-1][1].header.stamp) >= measured_at)
        return measured_pose, measured_at, scans[-1][1]

    share = Path(get_package_share_directory('rover_sim'))
    spheres = world_spheres(share/'worlds/desert.world')
    boxes = []
    mount = (0., 0., .2)

    def compare_geometry(name, measured_pose, measured_at, scan, objects):
        origin, matrix = sensor_geometry(measured_pose, mount)
        comparisons = []
        for object_name, center in objects:
            delta = tuple(center[i]-origin[i] for i in range(3))
            # Inverse rotation is the transpose. Actual body pitch and roll are
            # retained for every outgoing beam and sensor-height computation.
            local = tuple(sum(matrix[j][i]*delta[j] for j in range(3)) for i in range(3))
            bearing = math.atan2(local[1], local[0])
            closest = round((bearing-scan.angle_min)/scan.angle_increment)
            selected = []
            for index in range(max(0, closest-2), min(len(scan.ranges), closest+3)):
                angle = scan.angle_min+index*scan.angle_increment
                direction = rotate(matrix, (math.cos(angle), math.sin(angle), 0.))
                intersections = [(ray_sphere(origin, direction, c, radius), key)
                                 for key, c, radius in spheres]
                intersections += [(ray_box(origin, direction, c, size), key) for key, c, size in boxes]
                intersections.append((ray_box(origin, direction, (0., 0., -.25), (100., 100., .5)), 'sand_ground'))
                expected, nearest = min(intersections)
                if nearest != object_name or not scan.range_min <= expected <= scan.range_max:
                    continue
                actual = scan.ranges[index]
                selected.append({'index': index, 'angle': angle, 'expected': expected,
                                 'actual': actual if math.isfinite(actual) else None,
                                 'absolute_error': abs(actual-expected) if math.isfinite(actual) else None})
            comparisons.append({'object': object_name, 'selected_beams': selected,
                                'passed': len(selected) >= 3 and all(
                                    item['absolute_error'] is not None and item['absolute_error'] < .06
                                    for item in selected)})
        record(name, all(item['passed'] for item in comparisons),
               base_pose=measured_pose, sensor_world_origin=origin,
               pose_sim_stamp=measured_at, scan_sim_stamp=stamp_seconds(scan.header.stamp),
               tolerance_metres=.06, comparisons=comparisons)

    spawn = node.create_client(SpawnEntity, '/spawn_entity')
    delete = node.create_client(DeleteEntity, '/delete_entity')
    try:
        wait_for(lambda: len(clocks) >= 20 and epochs and heartbeats)
        if not args.performance_only:
            wait_for(lambda: len(scans) >= 10 and fixed_transforms)
        steady_started = time.monotonic()
        for window in range(args.windows):
            set_stage('performance', window=window+1)
            started = time.monotonic()
            wait_for(lambda: time.monotonic()-started >= args.duration, args.duration+5.)
            with history_lock:
                clock_window = [(wall, sim) for wall, sim in clocks if wall >= started]
                scan_window = [(wall, msg) for wall, msg in scans if wall >= started]
            heartbeat_window = [wall for wall in heartbeats[:] if wall >= started]
            if len(clock_window) < 2 or len(heartbeat_window) < 2:
                raise RuntimeError('Missing clock or actuator samples in performance window')
            rtf = ((clock_window[-1][1]-clock_window[0][1])/
                   (clock_window[-1][0]-clock_window[0][0]))
            gaps = [b-a for a, b in zip(heartbeat_window, heartbeat_window[1:])]
            clock_gaps = [b[0]-a[0] for a, b in zip(clock_window, clock_window[1:])]
            scan_gaps = [b[0]-a[0] for a, b in zip(scan_window, scan_window[1:])]
            rate_sim = rate_wall = None
            if len(scan_window) > 1:
                simulation_duration = (stamp_seconds(scan_window[-1][1].header.stamp)-
                                       stamp_seconds(scan_window[0][1].header.stamp))
                if simulation_duration <= 0:
                    raise RuntimeError('Non-increasing scan timestamps during steady measurement')
                rate_sim = (len(scan_window)-1)/simulation_duration
                rate_wall = (len(scan_window)-1)/(scan_window[-1][0]-scan_window[0][0])
            values = epoch_values(started)
            measurement = {'window': window+1, 'wall_seconds': clock_window[-1][0]-clock_window[0][0],
                           'real_time_factor': rtf, 'scan_count': len(scan_window),
                           'scan_rate_sim_hz': rate_sim, 'scan_rate_wall_hz': rate_wall,
                           'maximum_heartbeat_wall_gap': max(gaps),
                           'maximum_clock_wall_gap': max(clock_gaps),
                           'maximum_scan_wall_gap': max(scan_gaps, default=None), 'epochs': values}
            results['performance'].append(measurement)
            print(json.dumps(measurement), flush=True)
            record('performance_window_'+str(window+1), rtf > 0 and len(values) == 1
                   and max(gaps) < args.watchdog and (args.performance_only or
                   (rate_sim is not None and 9. <= rate_sim <= 11.)), **measurement)
        results['median_real_time_factor'] = statistics.median(
            item['real_time_factor'] for item in results['performance'])
        if args.performance_only:
            record('no_watchdog_false_trip_during_performance', len(epoch_values(steady_started)) == 1,
                   epochs=epoch_values(steady_started))
            results['passed'] = True
            return

        set_stage('interface')
        scan = scans[-1][1]
        topics = dict(node.get_topic_names_and_types())
        finite = [v for v in scan.ranges if math.isfinite(v)]
        valid = all((math.isfinite(v) and scan.range_min <= v <= scan.range_max)
                    or math.isinf(v) for v in scan.ranges)
        stamp = stamp_seconds(scan.header.stamp)
        # The independent DDS /clock and /scan deliveries have no cross-topic
        # ordering guarantee. Wait for the matching clock tick, not a sleep.
        wait_for(lambda: node.get_clock().now().nanoseconds/1e9 >= stamp)
        current = node.get_clock().now().nanoseconds/1e9
        scan_publishers = node.get_publishers_info_by_topic('/scan')
        record('scan_interface', topics.get('/scan') == ['sensor_msgs/msg/LaserScan']
               and len(scan_publishers) == 1 and scan.header.frame_id == 'lidar_link' and len(scan.ranges) == 720
               and valid and bool(finite) and 0 < stamp <= current+.002 and current-stamp < .3
               # Classic's URDF-to-SDF conversion rounds angles to six
               # significant digits before float32 LaserScan serialization.
               and math.isclose(scan.angle_min, -math.pi, abs_tol=1e-5)
               and math.isclose(scan.angle_max, math.pi, abs_tol=1e-5)
               and math.isclose(scan.angle_increment, 2*math.pi/719, abs_tol=1e-7)
               and math.isclose(scan.range_min, .12, abs_tol=1e-6)
               and math.isclose(scan.range_max, 30., abs_tol=1e-6),
               topic_type=topics.get('/scan'), frame_id=scan.header.frame_id,
               scan_publishers=[item.node_namespace+'/'+item.node_name for item in scan_publishers],
               samples=len(scan.ranges), angle_min=scan.angle_min, angle_max=scan.angle_max,
               angle_increment=scan.angle_increment, range_min=scan.range_min, range_max=scan.range_max,
               finite_returns=len(finite),
               positive_infinite_returns=sum(v == math.inf for v in scan.ranges),
               negative_infinite_returns=sum(v == -math.inf for v in scan.ranges),
               scan_sim_stamp=stamp, current_sim_time=current, age_sim_seconds=current-stamp)

        transform = buffer.lookup_transform('base_link', 'lidar_link', Time.from_msg(scan.header.stamp))
        t, q = transform.transform.translation, transform.transform.rotation
        publishers = node.get_publishers_info_by_topic('/tf_static')
        mounts = [item for item in fixed_transforms if item.header.frame_id == 'base_link']
        record('tf_fixed_mount_and_ownership', len(publishers) == 1
               and publishers[0].node_name == 'robot_state_publisher'
               and not dynamic_lidar_transforms
               and len(mounts) == len(fixed_transforms) and bool(mounts)
               and all(math.isclose(actual, expected, abs_tol=1e-9)
                       for actual, expected in zip((t.x, t.y, t.z, q.x, q.y, q.z, q.w),
                                                  (*mount, 0., 0., 0., 1.))),
               parent='base_link', child='lidar_link', translation=[t.x, t.y, t.z],
               quaternion=[q.x, q.y, q.z, q.w],
               tf_static_publishers=[item.node_namespace+'/'+item.node_name for item in publishers],
               transform_messages_received=len(fixed_transforms),
               competing_dynamic_lidar_transforms=len(dynamic_lidar_transforms))

        set_stage('terrain')
        actual_pose, at, scan = stopped_scan()
        compare_geometry('analytic_hill_returns', actual_pose, at, scan,
                         [(key, center) for key, center, radius in spheres])
        for label, center, size in [('front', (5.25, 0., 1.), (.5, 2., 2.)),
                                    ('left', (0., 4.25, 1.), (2., .5, 2.))]:
            set_stage('probe_insert', probe=label)
            name = 'lidar_validation_'+label+'_'+str(os.getpid())
            dimensions = ' '.join(map(str, size))
            sdf = ('<sdf version="1.6"><model name="'+name+'"><static>true</static>'
                   '<pose>0 0 0 0 0 0</pose><link name="box">'
                   '<collision name="collision"><geometry><box><size>'+dimensions+
                   '</size></box></geometry></collision><visual name="visual"><geometry>'
                   '<box><size>'+dimensions+'</size></box></geometry></visual></link></model></sdf>')
            request = SpawnEntity.Request(name=name, xml=sdf, reference_frame='world')
            request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = center
            request.initial_pose.orientation.w = 1.
            call(spawn, request)
            entities.append(name)
            boxes.append((name, center, size))
        set_stage('known_probes')
        actual_pose, at, scan_before = stopped_scan()
        compare_geometry('known_distance_obstacles', actual_pose, at, scan_before,
                         [(key, center) for key, center, size in boxes])

        set_stage('forward')
        start_pose = actual_pose
        run_sim(2., (.25, 0.))
        moved_pose, at, moved_scan = stopped_scan()
        dx, dy = moved_pose[0]-start_pose[0], moved_pose[1]-start_pose[1]
        body_x = math.cos(start_pose[5])*dx+math.sin(start_pose[5])*dy
        body_y = -math.sin(start_pose[5])*dx+math.cos(start_pose[5])*dy
        yaw = math.atan2(math.sin(moved_pose[5]-start_pose[5]), math.cos(moved_pose[5]-start_pose[5]))
        change = scan_change(scan_before, moved_scan)
        record('scan_changes_on_forward_motion', body_x > .15 and abs(body_y) < .05
               and abs(yaw) < .05 and change > .02,
               body_displacement=[body_x, body_y], yaw_change=yaw, mean_scan_change_metres=change)
        compare_geometry('obstacles_after_forward_motion', moved_pose, at, moved_scan,
                         [(key, center) for key, center, size in boxes])
        set_stage('yaw')
        run_sim(3., (0., .4))
        rotated_pose, at, rotated_scan = stopped_scan()
        yaw = math.atan2(math.sin(rotated_pose[5]-moved_pose[5]), math.cos(rotated_pose[5]-moved_pose[5]))
        translation = math.hypot(rotated_pose[0]-moved_pose[0], rotated_pose[1]-moved_pose[1])
        change = scan_change(moved_scan, rotated_scan)
        record('scan_changes_on_yaw_motion', yaw > .2 and translation < .1 and change > .05,
               yaw_change=yaw, center_displacement=translation, mean_scan_change_metres=change)
        compare_geometry('obstacles_after_yaw_motion', rotated_pose, at, rotated_scan,
                         [(key, center) for key, center, size in boxes])
        record('no_watchdog_false_trip_during_sensor_validation', len(epoch_values(steady_started)) == 1,
               epochs=epoch_values(steady_started))
        results['passed'] = True
    except Exception as error:
        results['passed'] = False
        results['error'] = repr(error)
        raise
    finally:
        set_stage('cleanup')
        command = (0., 0.)
        publish()
        cleanup_errors = []
        for name in reversed(entities):
            try:
                call(delete, DeleteEntity.Request(name=name))
            except Exception as error:
                cleanup_errors.append(name+': '+repr(error))
        results['temporary_entities_removed'] = not cleanup_errors
        if cleanup_errors:
            results['cleanup_errors'] = cleanup_errors
            results['passed'] = False
        if node.executor is not None:
            node.executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(json_safe(results), indent=2, allow_nan=False)+'\n')
        if cleanup_errors:
            raise RuntimeError('Failed to delete temporary validation entities: '+str(cleanup_errors))


if __name__ == '__main__':
    main()
