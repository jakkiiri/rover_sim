#!/usr/bin/env python3
"""Validate 3D clouds in a dedicated Classic desert launch (moves the rover).

Use an isolated ROS_DOMAIN_ID and GAZEBO_MASTER_URI with no teleop/other
command publisher. See docs/lidar3d.md for complete launch and run commands.
Analytical scene comparisons use Gazebo truth strictly for testing, not TF.
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

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import UInt64
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from rover_sim.lidar_postprocess import CLOUD_QOS, cloud_to_array
from validate_lidar import ray_box, ray_sphere, sensor_geometry, stamp_seconds, world_spheres
from validate_swerve import pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profile', choices=['lite', 'standard', 'high', 'dense'], default='lite')
    parser.add_argument('--duration', type=float, default=5.)
    parser.add_argument('--windows', type=int, default=2)
    parser.add_argument('--watchdog', type=float, default=.3)
    parser.add_argument('--performance-only', action='store_true', help='Sensor-disabled baseline')
    args = parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID') or not os.environ.get('GAZEBO_MASTER_URI'):
        parser.error('Set an isolated ROS_DOMAIN_ID and GAZEBO_MASTER_URI')
    if args.windows < 1 or not all(math.isfinite(v) and v > 0 for v in (args.duration, args.watchdog)):
        parser.error('duration, windows and watchdog must be positive and finite')
    channels, samples, vfov = {
        'lite': (16, 450, 30), 'standard': (16, 900, 30),
        'high': (32, 1024, 45), 'dense': (64, 1024, 45)}[args.profile]
    results = {'profile': args.profile, 'performance_only': args.performance_only,
               'checks': [], 'performance': [], 'passed': False}
    rclpy.init()
    node = Node('lidar3d_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    raw, clouds, clocks = deque(maxlen=60), deque(maxlen=60), deque(maxlen=20000)
    beats, epochs, fixed, dynamic, errors = deque(maxlen=20000), [], [], [], []
    lock = threading.Lock()
    state, command, entities = {}, [0., 0.], []
    mount = (0., 0., .4)

    def receive(history, msg):
        with lock:
            history.append((time.monotonic(), msg))

    node.create_subscription(PointCloud2, '/points_raw', lambda m: receive(raw, m), CLOUD_QOS)
    node.create_subscription(PointCloud2, '/points', lambda m: receive(clouds, m), CLOUD_QOS)
    node.create_subscription(ClockMessage, '/clock', lambda m: receive(clocks, m), qos_profile_sensor_data)
    node.create_subscription(JointState, '/joint_states', lambda m: state.update(
        {n: v for n, v in zip(m.name, m.velocity)}), 10)
    node.create_subscription(JointState, '/swerve/joint_commands', lambda m: beats.append(time.monotonic()), 10)
    node.create_subscription(UInt64, '/swerve/reset_epoch', lambda m: epochs.append((time.monotonic(), m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, '/tf_static', lambda m: fixed.extend(
        t for t in m.transforms if t.child_frame_id == 'lidar_link'),
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, '/tf', lambda m: dynamic.extend(
        t for t in m.transforms if t.child_frame_id == 'lidar_link'), qos_profile_sensor_data)
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)

    def publish():
        msg = Twist()
        msg.linear.x, msg.angular.z = command
        publisher.publish(msg)

    node.create_timer(.02, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def spin():
        try:
            rclpy.spin(node)
        except ExternalShutdownException:
            pass
        except Exception as error:
            errors.append(repr(error))

    worker = threading.Thread(target=spin, daemon=True)
    worker.start()

    def wait(predicate, timeout=30.):
        deadline = time.monotonic() + timeout
        while not predicate():
            if errors:
                raise RuntimeError(errors[-1])
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for 3D LiDAR validation condition')
            time.sleep(.01)

    def record(case, passed, **details):
        entry = dict(case=case, passed=bool(passed), **details)
        results['checks'].append(entry)
        print(json.dumps(entry, allow_nan=False), flush=True)
        if not passed:
            raise RuntimeError('Failed ' + case)

    def sim_now():
        return node.get_clock().now().nanoseconds / 1e9

    def run(seconds, velocity=(0., 0.)):
        command[:] = velocity
        start = sim_now()
        wait(lambda: sim_now() - start >= seconds, max(30., seconds * 30.))

    def epoch_values(start):
        before = [v for t, v in epochs if t < start]
        return set((before[-1:] if before else []) + [v for t, v in epochs if t >= start])

    def call(client, request):
        wait(client.service_is_ready)
        future = client.call_async(request)
        wait(future.done)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('Gazebo entity service failed: ' + str(response))

    def stopped_cloud():
        command[:] = (0., 0.)
        wait(lambda: len(state) == 8 and max(abs(v) for n, v in state.items()
                                           if n.endswith('_wheel_joint')) < .05)
        run(.4)
        base, at = pose()
        history = raw
        wait(lambda: history and stamp_seconds(history[-1][1].header.stamp) >= at)
        return base, history[-1][1]

    share = Path(get_package_share_directory('rover_sim'))
    spheres = world_spheres(share / 'worlds/desert.world')
    boxes = []

    def geometry(case, base, cloud, require_box=False):
        array = cloud_to_array(cloud)
        xyz = np.stack([array[k] for k in ('x', 'y', 'z')], axis=1).astype(float)
        xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
        origin, matrix = sensor_geometry(base, mount)
        grouped = {}
        self_hits = 0
        for point in xyz:
            length = float(np.linalg.norm(point))
            if not .5 <= length <= 100:
                continue
            direction = tuple(sum(matrix[i][j] * point[j] / length for j in range(3)) for i in range(3))
            intersections = [(ray_sphere(origin, direction, c, r), name) for name, c, r in spheres]
            intersections += [(ray_box(origin, direction, c, size), name) for name, c, size in boxes]
            intersections.append((ray_box(origin, direction, (0., 0., -.25), (100., 100., .5)), 'ground'))
            expected, name = min(intersections)
            if math.isfinite(expected) and .5 <= expected < 99.9:
                grouped.setdefault(name, []).append(abs(length - expected))
            body = point + np.asarray(mount)
            self_hits += int(abs(body[0]) < .55 and abs(body[1]) < .55 and -.35 < body[2] < .11)
        summary = {name: dict(count=len(values), median_error_m=float(np.median(values)),
                              p95_error_m=float(np.percentile(values, 95))) for name, values in grouped.items()}
        selected = [summary[name] for name in (['probe'] if require_box else ['ground'] + [s[0] for s in spheres])
                    if name in summary]
        needed = 1 if require_box else 4
        record(case, len(selected) == needed and self_hits == 0 and all(
            v['count'] >= 5 and v['median_error_m'] < .04 and v['p95_error_m'] < .09 for v in selected),
            base_pose=base, sensor_world_origin=list(origin), self_hits=self_hits, objects=summary)
        return xyz

    spawn = node.create_client(SpawnEntity, '/spawn_entity')
    delete = node.create_client(DeleteEntity, '/delete_entity')
    try:
        wait(lambda: len(clocks) >= 20 and beats and epochs and len(state) == 8)
        if not args.performance_only:
            wait(lambda: len(raw) >= 10 and len(clouds) >= 10 and fixed)
        steady = time.monotonic()
        for w in range(args.windows):
            start = time.monotonic()
            wait(lambda: time.monotonic() - start >= args.duration, args.duration + 5)
            with lock:
                ticks = [(t, stamp_seconds(m.clock)) for t, m in clocks if t >= start]
                scans = [(t, stamp_seconds(m.header.stamp)) for t, m in raw if t >= start]
            heartbeat = [t for t in list(beats) if t >= start]
            rtf = (ticks[-1][1] - ticks[0][1]) / (ticks[-1][0] - ticks[0][0])
            gap = max(b - a for a, b in zip(heartbeat, heartbeat[1:]))
            rate_sim = ((len(scans)-1) / (scans[-1][1]-scans[0][1])) if len(scans) > 1 else None
            rate_wall = ((len(scans)-1) / (scans[-1][0]-scans[0][0])) if len(scans) > 1 else None
            values = sorted(epoch_values(start))
            measurement = dict(real_time_factor=rtf, scan_rate_sim_hz=rate_sim,
                               scan_rate_wall_hz=rate_wall, maximum_heartbeat_wall_gap=gap, epochs=values)
            results['performance'].append(measurement)
            record('performance_window_' + str(w+1), rtf > 0 and len(values) == 1 and gap < args.watchdog
                   and (args.performance_only or rate_sim is not None and 9 <= rate_sim <= 11), **measurement)
        results['median_real_time_factor'] = statistics.median(v['real_time_factor'] for v in results['performance'])
        if args.performance_only:
            record('no_watchdog_false_trip', len(epoch_values(steady)) == 1)
            results['passed'] = True
            return

        raw_msg, msg = raw[-1][1], clouds[-1][1]
        array = cloud_to_array(msg)
        xyz = np.stack([array[k] for k in ('x', 'y', 'z')], axis=1)
        ranges = np.linalg.norm(xyz, axis=1)
        topics = dict(node.get_topic_names_and_types())
        record('cloud_interface', all(topics.get(t) == ['sensor_msgs/msg/PointCloud2']
                                     and len(node.get_publishers_info_by_topic(t)) == 1 for t in ('/points_raw', '/points'))
               and msg.header.frame_id == raw_msg.header.frame_id == 'lidar_link'
               and {'x', 'y', 'z', 'intensity', 'ring', 'time'} <= set(array.dtype.names)
               and len(xyz) > 100 and np.all(np.isfinite(xyz)) and np.all((ranges >= .49999) & (ranges <= 100.00001))
               and np.all(array['ring'] < channels) and np.all((array['time'] >= 0) & (array['time'] <= .10001)),
               fields=list(array.dtype.names), points=len(xyz), frame_id=msg.header.frame_id,
               rings_present=[int(v) for v in np.unique(array['ring'])])
        stamps = [stamp_seconds(m.header.stamp) for _, m in list(raw)]
        wait(lambda: sim_now() >= stamps[-1])
        raw_stamps = set(stamps)
        processed_stamps = {stamp_seconds(m.header.stamp) for _, m in list(clouds)}
        record('simulation_timestamps', stamps[0] > 0 and all(b > a for a, b in zip(stamps, stamps[1:]))
               and sim_now()-stamps[-1] < .5 and bool(raw_stamps & processed_stamps),
               latest_stamp=stamps[-1], current_sim_time=sim_now())
        transform = buffer.lookup_transform('base_link', 'lidar_link', Time.from_msg(msg.header.stamp))
        t, q = transform.transform.translation, transform.transform.rotation
        owners = node.get_publishers_info_by_topic('/tf_static')
        record('fixed_tf_and_ownership', len(owners) == 1 and owners[0].node_name == 'robot_state_publisher'
               and not dynamic and all(v.header.frame_id == 'base_link' for v in fixed)
               and np.allclose([t.x, t.y, t.z, q.x, q.y, q.z, q.w], [*mount, 0, 0, 0, 1]),
               translation=[t.x, t.y, t.z], owner=owners[0].node_name if owners else None)

        base, cloud = stopped_cloud()
        geometry('ground_and_hill_geometry', base, cloud)
        name = 'lidar3d_probe_' + str(os.getpid())
        size, center = (.5, 2., 2.), (5.25, 0., 1.)
        sdf = ('<sdf version="1.6"><model name="probe"><static>true</static><link name="box">'
               '<collision name="c"><geometry><box><size>.5 2 2</size></box></geometry></collision>'
               '<visual name="v"><geometry><box><size>.5 2 2</size></box></geometry></visual>'
               '</link></model></sdf>')
        request = SpawnEntity.Request(name=name, xml=sdf, reference_frame='world')
        request.initial_pose.position.x, request.initial_pose.position.y, request.initial_pose.position.z = center
        request.initial_pose.orientation.w = 1.
        call(spawn, request)
        entities.append(name)
        boxes.append(('probe', center, size))
        base, cloud = stopped_cloud()
        geometry('known_distance_box', base, cloud, True)
        run(2., (.25, 0.))
        moved, cloud = stopped_cloud()
        dx, dy = moved[0]-base[0], moved[1]-base[1]
        forward = math.cos(base[5])*dx + math.sin(base[5])*dy
        lateral = -math.sin(base[5])*dx + math.cos(base[5])*dy
        yaw = math.atan2(math.sin(moved[5]-base[5]), math.cos(moved[5]-base[5]))
        record('cloud_on_forward_motion', forward > .15 and abs(lateral) < .05 and abs(yaw) < .05,
               forward_m=forward, lateral_m=lateral, yaw_rad=yaw)
        geometry('box_after_forward_motion', moved, cloud, True)
        run(3., (0., .4))
        rotated, cloud = stopped_cloud()
        yaw = math.atan2(math.sin(rotated[5]-moved[5]), math.cos(rotated[5]-moved[5]))
        shift = math.hypot(rotated[0]-moved[0], rotated[1]-moved[1])
        record('cloud_on_yaw_motion', yaw > .2 and shift < .1, yaw_rad=yaw, center_shift_m=shift)
        geometry('box_after_yaw_motion', rotated, cloud, True)
        record('no_watchdog_false_trip', len(epoch_values(steady)) == 1, epochs=sorted(epoch_values(steady)))
        results['passed'] = True
    except Exception as error:
        results['error'] = repr(error)
        raise
    finally:
        command[:] = (0., 0.)
        publish()
        cleanup_errors = []
        for name in reversed(entities):
            try:
                call(delete, DeleteEntity.Request(name=name))
            except Exception as error:
                cleanup_errors.append(repr(error))
        results['temporary_entities_removed'] = not cleanup_errors
        if cleanup_errors:
            results['passed'] = False
            results['cleanup_errors'] = cleanup_errors
        if node.executor is not None:
            node.executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2, allow_nan=False) + '\n')
        if cleanup_errors:
            raise RuntimeError(str(cleanup_errors))


if __name__ == '__main__':
    main()
