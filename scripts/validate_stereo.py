#!/usr/bin/env python3
"""Validate the stereo camera against a dedicated headless Gazebo Classic launch.

Requires an isolated ROS_DOMAIN_ID and GAZEBO_MASTER_URI and no other /cmd_vel
publisher. Like validate_swerve.py it moves only proxy_rover with `gz model`
(between viewpoints) and uses Gazebo ground truth only for checking; nothing
here is a rover sensor or a TF broadcaster.

Full run (stereo launched): performance windows, interface, left/right sync,
camera_info, TF, depth and point cloud versus analytic ground truth from several
viewpoints, and error versus distance. --performance-only records RTF, LiDAR
rate and a resting LiDAR scan; run it with stereo:=false and pass the result to
a full run with --compare to check that stereo does not degrade the LiDAR.

Pass criteria were fixed before measuring (docs/stereo.md):
  heartbeat gap < 0.15 s, no watchdog epoch change, RTF(stereo on) >= 0.9 x RTF(off),
  LiDAR 9-11 Hz; per distance bin median |error| <= Z^2/(fx*B)*0.5 px + 0.02 m with
  >= 50 % valid pixels on textured surfaces; the usable range is the contiguous run of
  passing bins. Untextured hills and all-surface bins are reported, not judged.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import threading
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock as ClockMessage
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan, PointCloud2
from std_msgs.msg import String, UInt64
from stereo_msgs.msg import DisparityImage
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

from rover_sim.stereo_geometry import LEFT_FRAME, RIGHT_FRAME, baseline_from_urdf, link_transforms
from rover_sim.stereo_truth import depth_tolerance, error_bins, expected_depth, world_primitives
from validate_lidar import json_safe, rotation, stamp_seconds
from validate_swerve import pose

EDGES = [0.4, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 15.0]
MIN_BIN_SAMPLES = 30
REQUIRED_USABLE_RANGE = 4.0     # metres; fixed before measurement
HEARTBEAT_LIMIT = 0.15          # half of the default 0.3 s actuator watchdog
RTF_RATIO_LIMIT = 0.9
VIEWPOINTS = [('spawn', 0.0, 0.0, 0.0), ('forward_2m', 2.0, 0.0, 0.0),
              ('back_2m', -2.0, 0.0, 0.0), ('yaw_left', 0.0, 0.0, 0.35)]
SAMPLE_COLUMNS = 80             # ground-truth samples per image row (every 8th pixel at 640 px)
# Deep queue: this Python reader must not drop frames that the processor did receive.
RELIABLE = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def plain(value):
    """numpy scalars/arrays (e.g. np.True_) to plain Python so json can write them."""
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, np.ndarray):
        return plain(value.tolist())
    if isinstance(value, np.generic):
        return value.item()
    return value


def stamp_key(header):
    return (header.stamp.sec, header.stamp.nanosec)


def sim_rate(stamps):
    stamps = sorted(set(stamps))
    if len(stamps) < 2 or stamps[-1] <= stamps[0]:
        return None
    return (len(stamps) - 1) / (stamps[-1] - stamps[0])


def usable_range(bins):
    """Upper edge of the contiguous run of passing, well-sampled bins starting at the nearest one."""
    upper = None
    for item in bins:
        if item['samples'] < MIN_BIN_SAMPLES:
            if upper is None:
                continue        # nothing visible this close; keep looking outward
            break
        if not item['passed']:
            break
        upper = item['range_m'][1]
    return upper


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--performance-only', action='store_true')
    parser.add_argument('--compare', type=Path, help='Stereo-off --performance-only JSON for the non-interference check')
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
    if args.compare is not None and args.performance_only:
        parser.error('--compare belongs to the full (stereo-on) run')

    rclpy.init()
    node = Node('stereo_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    lock = threading.Lock()
    clocks, heartbeats, epochs = deque(maxlen=200000), deque(maxlen=200000), []
    scans = deque(maxlen=2000)
    stamps = {name: deque(maxlen=5000) for name in (
        'left_image', 'right_image', 'left_info', 'right_info', 'gazebo_right_info',
        'disparity', 'depth', 'points')}
    latest = {}
    feedback, static_tf, dynamic_tf = {}, [], []
    description = []
    command = (0., 0.)
    errors = []
    results = {'performance_only': args.performance_only, 'performance': [], 'checks': []}

    def keep(name, history=True):
        def callback(msg):
            with lock:
                if history and hasattr(msg, 'header'):
                    stamps[name].append((time.monotonic(), stamp_seconds(msg.header.stamp), stamp_key(msg.header)))
                latest[name] = msg
        return callback

    node.create_subscription(ClockMessage, '/clock', lambda m: clocks.append(
        (time.monotonic(), stamp_seconds(m.clock))), qos_profile_sensor_data)
    node.create_subscription(JointState, '/swerve/joint_commands', lambda m: heartbeats.append(time.monotonic()), 10)
    node.create_subscription(JointState, '/joint_states', lambda m: feedback.update(zip(m.name, m.velocity)), 10)
    node.create_subscription(UInt64, '/swerve/reset_epoch', lambda m: epochs.append((time.monotonic(), m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(LaserScan, '/scan', lambda m: scans.append((time.monotonic(), m)), qos_profile_sensor_data)
    node.create_subscription(String, '/robot_description', lambda m: description.append(m.data), LATCHED)
    node.create_subscription(TFMessage, '/tf_static', lambda m: static_tf.extend(m.transforms),
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TFMessage, '/tf', lambda m: dynamic_tf.extend(
        t for t in m.transforms if t.child_frame_id.startswith('stereo_')
        or t.child_frame_id in ('odom', 'map', 'world') or t.header.frame_id in ('odom', 'map', 'world')),
        qos_profile_sensor_data)
    if not args.performance_only:
        for name, kind, topic in (('left_image', Image, '/stereo/left/image_raw'),
                                  ('right_image', Image, '/stereo/right/image_raw'),
                                  ('left_info', CameraInfo, '/stereo/left/camera_info'),
                                  ('right_info', CameraInfo, '/stereo/right/camera_info'),
                                  ('gazebo_right_info', CameraInfo, '/stereo/gazebo/right/camera_info'),
                                  ('disparity', DisparityImage, '/stereo/disparity'),
                                  ('depth', Image, '/stereo/depth/image_raw'),
                                  ('points', PointCloud2, '/stereo/points2')):
            node.create_subscription(kind, topic, keep(name), RELIABLE)
        node.create_subscription(CameraInfo, '/stereo/depth/camera_info', keep('depth_info', False), RELIABLE)
    command_pub = node.create_publisher(Twist, '/cmd_vel', 1)

    def publish():
        msg = Twist()
        msg.linear.x, msg.angular.z = command
        command_pub.publish(msg)

    node.create_timer(.02, publish, clock=Clock(clock_type=ClockType.STEADY_TIME))
    buffer = Buffer()
    TransformListener(buffer, node)

    def spin():
        try:
            rclpy.spin(node)
        except Exception as error:   # surfaced by wait_for
            errors.append(repr(error))

    worker = threading.Thread(target=spin, daemon=True)
    worker.start()

    def sim_now():
        return node.get_clock().now().nanoseconds / 1e9

    def wait_for(predicate, timeout=30., what='condition'):
        deadline = time.monotonic() + timeout
        while not predicate():
            if errors:
                raise RuntimeError('Validation executor failed: ' + errors[-1])
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for ' + what)
            time.sleep(.01)

    def record(name, passed, **details):
        result = plain({'case': name, 'passed': bool(passed), **details})
        results['checks'].append(result)
        print(json.dumps(json_safe(result), allow_nan=False)[:2000], flush=True)
        if not passed:
            raise RuntimeError('Failed ' + name)

    def epoch_values(since):
        previous = [value for stamp, value in epochs if stamp < since]
        return list(dict.fromkeys(([previous[-1]] if previous else []) +
                                  [value for stamp, value in epochs if stamp >= since]))

    def run_sim(seconds):
        start = sim_now()
        wait_for(lambda: sim_now() - start >= seconds, max(30., seconds * 30.), 'simulation time')

    def stopped():
        wait_for(lambda: len(feedback) >= 8 and max(
            abs(v) for n, v in feedback.items() if n.endswith('_wheel_joint')) < .05, 30., 'wheels to stop')

    def resting_scan():
        stopped()
        run_sim(.5)
        measured_pose, at = pose()
        wait_for(lambda: scans and stamp_seconds(scans[-1][1].header.stamp) >= at, 10., 'scan after pose')
        scan = scans[-1][1]
        return {'pose': measured_pose, 'pose_sim_stamp': at, 'ranges': [
            v if math.isfinite(v) else None for v in scan.ranges], 'finite': sum(math.isfinite(v) for v in scan.ranges)}

    try:
        wait_for(lambda: len(clocks) >= 20 and epochs and heartbeats and description, 60., 'simulation and controller')
        urdf = description[-1]
        robot = ET.fromstring(urdf)
        lidar_present = robot.find("link[@name='lidar_link']") is not None
        stereo_present = robot.find("link[@name='stereo_link']") is not None
        results.update(lidar_present=lidar_present, stereo_present=stereo_present)
        if not args.performance_only and not stereo_present:
            raise RuntimeError('Full validation needs a launch with stereo:=true')
        if lidar_present:
            wait_for(lambda: len(scans) >= 10, 30., '/scan')
        sensor = robot.find("gazebo[@reference='stereo_link']/sensor") if stereo_present else None
        camera_rate = float(sensor.findtext('update_rate')) if sensor is not None else None
        if not args.performance_only:
            wait_for(lambda: all(len(stamps[k]) >= 5 for k in ('left_image', 'right_image', 'depth', 'points')),
                     60., 'stereo topics')
        started_all = time.monotonic()

        # 1. Performance windows: RTF, actuator heartbeats, watchdog, sensor rates.
        for window in range(args.windows):
            started = time.monotonic()
            wait_for(lambda: time.monotonic() - started >= args.duration, args.duration + 5., 'window')
            with lock:
                clock_window = [(w, s) for w, s in clocks if w >= started]
                scan_window = [(w, m) for w, m in scans if w >= started]
                stream = {k: [(w, s, key) for w, s, key in v if w >= started] for k, v in stamps.items()}
            beats = [w for w in list(heartbeats) if w >= started]
            if len(clock_window) < 2 or len(beats) < 2:
                raise RuntimeError('Missing clock or actuator samples in performance window')
            rtf = (clock_window[-1][1] - clock_window[0][1]) / (clock_window[-1][0] - clock_window[0][0])
            gap = max(b - a for a, b in zip(beats, beats[1:]))
            lidar_rate = sim_rate([stamp_seconds(m.header.stamp) for _, m in scan_window]) if lidar_present else None
            rates = {k: sim_rate([s for _, s, _ in v]) for k, v in stream.items()} if not args.performance_only else {}
            wall = {k: (len(v) - 1) / (v[-1][0] - v[0][0]) if len(v) > 1 else None for k, v in stream.items()
                    } if not args.performance_only else {}
            values = epoch_values(started)
            measurement = {'window': window + 1, 'real_time_factor': rtf, 'maximum_heartbeat_wall_gap': gap,
                           'lidar_rate_sim_hz': lidar_rate, 'stereo_rates_sim_hz': rates,
                           'stereo_rates_wall_hz': wall, 'epochs': values}
            results['performance'].append(measurement)
            ok = rtf > 0 and len(values) == 1 and gap < HEARTBEAT_LIMIT and gap < args.watchdog
            if lidar_present:
                ok = ok and lidar_rate is not None and 9. <= lidar_rate <= 11.
            if not args.performance_only:
                ok = ok and all(rates.get(k) and abs(rates[k] - camera_rate) <= .1 * camera_rate
                                for k in ('left_image', 'right_image', 'left_info', 'right_info'))
                ok = ok and all(rates.get(k) and rates[k] >= .9 * rates['left_image']
                                for k in ('disparity', 'depth', 'points'))
            record(f'performance_window_{window + 1}', ok, **measurement)
        results['median_real_time_factor'] = statistics.median(m['real_time_factor'] for m in results['performance'])
        if lidar_present:
            results['median_lidar_rate_sim_hz'] = statistics.median(
                m['lidar_rate_sim_hz'] for m in results['performance'])
            results['resting_scan'] = resting_scan()

        if args.performance_only:
            record('no_watchdog_trip', len(epoch_values(started_all)) == 1, epochs=epoch_values(started_all))
            results['passed'] = True
            return

        # 2. Interface: types, single publishers, namespaces, encodings.
        topics = dict(node.get_topic_names_and_types())
        expected_types = {'/stereo/left/image_raw': 'sensor_msgs/msg/Image',
                          '/stereo/right/image_raw': 'sensor_msgs/msg/Image',
                          '/stereo/left/camera_info': 'sensor_msgs/msg/CameraInfo',
                          '/stereo/right/camera_info': 'sensor_msgs/msg/CameraInfo',
                          '/stereo/disparity': 'stereo_msgs/msg/DisparityImage',
                          '/stereo/depth/image_raw': 'sensor_msgs/msg/Image',
                          '/stereo/depth/camera_info': 'sensor_msgs/msg/CameraInfo',
                          '/stereo/points2': 'sensor_msgs/msg/PointCloud2'}
        owners = {topic: [i.node_name for i in node.get_publishers_info_by_topic(topic)] for topic in expected_types}
        expected_owner = {'/stereo/left/image_raw': 'stereo_camera', '/stereo/left/camera_info': 'stereo_camera',
                          '/stereo/right/image_raw': 'stereo_camera_relay',
                          '/stereo/right/camera_info': 'stereo_camera_relay'}
        stereo_nodes = ('stereo_camera_relay', 'stereo_processor')
        published = {n: sorted(t for t, _ in node.get_publisher_names_and_types_by_node(n, '/')) for n in stereo_nodes}
        outside = {n: [t for t in topics_ if not t.startswith('/stereo/') and t not in ('/rosout', '/parameter_events')]
                   for n, topics_ in published.items()}
        with lock:
            left, depth, disparity, cloud = (latest['left_image'], latest['depth'], latest['disparity'],
                                             latest['points'])
        record('stereo_interface',
               all(topics.get(t) == [v] for t, v in expected_types.items())
               and all(len(o) == 1 for o in owners.values())
               and all(owners[t] == [n] for t, n in expected_owner.items())
               and all(owners[t] == ['stereo_processor'] for t in expected_types if t not in expected_owner)
               and not any(outside.values())
               and left.encoding == 'rgb8' and depth.encoding == '32FC1' and disparity.image.encoding == '32FC1'
               and (left.width, left.height) == (depth.width, depth.height)
               and [f.name for f in cloud.fields] == ['x', 'y', 'z', 'rgb'],
               topic_types={t: topics.get(t) for t in expected_types}, publishers=owners,
               stereo_node_publications=published, non_stereo_publications=outside,
               image_size=[left.width, left.height], encodings=[left.encoding, depth.encoding])

        # 3. Left/right capture-time synchronisation.
        with lock:
            keys = {name: {key for _, _, key in stamps[name]}
                    for name in ('left_image', 'right_image', 'left_info', 'right_info', 'depth')}
        # Only the span where every stream is being received: the validator's own
        # discovery and reader queue must not count as a system fault.
        start = max(min(v) for v in keys.values())
        end = min(max(v) for v in keys.values())
        keys = {name: {k for k in v if start <= k <= end} for name, v in keys.items()}
        left_keys, right_keys = keys['left_image'], keys['right_image']
        captures = keys['left_image'] | keys['right_image'] | keys['left_info'] | keys['right_info']
        common = left_keys & right_keys
        orphans = sorted(keys['depth'] - captures)
        record('left_right_timestamp_sync', len(common) >= .95 * len(left_keys | right_keys) and not orphans,
               span_sim_seconds=[start, end], left_frames=len(left_keys), right_frames=len(right_keys),
               identical_stamps=len(common), paired_fraction=len(common) / len(left_keys | right_keys),
               depth_frames=len(keys['depth']), depth_stamps_without_capture=orphans)

        # 4. camera_info consistency with the URDF.
        baseline = baseline_from_urdf(urdf)
        camera = sensor.find('camera')
        width, height = int(camera.findtext('image/width')), int(camera.findtext('image/height'))
        hfov = float(camera.findtext('horizontal_fov'))
        with lock:
            li, ri, gi, di = latest['left_info'], latest['right_info'], latest['gazebo_right_info'], latest['depth_info']
        fx_expected = width / (2 * math.tan(hfov / 2))
        conditions = {
            'left_frame': li.header.frame_id == LEFT_FRAME,
            'right_frame': ri.header.frame_id == RIGHT_FRAME,
            'depth_info_frame': di.header.frame_id == LEFT_FRAME,
            'sizes_match_urdf': (li.width, li.height) == (ri.width, ri.height) == (width, height),
            'fx_matches_hfov': math.isclose(li.p[0], fx_expected, rel_tol=1e-4),
            'shared_fx_fy': li.p[0] == ri.p[0] and li.p[5] == ri.p[5],
            'centred_principal_point': li.p[2] == ri.p[2] == (width - 1) / 2 and li.p[6] == ri.p[6] == (height - 1) / 2,
            'left_tx_zero': li.p[3] == 0,
            'right_tx_minus_fx_baseline': math.isclose(ri.p[3], -ri.p[0] * baseline, rel_tol=1e-9),
            'no_distortion': all(v == 0 for v in list(li.d) + list(ri.d)),
            'plugin_right_tx_was_zero': list(gi.p)[3] == 0,
            # DisparityImage.f and .t are float32 fields, so compare at float32 precision.
            'disparity_f_t': math.isclose(disparity.t, baseline, rel_tol=1e-6)
            and math.isclose(disparity.f, li.p[0], rel_tol=1e-6)}
        record('camera_info_consistency', all(conditions.values()), conditions=conditions,
               left_p=list(li.p), right_p=list(ri.p), expected_fx=fx_expected, urdf_baseline=baseline,
               camera_info_baseline=-ri.p[3] / ri.p[0], disparity_f_t=[disparity.f, disparity.t])

        # 5. TF: RSP owns fixed stereo frames; no world frames.
        chain = link_transforms(urdf)
        lookups = {}
        for frame in (LEFT_FRAME, RIGHT_FRAME, 'stereo_link'):
            transform = buffer.lookup_transform('base_link', frame, Time.from_msg(left.header.stamp))
            t = transform.transform.translation
            lookups[frame] = [t.x, t.y, t.z]
        tf_publishers = [i.node_name for i in node.get_publishers_info_by_topic('/tf_static')]
        stereo_static = [t for t in static_tf if t.child_frame_id.startswith('stereo_')]
        frames = {t.child_frame_id for t in static_tf} | {t.header.frame_id for t in static_tf}
        record('tf_resolution_and_ownership',
               all(np.allclose(lookups[f], chain[f][0], atol=1e-9) for f in lookups)
               and tf_publishers == ['robot_state_publisher'] and len(stereo_static) >= 3
               and not dynamic_tf and not frames & {'odom', 'map', 'world'},
               base_link_to_frame=lookups, urdf_translation={f: chain[f][0] for f in lookups},
               tf_static_publishers=tf_publishers, competing_dynamic_or_world_transforms=len(dynamic_tf))

        # 6. Depth and point cloud versus analytic ground truth.
        primitives = world_primitives(Path(get_package_share_directory('rover_sim')) / 'worlds' / 'desert.world')
        offset, optical = chain[LEFT_FRAME]
        intrinsics = dict(fx=li.p[0], fy=li.p[5], cx=li.p[2], cy=li.p[6])
        samples, per_model, sky, cloud_checks, views = [], {}, [0, 0], [], []
        for label, x, y, yaw in VIEWPOINTS:
            subprocess.run(['gz', 'model', '-m', 'proxy_rover', '-x', str(x), '-y', str(y), '-z', '0.34',
                            '-R', '0', '-P', '0', '-Y', str(yaw)], check=True, timeout=10)
            run_sim(1.5)
            stopped()
            measured_pose, at = pose()
            wait_for(lambda: stamps['depth'] and stamps['depth'][-1][1] >= at + .2, 20., 'fresh depth')
            with lock:
                depth_msg = latest['depth']
                cloud_msg = latest['points'] if stamp_key(latest['points'].header) == stamp_key(depth_msg.header) \
                    else None
            depth_image = np.frombuffer(bytes(depth_msg.data), '<f4').reshape(depth_msg.height, depth_msg.width)
            world_rotation = rotation(*measured_pose[3:])
            shift = [sum(world_rotation[i][k] * offset[k] for k in range(3)) for i in range(3)]
            origin = tuple(measured_pose[i] + shift[i] for i in range(3))
            camera_rotation = tuple(tuple(sum(world_rotation[i][k] * optical[k][j] for k in range(3))
                                          for j in range(3)) for i in range(3))
            view = []
            step = max(2, round(depth_msg.width / SAMPLE_COLUMNS))   # same density at any resolution
            for v in range(step // 2, depth_msg.height, step):
                for u in range(step // 2, depth_msg.width, step):
                    truth, model = expected_depth(u, v, intrinsics, origin, camera_rotation, primitives)
                    measured = float(depth_image[v, u])
                    measured = measured if math.isfinite(measured) else None
                    if model is None or not math.isfinite(truth):
                        sky[0] += 1
                        sky[1] += measured is not None
                        continue
                    view.append((truth, measured, model))
                    per_model.setdefault(model, []).append((truth, measured))
            samples += view
            # Cloud: every point must reproduce the depth image, and ground points must lie on z = 0.
            if cloud_msg is not None:
                xyz = np.frombuffer(bytes(cloud_msg.data), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                                                  ('rgb', '<u4')])
                pts = np.stack([xyz['x'], xyz['y'], xyz['z']], 1).astype(np.float64)
                u = intrinsics['fx'] * pts[:, 0] / pts[:, 2] + intrinsics['cx']
                v = intrinsics['fy'] * pts[:, 1] / pts[:, 2] + intrinsics['cy']
                ui, vi = np.rint(u).astype(int), np.rint(v).astype(int)
                reproject = float(np.max(np.abs(depth_image[vi, ui] - pts[:, 2]))) if len(pts) else None
                pixel_error = float(max(np.max(np.abs(u - ui)), np.max(np.abs(v - vi)))) if len(pts) else None
                world = pts @ np.array(camera_rotation).T + np.array(origin)
                ground = []
                for index in range(0, len(pts), max(1, len(pts) // 1500)):
                    truth, model = expected_depth(u[index], v[index], intrinsics, origin, camera_rotation, primitives)
                    if model == 'sand_ground' and truth < 4.0:
                        ground.append(abs(world[index, 2]))
                cloud_checks.append({'viewpoint': label, 'points': len(pts), 'max_depth_mismatch_m': reproject,
                                     'max_pixel_rounding': pixel_error, 'ground_points_checked': len(ground),
                                     'median_ground_height_m': float(np.median(ground)) if ground else None})
            views.append({'viewpoint': label, 'pose': measured_pose, 'pose_sim_stamp': at,
                          'depth_sim_stamp': stamp_seconds(depth_msg.header.stamp), 'sample_step_px': step,
                          'cloud_matched': cloud_msg is not None, 'bins': error_bins(
                              [(t, m) for t, m, name in view if not name.startswith('hill')],
                              EDGES, intrinsics['fx'], baseline)})
        # As planned: coverage and accuracy are judged on textured surfaces (sand and
        # stereo features). The plain-coloured hills, kept unchanged as requested, are
        # reported separately; all-surface bins are recorded for comparison.
        textured = [(t, m) for t, m, name in samples if not name.startswith('hill')]
        bins = error_bins(textured, EDGES, intrinsics['fx'], baseline)
        bins_all = error_bins([(t, m) for t, m, _ in samples], EDGES, intrinsics['fx'], baseline)
        upper = usable_range(bins)
        tolerances = [depth_tolerance(t, intrinsics['fx'], baseline) for t, _ in textured]
        in_range = [(t, m, tol) for (t, m), tol in zip(textured, tolerances) if upper and t < upper and m is not None]
        gross = sum(abs(m - t) > 3 * tol for t, m, tol in in_range) / len(in_range) if in_range else 1.0
        model_summary = {}
        for model, items in per_model.items():
            errs = sorted(abs(m - t) for t, m in items if m is not None)
            model_summary[model] = {'samples': len(items), 'valid_fraction': len(errs) / len(items),
                                    'median_abs_error_m': errs[len(errs) // 2] if errs else None,
                                    'truth_range_m': [min(t for t, _ in items), max(t for t, _ in items)]}
        hills = [(t, m) for t, m, name in samples if name.startswith('hill')]
        hill_errors = sorted(abs(m - t) for t, m in hills if m is not None)
        untextured = {'samples': len(hills), 'valid_fraction': len(hill_errors) / len(hills) if hills else None,
                      'median_abs_error_m': hill_errors[len(hill_errors) // 2] if hill_errors else None}
        results.update(error_vs_distance=bins, error_vs_distance_all_surfaces=bins_all,
                       usable_range_all_surfaces_m=usable_range(bins_all), untextured_hills=untextured,
                       usable_range_m=upper, per_object=model_summary, viewpoints=views)
        record('depth_vs_ground_truth', upper is not None and upper >= REQUIRED_USABLE_RANGE
               and gross < .05 and sky[1] <= .005 * max(1, sky[0]),
               usable_range_m=upper, required_usable_range_m=REQUIRED_USABLE_RANGE,
               gross_outlier_fraction_in_usable_range=gross, sky_pixels=sky[0], false_depth_on_sky=sky[1],
               error_vs_distance=bins, usable_range_all_surfaces_m=usable_range(bins_all),
               untextured_hills=untextured, per_object=model_summary)
        record('point_cloud_consistency', len(cloud_checks) == len(VIEWPOINTS) and all(
            c['points'] > 1000 and c['max_depth_mismatch_m'] < 1e-4 and c['max_pixel_rounding'] < 1e-2
            and c['ground_points_checked'] >= 50 and c['median_ground_height_m'] < .03 for c in cloud_checks),
            clouds=cloud_checks)

        subprocess.run(['gz', 'model', '-m', 'proxy_rover', '-x', '0', '-y', '0', '-z', '0.34',
                        '-R', '0', '-P', '0', '-Y', '0'], check=True, timeout=10)
        run_sim(1.5)
        record('no_watchdog_trip_during_stereo_validation', len(epoch_values(started_all)) == 1,
               epochs=epoch_values(started_all))

        # 7. Stereo must not degrade the LiDAR (needs the stereo-off baseline).
        if args.compare is not None:
            baseline_run = json.loads(args.compare.read_text())
            if baseline_run.get('stereo_present') or not baseline_run.get('performance_only'):
                raise RuntimeError('--compare must be a stereo:=false --performance-only result')
            ratio = results['median_real_time_factor'] / baseline_run['median_real_time_factor']
            details = {'rtf_stereo_off': baseline_run['median_real_time_factor'],
                       'rtf_stereo_on': results['median_real_time_factor'], 'rtf_ratio': ratio}
            ok = ratio >= RTF_RATIO_LIMIT
            if lidar_present:
                off, on = baseline_run['resting_scan'], results['resting_scan']
                pairs = [(a, b) for a, b in zip(off['ranges'], on['ranges']) if a is not None and b is not None]
                diffs = sorted(abs(a - b) for a, b in pairs)
                details.update(lidar_rate_off=baseline_run['median_lidar_rate_sim_hz'],
                               lidar_rate_on=results['median_lidar_rate_sim_hz'],
                               finite_returns_off=off['finite'], finite_returns_on=on['finite'],
                               pose_difference_m=math.dist(off['pose'][:2], on['pose'][:2]),
                               median_beam_difference_m=diffs[len(diffs) // 2] if diffs else None,
                               p99_beam_difference_m=diffs[int(.99 * (len(diffs) - 1))] if diffs else None)
                ok = ok and 9 <= details['lidar_rate_off'] <= 11 and 9 <= details['lidar_rate_on'] <= 11 \
                    and abs(on['finite'] - off['finite']) <= max(3, .02 * off['finite']) \
                    and details['median_beam_difference_m'] is not None \
                    and details['median_beam_difference_m'] < .02 and details['p99_beam_difference_m'] < .06
            record('stereo_does_not_degrade_lidar_or_rtf', ok, **details)
        results['passed'] = True
    except Exception as error:
        results['passed'] = False
        results['error'] = repr(error)
        raise
    finally:
        command = (0., 0.)
        publish()
        if node.executor is not None:
            node.executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(json_safe(plain(results)), indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
