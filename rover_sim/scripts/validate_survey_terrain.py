#!/usr/bin/env python3
"""Check the USGS world in a dedicated Fortress launch; moves the rover.

Requires isolated ROS_DOMAIN_ID / IGN_PARTITION and no teleop/planner.
Ground truth is used solely to validate collision, ground contact and LiDAR.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import UInt64
from tf2_ros import Buffer, TransformListener
import yaml

from rover_sim.lidar_postprocess import CLOUD_QOS, cloud_to_array
from rover_sim import sim_services
from rover_sim.survey_terrain import SurveySurface


def seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def position(odom):
    p = odom.pose.pose.position
    return np.array([p.x, p.y, p.z])


def yaw(odom):
    q = odom.pose.pose.orientation
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--expected-rate', type=float, default=5.)
    args = parser.parse_args()
    sim_services.require_isolation(parser)
    if not math.isfinite(args.expected_rate) or args.expected_rate <= 0:
        parser.error('expected-rate must be positive and finite')
    share = Path(get_package_share_directory('rover_sim'))
    surface = SurveySurface(share/'worlds/usgs.terrain.json')
    geometry = yaml.safe_load((share/'config/real_rover.yaml').read_text())
    results = {'checks': [], 'passed': False}
    odoms, clouds, beats, epochs = deque(maxlen=2000), deque(maxlen=100), deque(maxlen=5000), []
    feedback, errors, command = {}, [], [0., 0., 0.]
    rclpy.init()
    node = Node('survey_terrain_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    node.create_subscription(Odometry, '/ground_truth/odom', lambda m: odoms.append(m), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/points_raw', lambda m: clouds.append((time.monotonic(), m)), CLOUD_QOS)
    node.create_subscription(JointState, '/joint_states', lambda m: feedback.update(
        {n: v for n, v in zip(m.name, m.velocity)}), 10)
    node.create_subscription(JointState, '/swerve/joint_commands', lambda m: beats.append(time.monotonic()), 10)
    node.create_subscription(UInt64, '/swerve/reset_epoch', lambda m: epochs.append(m.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    publisher = node.create_publisher(Twist, '/cmd_vel', 1)
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def publish():
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = command
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

    def wait(predicate, timeout=90.):
        deadline = time.monotonic()+timeout
        while not predicate():
            if errors:
                raise RuntimeError(errors[-1])
            if time.monotonic() > deadline:
                raise RuntimeError('Timed out waiting for survey validation condition')
            time.sleep(.01)

    def record(case, passed, **details):
        entry = dict(case=case, passed=bool(passed), **details)
        results['checks'].append(entry)
        print(json.dumps(entry, allow_nan=False), flush=True)

    def sim_time():
        return node.get_clock().now().nanoseconds/1e9

    def run(duration, velocity=(0., 0., 0.)):
        command[:] = velocity
        start = sim_time()
        wait(lambda: sim_time()-start >= duration, max(60., duration*60))

    def stopped():
        run(1.)
        wait(lambda: max(abs(v) for n, v in feedback.items() if n.endswith('_wheel_joint')) < .08)
        return odoms[-1]

    def world_cloud(msg):
        stamp = seconds(msg.header.stamp)
        wait(lambda: seconds(odoms[-1].header.stamp) >= stamp)
        odom = min(list(odoms), key=lambda m: abs(seconds(m.header.stamp)-stamp))
        t = buffer.lookup_transform('base_link', msg.header.frame_id, Time())
        tr = t.transform.translation
        a = cloud_to_array(msg)
        xyz = np.stack([a[k] for k in ('x', 'y', 'z')], axis=1)
        mask = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 3.)
        xyz = xyz[mask]
        local = xyz@matrix(t.transform.rotation).T+np.array([tr.x, tr.y, tr.z])
        return local@matrix(odom.pose.pose.orientation).T+position(odom)

    def ground_check(case):
        stopped()
        stamp = sim_time()
        wait(lambda: seconds(clouds[-1][1].header.stamp) >= stamp)
        points = world_cloud(clouds[-1][1])
        points = points[::max(1, len(points)//600)]
        residuals = []
        for x, y, z in points:
            try:
                residuals.append(abs(z-surface.height(float(x), float(y))))
            except ValueError:
                continue
        median = float(np.median(residuals)) if residuals else None
        p95 = float(np.percentile(residuals, 95)) if residuals else None
        record(case, len(residuals) > 50 and median < .035 and p95 < .09,
               compared_points=len(residuals), median_vertical_error_m=median, p95_vertical_error_m=p95)

    spawn = node.create_client(SpawnEntity, sim_services.SPAWN)
    delete = node.create_client(DeleteEntity, sim_services.DELETE)
    entity = None

    def call(client, request):
        wait(client.service_is_ready)
        future = client.call_async(request)
        wait(future.done)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(str(response))

    try:
        wait(lambda: len(odoms) > 50 and len(clouds) > 5 and epochs and beats and feedback)
        run(2.)
        start_wall, start_sim, first_epoch = time.monotonic(), sim_time(), epochs[-1]
        run(3.)
        history = [(t, m) for t, m in list(clouds) if t >= start_wall]
        duration = time.monotonic()-start_wall
        rtf = (sim_time()-start_sim)/duration
        stamps = [seconds(m.header.stamp) for _, m in history]
        rate = (len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps) > 1 else 0.
        record('world_spawn_and_sensor_rate', len(stamps) > 2 and abs(rate-args.expected_rate) < .2*args.expected_rate,
               world='usgs', real_time_factor=rtf, raw_publish_rate_sim_hz=rate,
               frame=clouds[-1][1].header.frame_id, source_tiles=len(surface.metadata['inputs']))
        samples = [o for o in list(odoms) if seconds(o.header.stamp) >= start_sim]
        positions = np.array([position(o) for o in samples])
        drift = float(np.max(np.linalg.norm(positions[:, :2]-positions[0, :2], axis=1)))
        record('stable_rest', drift < .03 and float(np.std(positions[:, 2])) < .01,
               xy_drift_m=drift, base_z_std_m=float(np.std(positions[:, 2])))
        odom = odoms[-1]
        R, p = matrix(odom.pose.pose.orientation), position(odom)
        gaps = {}
        for prefix in ['front_left', 'front_right', 'rear_left', 'rear_right']:
            transform = buffer.lookup_transform('base_link', prefix+'_wheel', Time())
            tr = transform.transform.translation
            center = p+R@np.array([tr.x, tr.y, tr.z])
            gaps[prefix] = float(center[2]-geometry['wheel_radius']-surface.height(float(center[0]), float(center[1])))
        record('wheel_ground_contact', all(abs(v) < .035 for v in gaps.values()), wheel_bottom_gap_m=gaps)
        box = geometry['body']['chassis']
        clearances = []
        for sx in (-1, 1):
            for sy in (-1, 1):
                local = np.array(box['xyz'])+np.array([sx*box['size'][0]/2, sy*box['size'][1]/2, -box['size'][2]/2])
                corner = p+R@local
                clearances.append(float(corner[2]-surface.height(float(corner[0]), float(corner[1]))))
        record('chassis_clearance', min(clearances) > .04, minimum_clearance_m=min(clearances))
        ground_check('lidar_matches_survey_mesh')
        entity = 'survey_validation_probe_'+str(os.getpid())
        base = stopped()
        origin = position(base)
        center = origin[:2]+np.array([5.25, 0.])
        z = surface.height(float(center[0]), float(center[1]))+1.
        call(spawn, sim_services.spawn_request(entity, (
            '<sdf version="1.6"><model name="probe"><static>true</static><link name="box">'
            '<collision name="c"><geometry><box><size>.5 2 2</size></box></geometry></collision>'
            '<visual name="v"><geometry><box><size>.5 2 2</size></box></geometry></visual></link></model></sdf>'),
            (float(center[0]), float(center[1]), z)))
        run(.8)
        points = world_cloud(clouds[-1][1])
        on_face = points[(abs(points[:, 0]-(center[0]-.25)) < .12) & (abs(points[:, 1]-center[1]) < .9)
                         & (abs(points[:, 2]-z) < .9)]
        error = float(np.median(abs(on_face[:, 0]-(center[0]-.25)))) if len(on_face) else None
        record('known_distance_probe', len(on_face) >= 5 and error < .04,
               returns=len(on_face), face_distance_from_initial_base_m=5., median_face_error_m=error)
        call(delete, sim_services.delete_request(entity))
        entity = None
        for case, velocity in [('forward', (.2, 0., 0.)), ('backward', (-.2, 0., 0.)),
                               ('yaw_left', (0., 0., .3)), ('yaw_right', (0., 0., -.3)),
                               ('blocked_strafe', (0., .3, 0.))]:
            before = stopped()
            run(2., velocity)
            after = stopped()
            delta = position(after)-position(before)
            angle = yaw(before)
            forward = math.cos(angle)*delta[0]+math.sin(angle)*delta[1]
            lateral = -math.sin(angle)*delta[0]+math.cos(angle)*delta[1]
            turn = math.atan2(math.sin(yaw(after)-angle), math.cos(yaw(after)-angle))
            if velocity[0]:
                ok = forward*velocity[0] > .03 and abs(lateral) < .08 and abs(turn) < .1
            elif velocity[2]:
                ok = turn*velocity[2] > .03 and np.linalg.norm(delta[:2]) < .1
            else:
                ok = np.linalg.norm(delta[:2]) < .03 and abs(turn) < .03
            record(case, ok, forward_m=float(forward), lateral_m=float(lateral), yaw_change_rad=turn)
        ground_check('lidar_after_movement_matches_survey')
        record('watchdog_guard', epochs[-1] == first_epoch, initial_epoch=first_epoch, final_epoch=epochs[-1])
        results['passed'] = all(c['passed'] for c in results['checks'])
    except Exception as error:
        results['error'] = repr(error)
        raise
    finally:
        command[:] = (0., 0., 0.)
        publish()
        if entity:
            call(delete, sim_services.delete_request(entity))
        if node.executor is not None:
            node.executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.try_shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2, allow_nan=False)+'\n')
    if not results['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
