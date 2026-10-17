#!/usr/bin/env python3
"""Explicit Gazebo Classic motion check; requires the rover launch running.

Uses simulator ground truth and joint feedback for validation, not rover sensors.
Resets only the named proxy rover between cases. Normal cases use forward/yaw
commands only. Explicitly select blocked_lateral_* cases to inject prohibited
lateral input and verify the controller clamp. Saves results in the source tree
only when an output path is explicitly supplied.
"""
import argparse
import json
import re
import math
import subprocess
import time
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

CASES = [('forward', .4, 0, 0), ('backward', -.4, 0, 0),
         ('rotate_left', 0, 0, .5), ('rotate_right', 0, 0, -.5),
         ('forward_yaw', .3, 0, .3), ('zero', 0, 0, 0)]
BLOCKED_LATERAL_CASES = [('blocked_lateral_left', 0, .35, 0),
                         ('blocked_lateral_right', 0, -.35, 0),
                         ('blocked_lateral_forward', .4, .35, 0),
                         ('blocked_lateral_yaw', 0, -.35, .5)]


def pose():
    """Read model pose and its timestamp from the same Gazebo Pose_V packet."""
    stream = subprocess.check_output(
        ['gz', 'topic', '-e', '/gazebo/desert/pose/info', '-d', '1'],
        text=True, timeout=10)
    latest = None
    for packet in re.split(r'\ntime \{', '\n'+stream)[1:]:
        match = re.search(r'pose \{\s*name: "proxy_rover"[^{}]*'
                          r'position \{([^{}]+)\}\s*orientation \{([^{}]+)\}', packet)
        if match is None:
            continue
        def fields(text):
            return {key: float(value) for key, value in
                    re.findall(r'(\w+):\s*([-+0-9.eE]+)', text)}
        stamp = fields(packet.split('}', 1)[0])
        pos, q = fields(match[1]), fields(match[2])
        x, y, z, w = q['x'], q['y'], q['z'], q['w']
        roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
        pitch = math.asin(max(-1, min(1, 2*(w*y-z*x))))
        yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        latest = ([pos['x'], pos['y'], pos['z'], roll, pitch, yaw],
                  stamp.get('sec', 0)+stamp.get('nsec', 0)/1e9)
    if latest is None:
        raise RuntimeError('No proxy_rover pose in Gazebo pose stream')
    return latest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--cases', nargs='+', choices=[c[0] for c in CASES+BLOCKED_LATERAL_CASES],
                        help='Default: normal forward/yaw cases; blocked lateral cases require explicit selection')
    args = parser.parse_args()
    rclpy.init()
    node = Node('swerve_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    state, targets = {}, {}
    node.create_subscription(JointState, '/joint_states',
                             lambda m: state.update({n: [p, v] for n, p, v in zip(m.name, m.position, m.velocity)}), 10)
    node.create_subscription(JointState, '/swerve/joint_commands',
                             lambda m: targets.update({n: [p, v] for n, p, v in zip(m.name, m.position, m.velocity)}), 10)

    command = (0.0, 0.0, 0.0)

    def publish_command():
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = map(float, command)
        pub.publish(msg)

    node.create_timer(.02, publish_command)
    worker = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    worker.start()

    def run(seconds, velocity):
        nonlocal command
        command = velocity
        start = node.get_clock().now().nanoseconds/1e9
        deadline = time.monotonic()+max(20, seconds*10)
        while node.get_clock().now().nanoseconds/1e9-start < seconds:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation clock stalled')
            time.sleep(.005)

    results = []
    try:
        deadline = time.monotonic()+20
        while len(state) < 8:
            if time.monotonic() > deadline:
                raise RuntimeError('Missing joint feedback; start the rover launch first')
            time.sleep(.05)
        cases = CASES+BLOCKED_LATERAL_CASES if args.cases else CASES
        for name, vx, vy, wz in cases:
            if args.cases and name not in args.cases:
                continue
            run(.5, (0., 0., 0.))
            subprocess.run(['gz', 'model', '-m', 'proxy_rover', '-x', '0', '-y', '0',
                            '-z', '0.34', '-R', '0', '-P', '0', '-Y', '0'], check=True, timeout=10)
            run(2.0, (vx, vy, wz))
            a, t0 = pose()
            run(3.0, (vx, vy, wz))
            b, t1 = pose()
            duration = t1-t0
            dx, dy = b[0]-a[0], b[1]-a[1]
            bx, by = math.cos(a[5])*dx+math.sin(a[5])*dy, -math.sin(a[5])*dx+math.cos(a[5])*dy
            yaw = math.atan2(math.sin(b[5]-a[5]), math.cos(b[5]-a[5]))
            # Policy expectation is independent of the incoming lateral field:
            # the normal rover controller always uses body-frame vy=0.
            expected_x = vx*duration if wz == 0 else vx*math.sin(wz*duration)/wz
            expected_y = 0. if wz == 0 else vx*(1-math.cos(wz*duration))/wz
            # Gazebo transport pose sampling has finite latency; compare broad physical tolerances.
            translation_error = math.hypot(bx-expected_x, by-expected_y)
            heading_error = abs(yaw-wz*duration)
            signs_ok = all(abs(value[1]) < .1 or state.get(j, [0, 0])[1]*value[1] > 0
                           for j, value in targets.items() if j.endswith('_wheel_joint'))
            steering_error = max(abs(state[j][0]-t[0]) for j, t in targets.items() if j.endswith('_kingpin'))
            lateral_only = vy != 0 and vx == 0 and wz == 0
            drive_stopped = all(value[1] == 0. and abs(state[j][1]) < .05
                                for j, value in targets.items() if j.endswith('_wheel_joint'))
            result = dict(case=name, command=[vx, vy, wz], effective_command=[vx, 0., wz],
                          measured_duration=duration, start_pose=a, end_pose=b,
                          body_displacement=[bx, by], expected_displacement=[expected_x, expected_y],
                          yaw_change=yaw, translation_error=translation_error, heading_error=heading_error,
                          wheel_signs_correct=signs_ok, max_steering_error=steering_error,
                          joints=dict(state), targets=dict(targets),
                          lateral_only_drive_stopped=drive_stopped if lateral_only else None,
                          passed=translation_error < (.01 if lateral_only else .15)
                          and heading_error < (.01 if lateral_only else .08) and signs_ok
                          and steering_error < .1 and (not lateral_only or drive_stopped))
            results.append(result)
            print(json.dumps({k: v for k, v in result.items() if k not in ('joints', 'targets') }), flush=True)
            run(.5, (0., 0., 0.))
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(results, indent=2)+'\n')
        if not all(r['passed'] for r in results):
            raise SystemExit(1)
    finally:
        command = (0.0, 0.0, 0.0)
        pub.publish(Twist())
        node.executor.shutdown()
        worker.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
