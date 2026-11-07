#!/usr/bin/env python3
"""Launch every stereo x LiDAR combination headless and check that each starts and stops cleanly.

Requires an isolated ROS_DOMAIN_ID and GAZEBO_MASTER_URI; no simulation may be
running in that domain/master. For each combination it starts
`ros2 launch rover_sim rover_desert.launch.py gui:=false lidar_mode:=2d stereo:=S lidar:=L`,
waits for the actuator bridge, checks which sensor topics exist and carry data,
scans the log for errors, then stops the launch with SIGINT.
"""
import argparse
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState, LaserScan, PointCloud2

COMBINATIONS = [(True, True), (True, False), (False, True), (False, False)]
STEREO_TOPICS = ['/stereo/left/image_raw', '/stereo/right/image_raw', '/stereo/disparity',
                 '/stereo/depth/image_raw', '/stereo/points2']
# Errors that are not about this package's startup (none are expected; kept explicit).
ERROR_PATTERN = re.compile(r'\[ERROR\]|\[Err\]|Traceback|process has died|exception', re.IGNORECASE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--log-dir', type=Path, required=True, help='Where each launch log is written')
    parser.add_argument('--settle', type=float, default=12., help='Wall seconds after the bridge is ready')
    args = parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID') or not os.environ.get('GAZEBO_MASTER_URI'):
        parser.error('Set an isolated ROS_DOMAIN_ID and GAZEBO_MASTER_URI first')
    args.log_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    results = {'combinations': [], 'passed': False}
    try:
        for stereo, lidar in COMBINATIONS:
            name = f'stereo_{"on" if stereo else "off"}_lidar_{"on" if lidar else "off"}'
            log_path = args.log_dir / f'{name}.log'
            with open(log_path, 'w') as log:
                process = subprocess.Popen(
                    ['ros2', 'launch', 'rover_sim', 'rover_desert.launch.py', 'gui:=false',
                     'world:=desert', 'lidar_mode:=2d', f'stereo:={str(stereo).lower()}', f'lidar:={str(lidar).lower()}'],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            entry = {'combination': name, 'stereo': stereo, 'lidar': lidar, 'log': str(log_path)}
            node = Node('launch_matrix_' + name)
            seen = {}
            reliable = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
            node.create_subscription(JointState, '/joint_states', lambda m: seen.__setitem__('/joint_states', 1), 10)
            node.create_subscription(LaserScan, '/scan', lambda m: seen.__setitem__('/scan', 1), qos_profile_sensor_data)
            node.create_subscription(PointCloud2, '/stereo/points2', lambda m: seen.__setitem__('/stereo/points2', 1),
                                     reliable)
            node.create_subscription(Image, '/stereo/depth/image_raw',
                                     lambda m: seen.__setitem__('/stereo/depth/image_raw', 1), reliable)
            stop = threading.Event()

            def spin(node=node, stop=stop):
                while not stop.is_set():
                    rclpy.spin_once(node, timeout_sec=.05)
            spinner = threading.Thread(target=spin, daemon=True)
            spinner.start()
            try:
                deadline = time.monotonic() + 90
                while 'Eight-joint bridge ready' not in log_path.read_text():
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError(f'{name}: launch did not become ready')
                    time.sleep(.2)
                time.sleep(args.settle)
                # Only topics with a publisher count; this node's own subscriptions also create names.
                topics = {t for t, _ in node.get_topic_names_and_types() if node.count_publishers(t) > 0}
                expected_data = ['/joint_states'] + (['/scan'] if lidar else []) + (
                    ['/stereo/points2', '/stereo/depth/image_raw'] if stereo else [])
                entry.update(
                    scan_topic=('/scan' in topics), stereo_topics=sorted(t for t in topics if t.startswith('/stereo/')),
                    data_received=sorted(seen), expected_data=expected_data)
                ok = ('/scan' in topics) == lidar and all(t in topics for t in STEREO_TOPICS) == stereo
                ok = ok and (stereo or not any(t.startswith('/stereo/') for t in topics))
                ok = ok and all(t in seen for t in expected_data)
            finally:
                stop.set()
                spinner.join(timeout=2)
                node.destroy_node()
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=40)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                    entry['forced_kill'] = True
            text = log_path.read_text()
            marker = '[launch]: user interrupted with ctrl-c'
            startup, _, shutdown = text.partition(marker)
            errors = [line[:300] for line in startup.splitlines() if ERROR_PATTERN.search(line)]
            # Every node must also stop cleanly after Ctrl+C (no tracebacks, no non-zero exits).
            shutdown_errors = [line[:300] for line in shutdown.splitlines() if ERROR_PATTERN.search(line)]
            entry.update(shutdown_error_lines=shutdown_errors, saw_ctrl_c=bool(marker in text))
            leftovers = subprocess.run(['pgrep', '-g', str(process.pid)], capture_output=True, text=True).stdout.split()
            entry.update(startup_error_lines=errors, exit_code=process.returncode, leftover_processes=leftovers,
                         passed=ok and not errors and not shutdown_errors and entry['saw_ctrl_c']
                         and process.returncode == 0 and not leftovers and not entry.get('forced_kill'))
            results['combinations'].append(entry)
            print(json.dumps(entry), flush=True)
            time.sleep(3)
        results['passed'] = all(c['passed'] for c in results['combinations'])
    finally:
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2) + '\n')
    if not results['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
