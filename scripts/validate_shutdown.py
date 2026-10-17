#!/usr/bin/env python3
"""Real ROS/process shutdown regression, isolated from Gazebo and user nodes.

Run in an unused ROS_DOMAIN_ID. The child uses the installed controller; only
its destruction boundary is instrumented to inject real repeated signals at
the exact point that previously raised KeyboardInterrupt. Feedback is synthetic.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt64


CHILD = '''
import os
import signal
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
import rover_sim.controller as controller

description = xacro.process_file(str(Path(get_package_share_directory('rover_sim')) /
                                     'urdf/mars_rover.urdf.xacro'), mappings={'enable_lidar': 'false'}).toxml()
class Configured(controller.SwerveController):
    def declare_parameter(self, name, value=None, *args, **kwargs):
        if name == 'robot_description': value = description
        return super().declare_parameter(name, value, *args, **kwargs)
controller.SwerveController = Configured
original_destroy = Node.destroy_node
def destroy(node):
    print('SHUTDOWN_DESTRUCTION_BOUNDARY', flush=True)
    for sig in (signal.SIGINT, signal.SIGINT, signal.SIGTERM):
        os.kill(os.getpid(), sig)
    return original_destroy(node)
Node.destroy_node = destroy
controller.main(args=[])
print('SHUTDOWN_COMPLETE', flush=True)
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not os.environ.get('ROS_DOMAIN_ID'):
        parser.error('Choose an unused ROS_DOMAIN_ID first')
    rclpy.init()
    node = Node('shutdown_validation')
    command_pub = node.create_publisher(Twist, '/cmd_vel', 1)
    feedback_pub = node.create_publisher(JointState, '/joint_states', 10)
    epoch_pub = node.create_publisher(UInt64, '/swerve/reset_epoch',
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    samples = []
    def targets(msg):
        wheels = [abs(v) for name, v in zip(msg.name, msg.velocity) if name.endswith('_wheel_joint')]
        if len(wheels) == 4: samples.append((time.monotonic(), max(wheels)))
    node.create_subscription(JointState, '/swerve/joint_commands', targets, 10)
    prefixes = ['front_left', 'front_right', 'rear_left', 'rear_right']
    feedback = JointState()
    feedback.name = ([p+'_kingpin' for p in prefixes] + [p+'_wheel_joint' for p in prefixes] +
                     [p+'_suspension' for p in prefixes])
    feedback.position = [0.]*12
    feedback.velocity = [0.]*12
    results = []
    child = None

    def pump(predicate, tick=lambda: None, timeout=12):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= deadline: raise RuntimeError('Shutdown validation timed out')
            tick()
            rclpy.spin_once(node, timeout_sec=.02)

    try:
        for initialized in (True, False):
            samples.clear()
            child = subprocess.Popen([sys.executable, '-c', CHILD], stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True,
                                     cwd=get_package_share_directory('rover_sim'))
            epoch_pub.publish(UInt64(data=1 if initialized else 2))
            if initialized:
                def drive():
                    feedback.header.stamp = node.get_clock().now().to_msg()
                    feedback_pub.publish(feedback)
                    command = Twist()
                    command.linear.x = .4
                    command_pub.publish(command)
                pump(lambda: bool(samples) and samples[-1][1] > 1., drive)
            else:
                pump(lambda: feedback_pub.get_subscription_count() > 0)
            signal_at = time.monotonic()
            child.send_signal(signal.SIGINT)
            pump(lambda: child.poll() is not None)
            output = child.communicate(timeout=2)[0]
            # The stop is reliably acknowledged before destruction. Drain any
            # acknowledgement-before-Python-callback scheduling difference.
            if initialized:
                try:
                    pump(lambda: any(t >= signal_at and v == 0. for t, v in samples))
                except RuntimeError:
                    pass  # Preserve the child's exception and missing stop as a failed result.
            final = [v for t, v in samples if t >= signal_at]
            passed = (child.returncode == 0 and output.count('SHUTDOWN_DESTRUCTION_BOUNDARY') == 1
                      and 'SHUTDOWN_COMPLETE' in output and 'Traceback' not in output
                      and 'KeyboardInterrupt' not in output
                      and ((bool(final) and final[-1] == 0.) if initialized else not final))
            result = dict(case='repeated_sigint_moving' if initialized else 'repeated_sigint_before_feedback',
                          passed=passed, exit_code=child.returncode,
                          destruction_calls=output.count('SHUTDOWN_DESTRUCTION_BOUNDARY'),
                          final_drive_target=final[-1] if final else None, child_output=output)
            results.append(result)
            print(json.dumps(result), flush=True)
            if not passed: raise RuntimeError('Repeated-signal regression failed')
            child = None
    finally:
        if child is not None and child.poll() is None:
            child.kill()
            child.wait(timeout=3)
        node.destroy_node()
        rclpy.shutdown()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
