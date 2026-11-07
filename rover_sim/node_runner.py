"""Run a ROS node until SIGINT/SIGTERM without tracebacks on repeated signals.

Ctrl+C in a terminal signals the whole process group and `ros2 launch`
forwards another SIGINT, so a second signal can land inside node teardown.
As in rover_sim.controller, the handlers only set a flag; teardown runs once
and is never interrupted. The handlers stay installed until the process exits.
"""
import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def run_until_signal(make_node, args=None, poll=0.05):
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    node = None
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = make_node()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=poll)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.try_shutdown()
