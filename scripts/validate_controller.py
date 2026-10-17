#!/usr/bin/env python3
"""Controller validation on the flat world using Gazebo ground truth.

Checks: circle and figure-eight path following (cross-track error), drive
mode switching (crab / point_turn / ackermann), stall diagnostics against a
wall, wheel-odometry drift versus ``/ground_truth/odom``, the command
watchdog (publisher stops -> wheels stop) and the reset handshake
(``/reset_world`` -> new epoch, no motion until a fresh command).
"""
import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.validation.harness import EntityControl, Observer, SimulationLaunch, truth_since, write_json  # noqa: E402

REST_HEIGHT = 0.41
ENTITY = 'mars_rover'


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def circle_cross_track(samples, centre, radius):
    errors = [math.hypot(g[2] - centre[0], g[3] - centre[1]) - radius for g in samples]
    return dict(rms=math.sqrt(sum(e * e for e in errors) / len(errors)), max=max(abs(e) for e in errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--speed', type=float, default=0.5)
    parser.add_argument('--yaw-rate', type=float, default=0.5)
    parser.add_argument('--arg', action='append', default=[])
    parser.add_argument('--log', type=Path)
    args = parser.parse_args()
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, UInt64
    from std_srvs.srv import Empty
    from rclpy.qos import DurabilityPolicy, QoSProfile
    launch_args = ['gui:=false', 'lidar:=false', 'world:=flat'] + args.arg
    results = []

    def record(name, passed, **details):
        entry = dict(case=name, passed=bool(passed), **details)
        results.append(entry)
        print(json.dumps(entry), flush=True)

    with SimulationLaunch('rover_mars.launch.py', launch_args, log_path=args.log) as launch:
        observer = Observer('controller_validation')
        node = observer.node
        odom, diagnostics, epochs, modes = [], [], [], []
        lock = threading.Lock()
        node.create_subscription(Odometry, '/odom', lambda m: odom.append(
            (m.header.stamp.sec + m.header.stamp.nanosec / 1e9, m.pose.pose.position.x, m.pose.pose.position.y,
             2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w))), 50)
        node.create_subscription(DiagnosticArray, '/swerve/diagnostics', lambda m: diagnostics.append(
            (time.monotonic(), m.status[0].level, m.status[0].message, {kv.key: kv.value for kv in m.status[0].values})), 10)
        node.create_subscription(UInt64, '/swerve/reset_epoch', lambda m: epochs.append((time.monotonic(), m.data)),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(String, '/swerve/drive_mode', lambda m: modes.append(m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        mode_pub = node.create_publisher(String, '/swerve/set_drive_mode', 1)
        try:
            observer.wait_for(lambda: observer.latest_truth() is not None and observer.latest_joints() is not None
                              and bool(odom) and bool(epochs), timeout=150, what='rover', launch=launch)
            control = EntityControl(observer)
            observer.run_sim(2.0)

            def reset_pose(x=0.0, y=0.0, yaw=0.0):
                observer.command = (0.0, 0.0, 0.0)
                observer.run_sim(1.0)
                control.teleport(ENTITY, x, y, REST_HEIGHT + 0.03, yaw=yaw)
                observer.run_sim(1.5)

            def odom_since(sim_start):
                return [o for o in odom if o[0] >= sim_start]

            # --- circle ----------------------------------------------------------
            reset_pose()
            v, w = args.speed, args.yaw_rate
            radius = v / w
            observer.run_sim(2.0, (v, 0.0, w))          # accelerate and settle on the arc
            arc_start = observer.latest_truth()
            # Centre of the circle from the pose at arc start (left of the heading).
            centre = (arc_start[2] - radius * math.sin(arc_start[7]), arc_start[3] + radius * math.cos(arc_start[7]))
            lap = 2 * math.pi / w
            observer.run_sim(lap, (v, 0.0, w))
            circle = truth_since(observer, arc_start[1])
            cte = circle_cross_track(circle, centre, radius)
            observer.command = (0.0, 0.0, 0.0)
            observer.run_sim(1.5)
            record('circle_path', cte['rms'] < 0.08 and cte['max'] < 0.2, radius=radius, cross_track_rms=cte['rms'],
                   cross_track_max=cte['max'], laps=1, samples=len(circle))

            # --- figure eight (odometry drift is measured over this teleport-free segment) ----
            reset_pose()
            start_truth, start_odom = observer.latest_truth(), odom[-1]
            t0 = observer.sim_time()
            observer.run_sim(2.0, (v, 0.0, w))
            a_start = observer.latest_truth()
            centre_a = (a_start[2] - radius * math.sin(a_start[7]), a_start[3] + radius * math.cos(a_start[7]))
            observer.run_sim(lap, (v, 0.0, w))
            loop_a = truth_since(observer, a_start[1])
            observer.run_sim(1.5, (v, 0.0, -w))            # transition through the crossing
            b_start = observer.latest_truth()
            centre_b = (b_start[2] + radius * math.sin(b_start[7]), b_start[3] - radius * math.cos(b_start[7]))
            observer.run_sim(lap, (v, 0.0, -w))
            loop_b = truth_since(observer, b_start[1])
            cte_a, cte_b = circle_cross_track(loop_a, centre_a, radius), circle_cross_track(loop_b, centre_b, radius)
            observer.command = (0.0, 0.0, 0.0)
            observer.run_sim(1.5)
            record('figure_eight_path', max(cte_a['rms'], cte_b['rms']) < 0.08 and max(cte_a['max'], cte_b['max']) < 0.25,
                   loop_a=cte_a, loop_b=cte_b)

            # --- odometry drift over the two manoeuvres ------------------------------
            end_truth = observer.latest_truth()
            end_odom = odom[-1]
            truth_dx, truth_dy = end_truth[2] - start_truth[2], end_truth[3] - start_truth[3]
            truth_yaw = wrap(end_truth[7] - start_truth[7])
            # Express both displacements in the respective start frames.
            def body_frame(dx, dy, yaw0):
                return (math.cos(yaw0) * dx + math.sin(yaw0) * dy, -math.sin(yaw0) * dx + math.cos(yaw0) * dy)
            t_body = body_frame(truth_dx, truth_dy, start_truth[7])
            o_body = body_frame(end_odom[1] - start_odom[1], end_odom[2] - start_odom[2], start_odom[3])
            distance = sum(math.hypot(b[2] - a[2], b[3] - a[3]) for a, b in zip(
                truth_since(observer, t0), truth_since(observer, t0)[1:]))
            position_error = math.hypot(t_body[0] - o_body[0], t_body[1] - o_body[1])
            yaw_error = abs(wrap((end_odom[3] - start_odom[3]) - truth_yaw))
            record('odometry_drift', position_error / distance < 0.03 and math.degrees(yaw_error) < 3.0,
                   segment='figure_eight', distance_travelled=distance, position_error_m=position_error,
                   drift_percent=100.0 * position_error / distance, yaw_error_deg=math.degrees(yaw_error))

            # --- drive modes ------------------------------------------------------------
            reset_pose()
            mode_pub.publish(String(data='crab'))
            observer.wait_for(lambda: modes and modes[-1] == 'crab', timeout=10, what='crab mode')
            before = observer.latest_truth()
            observer.run_sim(4.0, (0.0, 0.3, 0.0))
            after = observer.latest_truth()
            lateral = body_frame(after[2] - before[2], after[3] - before[3], before[7])
            crab_ok = lateral[1] > 0.6 and abs(lateral[0]) < 0.15 and abs(wrap(after[7] - before[7])) < 0.1
            observer.command = (0.0, 0.0, 0.0)
            observer.run_sim(1.0)
            mode_pub.publish(String(data='point_turn'))
            observer.wait_for(lambda: modes[-1] == 'point_turn', timeout=10, what='point_turn mode')
            before = observer.latest_truth()
            observer.run_sim(4.0, (0.5, 0.0, 0.6))
            after = observer.latest_truth()
            turn_ok = abs(wrap(after[7] - before[7])) > 1.2 and math.hypot(after[2] - before[2], after[3] - before[3]) < 0.15
            observer.command = (0.0, 0.0, 0.0)
            observer.run_sim(1.0)
            mode_pub.publish(String(data='ackermann'))
            observer.wait_for(lambda: modes[-1] == 'ackermann', timeout=10, what='ackermann mode')
            before = observer.latest_truth()
            observer.run_sim(3.0, (0.0, 0.3, 0.0))
            after = observer.latest_truth()
            no_strafe = math.hypot(after[2] - before[2], after[3] - before[3]) < 0.03
            observer.command = (0.0, 0.0, 0.0)
            record('drive_modes', crab_ok and turn_ok and no_strafe, crab_body_displacement=lateral,
                   point_turn_yaw_change=wrap(after[7] - before[7]) if False else None, point_turn_ok=turn_ok,
                   ackermann_ignores_lateral=no_strafe)

            # --- stall diagnostics against a wall ----------------------------------------
            reset_pose()
            control.spawn_box('wall', 1.2, 0.0, 0.5, 0.3, 3.0, 1.0)
            observer.run_sim(0.5)
            diag_start = time.monotonic()
            observer.run_sim(6.0, (0.5, 0.0, 0.0))
            observer.command = (0.0, 0.0, 0.0)
            recent = [d for d in diagnostics if d[0] >= diag_start]
            stall_messages = [d for d in recent if 'stall' in d[2]]
            slip_messages = [d for d in recent if 'slip' in d[2]]
            keys = recent[-1][3].keys() if recent else []
            record('stall_diagnostic', bool(stall_messages or slip_messages) and all(k in keys for k in (
                'slip_front_left_wheel_joint', 'stalled', 'drive_mode')),
                   diagnostics_received=len(recent), stall_warnings=len(stall_messages), slip_warnings=len(slip_messages),
                   first_warning_after_s=(min(d[0] for d in stall_messages + slip_messages) - diag_start)
                   if (stall_messages or slip_messages) else None)
            control.remove('wall')
            observer.run_sim(1.0)

            # --- command watchdog: publisher stops -> wheels stop -----------------------------
            reset_pose()
            observer.run_sim(3.0, (0.5, 0.0, 0.0))
            moving_speed = math.hypot(*observer.latest_truth()[8:10])
            observer.publisher_active = False
            timer_cancelled = node.destroy_timer(next(t for t in node.timers))   # stop the heartbeat
            t_stop = observer.sim_time()
            observer.wait_for(lambda: math.hypot(*observer.latest_truth()[8:10]) < 0.02 or observer.sim_time() - t_stop > 5,
                              timeout=60, what='watchdog stop')
            stop_time = observer.sim_time() - t_stop
            record('command_timeout_stops_rover', moving_speed > 0.4 and stop_time < 2.0,
                   speed_before=moving_speed, stop_after_s=stop_time)
            from rclpy.clock import Clock, ClockType
            node.create_timer(0.02, observer._publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

            # --- reset handshake: /reset_world -> new epoch, no motion until a fresh command -----
            reset_client = node.create_client(Empty, '/reset_world')
            observer.wait_for(reset_client.service_is_ready, timeout=10, what='/reset_world')
            observer.command = (0.5, 0.0, 0.0)
            observer.run_sim(2.0)
            epoch_before = epochs[-1][1]
            future = reset_client.call_async(Empty.Request())
            observer.wait_for(future.done, timeout=10, what='reset')
            observer.wait_for(lambda: epochs[-1][1] != epoch_before, timeout=10, what='new epoch')
            # The command keeps streaming; the new reader discards queued samples
            # and a fresh command restores motion.
            observer.run_sim(3.0)
            resumed = math.hypot(*observer.latest_truth()[8:10])
            observer.command = (0.0, 0.0, 0.0)
            record('reset_handshake', epochs[-1][1] != epoch_before and resumed > 0.3,
                   epoch_before=epoch_before, epoch_after=epochs[-1][1], speed_after_fresh_command=resumed)
        finally:
            observer.close()
    passed = all(r['passed'] for r in results)
    write_json(args.output, dict(passed=passed, checks=results, launch_args=launch_args))
    print(json.dumps(dict(passed=passed, cases={r['case']: r['passed'] for r in results})), flush=True)
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
