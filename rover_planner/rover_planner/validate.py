"""Check the A* planner against a running simulation.

Run with the sim (and, unless --rates-only, the planner) already up:
  ros2 run rover_planner validate_planner --rates-only --output baseline.json
  ros2 run rover_planner validate_planner --compare baseline.json --output result.json

Checks, using the obstacle boxes listed in rover_sim's desert.world:
  sensors  /stereo/points2, /stereo/left/image_raw and /scan keep their frames and
           their sim-time rates (and, with --compare, match the no-planner baseline)
  grid     every world obstacle in the camera's view has obstacle cells on it, and
           obstacle cells away from known obstacles and hills are rare
  path     for a goal behind the brick block (the straight line to it drives the
           rover into the brick), every path published over 5 s is connected,
           ends at the goal and keeps the rover's real footprint, facing along
           the path, clear of every obstacle
  drive    (--drive, needs the path follower running) the rover reaches the goal,
           and at every /odom pose on the way its real footprint at its real
           heading is clear of every obstacle
Exits 1 if any check fails.
"""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan, PointCloud2

# Rover footprint without the planner's extra clearance (proxy_rover.urdf.xacro).
HALF_LENGTH, HALF_WIDTH = 0.58, 0.51
CAMERA_X, CAMERA_RANGE = 0.55, 6.0
# Bearings the stereo pair can measure: the image spans +-45 deg, but SGBM
# leaves the left num_disparities = 80 of 480 columns without a match, so the
# rover's left side ends at atan(160 / 240) = 33.7 deg (rover_sim stereo.yaml).
CAMERA_BEARINGS = (math.radians(-45.0), math.atan2(240 - 80, 240))
DEPTH_ERROR_K = 0.0087  # planner's stereo depth sigma = k * range^2
SENSORS = {
    '/stereo/points2': (PointCloud2, 'stereo_left_optical_frame', 15.0),
    '/stereo/left/image_raw': (Image, 'stereo_left_optical_frame', 15.0),
    '/scan': (LaserScan, 'lidar_link', 10.0),
}


def load_world():
    """Obstacle boxes (cx, cy, half_x, half_y) and hill spheres (cx, cy, cz, r)."""
    world = os.path.join(get_package_share_directory('rover_sim'), 'worlds', 'desert.world')
    boxes, hills = [], []
    for model in ET.parse(world).getroot().iter('model'):
        name = model.get('name')
        pose = [float(v) for v in model.find('pose').text.split()]
        geometry = model.find('link/collision/geometry')
        if name.startswith('hill_'):
            hills.append((pose[0], pose[1], pose[2], float(geometry.find('sphere/radius').text)))
        elif name.startswith('stereo_feature_'):
            if geometry.find('box') is not None:
                sx, sy, _ = (float(v) for v in geometry.find('box/size').text.split())
                boxes.append((name, pose[0], pose[1], sx / 2, sy / 2))
            else:
                r = float(geometry.find('cylinder/radius').text)
                boxes.append((name, pose[0], pose[1], r, r))
    return boxes, hills


def hill_height(x, y, hills):
    best = 0.0
    for cx, cy, cz, r in hills:
        d2 = (x - cx) ** 2 + (y - cy) ** 2
        if d2 < r * r:
            best = max(best, cz + math.sqrt(r * r - d2))
    return best


def footprint_hits(x, y, yaw, box):
    """Rover rectangle at (x, y, yaw) overlaps an axis-aligned obstacle box (SAT test)."""
    _, bx, by, bhx, bhy = box
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    rover_hx = HALF_LENGTH * c + HALF_WIDTH * s
    rover_hy = HALF_LENGTH * s + HALF_WIDTH * c
    if abs(x - bx) > rover_hx + bhx or abs(y - by) > rover_hy + bhy:
        return False
    # Box corners against the rover's own axes.
    dx, dy = bx - x, by - y
    cc, ss = math.cos(yaw), math.sin(yaw)
    for half, ax, ay in ((HALF_LENGTH, cc, ss), (HALF_WIDTH, -ss, cc)):
        proj = abs(dx * ax + dy * ay)
        box_proj = bhx * abs(ax) + bhy * abs(ay)
        if proj > half + box_proj:
            return False
    return True


class Probe(Node):
    def __init__(self):
        super().__init__('planner_validation', parameter_overrides=[
            Parameter('use_sim_time', value=True)])
        self.stamps = {topic: [] for topic in SENSORS}
        self.frames = {topic: set() for topic in SENSORS}
        for topic, (kind, _, _) in SENSORS.items():
            self.create_subscription(kind, topic, self.keeper(topic), qos_profile_sensor_data)
        self.odom = self.grid = self.path = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(OccupancyGrid, '/planner/grid', lambda m: setattr(self, 'grid', m), 1)
        self.create_subscription(PathMsg, '/planner/path', lambda m: setattr(self, 'path', m), 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def keeper(self, topic):
        def callback(msg):
            self.stamps[topic].append(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)
            self.frames[topic].add(msg.header.frame_id)
        return callback

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until is None


def plot(path, grid, boxes, pts, pose, goal, scale=70):
    """PNG: planner grid, true obstacle outlines from desert.world, A* path.

    Drawn with OpenCV (already a planner dependency). Colours: red obstacle
    cell, orange too close for the rover, blue outlines the real obstacles,
    grey box the rover, dashed grey the straight line, green the A* path.
    """
    import cv2
    rx, ry, yaw = pose
    x0, x1 = rx - 1.5, max(goal[0], rx + 6.0) + 0.5
    y0, y1 = min(goal[1], ry - 4.0) - 0.5, ry + 3.5
    width, height = int((x1 - x0) * scale), int((y1 - y0) * scale)

    def px(x, y):
        return int(round((x - x0) * scale)), int(round((y1 - y) * scale))

    img = np.full((height, width, 3), (230, 239, 244), np.uint8)  # BGR
    for gx in range(int(math.floor(x0)), int(math.ceil(x1)) + 1):  # 1 m grid
        cv2.line(img, px(gx, y0), px(gx, y1), (205, 205, 205), 1)
    for gy in range(int(math.floor(y0)), int(math.ceil(y1)) + 1):
        cv2.line(img, px(x0, gy), px(x1, gy), (205, 205, 205), 1)
    res, w, h = grid.info.resolution, grid.info.width, grid.info.height
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = np.array(grid.data, dtype=np.int16).reshape(h, w)
    for (gy, gx), v in np.ndenumerate(data):
        if v in (50, 100):
            colour = (47, 67, 181) if v == 100 else (141, 196, 242)
            cv2.rectangle(img, px(ox + gx * res, oy + (gy + 1) * res),
                          px(ox + (gx + 1) * res, oy + gy * res), colour, -1)
    for name, bx, by, bhx, bhy in boxes:
        cv2.rectangle(img, px(bx - bhx, by + bhy), px(bx + bhx, by - bhy), (87, 53, 29), 2)
        label = name.replace('stereo_feature_', '')
        cv2.putText(img, label, px(bx - bhx, by + bhy + 0.08), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (87, 53, 29), 1, cv2.LINE_AA)
    c, s = math.cos(yaw), math.sin(yaw)
    corners = np.array([px(rx + dx * c - dy * s, ry + dx * s + dy * c)
                        for dx, dy in ((HALF_LENGTH, HALF_WIDTH), (-HALF_LENGTH, HALF_WIDTH),
                                       (-HALF_LENGTH, -HALF_WIDTH), (HALF_LENGTH, -HALF_WIDTH))])
    cv2.polylines(img, [corners], True, (60, 60, 60), 2)
    for i in range(0, 40, 2):  # dashed straight line
        a = (rx + (goal[0] - rx) * i / 40, ry + (goal[1] - ry) * i / 40)
        b = (rx + (goal[0] - rx) * (i + 1) / 40, ry + (goal[1] - ry) * (i + 1) / 40)
        cv2.line(img, px(*a), px(*b), (130, 130, 130), 1, cv2.LINE_AA)
    cv2.polylines(img, [np.array([px(x, y) for x, y in pts])], False, (92, 158, 26), 3, cv2.LINE_AA)
    cv2.drawMarker(img, px(*goal), (92, 158, 26), cv2.MARKER_STAR, 22, 2)
    cv2.putText(img, 'red: obstacle cell  orange: too close  blue: real obstacle  green: A* path',
                (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
    cv2.putText(img, f'1 m grid, odom frame; rover at ({rx:.1f}, {ry:.1f})', (8, 36),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (40, 40, 40), 1, cv2.LINE_AA)
    cv2.imwrite(str(path), img)


def drive_check(probe, boxes, goal, timeout, record, results):
    """Follow /odom until the rover stops within 0.35 m of the goal."""
    trace, closest = [], {}
    start = time.monotonic()
    last = None
    while time.monotonic() - start < timeout:
        rclpy.spin_once(probe, timeout_sec=0.05)
        odom = probe.odom
        if odom is None or odom is last:
            continue
        last = odom
        p, q = odom.pose.pose.position, odom.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        trace.append((round(p.x, 3), round(p.y, 3), round(yaw, 3)))
        for b in boxes:
            if footprint_hits(p.x, p.y, yaw, b):
                closest[b[0]] = closest.get(b[0], 0) + 1
        speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
        if math.hypot(p.x - goal[0], p.y - goal[1]) < 0.35 and speed < 0.02:
            break
    elapsed = time.monotonic() - start
    x, y, _ = trace[-1] if trace else (math.nan, math.nan, 0)
    error = math.hypot(x - goal[0], y - goal[1])
    travelled = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(trace, trace[1:]))
    record('rover drives to the goal', error < 0.35, final_error_m=round(error, 3),
           wall_s=round(elapsed, 1), travelled_m=round(travelled, 2), poses=len(trace))
    record('rover never touches an obstacle on the way', not closest, overlapping_poses=closest)
    results['drive_trace'] = trace[::10]


def sensor_rates(probe, seconds):
    for topic in SENSORS:
        probe.stamps[topic].clear()
        probe.frames[topic].clear()
    probe.spin_for(seconds)
    rates = {}
    for topic, (_, frame, nominal) in SENSORS.items():
        stamps = sorted(set(probe.stamps[topic]))
        rate = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 2 else 0.0
        rates[topic] = {'rate_hz': round(rate, 2), 'nominal_hz': nominal,
                        'frames': sorted(probe.frames[topic]), 'expected_frame': frame}
    return rates


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rates-only', action='store_true', help='Only measure sensor rates (baseline)')
    parser.add_argument('--compare', type=Path, help='Baseline JSON from --rates-only without the planner')
    parser.add_argument('--duration', type=float, default=10.0, help='Wall seconds for the rate window')
    parser.add_argument('--plot', type=Path, help='Also save a PNG of grid, obstacles and path')
    parser.add_argument('--drive', action='store_true',
                        help='Also check the path follower drives to the goal without touching anything')
    parser.add_argument('--drive-timeout', type=float, default=120.0, help='Wall seconds')
    parser.add_argument('--goal', type=float, nargs=2, default=(5.5, -3.4),
                        help='Goal in odom; the default is behind the brick block')
    args = parser.parse_args()

    rclpy.init()
    probe = Probe()
    results = {}
    failures = []

    def record(name, passed, **details):
        results[name] = {'pass': bool(passed), **details}
        print(f"{'PASS' if passed else 'FAIL'} {name}: {json.dumps(details)[:400]}", flush=True)
        if not passed:
            failures.append(name)

    try:
        rates = sensor_rates(probe, args.duration)
        for topic, r in rates.items():
            ok = (r['frames'] == [r['expected_frame']]
                  and abs(r['rate_hz'] - r['nominal_hz']) <= 0.1 * r['nominal_hz'])
            record(f'sensor {topic}', ok, **r)
        if args.compare:
            baseline = json.loads(args.compare.read_text())['rates']
            for topic, r in rates.items():
                before = baseline[topic]['rate_hz']
                record(f'unchanged {topic}', abs(r['rate_hz'] - before) <= 0.05 * max(before, 1.0),
                       baseline_hz=before, with_planner_hz=r['rate_hz'])
        results['rates'] = rates
        if args.rates_only:
            return

        boxes, hills = load_world()
        if not probe.spin_for(30.0, lambda: probe.odom is not None and probe.grid is not None):
            record('planner running', False, reason='no /odom or /planner/grid within 30 s')
            return
        probe.spin_for(3.0)  # let a few clouds reach the grid
        pose = probe.odom.pose.pose
        rx, ry = pose.position.x, pose.position.y
        q = pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        cam_x, cam_y = rx + CAMERA_X * math.cos(yaw), ry + CAMERA_X * math.sin(yaw)

        # ---- grid against the world ----
        g = probe.grid
        res, w, h = g.info.resolution, g.info.width, g.info.height
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = np.array(g.data, dtype=np.int16).reshape(h, w)
        gy, gx = np.nonzero(data == 100)
        cells = np.column_stack([ox + (gx + 0.5) * res, oy + (gy + 0.5) * res])
        per_obstacle = {}
        for name, bx, by, bhx, bhy in boxes:
            bearing = math.atan2(by - cam_y, bx - cam_x) - yaw
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            visible = (CAMERA_BEARINGS[0] + 0.05 < bearing < CAMERA_BEARINGS[1] - 0.05
                       and math.hypot(bx - cam_x, by - cam_y) < CAMERA_RANGE - 0.3)
            on_it = int(np.sum((np.abs(cells[:, 0] - bx) <= bhx + 0.15)
                               & (np.abs(cells[:, 1] - by) <= bhy + 0.15))) if len(cells) else 0
            per_obstacle[name] = {'visible': visible, 'cells': on_it}
        missed = [n for n, v in per_obstacle.items() if v['visible'] and v['cells'] == 0]
        # Out-of-view obstacles are listed too: their cells show what is remembered.
        record('grid sees every visible obstacle', not missed,
               in_view=sum(v['visible'] for v in per_obstacle.values()),
               obstacles=per_obstacle, missed=missed)

        def explained(x, y):
            # The planner spreads cells +-sigma(range) along the camera ray on
            # purpose; allow that, 2 sigma of stereo noise, and 0.2 m of edges.
            tol = 0.2 + 3 * DEPTH_ERROR_K * math.hypot(x - cam_x, y - cam_y) ** 2
            near_box = any(abs(x - bx) <= bhx + tol and abs(y - by) <= bhy + tol
                           for _, bx, by, bhx, bhy in boxes)
            return near_box or hill_height(x, y, hills) > 0.08
        stray = [(round(float(x), 2), round(float(y), 2)) for x, y in cells if not explained(x, y)]
        share = len(stray) / max(len(cells), 1)
        record('grid has few false obstacles', share <= 0.05, obstacle_cells=len(cells),
               unexplained_cells=len(stray), unexplained_share=round(share, 3), examples=stray[:10])

        # ---- path to a goal behind the crate ----
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = probe.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = args.goal
        goal.pose.orientation.w = 1.0
        probe.path = None
        sent = time.monotonic()
        probe.goal_pub.publish(goal)
        got = probe.spin_for(10.0, lambda: probe.path is not None and len(probe.path.poses) > 0)
        latency = time.monotonic() - sent
        if not got:
            record('path found', False, reason='no non-empty /planner/path within 10 s')
            return
        # Every replan over the next 5 s must be valid, not just the first.
        paths, starts = [], []
        seen = None
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            rclpy.spin_once(probe, timeout_sec=0.05)
            if probe.path is not None and probe.path is not seen:
                seen = probe.path
                at = probe.odom.pose.pose.position
                starts.append((at.x, at.y))  # the rover may be moving (follower running)
                paths.append([(p.pose.position.x, p.pose.position.y,
                               2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                              for p in seen.poses])
        empty = sum(1 for pts in paths if not pts)
        starts = [st for st, pts in zip(starts, paths) if pts]
        paths = [pts for pts in paths if pts]
        worst = {'end_error_m': 0.0, 'start_error_m': 0.0, 'max_step_m': 0.0}
        collisions, lengths = set(), []
        for (sx, sy), pts in zip(starts, paths):
            steps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])] or [0.0]
            worst['end_error_m'] = max(worst['end_error_m'],
                                       math.hypot(pts[-1][0] - args.goal[0], pts[-1][1] - args.goal[1]))
            worst['start_error_m'] = max(worst['start_error_m'], math.hypot(pts[0][0] - sx, pts[0][1] - sy))
            worst['max_step_m'] = max(worst['max_step_m'], max(steps))
            lengths.append(sum(steps))
            collisions |= {b[0] for x, y, heading in pts for b in boxes
                           if footprint_hits(x, y, heading, b)}
        worst = {k: round(v, 3) for k, v in worst.items()}
        record('path found', len(paths) >= 5 and empty == 0 and worst['end_error_m'] <= 0.1
               and worst['start_error_m'] <= 0.1 and worst['max_step_m'] <= 1.5 * res + 1e-6,
               paths_checked=len(paths), empty_paths=empty, first_path_s=round(latency, 2),
               length_m=[round(min(lengths), 2), round(max(lengths), 2)], **worst)
        record('path keeps the rover clear of obstacles', not collisions,
               colliding_with=sorted(collisions))
        straight = [(rx + (args.goal[0] - rx) * i / 100, ry + (args.goal[1] - ry) * i / 100)
                    for i in range(101)]
        in_the_way = sorted({b[0] for x, y in straight for b in boxes if footprint_hits(x, y, yaw, b)})
        record('straight line to the goal is blocked (test is meaningful)', bool(in_the_way),
               straight_line_hits=in_the_way)
        pts = [(x, y) for x, y, _ in paths[-1]]
        if args.drive:
            drive_check(probe, boxes, args.goal, args.drive_timeout, record, results)
        if args.plot:
            plot(args.plot, probe.grid, boxes, pts, (rx, ry, yaw), args.goal)
        results['path'] = pts
    finally:
        results['failures'] = failures
        args.output.write_text(json.dumps(results, indent=2) + '\n')
        probe.destroy_node()
        rclpy.try_shutdown()
        print(f"{'ALL PASS' if not failures else 'FAILED: ' + ', '.join(failures)} -> {args.output}")
        sys.exit(1 if failures else 0)
