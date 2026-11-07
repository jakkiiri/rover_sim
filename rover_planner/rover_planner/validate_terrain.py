"""Check the slope-aware planner on rover_sim's Mars world.

Run with the Mars sim and the planner (with follow:=true for --drive) up:
  ros2 run rover_planner validate_terrain --goal 11.3 2.6 --drive --output hill.json --plot hill.png

Truth comes from the generator's files in rover_sim (worlds/mars_layout.json,
models/mars_terrain/heightmap.npy): the exact terrain heights and every rock
and shrub with its size.

Checks:
  slope grid  of the cells the planner marks too steep, few are gentle in truth
              (truth slope < limit - 5 deg; cells on rocks and boulders are left
              out); of the measured cells that are steeper than limit + 5 deg in
              truth, most are not drivable (too steep, obstacle, or too close to one)
  obstacles   every rock or shrub the rover cannot drive over (top above the
              step limit) that is in view has obstacle cells near it
  paths       every path published over 5 s stays on measured ground no steeper
              than limit + 3 deg (truth, at rover scale) and keeps the rover's body,
              facing along the path, clear of every such rock and shrub in the
              camera's view (paths may cross unseen ground; they are replanned)
  drive       (--drive) the rover reaches the goal, never touches a rock or
              shrub it cannot drive over, and never tilts more than limit + 5 deg
Exits 1 if any check fails.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from .terrain import fit_planes
from .validate import CAMERA_BEARINGS, CAMERA_RANGE, CAMERA_X, HALF_LENGTH, HALF_WIDTH

TOO_CLOSE, TOO_STEEP, OBSTACLE = 50, 80, 100


class Truth:
    """Mars world terrain heights, rover-scale slope and the rocks and shrubs."""

    def __init__(self, step_height, slope_window=1.0):
        share = Path(get_package_share_directory('rover_sim'))
        self.layout = json.loads((share / 'worlds' / 'mars_layout.json').read_text())
        t = self.layout['terrain']
        self.h = np.load(share / 'models' / 'mars_terrain' / 'heightmap.npy').astype(np.float64)
        self.spacing, self.n = t['spacing_m'], t['samples']
        cells = max(3, int(round(slope_window / self.spacing)) | 1)
        _, a, b, ok = fit_planes(self.h, np.ones_like(self.h, bool), cells, 0.3)
        self.slope = np.degrees(np.arctan(np.hypot(a, b) / self.spacing))
        self.objects = self.layout['objects']
        self.blocking = [o for o in self.objects if o['top_above_ground'] > step_height]

    def index(self, x, y):
        c = np.clip(np.round(np.asarray(x) / self.spacing + (self.n - 1) / 2).astype(int), 0, self.n - 1)
        r = np.clip(np.round(np.asarray(y) / self.spacing + (self.n - 1) / 2).astype(int), 0, self.n - 1)
        return r, c

    def slope_at(self, x, y):
        return self.slope[self.index(x, y)]

    def height_at(self, x, y):
        return self.h[self.index(x, y)]


def touches(x, y, yaw, obj, margin=0.0):
    """Rover rectangle at (x, y, yaw) overlaps a round object (closest-point test)."""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = obj['x'] - x, obj['y'] - y
    lx, ly = dx * c + dy * s, -dx * s + dy * c
    qx = min(max(lx, -HALF_LENGTH), HALF_LENGTH)
    qy = min(max(ly, -HALF_WIDTH), HALF_WIDTH)
    return math.hypot(lx - qx, ly - qy) < obj['radius'] + margin


def in_view(pose, x, y):
    rx, ry, yaw = pose
    cx, cy = rx + CAMERA_X * math.cos(yaw), ry + CAMERA_X * math.sin(yaw)
    bearing = math.atan2(math.sin(math.atan2(y - cy, x - cx) - yaw), math.cos(math.atan2(y - cy, x - cx) - yaw))
    return math.hypot(x - cx, y - cy) < CAMERA_RANGE - 0.5 and CAMERA_BEARINGS[0] < bearing < CAMERA_BEARINGS[1]


class Probe(Node):
    def __init__(self):
        super().__init__('terrain_validation', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.odom = self.grid = self.path = None
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(OccupancyGrid, '/planner/grid', lambda m: setattr(self, 'grid', m), 1)
        self.create_subscription(PathMsg, '/planner/path', lambda m: setattr(self, 'path', m), 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def spin_for(self, seconds, until=None):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if until is not None and until():
                return True
        return until is None


def pose_of(odom):
    p, q = odom.pose.pose.position, odom.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))
    return (p.x, p.y, yaw), tilt


def grid_cells(grid):
    res, w, h = grid.info.resolution, grid.info.width, grid.info.height
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    data = np.array(grid.data, np.int16).reshape(h, w)
    gy, gx = np.mgrid[0:h, 0:w]
    return data, ox + (gx + 0.5) * res, oy + (gy + 0.5) * res


def check_grid(truth, grid, pose, limit, record):
    data, x, y = grid_cells(grid)
    true_slope = truth.slope_at(x, y)
    steep = data == TOO_STEEP
    known = data >= 0
    near = np.hypot(x - pose[0], y - pose[1]) < 5.0
    # Cells on a rock or boulder are not bare terrain: its sides may rightly read as steep.
    on_object = np.zeros_like(steep)
    for o in truth.blocking:
        on_object |= np.hypot(x - o['x'], y - o['y']) < o['radius'] + 0.1
    false_steep = steep & ~on_object & (true_slope < limit - 5)
    really_steep = known & near & (true_slope > limit + 5)
    marked = really_steep & ((data == TOO_STEEP) | (data == OBSTACLE))
    # Safety is what matters: the rover's centre may not go there. Cells high on a
    # wall are often only "too close" (next to marked ones), which also blocks them.
    caught = really_steep & ((data == TOO_STEEP) | (data == OBSTACLE) | (data == TOO_CLOSE))
    false_share = float(false_steep.sum()) / max(int(steep.sum()), 1)
    caught_share = float(caught.sum()) / max(int(really_steep.sum()), 1)
    record('few gentle cells are marked too steep', false_share <= 0.10,
           too_steep_cells=int(steep.sum()), gentle_in_truth=int(false_steep.sum()), share=round(false_share, 3))
    record('measured steep ground is not drivable', really_steep.sum() == 0 or caught_share >= 0.8,
           steep_in_truth_within_5m=int(really_steep.sum()), blocked=int(caught.sum()),
           share=round(caught_share, 3), marked_too_steep=int(marked.sum()),
           marked_share=round(float(marked.sum()) / max(int(really_steep.sum()), 1), 3))
    seen = []
    missed = []
    for o in truth.blocking:
        if not in_view(pose, o['x'], o['y']):
            continue
        d = np.hypot(x - o['x'], y - o['y'])
        hit = bool(((data == OBSTACLE) & (d < o['radius'] + 0.3)).any())
        (seen if hit else missed).append(f"{o['kind']} at ({o['x']:.1f}, {o['y']:.1f}), "
                                        f"{o['top_above_ground']:.2f} m tall")
    record('rocks and shrubs in view are in the grid', not missed, in_view=len(seen) + len(missed),
           found=len(seen), missed=missed)


def measured_mask(grid, pts):
    """True for each (x, y) on a grid cell whose ground the planner has measured."""
    out = np.zeros(len(pts), bool)
    if grid is None:
        return out
    data, res = np.array(grid.data, np.int16).reshape(grid.info.height, grid.info.width), grid.info.resolution
    for k, (x, y) in enumerate(pts):
        c = int(math.floor((x - grid.info.origin.position.x) / res))
        r = int(math.floor((y - grid.info.origin.position.y) / res))
        out[k] = 0 <= r < data.shape[0] and 0 <= c < data.shape[1] and data[r, c] >= 0
    return out


def check_paths(probe, truth, limit, seconds, record):
    paths, bad_slope, bad_touch, unseen, steep_unmeasured = 0, [], [], set(), []
    worst = 0.0
    end = time.monotonic() + seconds
    last = None
    while time.monotonic() < end:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if probe.path is None or probe.path is last or not probe.path.poses:
            continue
        last = probe.path
        paths += 1
        pts = [(p.pose.position.x, p.pose.position.y) for p in last.poses]
        slopes = truth.slope_at(np.array([p[0] for p in pts]), np.array([p[1] for p in pts]))
        # Only ground the planner has measured counts; paths over unmeasured ground
        # are guesses that get replanned once the camera sees it.
        measured = measured_mask(probe.grid, pts)
        if measured.any():
            worst = max(worst, float(slopes[measured].max()))
        for i in np.nonzero(slopes > limit + 3)[0]:
            text = f'{slopes[i]:.1f} deg at ({pts[i][0]:.1f}, {pts[i][1]:.1f})'
            (bad_slope if measured[i] else steep_unmeasured).append(text)
        pose = pose_of(probe.odom)[0] if probe.odom else None
        for a, b in zip(pts, pts[1:]):
            yaw = math.atan2(b[1] - a[1], b[0] - a[0])
            for o in truth.blocking:
                if touches(b[0], b[1], yaw, o):
                    name = f"{o['kind']} at ({o['x']:.1f}, {o['y']:.1f})"
                    if pose is not None and in_view(pose, o['x'], o['y']):
                        bad_touch.append(name)
                    else:
                        unseen.add(name)
    record('paths published', paths > 0, paths=paths)
    record('paths avoid ground steeper than the limit', not bad_slope, steepest_on_paths_deg=round(worst, 1),
           too_steep=sorted(set(bad_slope))[:10], steep_but_unmeasured=sorted(set(steep_unmeasured))[:10])
    record('paths keep the rover clear of rocks and shrubs in view', not bad_touch,
           touching=sorted(set(bad_touch))[:10], crossing_unseen=sorted(unseen)[:10])
    return last


def drive(probe, truth, goal, limit, timeout, record, results):
    trace, touched, over_limit = [], {}, 0
    worst_tilt = 0.0
    start, last = time.monotonic(), None
    while time.monotonic() - start < timeout:
        rclpy.spin_once(probe, timeout_sec=0.05)
        odom = probe.odom
        if odom is None or odom is last:
            continue
        last = odom
        pose, tilt = pose_of(odom)
        trace.append((round(pose[0], 3), round(pose[1], 3), round(pose[2], 3), round(tilt, 1)))
        worst_tilt = max(worst_tilt, tilt)
        over_limit += tilt > limit + 5
        for o in truth.blocking:
            if touches(*pose, o):
                key = f"{o['kind']} at ({o['x']:.1f}, {o['y']:.1f})"
                touched[key] = touched.get(key, 0) + 1
        speed = math.hypot(odom.twist.twist.linear.x, odom.twist.twist.linear.y)
        if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) < 0.35 and speed < 0.05:
            break
    x, y = (trace[-1][0], trace[-1][1]) if trace else (math.nan, math.nan)
    error = math.hypot(x - goal[0], y - goal[1])
    travelled = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(trace, trace[1:]))
    climb = (float(truth.height_at(*goal)) - float(truth.height_at(trace[0][0], trace[0][1]))) if trace else 0.0
    record('rover drives to the goal', error < 0.35, final_error_m=round(error, 3),
           wall_s=round(time.monotonic() - start, 1), travelled_m=round(travelled, 2),
           height_change_m=round(climb, 2))
    record('rover never touches a rock or shrub it cannot drive over', not touched, touching_poses=touched)
    record('rover never tilts beyond the limit + 5 deg', over_limit == 0, max_tilt_deg=round(worst_tilt, 1),
           poses_over=over_limit)
    results['drive_trace'] = trace[::10]
    return trace


def plot(path_png, truth, grid, path, trace, goal, start_pose, limit, scale=28):
    """Top view: shaded terrain, planner grid, rocks and shrubs, path and drive."""
    import cv2
    xs = [start_pose[0], goal[0]] + [t[0] for t in trace]
    ys = [start_pose[1], goal[1]] + [t[1] for t in trace]
    x0, x1 = min(xs) - 5, max(xs) + 5
    y0, y1 = min(ys) - 5, max(ys) + 5
    w, h = int((x1 - x0) * scale), int((y1 - y0) * scale)
    gx, gy = np.meshgrid(x0 + (np.arange(w) + 0.5) / scale, y1 - (np.arange(h) + 0.5) / scale)
    s = truth.slope_at(gx, gy)
    shade = np.clip(1.0 - s / 60.0, 0.25, 1.0)
    img = np.stack([0.55 * shade, 0.65 * shade, 0.85 * shade], -1)            # BGR terrain
    img[s > limit] = img[s > limit] * 0.5 + np.array([0.55, 0.25, 0.55]) * 0.5  # truth too steep
    img = (img * 255).astype(np.uint8)

    def px(x, y):
        return int(round((x - x0) * scale)), int(round((y1 - y) * scale))

    if grid is not None:
        data, cx, cy = grid_cells(grid)
        colours = {OBSTACLE: (40, 50, 200), TOO_STEEP: (200, 60, 160), 50: (90, 170, 245)}
        res = grid.info.resolution
        for value, colour in colours.items():
            for x, y in zip(cx[data == value], cy[data == value]):
                cv2.rectangle(img, px(x - res / 2, y + res / 2), px(x + res / 2, y - res / 2), colour, -1)
    for o in truth.objects:
        colour = (60, 120, 60) if o['kind'] == 'shrub' else (30, 30, 30)
        thick = 2 if o in truth.blocking else 1
        cv2.circle(img, px(o['x'], o['y']), max(1, int(o['radius'] * scale)), colour, thick, cv2.LINE_AA)
    if trace:
        cv2.polylines(img, [np.array([px(t[0], t[1]) for t in trace])], False, (255, 255, 255), 2, cv2.LINE_AA)
    if path is not None and path.poses:
        pts = np.array([px(p.pose.position.x, p.pose.position.y) for p in path.poses])
        cv2.polylines(img, [pts], False, (60, 200, 60), 2, cv2.LINE_AA)
    cv2.drawMarker(img, px(*goal), (60, 200, 60), cv2.MARKER_STAR, 22, 2)
    cv2.drawMarker(img, px(start_pose[0], start_pose[1]), (255, 255, 255), cv2.MARKER_SQUARE, 14, 2)
    for i, text in enumerate([f'terrain shaded by true slope, purple tint: steeper than {limit:.0f} deg',
                              'grid: red obstacle, magenta too steep, orange too close',
                              'circles: rocks (black) and shrubs (green), thick = must avoid',
                              'white: drive, green: A* path just after the goal was sent; 1 px = %.0f mm' % (1000 / scale)]):
        cv2.putText(img, text, (8, 18 + 17 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (20, 20, 20), 1, cv2.LINE_AA)
    cv2.imwrite(str(path_png), img)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--goal', type=float, nargs=2, required=True, metavar=('X', 'Y'))
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plot', type=Path)
    parser.add_argument('--drive', action='store_true', help='also follow the drive (path follower running)')
    parser.add_argument('--drive-timeout', type=float, default=240.0, help='wall seconds')
    parser.add_argument('--path-seconds', type=float, default=5.0)
    parser.add_argument('--max-slope', type=float, default=25.0, help="planner's max_slope_deg")
    parser.add_argument('--step-height', type=float, default=0.12, help="planner's z_min")
    args = parser.parse_args()

    truth = Truth(args.step_height)
    rclpy.init()
    probe = Probe()
    results = {'goal': list(args.goal), 'max_slope_deg': args.max_slope, 'checks': []}

    def record(name, passed, **details):
        results['checks'].append({'check': name, 'passed': bool(passed), **details})
        print(f"[{'PASS' if passed else 'FAIL'}] {name}: {json.dumps(details)}", flush=True)

    if not probe.spin_for(30.0, lambda: probe.odom is not None and probe.grid is not None):
        sys.exit('no /odom or /planner/grid: start the sim and the planner first')
    start_pose, tilt = pose_of(probe.odom)
    results['start'] = {'x': round(start_pose[0], 2), 'y': round(start_pose[1], 2), 'tilt_deg': round(tilt, 1)}
    probe.spin_for(3.0)                                   # a few clouds into the grid
    check_grid(truth, probe.grid, start_pose, args.max_slope, record)

    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.position.x, goal.pose.position.y = args.goal
    goal.pose.orientation.w = 1.0
    probe.path = None
    for _ in range(3):
        probe.goal_pub.publish(goal)
        probe.spin_for(0.3)
    last_path = check_paths(probe, truth, args.max_slope, args.path_seconds, record)
    trace = []
    if args.drive:
        trace = drive(probe, truth, args.goal, args.max_slope, args.drive_timeout, record, results)
        pose, _ = pose_of(probe.odom)
        check_grid(truth, probe.grid, pose, args.max_slope, record)
    if args.plot:
        plot(args.plot, truth, probe.grid, last_path, trace, args.goal, start_pose, args.max_slope)
    results['passed'] = all(c['passed'] for c in results['checks'])
    args.output.write_text(json.dumps(results, indent=1))
    print('ALL PASS' if results['passed'] else 'SOME CHECKS FAILED')
    probe.destroy_node()
    rclpy.shutdown()
    return 0 if results['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
