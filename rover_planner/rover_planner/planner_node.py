"""A* planner node: stereo point cloud -> terrain + obstacle grid -> nav_msgs/Path.

The ground is measured, not assumed flat (terrain.py): slopes the rover can
climb cost more the steeper they are, slopes it cannot climb are blocked, and
obstacles are judged by their height above the local ground.

Subscribes (read only; nothing in the sim is changed):
  /stereo/points2  sensor_msgs/PointCloud2  terrain and obstacles (best-effort, latest only)
  /points          sensor_msgs/PointCloud2  3D LiDAR: terrain and obstacles all around
                   (lidar_topic; empty turns it off)
  /odom            nav_msgs/Odometry        rover pose (from ground_truth_odom)
  /goal_pose       geometry_msgs/PoseStamped  goals from RViz "2D Goal Pose"
Publishes:
  /planner/grid    nav_msgs/OccupancyGrid   100 obstacle, 80 too steep, 50 too close for the
                   rover, 1-49 slope cost, 0 flat, -1 ground not measured yet
  /planner/grid_markers  visualization_msgs/MarkerArray  the same grid as cubes on the ground
                   (RViz's Map display fails to colour grids on some Intel/Mesa drivers)
  /planner/path    nav_msgs/Path            A* path from the rover to the goal
  /planner/status  std_msgs/String          idle, path, no_path_can_turn (no path, but the
                   rover can turn in place to look around: path_follower does), no_path,
                   or goal_blocked (no safe point within goal_snap_radius of the goal)

Goals can also come from the goal_x / goal_y parameters, at start-up or live:
  ros2 param set /astar_planner goal_x 6.5
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
import numpy as np
import tf2_ros

from .astar import (OBSTACLE, GridAStar, circle_kernel, footprint_blocked,
                    footprint_kernel, nearest_valid_cell, plan_from)
from .occupancy import GridWindow, ObstacleMemory, transform_points
from .terrain import ElevationMap, TerrainModel, height_sigma

# Values in /planner/grid besides OBSTACLE (100) and free (0).
TOO_STEEP = 80     # ground steeper than max_slope, or a drop-off
TOO_CLOSE = 50     # the rover would touch an obstacle or put a wheel on too-steep ground
SLOPE_COST_MAX = 49  # slope cost is shown as 1..49

PARAMETERS = {
    'cloud_topic': '/stereo/points2',
    'lidar_topic': '/points',    # 3D LiDAR cloud; '' uses the stereo camera only
    'odom_topic': '/odom',
    'goal_topic': '/goal_pose',
    'global_frame': 'odom',
    'resolution': 0.1,           # m per cell
    'grid_size': 12.0,           # m, square window centred on the rover
    'z_min': 0.12,               # m above the local ground: taller points are obstacles
    'z_max': 1.5,                # m above ground; higher points can be driven under
    'ground_tolerance': 0.05,    # m; points this close to the local ground are ground
    'max_slope_deg': 25.0,       # steepest ground the rover may drive on (measured, see README)
    'slope_cost': 3.0,           # a step at max_slope costs (1 + slope_cost) flat steps
    'slope_window': 1.0,         # m, square fitted for the rover's tilt (its wheelbase)
    'ground_window': 2.0,        # m, square fitted for the ground height under obstacles
    'terrain_min_fraction': 0.3,  # share of a window that must be measured to fit a plane
    'terrain_outlier': 0.08,     # m above the first plane fit: left out of the refit (rocks)
    'height_error_floor': 0.02,  # m, smallest stereo height error (measured ground noise)
    'elevation_size': 30.0,      # m, square of remembered ground heights around the rover
    # Drop-offs: ground this close to the camera is always measured when drivable
    # (camera 0.30 m up sees downslopes up to atan(0.30 / 0.9) = 18 deg past an edge).
    'drop_check_near': 0.45,     # m from the camera; nearer ground is below the image
    'drop_check_far': 0.9,       # m from the camera
    'drop_check_views': 3,       # clouds that must have looked at a cell
    'camera_bearings_deg': [-40.0, 29.0],  # usable view, right / left (stereo blind band, 5 deg margin)
    'max_range': 6.0,            # m from the camera
    'min_points_per_cell': 2,    # per cloud, rejects isolated stereo mismatches
    'confirm_hits': 3,           # clouds a cell must be seen in before it counts
    'depth_error_k': 0.0087,     # stereo depth sigma = k * range^2 (m); 0 turns spreading off
    'clear_misses': 3,           # clouds in a row showing ground in a cell that clear it
    'unconfirmed_s': 2.0,        # s before a cell seen in fewer than confirm_hits clouds is dropped
    'cloud_rate': 5.0,           # Hz of clouds added to the grid (camera runs at 15)
    'lidar_rate': 10.0,          # Hz of LiDAR clouds added to the grid
    'lidar_max_range': 10.0,     # m from the LiDAR
    'lidar_range_sigma': 0.03,   # m, LiDAR range error (sensor 0.02 + post-processing ~0.01)
    'terrain_cache_s': 0.2,      # s a terrain fit is reused for (one fit takes ~40 ms)
    'plan_rate': 2.0,            # Hz of grid publishing and replanning
    'footprint_half_length': 0.58,  # m, base_link to front of wheels / camera
    'footprint_half_width': 0.51,   # m, base_link to outside of the wheels
    'inflation': 0.15,           # m of clearance added on every side
    'footprint_shape': 'circle',  # 'circle' fits any heading; 'rectangle' uses the current heading
    'goal_tolerance': 0.3,       # m; closer than this counts as reached
    'goal_snap_radius': 1.5,     # m; a blocked goal is replaced by the nearest safe cell this close
    'goal_snap_clearance': 0.2,  # m kept from blocked cells by that safe cell, when there is room
    'goal_x': math.nan,
    'goal_y': math.nan,
}


def goal_snap_mask(shape, cell, radius_cells):
    """Cells within radius_cells of cell (gx, gy), as a [gy, gx] mask."""
    rows, cols = np.mgrid[0:shape[0], 0:shape[1]]
    return np.hypot(cols - cell[0], rows - cell[1]) <= radius_cells


def cloud_to_xyz(msg):
    """Nx3 float64 x, y, z of a PointCloud2, NaN points dropped.

    read_points_numpy needs every field to share one type, and the stereo
    cloud also carries a uint32 rgb field, so read a structured array instead.
    """
    pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.column_stack([pts['x'], pts['y'], pts['z']]).astype(np.float64)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class AStarPlanner(Node):
    def __init__(self):
        super().__init__('astar_planner')
        for name, default in PARAMETERS.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in PARAMETERS}
        self.p = p
        if p['resolution'] <= 0 or p['grid_size'] < 1.0 or p['cloud_rate'] <= 0 or p['plan_rate'] <= 0:
            raise ValueError('resolution, cloud_rate and plan_rate must be positive; grid_size >= 1 m')
        if p['footprint_shape'] not in ('circle', 'rectangle'):
            raise ValueError("footprint_shape must be 'circle' or 'rectangle'")
        self.frame = p['global_frame']
        self.size = int(round(p['grid_size'] / p['resolution']))
        self.memory = ObstacleMemory(p['resolution'], p['unconfirmed_s'], p['min_points_per_cell'],
                                     p['confirm_hits'], p['depth_error_k'], p['clear_misses'])
        # Each cell's height is a running mean; capping its weight at about ten good
        # measurements keeps it able to follow new, better (closer) views.
        self.elevation = ElevationMap(p['resolution'], int(round(p['elevation_size'] / p['resolution'])),
                                      max_weight=10.0 / p['height_error_floor'] ** 2)
        self.max_slope = math.radians(p['max_slope_deg'])
        self.terrain = None

        self.pose = None          # (x, y, yaw) in the global frame
        self.goal = None          # (x, y) in the global frame
        self.snapped_goal = None  # safe point used instead of a blocked goal
        self.last_cloud_s = -math.inf
        self.last_lidar_s = -math.inf
        self.terrain_cache = None  # (window origin, sim time, TerrainModel)
        self.stats = {'clouds': 0, 'lidar_clouds': 0, 'tf_skips': 0, 'plans': 0, 'plan_ms': 0.0}
        self.last_warning = None  # avoids repeating the same warning every plan

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Best effort, latest only: never slows the stereo publisher down.
        cloud_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PointCloud2, p['cloud_topic'], self.on_cloud, cloud_qos)
        if p['lidar_topic']:
            # Reliable, like the LiDAR's own viewer: its clouds are fragmented on the
            # wire and best effort drops whole sweeps when one fragment is lost.
            lidar_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                   reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(PointCloud2, p['lidar_topic'], self.on_lidar, lidar_qos)
        self.create_subscription(Odometry, p['odom_topic'], self.on_odom, 10)
        self.create_subscription(PoseStamped, p['goal_topic'], self.on_goal_pose, 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, '/planner/grid', 1)
        self.path_pub = self.create_publisher(Path, '/planner/path', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/planner/grid_markers', 1)
        self.status_pub = self.create_publisher(String, '/planner/status', 1)
        self.add_on_set_parameters_callback(self.on_set_parameters)

        if math.isfinite(p['goal_x']) and math.isfinite(p['goal_y']):
            self.set_goal(p['goal_x'], p['goal_y'], 'parameter')
        self.create_timer(1.0 / p['plan_rate'], self.plan)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"A* planner: {self.size}x{self.size} cells of {p['resolution']} m in '{self.frame}', "
            f"obstacles {p['z_min']}-{p['z_max']} m above the ground within {p['max_range']} m, "
            f"slopes up to {p['max_slope_deg']} deg. "
            'Set a goal with RViz "2D Goal Pose".')

    # ---- inputs ---------------------------------------------------------

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw_of(q))

    def on_cloud(self, msg):
        """Stereo cloud: terrain, obstacles and drop-off views (at most cloud_rate Hz)."""
        stamp_s = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.pose is None or stamp_s - self.last_cloud_s < 1.0 / self.p['cloud_rate'] - 1e-3:
            return
        placed = self.place_cloud(msg, self.p['max_range'])
        if placed is None:
            return
        self.last_cloud_s = stamp_s
        xyz, sensor, r = placed
        p = self.p
        # Cells the camera looked at up close, for drop-off detection.
        fx = 2 * (r.x * r.z + r.y * r.w)          # optical axis (z) in the fixed frame
        fy = 2 * (r.y * r.z - r.x * r.w)
        b0, b1 = (math.radians(a) for a in p['camera_bearings_deg'])
        self.elevation.recentre(self.pose[0], self.pose[1], margin=self.size // 2 + 15)
        self.elevation.mark_viewed(sensor[:2], math.atan2(fy, fx), p['drop_check_near'],
                                   p['drop_check_far'], b0, b1)
        self.add_points(xyz, sensor, stamp_s, height_sigma(xyz, sensor, p['depth_error_k'],
                                                           p['height_error_floor']), None)
        self.stats['clouds'] += 1

    def on_lidar(self, msg):
        """3D LiDAR cloud: terrain and obstacles all around (at most lidar_rate Hz).

        Its range error is small and does not grow with range, so heights get the
        error floor and obstacles are not spread along the ray. It does not mark
        cells for drop-off detection: its lowest beam meets flat ground about 2.7 m
        out, so it never sees the ground right next to the rover.
        """
        stamp_s = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        if self.pose is None or stamp_s - self.last_lidar_s < 1.0 / self.p['lidar_rate'] - 1e-3:
            return
        placed = self.place_cloud(msg, self.p['lidar_max_range'])
        if placed is None:
            return
        self.last_lidar_s = stamp_s
        xyz, sensor, _ = placed
        p = self.p
        self.elevation.recentre(self.pose[0], self.pose[1], margin=self.size // 2 + 15)
        self.add_points(xyz, sensor, stamp_s, np.full(len(xyz), p['height_error_floor']),
                        p['lidar_range_sigma'], ground_only=True)
        self.stats['lidar_clouds'] += 1

    def place_cloud(self, msg, max_range):
        """(points in the fixed frame, sensor position, sensor rotation), or None."""
        try:
            # No waiting: a blocking lookup would stall this single-threaded node.
            tf = self.tf_buffer.lookup_transform(self.frame, msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp))
        except tf2_ros.TransformException:
            self.stats['tf_skips'] += 1  # the next cloud will do
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        sensor = (t.x, t.y, t.z)
        xyz = transform_points(cloud_to_xyz(msg), sensor, (r.x, r.y, r.z, r.w))
        return self.drop_far_and_own_points(xyz, sensor, max_range), sensor, r

    def add_points(self, xyz, sensor, stamp_s, sigma_z, range_sigma, ground_only=False):
        """Ground heights, then obstacle and ground cells, from one placed cloud.

        ground_only: leave points already known to be obstacles (above z_min over
        the current ground) out of the ground heights. The LiDAR samples rock tops
        densely; in the height map they turned rocks into steep mounds of
        terrain, which a gentle slope fit can hide. The stereo camera keeps adding
        every point: it is what maps hill and mesa flanks as ground.
        """
        p = self.p
        window = GridWindow(self.pose[0], self.pose[1], p['resolution'], self.size)
        if ground_only:
            above = self.terrain_model(window, cached=True).height_above_ground(xyz)
            keep = ~(above > p['z_min'])                   # unknown (NaN) ground counts as kept
            self.elevation.add(xyz[keep], sigma_z[keep], p['min_points_per_cell'])
        else:
            # Rock and shrub points go in too; the plane fits leave them out.
            self.elevation.add(xyz, sigma_z, p['min_points_per_cell'])
        # Height of every point above the ground under it.
        terrain = self.terrain_model(window, cached=True)
        above = terrain.height_above_ground(xyz)
        known = np.isfinite(above)
        obstacle = known & (above > p['z_min']) & (above < p['z_max'])
        ground = known & (np.abs(above) < p['ground_tolerance'])
        # Obstacle memory, with heights above the ground.
        points = np.column_stack([xyz[obstacle, :2], above[obstacle]])
        self.memory.add(points, stamp_s, sensor[:2], xyz[ground], range_sigma=range_sigma)

    def drop_far_and_own_points(self, xyz, sensor, max_range=None):
        """Finite points within max_range of the sensor that are not on the rover."""
        max_range = self.p['max_range'] if max_range is None else max_range
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        xyz = xyz[np.linalg.norm(xyz - np.asarray(sensor), axis=1) <= max_range]
        rx, ry, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = xyz[:, 0] - rx, xyz[:, 1] - ry
        on_rover = ((np.abs(dx * c + dy * s) <= self.p['footprint_half_length'])
                    & (np.abs(-dx * s + dy * c) <= self.p['footprint_half_width']))
        return xyz[~on_rover]

    def terrain_model(self, window, cached=False):
        """Terrain over the window. cached=True reuses a fit of the same window made
        less than terrain_cache_s ago: clouds arrive at up to 15 Hz and one fit
        takes about 40 ms, while the ground changes slowly."""
        now = self.get_clock().now().nanoseconds * 1e-9
        origin = (window.origin_ix, window.origin_iy)
        if (cached and self.terrain_cache is not None and self.terrain_cache[0] == origin
                and 0 <= now - self.terrain_cache[1] < self.p['terrain_cache_s']):
            return self.terrain_cache[2]
        terrain = self._fit_terrain(window)
        self.terrain_cache = (origin, now, terrain)
        return terrain

    def _fit_terrain(self, window):
        p = self.p
        return TerrainModel(self.elevation, window.origin_ix, window.origin_iy, self.size, self.size,
                            slope_window=p['slope_window'], ground_window=p['ground_window'],
                            min_fraction=p['terrain_min_fraction'], outlier=p['terrain_outlier'],
                            unseen_views=p['drop_check_views'])

    def on_goal_pose(self, msg):
        x, y = msg.pose.position.x, msg.pose.position.y
        frame = msg.header.frame_id or self.frame
        if frame != self.frame:
            try:
                tf = self.tf_buffer.lookup_transform(self.frame, frame, Time())
            except tf2_ros.TransformException as exc:
                self.get_logger().warn(f'Ignoring goal in {frame}: {exc}')
                return
            t, r = tf.transform.translation, tf.transform.rotation
            x, y, _ = transform_points(np.array([[x, y, msg.pose.position.z]]),
                                       (t.x, t.y, t.z), (r.x, r.y, r.z, r.w))[0]
        self.set_goal(x, y, 'RViz')

    def on_set_parameters(self, params):
        """Live goal changes through goal_x / goal_y; other parameters are fixed."""
        values = {p.name: p.value for p in params}
        for name in values:
            if name in PARAMETERS and name not in ('goal_x', 'goal_y'):
                return SetParametersResult(successful=False,
                                           reason=f'{name} is read at start-up only')
        if 'goal_x' in values or 'goal_y' in values:
            gx = float(values.get('goal_x', self.get_parameter('goal_x').value))
            gy = float(values.get('goal_y', self.get_parameter('goal_y').value))
            if math.isfinite(gx) and math.isfinite(gy):
                self.set_goal(gx, gy, 'parameter')
        return SetParametersResult(successful=True)

    def set_goal(self, x, y, source):
        self.goal = (float(x), float(y))
        self.snapped_goal = None
        self.last_warning = None
        self.get_logger().info(f'New goal from {source}: ({x:.2f}, {y:.2f}) in {self.frame}')

    # ---- planning -------------------------------------------------------

    def plan(self):
        """Publish the current grid and, with a goal, a fresh A* path."""
        if self.pose is None:
            return
        p = self.p
        now = self.get_clock().now()
        rx, ry, yaw = self.pose
        # Keep cells that could re-enter the window soon; drop the rest.
        self.memory.prune(now.nanoseconds * 1e-9, (rx, ry), p['grid_size'])
        window = GridWindow(rx, ry, p['resolution'], self.size)
        occupancy, _ = self.memory.grid(window.origin_ix, window.origin_iy, self.size, self.size)
        terrain = self.terrain = self.terrain_model(window)
        # Too steep: ground where the rover, standing there, would tilt more than
        # max_slope, and drop-offs (ground next to the camera that it never saw).
        # Grown by the footprint like obstacles: no wheel may go onto it either (a
        # wheel against the foot of a steep wall jams its steering).
        steep = (terrain.slope_ok & (terrain.slope > self.max_slope)) | terrain.unseen
        kernel = self.kernel(yaw)
        steep_zone = footprint_blocked(np.where(steep, OBSTACLE, 0).astype(np.int8), kernel)
        blocked = footprint_blocked(occupancy, kernel) | steep_zone
        # Slope cost: 0 on the flat, slope_cost at max_slope, growing with the square.
        cost = p['slope_cost'] * np.square(np.minimum(terrain.slope / self.max_slope, 1.0))
        height = np.where(terrain.ground_ok, terrain.ground, np.nan)
        self.publish_grid(window, occupancy, blocked, steep, cost, terrain, now)
        if self.goal is None:
            self.publish_status('idle')
            return
        target = self.goal if self.snapped_goal is None else self.snapped_goal
        if (math.hypot(self.goal[0] - rx, self.goal[1] - ry) < p['goal_tolerance']
                or math.hypot(target[0] - rx, target[1] - ry) < p['goal_tolerance']):
            where = 'Goal' if target is self.goal else 'Safe point near the goal'
            self.get_logger().info(f'{where} ({target[0]:.2f}, {target[1]:.2f}) reached')
            self.goal = self.snapped_goal = None
            self.publish_path([], window, now)
            self.publish_status('idle')
            return

        start = window.world_to_grid(rx, ry)
        goal = window.world_to_grid(*self.goal)
        search = GridAStar(occupancy, height, blocked, cost)
        if not search.in_grid(goal):
            # Planning horizon is the window: aim for the free cell nearest the
            # goal and replan as the rover (and window) moves on. Only cells the
            # rover can reach count, or it may aim for a mesa top it cannot climb.
            goal = nearest_valid_cell(blocked, goal, reachable_from=start)
        elif not search.is_pose_valid(goal):
            # A goal the rover cannot stand on (next to a rock, on a steep flank):
            # go to the nearest reachable safe cell within goal_snap_radius instead.
            near = self.snap_goal(search, blocked, window, goal, start)
            if near is None:
                self.warn_once(f"Goal is inside an obstacle, too close to one, or too steep, and no "
                               f"safe point is within {p['goal_snap_radius']} m of it; no path")
                self.publish_path([], window, now)
                self.publish_status('goal_blocked')  # nothing to look for: do not turn around
                return
            self.warn_once('Goal is inside an obstacle, too close to one, or too steep; '
                           'heading for the nearest safe point instead')
            goal = near
        if goal is None or not search.is_pose_valid(goal):
            self.warn_once('No reachable free cell towards the goal; no path')
            self.publish_path([], window, now)
            self.publish_no_path(occupancy, terrain, start)
            return
        # The rover's real body at its heading, no inflation: the hard limit.
        body = footprint_blocked(occupancy, footprint_kernel(
            p['footprint_half_length'], p['footprint_half_width'], yaw, p['resolution']))

        t0 = time.perf_counter()
        turn = self.turn_blocked(occupancy)
        cells, problem = plan_from(search, start, goal, body, turn, yaw)
        self.stats['plan_ms'] = (time.perf_counter() - t0) * 1e3
        self.stats['plans'] += 1
        if problem:
            self.warn_once(f'No path: {problem}')
            self.publish_no_path(occupancy, terrain, start)
        else:
            self.last_warning = None
            self.publish_status('path')
        self.publish_path(cells, window, now, terrain)

    def snap_goal(self, search, blocked, window, goal, start):
        """Nearest safe reachable cell within goal_snap_radius of a blocked goal, or None.

        Once chosen, the point is kept while it stays safe: re-choosing it every
        plan made it hop round the rock as more of the rock came into view, and
        the rover drove round after it. It also keeps goal_snap_clearance from
        anything blocked when it can, because the grid only holds the parts of a
        rock taller than z_min and the rover parked against the low flank.
        """
        p = self.p
        if self.snapped_goal is not None:
            cell = window.world_to_grid(*self.snapped_goal)
            if search.in_grid(cell) and search.is_pose_valid(cell):
                return cell
        snap = goal_snap_mask(blocked.shape, goal, p['goal_snap_radius'] / p['resolution'])
        margin = footprint_blocked(np.where(blocked, OBSTACLE, 0).astype(np.int8),
                                   circle_kernel(p['goal_snap_clearance'], p['resolution']))
        near = nearest_valid_cell(blocked, goal, reachable_from=start, allowed=snap & ~margin)
        if near is None:
            near = nearest_valid_cell(blocked, goal, reachable_from=start, allowed=snap)
        if near is not None:
            self.snapped_goal = window.grid_to_world(*near)
            self.get_logger().info(
                f'Safe point near the goal: ({self.snapped_goal[0]:.2f}, {self.snapped_goal[1]:.2f}), '
                f"{math.hypot(near[0] - goal[0], near[1] - goal[1]) * p['resolution']:.1f} m from it")
        return near

    def publish_no_path(self, occupancy, terrain, start):
        """No path. Say whether turning in place to look around is safe.

        A false obstacle or drop-off behind the camera can close the only way on,
        and only seeing ground there clears it. Turning sweeps the disc around the
        rover's corners (no inflation): it must hold no obstacle, and the rover
        must stand on ground it may drive on.
        """
        gx, gy = start
        level = not terrain.slope_ok[gy, gx] or terrain.slope[gy, gx] <= self.max_slope
        turn_ok = not self.turn_blocked(occupancy)[gy, gx]
        self.publish_status('no_path_can_turn' if level and turn_ok else 'no_path')

    def turn_blocked(self, occupancy):
        """Cells where turning in place would sweep the body's corners (no
        inflation) into an obstacle.

        Steep ground does not count: a corner passing over it is not a
        collision (the wheels stay within 0.59 m of the centre), and counting it
        left the rover stuck for good beside a crater wall, unable to turn and
        with no straight way out of the margin."""
        p = self.p
        disc = circle_kernel(math.hypot(p['footprint_half_length'], p['footprint_half_width']),
                             p['resolution'])
        return footprint_blocked(occupancy, disc)

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def kernel(self, yaw):
        """Cells the rover covers around its centre (plus inflation)."""
        p = self.p
        hl, hw = p['footprint_half_length'], p['footprint_half_width']
        if p['footprint_shape'] == 'circle':
            return circle_kernel(math.hypot(hl, hw) + p['inflation'], p['resolution'])
        return footprint_kernel(hl + p['inflation'], hw + p['inflation'], yaw, p['resolution'])

    def warn_once(self, text):
        if text != self.last_warning:
            self.get_logger().warn(text)
            self.last_warning = text

    # ---- outputs --------------------------------------------------------

    def publish_grid(self, window, occupancy, blocked, steep, cost, terrain, now):
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.header.stamp = now.to_msg()
        msg.info.resolution = float(self.p['resolution'])
        msg.info.width = msg.info.height = self.size
        msg.info.origin.position.x, msg.info.origin.position.y = window.origin_xy
        msg.info.origin.orientation.w = 1.0
        scale = SLOPE_COST_MAX / max(self.p['slope_cost'], 1e-9)
        data = np.where(cost > 0, np.clip(np.ceil(cost * scale), 0, SLOPE_COST_MAX), 0).astype(np.int8)
        data[~terrain.slope_ok] = -1          # ground not measured yet (planned through as free)
        data[blocked] = TOO_CLOSE
        data[steep] = TOO_STEEP
        data[occupancy >= OBSTACLE] = OBSTACLE
        msg.data = data.ravel().tolist()  # row-major from the lower-left cell
        self.grid_pub.publish(msg)
        self.publish_markers(window, data, terrain, now)

    def publish_markers(self, window, data, terrain, now):
        """The grid as cube lists lying on the measured ground.

        Red: obstacles. Purple: too steep. Orange: too close to an obstacle.
        Slope cost: green (gentle) to yellow (near the limit), from 5 deg up.
        """
        markers = MarkerArray()
        res = float(self.p['resolution'])
        ground = np.where(terrain.ground_ok, terrain.ground, terrain.elevation)
        layers = [(OBSTACLE, (0.80, 0.18, 0.12, 0.95), 0.06),
                  (TOO_STEEP, (0.62, 0.22, 0.85, 0.75), 0.04),
                  (TOO_CLOSE, (1.00, 0.62, 0.20, 0.30), 0.02)]
        for marker_id, (value, rgba, height) in enumerate(layers):
            gy, gx = np.nonzero(data == value)
            markers.markers.append(self.cube_list(marker_id, window, gx, gy, ground, height, rgba, now))
        shown = math.radians(5.0) / self.max_slope
        level = np.clip(terrain.slope / self.max_slope, 0, 1)
        gy, gx = np.nonzero((data > 0) & (data <= SLOPE_COST_MAX) & (level >= shown))
        m = self.cube_list(len(layers), window, gx, gy, ground, 0.02, (0.0, 0.0, 0.0, 0.55), now)
        for x, y in zip(gx, gy):
            t = float(level[y, x])
            m.colors.append(ColorRGBA(r=0.2 + 0.8 * t, g=0.85, b=0.25 * (1 - t), a=0.55))
        markers.markers.append(m)
        self.marker_pub.publish(markers)

    def cube_list(self, marker_id, window, gx, gy, ground, height, rgba, now):
        res = float(self.p['resolution'])
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = now.to_msg()
        m.ns = 'planner_grid'
        m.id = marker_id
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = res
        m.scale.z = height
        m.color.r, m.color.g, m.color.b, m.color.a = (float(c) for c in rgba)
        for x, y in zip(gx, gy):
            wx, wy = window.grid_to_world(int(x), int(y))
            m.points.append(Point(x=wx, y=wy, z=float(ground[y, x]) + height / 2.0))
        return m

    def publish_path(self, cells, window, now, terrain=None):
        msg = Path()
        msg.header.frame_id = self.frame
        msg.header.stamp = now.to_msg()
        points = [window.grid_to_world(gx, gy) for gx, gy in cells]
        for i, (x, y) in enumerate(points):
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x, pose.pose.position.y = x, y
            gx, gy = cells[i]
            if terrain is not None and terrain.ground_ok[gy, gx]:
                pose.pose.position.z = float(terrain.ground[gy, gx]) + 0.05  # drawn on the ground
            # Face the next point; the last pose keeps the final segment's heading.
            if i + 1 < len(points):
                (ax, ay), (bx, by) = (x, y), points[i + 1]
            elif i > 0:
                (ax, ay), (bx, by) = points[i - 1], (x, y)
            else:
                (ax, ay), (bx, by) = (0.0, 0.0), (1.0, 0.0)
            heading = math.atan2(by - ay, bx - ax)
            pose.pose.orientation.z = math.sin(heading / 2.0)
            pose.pose.orientation.w = math.cos(heading / 2.0)
            msg.poses.append(pose)
        self.path_pub.publish(msg)

    def report(self):
        s = self.stats
        self.get_logger().info(
            f"Last 10 s: {s['clouds']} stereo and {s['lidar_clouds']} LiDAR clouds used "
            f"({s['tf_skips']} skipped waiting for TF), "
            f"{s['plans']} plans (last {s['plan_ms']:.0f} ms), "
            f"{len(self.memory.confirmed())} obstacle cells confirmed, {len(self.memory.cells)} seen"
            + self.terrain_summary())
        s.update(clouds=0, lidar_clouds=0, tf_skips=0, plans=0)

    def terrain_summary(self):
        t = self.terrain
        if t is None or not t.slope_ok.any():
            return ''
        steep = int((t.slope_ok & (t.slope > self.max_slope)).sum())
        return (f'; terrain known in {int(t.slope_ok.sum())} cells, steepest '
                f'{math.degrees(float(t.slope[t.slope_ok].max())):.0f} deg, {steep} too steep, '
                f'{int(t.unseen.sum())} drop-off')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
