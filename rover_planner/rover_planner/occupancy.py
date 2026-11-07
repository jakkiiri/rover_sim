"""Turn stereo point clouds into a 2D obstacle grid around the rover.

All coordinates here are in the planner's fixed frame (odom), where the
desert ground is flat at z = 0, so a point's z is its height above ground.

Cells use global integer indices: cell (ix, iy) covers
[ix * res, (ix + 1) * res) x [iy * res, (iy + 1) * res). A grid window is a
block of those cells, so obstacles stay put in the world as the window
follows the rover and an OccupancyGrid lines up with them exactly.
"""

import math

import numpy as np

from .astar import OBSTACLE


def quaternion_matrix(x, y, z, w):
    """3x3 rotation matrix of a unit quaternion."""
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_points(xyz, translation, quaternion):
    """Apply a TF transform (translation xyz, quaternion xyzw) to Nx3 points."""
    rotation = quaternion_matrix(*quaternion)
    return xyz @ rotation.T + np.asarray(translation, dtype=np.float64)


def filter_obstacle_points(xyz, sensor_xyz, rover_pose, *, z_min, z_max, max_range,
                           half_length, half_width):
    """Keep points that can be obstacles.

    xyz: Nx3 points in the fixed frame. sensor_xyz: camera position in that
    frame. rover_pose: (x, y, yaw). Drops points that are not finite, outside
    the height band (ground and overhead), further than max_range from the
    camera (stereo depth error grows with the square of range), or on the
    rover's own footprint.
    """
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    xyz = xyz[(xyz[:, 2] > z_min) & (xyz[:, 2] < z_max)]
    rng = np.linalg.norm(xyz - np.asarray(sensor_xyz), axis=1)
    xyz = xyz[rng <= max_range]
    rx, ry, yaw = rover_pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx = xyz[:, 0] - rx
    dy = xyz[:, 1] - ry
    on_rover = (np.abs(dx * c + dy * s) <= half_length) & (np.abs(-dx * s + dy * c) <= half_width)
    return xyz[~on_rover]


def ground_points(xyz, sensor_xyz, *, z_min, max_range):
    """Points on the ground (|z| < z_min) within max_range of the camera.

    A ground point means the camera saw the ground at that spot, so nothing
    stands there; ObstacleMemory uses this to clear obstacle cells.
    """
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    xyz = xyz[np.abs(xyz[:, 2]) < z_min]
    return xyz[np.linalg.norm(xyz - np.asarray(sensor_xyz), axis=1) <= max_range]


class ObstacleMemory:
    """Obstacle cells, kept until the camera sees the ground there again.

    Filters, in order:
      - one cloud must put at least `min_points` points in a cell;
      - a cell must be seen in at least `confirm_hits` clouds before it counts
        (real obstacles show up in every cloud, stereo mismatches rarely repeat);
      - a cell is cleared after `clear_misses` clouds in a row that show ground
        in it and no obstacle. Cells out of view or hidden behind something
        are never seen as ground, so they are kept: there is deliberately no
        timeout, because a parked rover must not forget the rock beside it;
      - a cell not yet confirmed is dropped `unconfirmed_s` after it was seen.

    Stereo depth error grows with the square of range (sigma = k * r^2) and lies
    along the camera ray, so with depth_error_k > 0 each confirmed cell is drawn
    spread +-sigma along the ray it was last seen on. Near obstacles stay tight;
    far ones, whose position is less certain, get longer.
    """

    # Samples across +-1 sigma when spreading a cell along the camera ray.
    SPREAD_STEPS = np.linspace(-1.0, 1.0, 7)
    # Fields of a cell record.
    SEEN, HEIGHT, HITS, UX, UY, SIGMA, MISSES = range(7)

    def __init__(self, resolution, unconfirmed_s, min_points, confirm_hits=1, depth_error_k=0.0,
                 clear_misses=3):
        self.resolution = resolution
        self.unconfirmed_s = unconfirmed_s
        self.min_points = min_points
        self.confirm_hits = confirm_hits
        self.depth_error_k = depth_error_k
        self.clear_misses = clear_misses
        # (ix, iy) -> [last_seen_s, height_m, hits, ray_unit_x, ray_unit_y, sigma_m, misses]
        self.cells = {}

    def _cells_with_points(self, xyz):
        ij = np.floor(xyz[:, :2] / self.resolution).astype(np.int64)
        cells, inverse, counts = np.unique(ij, axis=0, return_inverse=True, return_counts=True)
        return cells, inverse.reshape(-1), counts

    def add(self, xyz, stamp_s, sensor_xy=(0.0, 0.0), ground_xyz=None, range_sigma=None):
        """Add one cloud: obstacle points and (optionally) ground points, fixed frame.

        range_sigma: the sensor's range error (m), constant with range (LiDAR).
        None uses the stereo model, depth_error_k * r^2.
        """
        hit = set()
        if xyz.shape[0]:
            cells, inverse, counts = self._cells_with_points(xyz)
            heights = np.zeros(len(cells))
            np.maximum.at(heights, inverse, xyz[:, 2])
            keep = counts >= self.min_points
            cells, heights = cells[keep], heights[keep]
            ray = (cells + 0.5) * self.resolution - np.asarray(sensor_xy, dtype=np.float64)
            rng = np.maximum(np.hypot(ray[:, 0], ray[:, 1]), 1e-6)
            for (ix, iy), h, (dx, dy), r in zip(cells, heights, ray, rng):
                key = (int(ix), int(iy))
                hit.add(key)
                old = self.cells.get(key)
                hits = old[self.HITS] + 1 if old is not None else 1
                height = max(float(h), old[self.HEIGHT]) if old is not None else float(h)
                sigma = self.depth_error_k * r * r if range_sigma is None else range_sigma
                self.cells[key] = [stamp_s, height, hits, dx / r, dy / r, sigma, 0]
        if ground_xyz is not None and ground_xyz.shape[0] and self.cells:
            cells, _, counts = self._cells_with_points(ground_xyz)
            ground = {(int(ix), int(iy)) for ix, iy in cells[counts >= self.min_points]}
            for key in [k for k in self.cells if k in ground and k not in hit]:
                record = self.cells[key]
                record[self.MISSES] += 1
                if record[self.MISSES] >= self.clear_misses:
                    del self.cells[key]

    def prune(self, now_s, centre_xy=None, keep_radius=None):
        """Drop stale unconfirmed cells and (optionally) all cells far from centre_xy."""
        cutoff = now_s - self.unconfirmed_s
        cells = {k: v for k, v in self.cells.items()
                 if v[self.HITS] >= self.confirm_hits or v[self.SEEN] >= cutoff}
        if centre_xy is not None and keep_radius is not None:
            r2 = (keep_radius / self.resolution) ** 2
            cx, cy = centre_xy[0] / self.resolution, centre_xy[1] / self.resolution
            cells = {k: v for k, v in cells.items() if (k[0] - cx) ** 2 + (k[1] - cy) ** 2 <= r2}
        self.cells = cells

    def clear(self):
        self.cells.clear()

    def confirmed(self):
        return {k: v for k, v in self.cells.items() if v[self.HITS] >= self.confirm_hits}

    def grid(self, origin_ix, origin_iy, width, height):
        """Occupancy (int8: 0 or OBSTACLE) and height arrays for one window, [gy, gx]."""
        occupancy = np.zeros((height, width), dtype=np.int8)
        heights = np.zeros((height, width), dtype=np.float32)
        confirmed = self.confirmed()
        if not confirmed:
            return occupancy, heights
        keys = np.array(list(confirmed.keys()), dtype=np.float64)
        vals = np.array(list(confirmed.values()), dtype=np.float64)
        centres = (keys + 0.5) * self.resolution
        offsets = vals[:, 5:6] * self.SPREAD_STEPS[None, :]                  # N x S metres
        pts = centres[:, None, :] + offsets[:, :, None] * vals[:, None, 3:5]  # N x S x 2
        ij = np.floor(pts.reshape(-1, 2) / self.resolution).astype(np.int64)
        gx = ij[:, 0] - origin_ix
        gy = ij[:, 1] - origin_iy
        h = np.repeat(vals[:, 1], len(self.SPREAD_STEPS)).astype(np.float32)
        inside = (gx >= 0) & (gx < width) & (gy >= 0) & (gy < height)
        gx, gy, h = gx[inside], gy[inside], h[inside]
        occupancy[gy, gx] = OBSTACLE
        np.maximum.at(heights, (gy, gx), h)
        return occupancy, heights


class GridWindow:
    """A size x size cell window centred on the rover's cell."""

    def __init__(self, rover_x, rover_y, resolution, size):
        self.resolution = resolution
        self.size = size
        self.origin_ix = math.floor(rover_x / resolution) - size // 2
        self.origin_iy = math.floor(rover_y / resolution) - size // 2

    @property
    def origin_xy(self):
        """World position of the window's lower-left corner."""
        return self.origin_ix * self.resolution, self.origin_iy * self.resolution

    def world_to_grid(self, x, y):
        """Window cell holding (x, y); may fall outside the window."""
        return (math.floor(x / self.resolution) - self.origin_ix,
                math.floor(y / self.resolution) - self.origin_iy)

    def grid_to_world(self, gx, gy):
        """Centre of a window cell."""
        return ((gx + self.origin_ix + 0.5) * self.resolution,
                (gy + self.origin_iy + 0.5) * self.resolution)
