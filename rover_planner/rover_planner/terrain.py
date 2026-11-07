"""Ground elevation, slope and "height above ground" from stereo points.

The flat-ground grid treated anything 0.06 m above z = 0 as an obstacle, so a
hill was a wall. Here the ground is measured instead:

1. Elevation. Every stereo point (fixed frame) is binned into cells; each
   cloud's mean height per cell is blended into a running elevation map,
   weighted by how well stereo measures height at that range.
2. Local ground planes. At every cell a plane z = c + a x + b y is fitted by
   least squares to the elevation of the cells in a square window around it,
   all cells at once with box filters. Cells far above the first fit (rocks,
   shrubs) are left out and the plane is fitted again, so a rock does not
   tilt the ground under it.
   - The slope window is the size of the rover's wheelbase: its plane is the
     tilt the rover would have standing on that cell.
   - The ground window is larger: its plane is the ground height used to
     judge how tall things are.
3. Height above ground of each point = its z minus the ground plane at its
   cell. Points higher than the rover can drive over are obstacle points;
   points near zero are ground (and clear old obstacle cells). This replaces
   "z above 0" and works on hills.
4. Drop-offs. The camera is low (0.30 m), so ground falling away behind an
   edge (a crater wall) is never seen. Ground close to the camera and inside
   its field of view is always measured when it is drivable, so cells there
   that were looked at in several clouds and still have no height are a
   drop-off ("unseen") and are treated as too steep.

Cells use the same global indices as occupancy.py: cell (ix, iy) covers
[ix * res, (ix + 1) * res) x [iy * res, (iy + 1) * res).
"""

import math

import cv2
import numpy as np


class ElevationMap:
    """Running per-cell mean ground height in a square that follows the rover.

    The arrays cover `size` x `size` cells; when the rover nears an edge they
    are re-centred on it and cells that fall off are forgotten.
    """

    def __init__(self, resolution, size, max_weight):
        self.resolution = resolution
        self.size = size
        self.max_weight = max_weight
        self.origin = None                      # global (ix, iy) of array cell [0, 0]
        self.weight = np.zeros((size, size), np.float64)
        self.height = np.zeros((size, size), np.float64)
        self.viewed = np.zeros((size, size), np.int16)   # clouds that looked at the cell up close

    def recentre(self, x, y, margin):
        """Keep (x, y) at least `margin` cells from every edge."""
        ix, iy = math.floor(x / self.resolution), math.floor(y / self.resolution)
        if self.origin is not None:
            ox, oy = self.origin
            if margin <= ix - ox < self.size - margin and margin <= iy - oy < self.size - margin:
                return
        new = (ix - self.size // 2, iy - self.size // 2)
        arrays = [np.zeros_like(a) for a in (self.weight, self.height, self.viewed)]
        if self.origin is not None:
            dx, dy = new[0] - self.origin[0], new[1] - self.origin[1]
            src_x, dst_x = slice(max(dx, 0), self.size + min(dx, 0)), slice(max(-dx, 0), self.size + min(-dx, 0))
            src_y, dst_y = slice(max(dy, 0), self.size + min(dy, 0)), slice(max(-dy, 0), self.size + min(-dy, 0))
            if abs(dx) < self.size and abs(dy) < self.size:
                for new_a, old_a in zip(arrays, (self.weight, self.height, self.viewed)):
                    new_a[dst_y, dst_x] = old_a[src_y, src_x]
        self.origin = new
        self.weight, self.height, self.viewed = arrays

    def mark_viewed(self, camera_xy, heading, near, far, bearing_min, bearing_max):
        """Count one look at the cells between near and far metres from the camera
        (horizontally) within its usable bearings (rad, left positive)."""
        if self.origin is None:
            return
        res = self.resolution
        n = int(math.ceil(far / res)) + 1
        cx, cy = math.floor(camera_xy[0] / res), math.floor(camera_xy[1] / res)
        iy, ix = np.mgrid[cy - n:cy + n + 1, cx - n:cx + n + 1]
        dx, dy = (ix + 0.5) * res - camera_xy[0], (iy + 0.5) * res - camera_xy[1]
        d = np.hypot(dx, dy)
        bearing = np.angle(np.exp(1j * (np.arctan2(dy, dx) - heading)))
        seen = (d >= near) & (d <= far) & (bearing >= bearing_min) & (bearing <= bearing_max)
        ax, ay = ix[seen] - self.origin[0], iy[seen] - self.origin[1]
        inside = (ax >= 0) & (ax < self.size) & (ay >= 0) & (ay < self.size)
        self.viewed[ay[inside], ax[inside]] = np.minimum(self.viewed[ay[inside], ax[inside]] + 1, 1000)

    def add(self, xyz, sigma_z, min_points):
        """Blend one cloud in. xyz: Nx3 fixed frame; sigma_z: N height errors (m)."""
        if self.origin is None or not len(xyz):
            return
        ij = np.floor(xyz[:, :2] / self.resolution).astype(np.int64) - np.asarray(self.origin)
        inside = (ij >= 0).all(axis=1) & (ij < self.size).all(axis=1)
        ij, z, w = ij[inside], xyz[inside, 2], 1.0 / np.square(sigma_z[inside])
        if not len(z):
            return
        flat = ij[:, 1] * self.size + ij[:, 0]
        count = np.bincount(flat, minlength=self.size * self.size)
        sum_z = np.bincount(flat, weights=z, minlength=self.size * self.size)
        # A cell's measurement is its mean height; its weight is the mean point weight
        # (points in one cell share their error, so it is not divided by the count).
        sum_w = np.bincount(flat, weights=w, minlength=self.size * self.size)
        cells = np.nonzero(count >= min_points)[0]
        mean = sum_z[cells] / count[cells]
        w_cell = sum_w[cells] / count[cells]
        rows, cols = np.divmod(cells, self.size)
        old_w = self.weight[rows, cols]
        new_w = old_w + w_cell
        self.height[rows, cols] += (mean - self.height[rows, cols]) * (w_cell / new_w)
        self.weight[rows, cols] = np.minimum(new_w, self.max_weight)

    def window(self, origin_ix, origin_iy, width, height, with_views=False):
        """(heights, known mask[, view counts]) for a window of global cells, [gy, gx]."""
        h = np.zeros((height, width), np.float64)
        known = np.zeros((height, width), bool)
        views = np.zeros((height, width), np.int16)
        ox, oy = (origin_ix - self.origin[0], origin_iy - self.origin[1]) if self.origin else (0, 0)
        x0, y0 = max(ox, 0), max(oy, 0)
        x1, y1 = min(ox + width, self.size), min(oy + height, self.size)
        if self.origin is not None and x1 > x0 and y1 > y0:
            h[y0 - oy:y1 - oy, x0 - ox:x1 - ox] = self.height[y0:y1, x0:x1]
            known[y0 - oy:y1 - oy, x0 - ox:x1 - ox] = self.weight[y0:y1, x0:x1] > 0
            views[y0 - oy:y1 - oy, x0 - ox:x1 - ox] = self.viewed[y0:y1, x0:x1]
        return (h, known, views) if with_views else (h, known)


def fit_planes(height, mask, cells, min_fraction):
    """Least-squares plane at every cell over a cells x cells window of masked cells.

    Coordinates are in cells relative to the array. Returns (value at the cell,
    slope a = dz/dx per cell, slope b = dz/dy per cell, valid mask). A plane is
    valid when at least min_fraction of its window is known and the known
    cells are not all on one line.
    """
    rows, cols = np.mgrid[0:height.shape[0], 0:height.shape[1]].astype(np.float64)
    m = mask.astype(np.float64)
    z = np.where(mask, height, 0.0)

    def box(a):
        return cv2.boxFilter(a, cv2.CV_64F, (cells, cells), normalize=False,
                             borderType=cv2.BORDER_CONSTANT)

    # Shift coordinates to each window's centre so the normal equations stay well scaled:
    # sums of (x - xc) follow from sums of x.
    n = box(m)
    sx, sy = box(m * cols) - n * cols, box(m * rows) - n * rows
    sxx = box(m * cols * cols) - 2 * cols * box(m * cols) + n * cols * cols
    syy = box(m * rows * rows) - 2 * rows * box(m * rows) + n * rows * rows
    sxy = (box(m * cols * rows) - cols * box(m * rows) - rows * box(m * cols) + n * cols * rows)
    sz = box(z)
    sxz = box(z * cols) - cols * sz
    syz = box(z * rows) - rows * sz
    a_mat = np.stack([np.stack([n, sx, sy], -1), np.stack([sx, sxx, sxy], -1),
                      np.stack([sy, sxy, syy], -1)], -2)
    rhs = np.stack([sz, sxz, syz], -1)
    det = np.linalg.det(a_mat)
    # Known cells spanning less than a few cells in some direction give no slope.
    valid = (n >= min_fraction * cells * cells) & (np.abs(det) > 1e-6 * np.maximum(n, 1) ** 3)
    a_mat[~valid] = np.eye(3)
    rhs[~valid] = 0.0
    sol = np.linalg.solve(a_mat, rhs[..., None])[..., 0]
    return sol[..., 0], sol[..., 1], sol[..., 2], valid


def robust_planes(height, known, cells, min_fraction, outlier):
    """fit_planes, refitted without cells more than `outlier` m above the first fit."""
    value, a, b, valid = fit_planes(height, known, cells, min_fraction)
    keep = known & ~(valid & (height - value > outlier))
    return fit_planes(height, keep, cells, min_fraction)


class TerrainModel:
    """Ground height and slope for one window, from an ElevationMap.

    Attributes, all [gy, gx] over the window:
      ground   ground height under each cell (m), from the ground window
      ground_ok  ground is known there
      slope    tilt of the wheelbase-sized plane (rad); 0 where unknown
      slope_ok   slope is known there
      grad     (dz/dx, dz/dy) of that plane
      unseen   looked at up close but never measured: a drop-off
    """

    def __init__(self, elevation, origin_ix, origin_iy, width, height, *,
                 slope_window, ground_window, min_fraction, outlier, unseen_views=3):
        res = elevation.resolution
        self.origin_ix, self.origin_iy, self.resolution = origin_ix, origin_iy, res
        self.width, self.height = width, height
        pad = int(math.ceil(max(slope_window, ground_window) / res / 2)) + 1
        h, known, views = elevation.window(origin_ix - pad, origin_iy - pad, width + 2 * pad,
                                           height + 2 * pad, with_views=True)
        inner = (slice(pad, pad + height), slice(pad, pad + width))

        def odd(metres):
            return max(3, int(round(metres / res)) | 1)

        g, _, _, g_ok = robust_planes(h, known, odd(ground_window), min_fraction, outlier)
        s_val, sa, sb, s_ok = robust_planes(h, known, odd(slope_window), min_fraction, outlier)
        self.ground = g[inner]
        self.ground_ok = g_ok[inner]
        # Where the big window has too little data, the small one may still know the ground.
        fill = ~self.ground_ok & s_ok[inner]
        self.ground[fill] = s_val[inner][fill]
        self.ground_ok |= fill
        self.grad = np.stack([sa[inner], sb[inner]], -1) / res
        self.slope_ok = s_ok[inner]
        self.slope = np.where(self.slope_ok, np.arctan(np.hypot(self.grad[..., 0], self.grad[..., 1])), 0.0)
        self.elevation = h[inner]
        self.known = known[inner]
        # Looked at up close several times and still no ground within a cell of it:
        # a drop-off. (One cell of slack: stereo leaves small holes.)
        near_known = cv2.dilate(known.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
        self.unseen = ((views >= unseen_views) & ~near_known)[inner]

    def height_above_ground(self, xyz):
        """Each point's height above the ground under it; NaN where unknown or off the window."""
        gx = np.floor(xyz[:, 0] / self.resolution).astype(np.int64) - self.origin_ix
        gy = np.floor(xyz[:, 1] / self.resolution).astype(np.int64) - self.origin_iy
        out = np.full(len(xyz), np.nan)
        inside = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)
        ok = np.zeros(len(xyz), bool)
        ok[inside] = self.ground_ok[gy[inside], gx[inside]]
        out[ok] = xyz[ok, 2] - self.ground[gy[ok], gx[ok]]
        return out


def directional_slopes(grad, directions):
    """Pitch and roll (rad) of a rover driving each way over planes of gradient grad.

    grad: [..., 2] dz/dx, dz/dy. directions: list of (dx, dy). Returns two arrays
    [len(directions), ...]: pitch (positive = nose up) and |roll|.
    """
    pitch, roll = [], []
    for dx, dy in directions:
        n = math.hypot(dx, dy)
        ux, uy = dx / n, dy / n
        along = grad[..., 0] * ux + grad[..., 1] * uy
        across = -grad[..., 0] * uy + grad[..., 1] * ux
        # Tilt of the plane measured along each body axis.
        pitch.append(np.arctan(along))
        roll.append(np.abs(np.arctan(across / np.sqrt(1 + along * along))))
    return np.array(pitch), np.array(roll)


def height_sigma(xyz, sensor_xyz, depth_error_k, floor):
    """Stereo height error per point: range error k r^2 along the ray, projected on z."""
    ray = xyz - np.asarray(sensor_xyz)
    r = np.maximum(np.linalg.norm(ray, axis=1), 1e-6)
    return np.hypot(floor, depth_error_k * r * r * np.abs(ray[:, 2]) / r)
