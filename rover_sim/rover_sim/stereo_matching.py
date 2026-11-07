"""Stereo matching, depth and point-cloud math on plain arrays (no ROS, no cv_bridge).

Every function either returns a result it has checked or raises ValueError, so
the ROS wrapper can publish nothing instead of guessing.
"""
from dataclasses import dataclass
import math
import struct

import numpy as np

CHANNELS = {'mono8': 1, 'rgb8': 3, 'bgr8': 3}
MATCHER_DEFAULTS = dict(min_disparity=0, num_disparities=80, block_size=5,
                        uniqueness_ratio=10, speckle_window_size=100, speckle_range=2,
                        disp12_max_diff=1, pre_filter_cap=63, min_valid_disparity=2.0,
                        min_texture=4.0, texture_window=5, edge_erosion=2, cloud_stride=2)
INVALID_DISPARITY = -1.0
POINT_STEP = 16  # x, y, z float32 + packed rgb


def _integer(config, name, low, high=None):
    value = config[name]
    if isinstance(value, bool) or not isinstance(value, int) or value < low or (high is not None and value > high):
        bound = f'from {low} to {high}' if high is not None else f'at least {low}'
        raise ValueError(f'{name} must be an integer {bound}')
    return value


def validate_matcher_config(config):
    """Return a complete, checked matcher configuration."""
    unknown = set(config) - set(MATCHER_DEFAULTS)
    if unknown:
        raise ValueError(f'Unknown matcher parameters: {sorted(unknown)}')
    merged = dict(MATCHER_DEFAULTS, **config)
    _integer(merged, 'min_disparity', 0, 256)
    if _integer(merged, 'num_disparities', 16, 512) % 16:
        raise ValueError('num_disparities must be a multiple of 16')
    if _integer(merged, 'block_size', 3, 11) % 2 == 0:
        raise ValueError('block_size must be odd')
    _integer(merged, 'uniqueness_ratio', 0, 100)
    _integer(merged, 'speckle_window_size', 0, 10000)
    _integer(merged, 'speckle_range', 0, 64)
    _integer(merged, 'disp12_max_diff', -1, 512)
    _integer(merged, 'pre_filter_cap', 1, 255)
    _integer(merged, 'cloud_stride', 1, 16)
    _integer(merged, 'edge_erosion', 0, 5)
    if _integer(merged, 'texture_window', 3, 31) % 2 == 0:
        raise ValueError('texture_window must be odd')
    for name, positive in (('min_valid_disparity', True), ('min_texture', False)):
        value = merged[name]
        if (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
                or value < 0 or (positive and value == 0)):
            raise ValueError(f'{name} must be a finite {"positive" if positive else "nonnegative"} number')
        merged[name] = float(value)
    return merged


def image_array(encoding, width, height, step, data):
    """View raw ROS image bytes as an (h, w[, 3]) uint8 array, checking every size field."""
    if encoding not in CHANNELS:
        raise ValueError(f'Unsupported image encoding {encoding!r}')
    if width <= 0 or height <= 0:
        raise ValueError('Image must have positive size')
    channels = CHANNELS[encoding]
    if step < width * channels:
        raise ValueError('Image step is smaller than one row')
    buffer = np.frombuffer(bytes(data) if not isinstance(data, (bytes, bytearray, memoryview)) else data,
                           dtype=np.uint8)
    if buffer.size != step * height:
        raise ValueError('Image data length does not match step * height')
    rows = buffer.reshape(height, step)[:, :width * channels]
    return rows.reshape(height, width) if channels == 1 else rows.reshape(height, width, channels)


def to_gray(array, encoding):
    if encoding == 'mono8':
        return np.ascontiguousarray(array)
    rgb = array if encoding == 'rgb8' else array[:, :, ::-1]
    # ITU-R BT.601 luma, the same weights OpenCV uses.
    gray = 0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1] + 0.114 * rgb[:, :, 2]
    return np.ascontiguousarray(np.clip(np.rint(gray), 0, 255).astype(np.uint8))


@dataclass(frozen=True)
class PairGeometry:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    baseline: float


def check_pair(left, right, expected_baseline=None, baseline_tolerance=1e-3):
    """Check a synchronized left/right pair; left/right are dicts of header and camera_info fields.

    Required keys: stamp (sec, nanosec), image_frame, info_frame, width, height,
    info_width, info_height, d, k, r, p. Raises ValueError with the first problem.
    """
    stamps = {left['stamp'], right['stamp'], left['info_stamp'], right['info_stamp']}
    if len(stamps) != 1:
        raise ValueError('Left/right images and camera_info must have identical stamps')
    for side in (left, right):
        if side['image_frame'] != side['info_frame']:
            raise ValueError('Image and camera_info frames differ')
        if (side['width'], side['height']) != (side['info_width'], side['info_height']):
            raise ValueError('Image size differs from camera_info size')
        values = list(side['d']) + list(side['k']) + list(side['r']) + list(side['p'])
        if len(side['k']) != 9 or len(side['r']) != 9 or len(side['p']) != 12 or not all(
                math.isfinite(v) for v in values):
            raise ValueError('camera_info matrices must be complete and finite')
        if any(abs(v) > 1e-12 for v in side['d']):
            raise ValueError('Distorted images are not supported; this pipeline assumes rectified input')
        if any(abs(a - b) > 1e-9 for a, b in zip(side['r'], (1, 0, 0, 0, 1, 0, 0, 0, 1))):
            raise ValueError('Rectification rotation must be identity')
    if left['image_frame'] == right['image_frame']:
        raise ValueError('Left and right images must use different optical frames')
    if (left['width'], left['height']) != (right['width'], right['height']):
        raise ValueError('Left and right images differ in size')
    lp, rp = left['p'], right['p']
    for index in (0, 2, 5, 6):
        if abs(lp[index] - rp[index]) > 1e-6:
            raise ValueError('Left and right projections must share fx, fy, cx and cy')
    if lp[0] <= 0 or lp[5] <= 0:
        raise ValueError('Focal lengths must be positive')
    if abs(lp[3]) > 1e-9:
        raise ValueError('Left projection Tx must be zero')
    if not rp[3] < 0:
        raise ValueError('Right projection Tx must be negative (Tx = -fx * baseline)')
    baseline = -rp[3] / rp[0]
    if expected_baseline is not None and abs(baseline - expected_baseline) > baseline_tolerance:
        raise ValueError(f'camera_info baseline {baseline:.4f} m differs from expected {expected_baseline:.4f} m')
    return PairGeometry(left['width'], left['height'], lp[0], lp[5], lp[2], lp[6], baseline)


def create_matcher(cv2, config):
    c = validate_matcher_config(config)
    area = c['block_size'] ** 2
    return cv2.StereoSGBM_create(
        minDisparity=c['min_disparity'], numDisparities=c['num_disparities'], blockSize=c['block_size'],
        P1=8 * area, P2=32 * area, disp12MaxDiff=c['disp12_max_diff'], preFilterCap=c['pre_filter_cap'],
        uniquenessRatio=c['uniqueness_ratio'], speckleWindowSize=c['speckle_window_size'],
        speckleRange=c['speckle_range'], mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY)


def valid_window(width, height, config):
    """(x, y, w, h) of pixels where a full search window fits; outside it SGBM output is not trusted."""
    c = validate_matcher_config(config)
    half = c['block_size'] // 2
    x = c['min_disparity'] + c['num_disparities'] - 1 + half
    w, h = width - x - half, height - 2 * half
    if w <= 0 or h <= 0:
        raise ValueError('Image is too small for the configured disparity search')
    return x, half, w, h


def disparity_from_sgbm(raw, config):
    """Convert SGBM fixed-point output (x16) to float pixels; invalid or outside-window pixels become -1."""
    c = validate_matcher_config(config)
    if raw.dtype != np.int16 or raw.ndim != 2:
        raise ValueError('SGBM output must be a 2-D int16 array')
    disparity = raw.astype(np.float32) / 16.0
    x, y, w, h = valid_window(raw.shape[1], raw.shape[0], c)
    keep = np.zeros(raw.shape, dtype=bool)
    keep[y:y + h, x:x + w] = True
    keep &= disparity >= max(c['min_disparity'], c['min_valid_disparity'])
    keep &= disparity <= c['min_disparity'] + c['num_disparities']
    return np.where(keep, disparity, np.float32(INVALID_DISPARITY)).astype(np.float32)


def local_std(gray, window):
    """Standard deviation of gray levels in a window x window box around each pixel (edges replicated)."""
    half = window // 2
    padded = np.pad(gray.astype(np.float64), half, mode='edge')
    total = np.zeros((padded.shape[0] + 1, padded.shape[1] + 1))
    squares = np.zeros_like(total)
    total[1:, 1:] = padded.cumsum(0).cumsum(1)
    squares[1:, 1:] = (padded * padded).cumsum(0).cumsum(1)

    def box(table):
        return (table[window:, window:] - table[:-window, window:]
                - table[window:, :-window] + table[:-window, :-window])
    count = window * window
    mean = box(total) / count
    return np.sqrt(np.maximum(box(squares) / count - mean * mean, 0.0))


def erode(mask, radius):
    """Binary erosion with a square (2r+1) window; outside the image counts as False."""
    if radius == 0:
        return mask.copy()
    padded = np.pad(mask, radius, mode='constant', constant_values=False)
    out = np.ones_like(mask)
    height, width = mask.shape
    for dy in range(2 * radius + 1):
        for dx in range(2 * radius + 1):
            out &= padded[dy:dy + height, dx:dx + width]
    return out


def compute_disparity(matcher, left_gray, right_gray, config):
    """SGBM disparity with two fail-closed filters.

    - Texture: on flat surfaces (sky, plain-coloured hills) SGBM matches camera
      noise and returns confident but wrong disparities, so patches with less
      than min_texture grey-level variation are invalid.
    - Edge erosion: block matching spreads a near surface's disparity a pixel or
      two past its silhouette ("edge fattening"), so the valid region is shrunk
      by edge_erosion pixels.
    """
    c = validate_matcher_config(config)
    if left_gray.shape != right_gray.shape or left_gray.ndim != 2:
        raise ValueError('Matcher needs two gray images of the same size')
    disparity = disparity_from_sgbm(matcher.compute(left_gray, right_gray), c)
    if c['min_texture'] > 0:
        disparity[local_std(left_gray, c['texture_window']) < c['min_texture']] = INVALID_DISPARITY
    if c['edge_erosion'] > 0:
        disparity[~erode(disparity != INVALID_DISPARITY, c['edge_erosion'])] = INVALID_DISPARITY
    return disparity


def disparity_to_depth(disparity, fx, baseline, min_valid_disparity):
    """Depth Z = fx * B / d in metres; anything not strictly valid becomes NaN."""
    if not (math.isfinite(fx) and fx > 0 and math.isfinite(baseline) and baseline > 0
            and math.isfinite(min_valid_disparity) and min_valid_disparity > 0):
        raise ValueError('fx, baseline and min_valid_disparity must be finite and positive')
    d = np.asarray(disparity, dtype=np.float32)
    valid = np.isfinite(d) & (d >= min_valid_disparity)
    depth = np.full(d.shape, np.nan, dtype=np.float32)
    depth[valid] = np.float32(fx * baseline) / d[valid]
    return depth


def depth_to_points(depth, fx, fy, cx, cy, stride=1):
    """Back-project valid depth pixels into the optical frame. Returns (points Nx3, rows, cols)."""
    if stride < 1:
        raise ValueError('stride must be at least 1')
    rows, cols = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    z = depth[rows, cols]
    valid = np.isfinite(z) & (z > 0)
    rows, cols, z = rows[valid], cols[valid], z[valid]
    x = (cols.astype(np.float32) - np.float32(cx)) * z / np.float32(fx)
    y = (rows.astype(np.float32) - np.float32(cy)) * z / np.float32(fy)
    return np.stack([x, y, z], axis=1).astype(np.float32), rows, cols


def pack_xyzrgb(points, colors):
    """Pack Nx3 float32 points and Nx3 uint8 RGB into PointCloud2 bytes (point_step 16)."""
    points = np.asarray(points, dtype=np.float32)
    colors = np.asarray(colors, dtype=np.uint8)
    if points.ndim != 2 or points.shape[1] != 3 or colors.shape != points.shape:
        raise ValueError('points and colors must both be Nx3')
    record = np.zeros(points.shape[0], dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4')])
    record['x'], record['y'], record['z'] = points[:, 0], points[:, 1], points[:, 2]
    record['rgb'] = ((colors[:, 0].astype(np.uint32) << 16) | (colors[:, 1].astype(np.uint32) << 8)
                     | colors[:, 2].astype(np.uint32))
    return record.tobytes()


def unpack_xyz(data, count, point_step=POINT_STEP):
    """Inverse of pack_xyzrgb for x, y, z (used by tests and the validator)."""
    if len(data) != count * point_step:
        raise ValueError('PointCloud2 data length does not match point count')
    return [struct.unpack_from('<fff', data, i * point_step) for i in range(count)]
