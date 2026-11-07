# MODIFIED from jakkiiri/rover_sim, Apache-2.0, commit
# 4b503e9a0232867b749f1e1f6f7a8958ed70a3a8 (rover_sim/lidar_postprocess.py).
# Local changes: robust PointCloud2 decoding, output bounds, consistent scan
# metadata, validated read-only parameters and ROS shutdown handling, beam
# fans not centred on the horizon (vertical_min_deg, for the downward dome) and
# a box that drops the rover's own body (self_filter_box).
# See THIRD_PARTY_NOTICES.md and LICENSES/Apache-2.0.txt.
"""Lightweight LiDAR realism post-processing (numpy-vectorised).

Subscribes to the raw Gazebo ray cloud (``/points_raw``) and republishes
``/points`` with:

* ``ring`` (uint16) and per-point ``time`` (float32, seconds from the sweep
  start, from the azimuth) fields that the Gazebo plugin does not provide,
* range-dependent Gaussian range noise ``sigma = sigma0 + sigma_per_metre * r``
  added along the ray (the sensor's own fixed-sigma noise remains),
* dropout / missing returns with probability ``p = p0 + p_per_metre * r``,
* an intensity model ``I = albedo * cos(incidence-free) / r^2`` scaled to [0, 1]
  when the sensor provides none,
* returns inside ``self_filter_box`` (the rover's own body, in the sensor
  frame) removed: a downward-looking dome sees the chassis and wheels,
* optionally a 2-D ``/scan`` LaserScan from the ring closest to 0 deg
  elevation (fallback when ``pointcloud_to_laserscan`` is not installed).

Not modelled: dust attenuation, multipath, rolling-shutter spin distortion.
The pass uses vectorised numpy operations; actual cost depends on cloud size
and host load. Per-point time is synthetic and does not imply motion deskewing.
"""
import math
import signal

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

# Point clouds are large (up to 55 800 points per sweep) and get fragmented on
# the wire; best-effort subscribers drop whole sweeps whenever one fragment is
# lost (measured on Fortress: 4.3 Hz received of 10 Hz published). Reliable
# QoS delivers every sweep from both backends (their publishers are reliable).
CLOUD_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=5)
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

NUMPY_TYPES = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2', PointField.UINT16: 'u2',
               PointField.INT32: 'i4', PointField.UINT32: 'u4', PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}


def cloud_to_array(msg):
    """Decode byte order and padded rows; reject malformed clouds explicitly."""
    if msg.point_step <= 0 or msg.row_step < msg.width * msg.point_step:
        raise ValueError('Invalid PointCloud2 point_step/row_step')
    if len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Truncated PointCloud2 data')
    names = [f.name for f in msg.fields]
    if len(set(names)) != len(names) or not {'x', 'y', 'z'} <= set(names):
        raise ValueError('PointCloud2 needs unique fields including x, y, z')
    endian = '>' if msg.is_bigendian else '<'
    for field in msg.fields:
        if (field.datatype not in NUMPY_TYPES or field.count != 1 or field.offset < 0
                or field.offset + np.dtype(NUMPY_TYPES[field.datatype]).itemsize > msg.point_step):
            raise ValueError('Unsupported PointCloud2 field layout')
    dtype = np.dtype({'names': [f.name for f in msg.fields],
                      'formats': [endian + NUMPY_TYPES[f.datatype] for f in msg.fields],
                      'offsets': [f.offset for f in msg.fields], 'itemsize': msg.point_step})
    return np.ndarray((msg.height, msg.width), dtype=dtype, buffer=bytes(msg.data),
                      strides=(msg.row_step, msg.point_step)).reshape(-1)


def process_cloud(points, rng, channels, vertical_fov, rate, sigma0, sigma_per_metre, p0, p_per_metre,
                  min_range, max_range, vertical_shift_steps=0.0, vertical_min=None, self_box=None):
    """Pure function: returns (xyz, intensity, ring, time) arrays of kept points.

    ``vertical_min`` is the lowest beam's elevation (rad); None means the fan
    is centred on the horizon (-vertical_fov / 2). ``self_box`` is
    (xmin, xmax, ymin, ymax, zmin, zmax) in the sensor frame; returns inside it
    are dropped. None or a zero-volume box disables it.

    ``vertical_shift_steps`` is the reference branch's optional legacy beam
    correction. Normal local launches set it to zero: this host already
    reports correct GPU angles. If explicitly enabled, points shifted above
    the top beam are dropped.
    """
    xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float64)
    finite = np.all(np.isfinite(xyz), axis=1)
    xyz = xyz[finite]
    r = np.linalg.norm(xyz, axis=1)
    keep = (r >= min_range) & (r <= max_range) & (r > 1e-6)
    if self_box is not None and self_box[0] < self_box[1] and self_box[2] < self_box[3] \
            and self_box[4] < self_box[5]:
        keep &= ~((xyz[:, 0] >= self_box[0]) & (xyz[:, 0] <= self_box[1])
                  & (xyz[:, 1] >= self_box[2]) & (xyz[:, 1] <= self_box[3])
                  & (xyz[:, 2] >= self_box[4]) & (xyz[:, 2] <= self_box[5]))
    xyz, r = xyz[keep], r[keep]
    if 'intensity' in points.dtype.names:
        intensity = points['intensity'][finite][keep].astype(np.float32)
    else:
        intensity = None
    azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
    elevation = np.arcsin(np.clip(xyz[:, 2] / np.maximum(r, 1e-9), -1, 1))
    step = vertical_fov / max(channels - 1, 1)
    if vertical_min is None:
        vertical_min = -vertical_fov / 2
    if vertical_shift_steps:
        elevation = elevation + vertical_shift_steps * step
        inside = elevation <= vertical_min + vertical_fov + 0.25 * step
        xyz, r, azimuth, elevation = xyz[inside], r[inside], azimuth[inside], elevation[inside]
        if intensity is not None:
            intensity = intensity[inside]
        xyz = np.stack([r * np.cos(elevation) * np.cos(azimuth), r * np.cos(elevation) * np.sin(azimuth),
                        r * np.sin(elevation)], axis=1)
    ring = (np.clip(np.rint((elevation - vertical_min) / step), 0, channels - 1)
            .astype(np.uint16) if channels > 1 else np.zeros(len(r), dtype=np.uint16))
    # Dropout and range-dependent noise.
    p_drop = np.clip(p0 + p_per_metre * r, 0.0, 0.95)
    survive = rng.random(len(r)) >= p_drop
    xyz, r, azimuth, ring = xyz[survive], r[survive], azimuth[survive], ring[survive]
    if intensity is not None:
        intensity = intensity[survive]
    sigma = sigma0 + sigma_per_metre * r
    scale = 1.0 + rng.normal(0.0, 1.0, len(r)) * sigma / np.maximum(r, 1e-9)
    # Noise must never flip a ray or publish returns outside the advertised range.
    noisy_range = r * scale
    valid = np.isfinite(noisy_range) & (noisy_range >= min_range) & (noisy_range <= max_range)
    xyz, r, azimuth, ring, scale = xyz[valid], r[valid], azimuth[valid], ring[valid], scale[valid]
    if intensity is not None:
        intensity = intensity[valid]
    xyz = xyz * scale[:, None]
    if intensity is None or not np.any(intensity):
        albedo = 0.25 + 0.1 * rng.random(len(r))
        intensity = np.clip(albedo / np.maximum(r, 0.5) ** 2 * 25.0, 0.0, 1.0).astype(np.float32)
    intensity = np.nan_to_num(intensity, nan=0.0, posinf=1.0, neginf=0.0)
    time = (((azimuth + math.pi) / (2 * math.pi)) / rate).astype(np.float32)
    return xyz.astype(np.float32), intensity.astype(np.float32), ring, time


def make_cloud(header, xyz, intensity, ring, time):
    msg = PointCloud2()
    msg.header = header
    msg.height, msg.width = 1, len(xyz)
    msg.fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                  PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                  PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                  PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                  PointField(name='ring', offset=16, datatype=PointField.UINT16, count=1),
                  PointField(name='time', offset=18, datatype=PointField.FLOAT32, count=1)]
    msg.is_bigendian = False
    msg.point_step = 22
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    data = np.zeros(len(xyz), dtype=np.dtype({'names': ['x', 'y', 'z', 'intensity', 'ring', 'time'],
                                              'formats': ['<f4', '<f4', '<f4', '<f4', '<u2', '<f4'],
                                              'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 22}))
    data['x'], data['y'], data['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    data['intensity'], data['ring'], data['time'] = intensity, ring, time
    msg.data = data.tobytes()
    return msg


def make_scan(header, xyz, ring, scan_ring, samples, min_range, max_range):
    """Planar LaserScan from one ring (nearest-bin binning of the azimuths)."""
    msg = LaserScan()
    msg.header = header
    msg.angle_min = -math.pi
    msg.angle_increment = 2 * math.pi / samples
    msg.angle_max = msg.angle_min + (samples - 1) * msg.angle_increment
    msg.range_min, msg.range_max = float(min_range), float(max_range)
    ranges = np.full(samples, math.inf, dtype=np.float32)
    selected = xyz[ring == scan_ring]
    if len(selected):
        azimuth = np.arctan2(selected[:, 1], selected[:, 0])
        r = np.hypot(selected[:, 0], selected[:, 1])
        index = np.rint((azimuth + math.pi) / msg.angle_increment).astype(int) % samples
        valid = np.isfinite(r) & (r >= min_range) & (r <= max_range)
        np.minimum.at(ranges, index[valid], r[valid])
    msg.ranges = ranges.tolist()
    return msg


class LidarPostprocess(Node):
    def __init__(self):
        super().__init__('lidar_postprocess')
        p = {name: self.declare_parameter(
            name, value, ParameterDescriptor(read_only=True)).value for name, value in dict(
            input_topic='/points_raw', output_topic='/points', channels=16, vertical_fov_deg=30.0, rate=10.0,
            sigma0=0.01, sigma_per_metre=0.0003, dropout_base=0.02, dropout_per_metre=0.002,
            min_range=0.5, max_range=100.0, seed=1, publish_scan=False, scan_samples=720,
            vertical_shift_steps=0.0, vertical_min_deg=float('nan'),
            self_filter_box=[0.0] * 6).items()}
        for key in ('channels', 'scan_samples'):
            if p[key] <= 0 or (key == 'channels' and p[key] > 65535):
                raise ValueError(f'{key} must be positive and channels must fit uint16')
        for key in ('rate', 'vertical_fov_deg', 'max_range'):
            if not math.isfinite(p[key]) or p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('sigma0', 'sigma_per_metre', 'dropout_base', 'dropout_per_metre', 'min_range'):
            if not math.isfinite(p[key]) or p[key] < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        if not math.isfinite(p['vertical_shift_steps']) or abs(p['vertical_shift_steps']) > 2:
            raise ValueError('vertical_shift_steps must be finite and within [-2, 2]')
        if p['vertical_fov_deg'] >= 180 or p['min_range'] >= p['max_range']:
            raise ValueError('vertical_fov_deg must be below 180 and min_range below max_range')
        if math.isfinite(p['vertical_min_deg']) and not (
                -90 <= p['vertical_min_deg'] and p['vertical_min_deg'] + p['vertical_fov_deg'] <= 90):
            raise ValueError('vertical_min_deg + vertical_fov_deg must stay within [-90, 90] deg')
        if len(p['self_filter_box']) != 6 or not all(math.isfinite(v) for v in p['self_filter_box']):
            raise ValueError('self_filter_box must be 6 finite numbers: xmin xmax ymin ymax zmin zmax')
        if p['dropout_base'] > 1 or p['seed'] < 0:
            raise ValueError('dropout_base must be <= 1 and seed nonnegative')
        if not p['input_topic'] or not p['output_topic'] or p['input_topic'] == p['output_topic']:
            raise ValueError('Input/output topics must be nonempty and different')
        self.p = p
        self.rng = np.random.default_rng(int(p['seed']))
        # Lowest beam elevation; NaN means a fan centred on the horizon.
        self.vertical_min = (math.radians(p['vertical_min_deg']) if math.isfinite(p['vertical_min_deg'])
                             else -math.radians(p['vertical_fov_deg']) / 2)
        self.self_box = tuple(float(v) for v in p['self_filter_box'])
        # The ring closest to 0 deg elevation.
        step = math.radians(p['vertical_fov_deg']) / max(p['channels'] - 1, 1)
        self.scan_ring = int(min(max(round(-self.vertical_min / step), 0), p['channels'] - 1))
        self.publisher = self.create_publisher(PointCloud2, p['output_topic'], CLOUD_QOS)
        self.scan_publisher = (self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
                               if p['publish_scan'] else None)
        self.create_subscription(PointCloud2, p['input_topic'], self.on_cloud, CLOUD_QOS)
        self.get_logger().info(f"post-processing {p['input_topic']} -> {p['output_topic']} "
                               f"(channels {p['channels']}, scan {'on' if p['publish_scan'] else 'off'})")

    def on_cloud(self, msg):
        p = self.p
        try:
            points = cloud_to_array(msg)
        except (ValueError, TypeError) as error:
            self.get_logger().warning(f'Discarding malformed LiDAR cloud: {error}', throttle_duration_sec=5.0)
            return
        xyz, intensity, ring, time = process_cloud(
            points, self.rng, int(p['channels']), math.radians(p['vertical_fov_deg']), p['rate'], p['sigma0'],
            p['sigma_per_metre'], p['dropout_base'], p['dropout_per_metre'], p['min_range'], p['max_range'],
            float(p['vertical_shift_steps']), self.vertical_min, self.self_box)
        self.publisher.publish(make_cloud(msg.header, xyz, intensity, ring, time))
        if self.scan_publisher is not None:
            self.scan_publisher.publish(make_scan(msg.header, xyz, ring, self.scan_ring, int(p['scan_samples']),
                                                  p['min_range'], p['max_range']))


def main(args=None):
    node = None
    stopping = False

    def request_stop(signum, frame):
        nonlocal stopping
        stopping = True

    # Signals only set a flag, including during destruction. This keeps repeated
    # Ctrl+C from interrupting cleanup or destroying resources more than once.
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, request_stop)
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = LidarPostprocess()
        while rclpy.ok() and not stopping:
            rclpy.spin_once(node, timeout_sec=.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
