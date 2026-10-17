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
* optionally a 2-D ``/scan`` LaserScan from the ring closest to 0 deg
  elevation (fallback when ``pointcloud_to_laserscan`` is not installed).

Not modelled: dust attenuation, multipath, rolling-shutter spin distortion.
The whole pass is a handful of numpy operations per cloud (≈1 ms for 14 400
points), so the CPU cost is bounded and small.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

NUMPY_TYPES = {PointField.INT8: 'i1', PointField.UINT8: 'u1', PointField.INT16: 'i2', PointField.UINT16: 'u2',
               PointField.INT32: 'i4', PointField.UINT32: 'u4', PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8'}


def cloud_to_array(msg):
    """View a PointCloud2 as a structured numpy array without copying fields."""
    dtype = np.dtype({'names': [f.name for f in msg.fields],
                      'formats': [NUMPY_TYPES[f.datatype] for f in msg.fields],
                      'offsets': [f.offset for f in msg.fields], 'itemsize': msg.point_step})
    return np.frombuffer(bytes(msg.data), dtype=dtype, count=msg.width * msg.height)


def process_cloud(points, rng, channels, vertical_fov, rate, sigma0, sigma_per_metre, p0, p_per_metre,
                  min_range, max_range):
    """Pure function: returns (xyz, intensity, ring, time) arrays of kept points."""
    xyz = np.stack([points['x'], points['y'], points['z']], axis=1).astype(np.float64)
    finite = np.all(np.isfinite(xyz), axis=1)
    xyz = xyz[finite]
    r = np.linalg.norm(xyz, axis=1)
    keep = (r >= min_range) & (r <= max_range) & (r > 1e-6)
    xyz, r = xyz[keep], r[keep]
    if 'intensity' in points.dtype.names:
        intensity = points['intensity'][finite][keep].astype(np.float32)
    else:
        intensity = None
    azimuth = np.arctan2(xyz[:, 1], xyz[:, 0])
    elevation = np.arcsin(np.clip(xyz[:, 2] / np.maximum(r, 1e-9), -1, 1))
    step = vertical_fov / max(channels - 1, 1)
    ring = np.clip(np.rint((elevation + vertical_fov / 2) / step), 0, channels - 1).astype(np.uint16)
    # Dropout and range-dependent noise.
    p_drop = np.clip(p0 + p_per_metre * r, 0.0, 0.95)
    survive = rng.random(len(r)) >= p_drop
    xyz, r, azimuth, ring = xyz[survive], r[survive], azimuth[survive], ring[survive]
    if intensity is not None:
        intensity = intensity[survive]
    sigma = sigma0 + sigma_per_metre * r
    scale = 1.0 + rng.normal(0.0, 1.0, len(r)) * sigma / np.maximum(r, 1e-9)
    xyz = xyz * scale[:, None]
    if intensity is None or not np.any(intensity):
        albedo = 0.25 + 0.1 * rng.random(len(r))
        intensity = np.clip(albedo / np.maximum(r, 0.5) ** 2 * 25.0, 0.0, 1.0).astype(np.float32)
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
                                              'formats': ['f4', 'f4', 'f4', 'f4', 'u2', 'f4'],
                                              'offsets': [0, 4, 8, 12, 16, 18], 'itemsize': 22}))
    data['x'], data['y'], data['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    data['intensity'], data['ring'], data['time'] = intensity, ring, time
    msg.data = data.tobytes()
    return msg


def make_scan(header, xyz, ring, scan_ring, samples, min_range, max_range):
    """Planar LaserScan from one ring (nearest-bin binning of the azimuths)."""
    msg = LaserScan()
    msg.header = header
    msg.angle_min, msg.angle_max = -math.pi, math.pi
    msg.angle_increment = 2 * math.pi / samples
    msg.range_min, msg.range_max = float(min_range), float(max_range)
    ranges = np.full(samples, math.inf, dtype=np.float32)
    selected = xyz[ring == scan_ring]
    if len(selected):
        azimuth = np.arctan2(selected[:, 1], selected[:, 0])
        r = np.hypot(selected[:, 0], selected[:, 1])
        index = np.clip(((azimuth + math.pi) / msg.angle_increment).astype(int), 0, samples - 1)
        order = np.argsort(-r)                # nearest wins when bins collide
        ranges[index[order]] = r[order]
    msg.ranges = ranges.tolist()
    return msg


class LidarPostprocess(Node):
    def __init__(self):
        super().__init__('lidar_postprocess')
        p = {name: self.declare_parameter(name, value).value for name, value in dict(
            input_topic='/points_raw', output_topic='/points', channels=16, vertical_fov_deg=30.0, rate=10.0,
            sigma0=0.01, sigma_per_metre=0.0003, dropout_base=0.02, dropout_per_metre=0.002,
            min_range=0.5, max_range=100.0, seed=1, publish_scan=False, scan_samples=720).items()}
        for key in ('channels', 'scan_samples'):
            if p[key] <= 0:
                raise ValueError(f'{key} must be positive')
        for key in ('rate', 'vertical_fov_deg', 'max_range'):
            if not math.isfinite(p[key]) or p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        for key in ('sigma0', 'sigma_per_metre', 'dropout_base', 'dropout_per_metre', 'min_range'):
            if not math.isfinite(p[key]) or p[key] < 0:
                raise ValueError(f'{key} must be finite and nonnegative')
        self.p = p
        self.rng = np.random.default_rng(int(p['seed']))
        self.scan_ring = int(round((p['channels'] - 1) / 2.0))
        self.publisher = self.create_publisher(PointCloud2, p['output_topic'], qos_profile_sensor_data)
        self.scan_publisher = (self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
                               if p['publish_scan'] else None)
        self.create_subscription(PointCloud2, p['input_topic'], self.on_cloud, qos_profile_sensor_data)
        self.get_logger().info(f"post-processing {p['input_topic']} -> {p['output_topic']} "
                               f"(channels {p['channels']}, scan {'on' if p['publish_scan'] else 'off'})")

    def on_cloud(self, msg):
        p = self.p
        points = cloud_to_array(msg)
        xyz, intensity, ring, time = process_cloud(
            points, self.rng, int(p['channels']), math.radians(p['vertical_fov_deg']), p['rate'], p['sigma0'],
            p['sigma_per_metre'], p['dropout_base'], p['dropout_per_metre'], p['min_range'], p['max_range'])
        self.publisher.publish(make_cloud(msg.header, xyz, intensity, ring, time))
        if self.scan_publisher is not None:
            self.scan_publisher.publish(make_scan(msg.header, xyz, ring, self.scan_ring, int(p['scan_samples']),
                                                  p['min_range'], p['max_range']))


def main(args=None):
    rclpy.init(args=args)
    node = LidarPostprocess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
