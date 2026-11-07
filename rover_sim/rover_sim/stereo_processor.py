"""Disparity, depth image and point cloud from the synchronized stereo pair.

A thin ROS wrapper around rover_sim.stereo_matching (OpenCV SGBM). It never
uses cv_bridge, which is broken on this machine with numpy 2. Any
inconsistent or unsupported input is logged and nothing is published.
"""
import array
import time

import message_filters
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from stereo_msgs.msg import DisparityImage

from rover_sim.node_runner import run_until_signal
from rover_sim.stereo_geometry import baseline_from_urdf
from rover_sim.stereo_matching import (
    MATCHER_DEFAULTS, POINT_STEP, check_pair, compute_disparity, create_matcher, depth_to_points,
    disparity_to_depth, image_array, pack_xyzrgb, to_gray, valid_window, validate_matcher_config)

INPUT_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
OUTPUT_QOS = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
CLOUD_FIELDS = [PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
                for name, offset in (('x', 0), ('y', 4), ('z', 8))] + [
               PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]


def side_fields(image, info):
    """Plain-data view of an image + camera_info for check_pair."""
    return dict(stamp=(image.header.stamp.sec, image.header.stamp.nanosec),
                info_stamp=(info.header.stamp.sec, info.header.stamp.nanosec),
                image_frame=image.header.frame_id, info_frame=info.header.frame_id,
                width=image.width, height=image.height, info_width=info.width, info_height=info.height,
                d=list(info.d), k=list(info.k), r=list(info.r), p=list(info.p))


def byte_field(data):
    """rclpy validates a bytes value byte by byte (~70 ms per MB); array('B') is copied directly."""
    return array.array('B', data)


def float_image(header, values):
    msg = Image()
    msg.header = header
    msg.height, msg.width = values.shape
    msg.encoding = '32FC1'
    msg.is_bigendian = 0
    msg.step = 4 * msg.width
    msg.data = byte_field(values.astype('<f4').tobytes())
    return msg


class StereoProcessor(Node):
    def __init__(self):
        super().__init__('stereo_processor')
        description = self.declare_parameter('robot_description', '',
                                             ParameterDescriptor(read_only=True)).value
        self.expected_baseline = baseline_from_urdf(description)
        self.baseline_tolerance = self.declare_parameter('baseline_tolerance', 0.002).value
        values = {name: self.declare_parameter(name, default).value for name, default in MATCHER_DEFAULTS.items()}
        self.config = validate_matcher_config(values)   # invalid configuration: node exits
        import cv2  # imported here so the ROS-free modules stay importable without OpenCV
        self.matcher = create_matcher(cv2, self.config)
        self.disparity_pub = self.create_publisher(DisparityImage, '/stereo/disparity', OUTPUT_QOS)
        self.depth_pub = self.create_publisher(Image, '/stereo/depth/image_raw', OUTPUT_QOS)
        self.depth_info_pub = self.create_publisher(CameraInfo, '/stereo/depth/camera_info', OUTPUT_QOS)
        self.cloud_pub = self.create_publisher(PointCloud2, '/stereo/points2', OUTPUT_QOS)
        subscribers = [message_filters.Subscriber(self, kind, topic, qos_profile=INPUT_QOS)
                       for kind, topic in ((Image, '/stereo/left/image_raw'), (CameraInfo, '/stereo/left/camera_info'),
                                           (Image, '/stereo/right/image_raw'),
                                           (CameraInfo, '/stereo/right/camera_info'))]
        # Exact stamps only: the multicamera sensor gives both images one capture time.
        self.sync = message_filters.TimeSynchronizer(subscribers, 4)
        self.sync.registerCallback(self.on_pair)
        self.stats = dict(processed=0, rejected=0, seconds=0.0)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f'Stereo processor ready: SGBM {self.config}, '
                               f'expected baseline {self.expected_baseline:.4f} m')

    def report(self):
        s = self.stats
        mean = 1000 * s['seconds'] / s['processed'] if s['processed'] else 0.0
        self.get_logger().info(f"Last 10 s: {s['processed']} pairs ({mean:.1f} ms mean), {s['rejected']} rejected")
        self.stats = dict(processed=0, rejected=0, seconds=0.0)

    def reject(self, reason):
        self.stats['rejected'] += 1
        self.get_logger().warning(f'Stereo pair rejected, nothing published: {reason}', throttle_duration_sec=5.0)

    def on_pair(self, left, left_info, right, right_info):
        started = time.perf_counter()
        try:
            geometry = check_pair(side_fields(left, left_info), side_fields(right, right_info),
                                  self.expected_baseline, self.baseline_tolerance)
            if left.encoding != right.encoding:
                raise ValueError('Left and right encodings differ')
            left_array = image_array(left.encoding, left.width, left.height, left.step, left.data)
            right_array = image_array(right.encoding, right.width, right.height, right.step, right.data)
            disparity = compute_disparity(self.matcher, to_gray(left_array, left.encoding),
                                          to_gray(right_array, right.encoding), self.config)
            depth = disparity_to_depth(disparity, geometry.fx, geometry.baseline,
                                       self.config['min_valid_disparity'])
            points, rows, cols = depth_to_points(depth, geometry.fx, geometry.fy, geometry.cx, geometry.cy,
                                                 self.config['cloud_stride'])
            if left.encoding == 'mono8':
                colors = left_array[rows, cols][:, None].repeat(3, axis=1)
            else:
                colors = left_array[rows, cols] if left.encoding == 'rgb8' else left_array[rows, cols][:, ::-1]
        except ValueError as error:
            self.reject(str(error))
            return
        self.publish(left.header, left_info, geometry, disparity, depth, points, colors)
        self.stats['processed'] += 1
        self.stats['seconds'] += time.perf_counter() - started

    def publish(self, header, left_info, geometry, disparity, depth, points, colors):
        msg = DisparityImage()
        msg.header = header
        msg.image = float_image(header, disparity)
        msg.f, msg.t = float(geometry.fx), float(geometry.baseline)
        x, y, w, h = valid_window(geometry.width, geometry.height, self.config)
        msg.valid_window.x_offset, msg.valid_window.y_offset = x, y
        msg.valid_window.width, msg.valid_window.height = w, h
        msg.min_disparity = float(self.config['min_disparity'])
        msg.max_disparity = float(self.config['min_disparity'] + self.config['num_disparities'])
        msg.delta_d = 1.0 / 16.0
        self.disparity_pub.publish(msg)
        self.depth_pub.publish(float_image(header, depth))
        self.depth_info_pub.publish(left_info)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = 1, len(points)
        cloud.fields = CLOUD_FIELDS
        cloud.is_bigendian = False
        cloud.point_step = POINT_STEP
        cloud.row_step = POINT_STEP * len(points)
        cloud.is_dense = True   # invalid pixels are omitted, never NaN points
        cloud.data = byte_field(pack_xyzrgb(points, colors))
        self.cloud_pub.publish(cloud)


def main(args=None):
    run_until_signal(StereoProcessor, args)
