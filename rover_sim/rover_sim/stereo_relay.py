"""Republish the Fortress stereo streams under /stereo/{left,right}/ with exact camera_info.

Gazebo Fortress publishes each camera on /stereo/gazebo/<side>/{image_raw,
camera_info} (bridged by ros_gz_bridge). Frames and stamps are already right
(the sensors set ignition_frame_id; both are rendered in one pass, so a pair
shares one stamp), but camera_info puts the principal point at (w/2, h/2),
half a pixel off the symmetric frustum's centre ((w-1)/2, (h-1)/2). The relay
rewrites K and P (principal point; Tx = 0 left, -fx * baseline right, with the
baseline read from robot_description) and passes images through untouched.
Logic: rover_sim.stereo_geometry.
"""
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from rover_sim.node_runner import run_until_signal
from rover_sim.stereo_geometry import LEFT_FRAME, RIGHT_FRAME, baseline_from_urdf, corrected_camera_info

# Reliable, keep last 5 (as the bridge publishes). Best-effort readers lost
# about half of the 500 KB frames in testing.
STREAM_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
SIDES = {'left': LEFT_FRAME, 'right': RIGHT_FRAME}


class StereoCameraRelay(Node):
    def __init__(self):
        super().__init__('stereo_camera_relay')
        description = self.declare_parameter('robot_description', '',
                                             ParameterDescriptor(read_only=True)).value
        # Fails closed: no stereo frames in the description means no relay.
        self.baseline = baseline_from_urdf(description)
        self.image_pubs, self.info_pubs = {}, {}
        for side in SIDES:
            self.image_pubs[side] = self.create_publisher(Image, f'/stereo/{side}/image_raw', STREAM_QOS)
            self.info_pubs[side] = self.create_publisher(CameraInfo, f'/stereo/{side}/camera_info', STREAM_QOS)
            self.create_subscription(Image, f'/stereo/gazebo/{side}/image_raw',
                                     lambda msg, side=side: self.on_image(side, msg), STREAM_QOS)
            self.create_subscription(CameraInfo, f'/stereo/gazebo/{side}/camera_info',
                                     lambda msg, side=side: self.on_info(side, msg), STREAM_QOS)
        self.get_logger().info(f'Stereo relay ready; baseline {self.baseline:.4f} m from robot_description')

    def on_image(self, side, msg):
        msg.header.frame_id = SIDES[side]
        self.image_pubs[side].publish(msg)

    def on_info(self, side, msg):
        try:
            k, p = corrected_camera_info(msg.k, msg.p, msg.width, msg.height, self.baseline, side)
        except ValueError as error:
            self.get_logger().warning(f'Dropping {side} camera_info: {error}', throttle_duration_sec=5.0)
            return
        msg.k, msg.p = k, p
        msg.header.frame_id = SIDES[side]
        self.info_pubs[side].publish(msg)


def main(args=None):
    run_until_signal(StereoCameraRelay, args)
