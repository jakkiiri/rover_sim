"""Republish the right Gazebo camera with its own optical frame and Tx = -fx * baseline.

libgazebo_ros_camera.so (multicamera mode) labels both cameras with one frame
and gives both the same Tx, so the plugin's right stream is remapped to
/stereo/gazebo/right/* and corrected here. Stamps are copied unchanged, so the
left/right pair keeps its single capture time. Logic: rover_sim.stereo_geometry.
"""
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from rover_sim.node_runner import run_until_signal
from rover_sim.stereo_geometry import RIGHT_FRAME, baseline_from_urdf, right_projection

# Same as the plugin's publishers: reliable, keep last 5, volatile. Reliable
# matters: best-effort readers lost about half of the 900 KB frames in testing.
STREAM_QOS = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)


class StereoCameraRelay(Node):
    def __init__(self):
        super().__init__('stereo_camera_relay')
        description = self.declare_parameter('robot_description', '',
                                             ParameterDescriptor(read_only=True)).value
        # Fails closed: no stereo frames in the description means no relay.
        self.baseline = baseline_from_urdf(description)
        self.image_pub = self.create_publisher(Image, '/stereo/right/image_raw', STREAM_QOS)
        self.info_pub = self.create_publisher(CameraInfo, '/stereo/right/camera_info', STREAM_QOS)
        self.create_subscription(Image, '/stereo/gazebo/right/image_raw', self.on_image, STREAM_QOS)
        self.create_subscription(CameraInfo, '/stereo/gazebo/right/camera_info', self.on_info, STREAM_QOS)
        self.get_logger().info(f'Right camera relay ready; baseline {self.baseline:.4f} m from robot_description')

    def on_image(self, msg):
        msg.header.frame_id = RIGHT_FRAME
        self.image_pub.publish(msg)

    def on_info(self, msg):
        try:
            p = right_projection(msg.p, self.baseline)
        except ValueError as error:
            self.get_logger().warning(f'Dropping right camera_info: {error}', throttle_duration_sec=5.0)
            return
        msg.p = p
        msg.header.frame_id = RIGHT_FRAME
        self.info_pub.publish(msg)


def main(args=None):
    run_until_signal(StereoCameraRelay, args)
