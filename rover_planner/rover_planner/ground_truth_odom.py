"""Gazebo ground-truth pose -> /odom and the odom -> base_root TF.

rover_sim has no odometry or world frame, so the planner had nowhere fixed to
put goals and obstacles. The ground_truth plugin in proxy_rover.urdf.xacro
publishes the rover's exact pose (body base_root,
which Gazebo merges base_link into) in Gazebo's world frame on
/ground_truth/odom. This node republishes it as:
  /odom  nav_msgs/Odometry, frame 'odom', child 'base_root'
  TF     odom -> base_root (base_root -> base_link is a fixed identity joint)
'odom' is therefore Gazebo's world frame, where the ground is flat at z = 0.
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import tf2_ros


class GroundTruthOdom(Node):
    def __init__(self):
        super().__init__('ground_truth_odom')
        self.declare_parameter('input_topic', '/ground_truth/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_root')
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_pub = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Odometry, self.get_parameter('input_topic').value, self.on_odom, 10)
        self.got_first = False

    def on_odom(self, msg):
        if not self.got_first:
            self.got_first = True
            self.get_logger().info(f'Ground truth received; publishing /odom and '
                                   f'{self.odom_frame} -> {self.base_frame}')
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        self.odom_pub.publish(msg)

        tf = TransformStamped()
        tf.header = msg.header
        tf.child_frame_id = self.base_frame
        p = msg.pose.pose.position
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = p.x, p.y, p.z
        tf.transform.rotation = msg.pose.pose.orientation
        self.tf_pub.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthOdom()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
