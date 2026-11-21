"""Gazebo ground-truth pose -> /odom and the odom -> base_root TF.

rover_sim has no odometry or world frame, so the planner had nowhere fixed to
put goals and obstacles. The ground_truth plugin in proxy_rover.urdf.xacro
publishes the rover's exact pose (body base_root,
which Gazebo merges base_link into) in Gazebo's world frame on
/ground_truth/odom. This node republishes it as:
  /odom  nav_msgs/Odometry, frame 'odom', child 'base_root'
  TF     odom -> base_root (base_root -> base_link is a fixed identity joint)
'odom' is therefore Gazebo's world frame. On the USGS terrain the ground is
not at z = 0 (the rover can sit ~15 m below it), and RViz's "2D Goal Pose"
tool projects clicks onto z = 0 of its fixed frame, so it found no ground.
The node also publishes:
  TF     odom -> goal_plane, a pure z shift to the ground under the rover
         (base z + ground_z), re-levelled only when it moves more than
         plane_step so the RViz scene does not bob. RViz uses it as its fixed
         frame; goals arrive in goal_plane and the planner transforms them.
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
        self.declare_parameter('goal_plane_frame', 'goal_plane')
        self.declare_parameter('ground_z', -0.3457)   # m, ground below base_link (real_rover.yaml)
        self.declare_parameter('plane_step', 0.25)    # m, re-level threshold
        self.plane_frame = self.get_parameter('goal_plane_frame').value
        self.ground_z = float(self.get_parameter('ground_z').value)
        self.plane_step = float(self.get_parameter('plane_step').value)
        self.plane_z = None
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

        ground = p.z + self.ground_z
        if self.plane_z is None or abs(ground - self.plane_z) > self.plane_step:
            self.plane_z = ground
        plane = TransformStamped()
        plane.header = msg.header
        plane.child_frame_id = self.plane_frame
        plane.transform.translation.z = self.plane_z
        plane.transform.rotation.w = 1.0
        self.tf_pub.sendTransform([tf, plane])


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
