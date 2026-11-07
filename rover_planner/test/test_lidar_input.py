"""The planner node takes the 3D LiDAR cloud: obstacles all around, no drop-off views."""
import math

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from rover_planner.planner_node import AStarPlanner

LIDAR_HEIGHT = 0.73  # lidar_link above the ground (0.40 m over base_link, 0.33 m up)
FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
          for i, n in enumerate(('x', 'y', 'z', 'intensity'))]


@pytest.fixture
def node():
    if not rclpy.ok():
        rclpy.init()
    planner = AStarPlanner()
    planner.pose = (0.0, 0.0, 0.0)
    yield planner
    planner.destroy_node()


def sweep(stamp_s, box=None):
    """Flat ground 3-6 m out (rings 5 cm apart), all around, plus an optional box (x, y, height)."""
    pts = []
    for r in np.arange(3.0, 6.0, 0.05):
        for a in np.linspace(-math.pi, math.pi, 720, endpoint=False):
            pts.append((r * math.cos(a), r * math.sin(a), -LIDAR_HEIGHT, 0.5))
    if box is not None:
        bx, by, h = box
        for dx in np.arange(-0.15, 0.16, 0.05):
            for dy in np.arange(-0.15, 0.16, 0.05):
                for z in np.arange(0.0, h, 0.05):
                    pts.append((bx + dx, by + dy, z - LIDAR_HEIGHT, 0.5))
    stamp = TimeMsg(sec=int(stamp_s), nanosec=int((stamp_s % 1) * 1e9))
    return point_cloud2.create_cloud(Header(frame_id='lidar_link', stamp=stamp), FIELDS, pts)


def give_tf(node, msg):
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = 'odom', 'lidar_link'
    tf.header.stamp = msg.header.stamp
    tf.transform.translation.z = LIDAR_HEIGHT
    tf.transform.rotation.w = 1.0
    node.tf_buffer.set_transform(tf, 'test')


def test_lidar_finds_an_obstacle_behind_the_rover(node):
    for i in range(4):
        msg = sweep(1.0 + 0.1 * i, box=(-4.0, 0.0, 0.4))   # behind: the camera never sees it
        give_tf(node, msg)
        node.terrain_cache = None
        node.on_lidar(msg)
    assert node.stats['lidar_clouds'] == 4
    confirmed = node.memory.confirmed()
    cells = np.array(list(confirmed))
    assert len(cells) and np.all(np.abs(cells[:, 0] * 0.1 + 0.05 + 4.0) < 0.3)
    assert node.elevation.viewed.max() == 0          # never used for drop-off detection


def test_lidar_ground_clears_a_stale_cell_behind_the_rover(node):
    node.memory.add(np.array([[-4.05, 0.05, 0.3]] * 2), 0.5, sensor_xy=(0.0, 0.0))
    for i in range(3):
        node.memory.add(np.array([[-4.05, 0.05, 0.3]] * 2), 0.6 + 0.1 * i, sensor_xy=(0.0, 0.0))
    assert node.memory.confirmed()
    for i in range(4):
        msg = sweep(2.0 + 0.1 * i)                     # now flat ground where the cell was
        give_tf(node, msg)
        node.terrain_cache = None
        node.on_lidar(msg)
    assert not node.memory.confirmed()


def test_lidar_clouds_are_rate_limited(node):
    for stamp in (3.0, 3.02, 3.05, 3.1):
        msg = sweep(stamp)
        give_tf(node, msg)
        node.on_lidar(msg)
    assert node.stats['lidar_clouds'] == 2             # 10 Hz: 3.0 and 3.1


def _see_box(node, box):
    for i in range(4):
        msg = sweep(5.0 + 0.1 * i, box=box)
        give_tf(node, msg)
        node.terrain_cache = None
        node.on_lidar(msg)
    assert node.memory.confirmed()


def _plan_status(node):
    sent = []
    node.status_pub.publish = lambda msg: sent.append(msg.data)
    paths = []
    node.path_pub.publish = lambda msg: paths.append(msg)
    node.plan()
    return sent[-1], paths[-1]


def test_goal_on_a_rock_goes_to_the_nearest_safe_point(node):
    _see_box(node, (4.0, 0.0, 0.4))
    node.goal = (4.0, 0.0)                               # clicked right on the rock
    status, path = _plan_status(node)
    assert status == 'path' and path.poses
    end = path.poses[-1].pose.position
    gap = math.hypot(end.x - 4.0, end.y)
    assert 0.5 < gap <= node.p['goal_snap_radius'] + 0.1   # stops short, clear of the rock


def test_goal_with_no_safe_point_nearby_stops_without_look_around(node):
    _see_box(node, (4.0, 0.0, 0.4))
    node.p['goal_snap_radius'] = 0.2
    node.goal = (4.0, 0.0)
    status, path = _plan_status(node)
    assert status == 'goal_blocked' and not path.poses


def test_safe_point_is_kept_and_reaching_it_ends_the_goal(node):
    _see_box(node, (4.0, 0.0, 0.4))
    node.goal = (4.0, 0.0)
    _plan_status(node)
    first = node.snapped_goal
    assert first is not None
    _plan_status(node)
    assert node.snapped_goal == first                    # not re-chosen every plan
    node.pose = (first[0], first[1], 0.0)
    status, _ = _plan_status(node)
    assert status == 'idle' and node.goal is None and node.snapped_goal is None
