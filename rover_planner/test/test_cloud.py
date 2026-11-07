import math

import numpy as np
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from rover_planner.planner_node import cloud_to_xyz


def test_reads_stereo_layout_with_uint32_rgb():
    # Same layout as /stereo/points2: float32 x, y, z then uint32 rgb, 16 bytes.
    fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
    points = [(1.0, 2.0, 3.0, 0xFF0000), (math.nan, 0.0, 0.0, 0), (4.0, 5.0, 6.0, 0x00FF00)]
    msg = point_cloud2.create_cloud(Header(frame_id='stereo_left_optical_frame'), fields, points)
    assert msg.point_step == 16
    xyz = cloud_to_xyz(msg)
    assert xyz.dtype == np.float64
    assert xyz.tolist() == [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]]
