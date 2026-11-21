"""validate_planner checks the sensors the robot has, at their configured rates."""
from sensor_msgs.msg import Image, LaserScan, PointCloud2

from rover_planner.validate import expected_sensors


def sensors(lidar_topic, lidar_rate=5.0, stereo_rate=2.0):
    found = {'stereo_left': dict(type='camera', topic='stereo/gazebo/left/image_raw', rate=stereo_rate,
                                 frame='stereo_left_optical_frame'),
             'stereo_right': dict(type='camera', topic='stereo/gazebo/right/image_raw', rate=stereo_rate,
                                  frame='stereo_right_optical_frame')}
    if lidar_topic:
        found['lidar'] = dict(type='gpu_lidar', topic=lidar_topic, rate=lidar_rate, frame='lidar_link')
    return found


def test_3d_lidar_is_checked_on_points():
    assert expected_sensors(sensors('rover/lidar3d')) == {
        '/stereo/points2': (PointCloud2, 'stereo_left_optical_frame', 2.0),
        '/stereo/left/image_raw': (Image, 'stereo_left_optical_frame', 2.0),
        '/points': (PointCloud2, 'lidar_link', 5.0)}


def test_2d_lidar_is_checked_on_scan():
    assert expected_sensors(sensors('rover/scan', 10.0))['/scan'] == (LaserScan, 'lidar_link', 10.0)


def test_missing_sensors_are_not_checked():
    assert set(expected_sensors(sensors(None))) == {'/stereo/points2', '/stereo/left/image_raw'}
    assert expected_sensors({}) == {}
