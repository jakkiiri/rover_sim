"""3-D LiDAR description, IMU, mast geometry, post-processing and RViz checks."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml

from rover_sim.kinematics import modules_from_urdf
from rover_sim.lidar_postprocess import cloud_to_array, make_cloud, make_scan, process_cloud

PACKAGE = Path(__file__).resolve().parents[1]
ROBOT = str(PACKAGE / 'urdf' / 'mars_rover.urdf.xacro')
PROFILES = {'lite': (16, 450, 30.0), 'standard': (16, 900, 30.0), 'high': (32, 1024, 45.0)}


def expand(**mappings):
    return ET.fromstring(xacro.process_file(ROBOT, mappings={k: str(v) for k, v in mappings.items()}).toxml())


@pytest.fixture(scope='module')
def descriptions():
    return {enabled: expand(enable_lidar=str(enabled).lower()) for enabled in (True, False)}


@pytest.fixture(scope='module')
def rviz():
    return yaml.safe_load((PACKAGE / 'rviz' / 'rover_mars.rviz').read_text())['Visualization Manager']


def sensor_of(root):
    sensors = root.findall("gazebo[@reference='lidar_link']/sensor")
    assert len(sensors) == 1
    return sensors[0]


def test_lidar_enabled_by_default():
    root = expand()
    assert len(root.findall("link[@name='lidar_link']")) == 1
    assert len(root.findall("joint[@name='lidar_joint']")) == 1
    assert sensor_of(root).get('type') == 'ray'


def test_disabled_lidar_has_no_sensor_or_extra_tf_frame(descriptions):
    root = descriptions[False]
    assert root.find("link[@name='lidar_link']") is None
    assert root.find("joint[@name='lidar_joint']") is None
    assert root.findall("gazebo[@reference='lidar_link']") == []
    assert root.find(".//plugin[@filename='libgazebo_ros_ray_sensor.so']") is None
    assert root.find(".//plugin[@filename='librover_swerve_actuators.so']") is not None
    assert root.find(".//plugin[@filename='libgazebo_ros_imu_sensor.so']") is not None


@pytest.mark.parametrize('profile', list(PROFILES))
def test_profiles_set_channels_samples_and_fov(profile):
    channels, samples, fov = PROFILES[profile]
    sensor = sensor_of(expand(lidar_profile=profile))
    horizontal, vertical = sensor.find('ray/scan/horizontal'), sensor.find('ray/scan/vertical')
    assert int(horizontal.findtext('samples')) == samples
    assert int(vertical.findtext('samples')) == channels
    assert (float(horizontal.findtext('min_angle')), float(horizontal.findtext('max_angle'))) == pytest.approx((-math.pi, math.pi))
    assert math.degrees(float(vertical.findtext('max_angle')) - float(vertical.findtext('min_angle'))) == pytest.approx(fov)
    assert float(sensor.findtext('update_rate')) == 10


@pytest.mark.parametrize('gpu,expected', [('true', 'gpu_ray'), ('false', 'ray'), ('auto', 'ray')])
def test_gpu_selection(gpu, expected):
    assert sensor_of(expand(lidar_gpu=gpu)).get('type') == expected


def test_range_noise_and_plugin_interface(descriptions):
    sensor = sensor_of(descriptions[True])
    limits = sensor.find('ray/range')
    assert 0.3 <= float(limits.findtext('min')) <= 0.9
    assert float(limits.findtext('max')) == pytest.approx(100.0)
    noise = sensor.find('ray/noise')
    assert noise.findtext('type') == 'gaussian' and float(noise.findtext('mean')) == 0
    assert 0.01 <= float(noise.findtext('stddev')) <= 0.03
    plugin = sensor.find('plugin')
    assert plugin.get('filename') == 'libgazebo_ros_ray_sensor.so'
    assert plugin.findtext('output_type') == 'sensor_msgs/PointCloud2'
    assert plugin.findtext('frame_name') == 'lidar_link'
    assert '~/out:=/points_raw' in [r.text.strip() for r in plugin.findall('ros/remapping')]


def test_mast_mount_clears_the_body_and_sees_the_ground(descriptions):
    root = descriptions[True]
    joint = root.find("joint[@name='lidar_joint']")
    assert joint.get('type') == 'fixed' and joint.find('parent').get('link') == 'base_link'
    xyz = [float(v) for v in joint.find('origin').get('xyz').split()]
    body = root.find("link[@name='base_link']/collision/geometry/box")
    length, width, height = (float(v) for v in body.get('size').split())
    deck_top = height / 2 + 0.02
    assert xyz[2] - deck_top >= 0.5                     # 0.5-0.8 m above the deck
    assert xyz[2] - deck_top <= 0.85
    assert abs(xyz[1]) < 1e-9 and 0 < xyz[0] < length / 2   # front-centre
    # The -15 deg beam must clear the deck corner furthest from the mast.
    far_corner = math.hypot(length / 2 + 0.05 + xyz[0], width / 2 + 0.07)
    drop_at_corner = far_corner * math.tan(math.radians(15))
    assert xyz[2] - drop_at_corner > deck_top
    ground_height = xyz[2] + 0.41        # base_link rests 0.41 m above ground
    assert 3.0 <= ground_height / math.tan(math.radians(15)) <= 6.0


def test_mast_height_is_a_parameter():
    joint = expand(mast_height=0.8).find("joint[@name='lidar_joint']")
    assert float(joint.find('origin').get('xyz').split()[2]) == pytest.approx(0.11 + 0.02 + 0.8 + 0.04)


def test_sensor_mass_and_inertia_are_physical(descriptions):
    inertial = descriptions[True].find("link[@name='lidar_link']/inertial")
    mass = float(inertial.find('mass').get('value'))
    assert 0 < mass < 2
    inertia = {k: float(v) for k, v in inertial.find('inertia').attrib.items()}
    xx, yy, zz = inertia['ixx'], inertia['iyy'], inertia['izz']
    assert min(xx, yy, zz) > 0 and max(xx, yy, zz) <= (xx + yy + zz) / 2 + 1e-12


def test_imu_on_chassis_with_noise_and_bias(descriptions):
    root = descriptions[True]
    sensors = root.findall("gazebo[@reference='imu_link']/sensor")
    assert len(sensors) == 1 and sensors[0].get('type') == 'imu'
    assert float(sensors[0].findtext('update_rate')) >= 50
    plugin = sensors[0].find('plugin')
    assert plugin.get('filename') == 'libgazebo_ros_imu_sensor.so'
    assert '~/out:=/imu/data' in [r.text.strip() for r in plugin.findall('ros/remapping')]
    for axis in ('x', 'y', 'z'):
        for group in ('angular_velocity', 'linear_acceleration'):
            noise = sensors[0].find(f'imu/{group}/{axis}/noise')
            assert float(noise.findtext('stddev')) > 0 and float(noise.findtext('bias_mean')) > 0
    joint = root.find("joint[@name='imu_joint']")
    assert joint.get('type') == 'fixed' and joint.find('parent').get('link') == 'base_link'


def test_sensor_addition_preserves_swerve_geometry_and_actuators(descriptions):
    assert modules_from_urdf(ET.tostring(descriptions[False])) == modules_from_urdf(ET.tostring(descriptions[True]))
    moving = [j for j in descriptions[False].findall('joint') if j.get('type') != 'fixed']
    assert len(moving) == 12
    for joint in moving:
        counterpart = descriptions[True].find(f"joint[@name='{joint.get('name')}']")
        assert ET.tostring(counterpart) == ET.tostring(joint)


def test_tf_tree_is_connected_and_rooted(descriptions):
    root = descriptions[True]
    links = {link.get('name') for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    assert links - children == {'base_root'}
    assert {'base_link', 'lidar_link', 'imu_link'} <= children
    assert 'odom' not in links and 'map' not in links


# --- post-processing --------------------------------------------------------

def synthetic_cloud(count=2000, seed=3):
    rng = np.random.default_rng(seed)
    azimuth = rng.uniform(-math.pi, math.pi, count)
    elevation = np.deg2rad(rng.integers(0, 16, count) * 2.0 - 15.0)
    r = rng.uniform(1.0, 40.0, count)
    xyz = np.stack([r * np.cos(elevation) * np.cos(azimuth), r * np.cos(elevation) * np.sin(azimuth),
                    r * np.sin(elevation)], axis=1)
    points = np.zeros(count, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    points['x'], points['y'], points['z'] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    return points, r, elevation


def test_postprocess_adds_ring_time_intensity_and_dropout():
    points, r, elevation = synthetic_cloud()
    rng = np.random.default_rng(0)
    xyz, intensity, ring, time = process_cloud(points, rng, 16, math.radians(30), 10.0, 0.0, 0.0, 0.1, 0.0, 0.5, 100.0)
    assert 0.8 * len(points) < len(xyz) < 0.95 * len(points)     # ~10 % dropout
    assert ring.dtype == np.uint16 and ring.min() == 0 and ring.max() == 15
    assert np.all((time >= 0) & (time <= 0.1)) and time.dtype == np.float32
    assert np.all((intensity > 0) & (intensity <= 1))
    # Rings follow elevation; no noise was requested so geometry is exact.
    expected_ring = np.rint((np.arcsin(xyz[:, 2] / np.linalg.norm(xyz, axis=1)) + math.radians(15)) / math.radians(2))
    assert np.array_equal(ring, expected_ring.astype(np.uint16))


def test_postprocess_noise_is_range_dependent_and_seeded():
    points, r, _ = synthetic_cloud(count=20000)
    out_a = process_cloud(points, np.random.default_rng(5), 16, math.radians(30), 10.0, 0.01, 0.001, 0.0, 0.0, 0.5, 100.0)
    out_b = process_cloud(points, np.random.default_rng(5), 16, math.radians(30), 10.0, 0.01, 0.001, 0.0, 0.0, 0.5, 100.0)
    assert np.array_equal(out_a[0], out_b[0])
    noisy_r = np.linalg.norm(out_a[0], axis=1)
    near, far = r < 5.0, r > 30.0
    assert np.std(noisy_r[near] - r[near]) < np.std(noisy_r[far] - r[far])
    assert np.std(noisy_r[far] - r[far]) == pytest.approx(0.01 + 0.001 * r[far].mean(), rel=0.25)


def test_cloud_round_trip_and_scan():
    from std_msgs.msg import Header
    points, _, _ = synthetic_cloud(count=500)
    xyz, intensity, ring, time = process_cloud(points, np.random.default_rng(1), 16, math.radians(30), 10.0,
                                               0.0, 0.0, 0.0, 0.0, 0.5, 100.0)
    msg = make_cloud(Header(frame_id='lidar_link'), xyz, intensity, ring, time)
    assert [f.name for f in msg.fields] == ['x', 'y', 'z', 'intensity', 'ring', 'time']
    assert msg.point_step == 22 and msg.width == len(xyz) and len(msg.data) == 22 * len(xyz)
    back = cloud_to_array(msg)
    assert np.allclose(back['x'], xyz[:, 0]) and np.array_equal(back['ring'], ring)
    scan = make_scan(msg.header, xyz, ring, 8, 360, 0.5, 100.0)
    assert len(scan.ranges) == 360 and scan.angle_increment == pytest.approx(math.pi / 180)
    finite = [v for v in scan.ranges if math.isfinite(v)]
    assert 0 < len(finite) <= int(np.sum(ring == 8))


# --- RViz -------------------------------------------------------------------

def test_rviz_uses_odom_frame_and_visible_accumulating_cloud(rviz):
    assert rviz['Global Options']['Fixed Frame'] == 'odom'
    clouds = [d for d in rviz['Displays'] if d.get('Class') == 'rviz_default_plugins/PointCloud2' and d['Enabled']]
    assert len(clouds) == 1
    cloud = clouds[0]
    assert cloud['Topic']['Value'] == '/points'
    assert cloud['Topic']['Reliability Policy'] == 'Best Effort'
    assert cloud['Style'] in ('Boxes', 'Spheres')
    assert 0.04 <= cloud['Size (m)'] <= 0.06
    assert 2 <= cloud['Decay Time'] <= 5
    assert cloud['Color Transformer'] in ('AxisColor', 'Intensity')


def test_rviz_shows_tf_robot_and_odometry(rviz):
    classes = {d['Class'] for d in rviz['Displays'] if d.get('Enabled')}
    assert {'rviz_default_plugins/TF', 'rviz_default_plugins/RobotModel', 'rviz_default_plugins/Odometry'} <= classes
    odom = [d for d in rviz['Displays'] if d.get('Class') == 'rviz_default_plugins/Odometry']
    assert {d['Topic']['Value'] for d in odom} == {'/odom', '/ground_truth/odom'}
    model = next(d for d in rviz['Displays'] if d.get('Class') == 'rviz_default_plugins/RobotModel')
    assert model['Description Topic']['Durability Policy'] == 'Transient Local'
