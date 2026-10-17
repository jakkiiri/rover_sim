"""Check the LiDAR description and local RViz interface without a simulator."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

from rover_sim.kinematics import modules_from_urdf


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def descriptions():
    path = str(PACKAGE / 'urdf' / 'proxy_rover.urdf.xacro')
    return {
        enabled: ET.fromstring(xacro.process_file(
            path, mappings={'enable_lidar': str(enabled).lower()}).toxml())
        for enabled in (True, False)
    }


@pytest.fixture(scope='module')
def sensor(descriptions):
    sensors = descriptions[True].findall("gazebo[@reference='lidar_link']/sensor")
    assert len(sensors) == 1
    return sensors[0]


@pytest.fixture(scope='module')
def rviz():
    return yaml.safe_load((PACKAGE / 'rviz' / 'rover_lidar.rviz').read_text())[
        'Visualization Manager']


def test_lidar_enabled_by_default():
    path = str(PACKAGE / 'urdf' / 'proxy_rover.urdf.xacro')
    root = ET.fromstring(xacro.process_file(path).toxml())
    assert len(root.findall("link[@name='lidar_link']")) == 1
    assert len(root.findall("joint[@name='lidar_joint']")) == 1


def test_disabled_lidar_has_no_sensor_or_extra_tf_frame(descriptions):
    root = descriptions[False]
    assert root.find("link[@name='lidar_link']") is None
    assert root.find("joint[@name='lidar_joint']") is None
    assert root.findall('.//sensor') == []
    assert root.find(".//plugin[@filename='libgazebo_ros_ray_sensor.so']") is None
    assert root.find(".//plugin[@filename='librover_swerve_actuators.so']") is not None


def test_fixed_mount_is_above_chassis_with_ros_axes(descriptions):
    root = descriptions[True]
    joint = root.find("joint[@name='lidar_joint']")
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'base_link'
    assert joint.find('child').get('link') == 'lidar_link'
    origin = joint.find('origin')
    xyz = [float(value) for value in origin.get('xyz').split()]
    rpy = [float(value) for value in origin.get('rpy', '0 0 0').split()]
    assert xyz == pytest.approx([0, 0, 0.20])
    assert rpy == pytest.approx([0, 0, 0])
    chassis = root.find("link[@name='base_link']/collision/geometry/box")
    assert xyz[2] > float(chassis.get('size').split()[2]) / 2


def test_sensor_mass_and_inertia_are_physical(descriptions):
    inertial = descriptions[True].find("link[@name='lidar_link']/inertial")
    mass = float(inertial.find('mass').get('value'))
    assert math.isfinite(mass) and 0 < mass < 1
    inertia = {key: float(value) for key, value in inertial.find('inertia').attrib.items()}
    assert all(math.isfinite(value) for value in inertia.values())
    xx, yy, zz = (inertia[key] for key in ('ixx', 'iyy', 'izz'))
    xy, xz, yz = (inertia[key] for key in ('ixy', 'ixz', 'iyz'))
    assert min(xx, yy, zz) > 0
    assert xx * yy - xy * xy > 0
    determinant = xx * yy * zz + 2 * xy * xz * yz - xx * yz**2 - yy * xz**2 - zz * xy**2
    assert determinant > 0
    assert max(xx, yy, zz) <= (xx + yy + zz) / 2 + 1e-12


def test_cpu_ray_is_single_horizontal_360_degree_scan(sensor):
    assert sensor.get('type') == 'ray'
    horizontal = sensor.find('ray/scan/horizontal')
    samples = int(horizontal.findtext('samples'))
    resolution = float(horizontal.findtext('resolution'))
    minimum = float(horizontal.findtext('min_angle'))
    maximum = float(horizontal.findtext('max_angle'))
    assert samples == 720
    assert resolution == 1
    assert (minimum, maximum) == pytest.approx((-math.pi, math.pi))
    assert math.degrees((maximum - minimum) / (samples - 1)) == pytest.approx(360 / 719)
    vertical = sensor.find('ray/scan/vertical')
    if vertical is not None:
        assert int(vertical.findtext('samples')) == 1
        assert float(vertical.findtext('min_angle')) == 0
        assert float(vertical.findtext('max_angle')) == 0


def test_rate_and_range_are_explicit(sensor):
    assert sensor.findtext('always_on').lower() in ('true', '1')
    assert float(sensor.findtext('update_rate')) == 10
    limits = sensor.find('ray/range')
    assert float(limits.findtext('min')) == pytest.approx(0.12)
    assert float(limits.findtext('max')) == pytest.approx(30)
    assert 0 < float(limits.findtext('resolution')) <= 0.01


def test_range_noise_is_small_unbiased_gaussian(sensor):
    noise = sensor.find('ray/noise')
    assert noise.findtext('type') == 'gaussian'
    assert float(noise.findtext('mean')) == 0
    assert float(noise.findtext('stddev')) == pytest.approx(0.01)


def test_classic_plugin_publishes_required_ros_interface(sensor):
    plugins = sensor.findall('plugin')
    assert len(plugins) == 1
    plugin = plugins[0]
    assert plugin.get('filename') == 'libgazebo_ros_ray_sensor.so'
    assert plugin.findtext('output_type') == 'sensor_msgs/LaserScan'
    assert plugin.findtext('frame_name') == 'lidar_link'
    remappings = [element.text.strip() for element in plugin.findall('ros/remapping')]
    assert '~/out:=/scan' in remappings


def test_sensor_addition_preserves_actual_swerve_geometry(descriptions):
    before = modules_from_urdf(ET.tostring(descriptions[False]))
    after = modules_from_urdf(ET.tostring(descriptions[True]))
    assert after == before


def test_sensor_addition_preserves_moving_joints_and_actuator_plugin(descriptions):
    before, after = descriptions[False], descriptions[True]
    moving = [joint for joint in before.findall('joint') if joint.get('type') != 'fixed']
    assert len(moving) == 8
    for joint in moving:
        counterpart = after.find(f"joint[@name='{joint.get('name')}']")
        assert ET.tostring(counterpart) == ET.tostring(joint)
    path = ".//plugin[@filename='librover_swerve_actuators.so']"
    assert ET.tostring(after.find(path)) == ET.tostring(before.find(path))


def test_robot_tf_tree_remains_connected_without_odom(descriptions):
    root = descriptions[True]
    links = {link.get('name') for link in root.findall('link')}
    children = {joint.find('child').get('link') for joint in root.findall('joint')}
    assert links - children == {'base_root'}
    assert 'base_link' in children and 'lidar_link' in children
    assert 'odom' not in links and 'map' not in links
    assert root.find("joint[@name='base_root_joint']/child").get('link') == 'base_link'
    assert len([joint for joint in root.findall('joint')
                if joint.find('child').get('link') == 'lidar_link']) == 1


def test_rviz_uses_robot_local_frame(rviz):
    assert rviz['Global Options']['Fixed Frame'] == 'base_link'


def test_rviz_laserscan_uses_sensor_qos_and_current_scan_only(rviz):
    scans = [display for display in rviz['Displays']
             if display.get('Class') == 'rviz_default_plugins/LaserScan']
    assert len(scans) == 1
    scan = scans[0]
    assert scan['Enabled'] is True
    assert scan['Topic']['Value'] == '/scan'
    assert scan['Topic']['Reliability Policy'] == 'Best Effort'
    assert scan['Topic']['Durability Policy'] == 'Volatile'
    assert scan['Decay Time'] == 0
    assert scan.get('Use Fixed Frame', True) is True


def test_rviz_robot_model_receives_latched_description(rviz):
    models = [display for display in rviz['Displays']
              if display.get('Class') == 'rviz_default_plugins/RobotModel']
    assert len(models) == 1
    model = models[0]
    assert model['Enabled'] is True
    assert model['Description Source'] == 'Topic'
    assert model['Description Topic']['Value'] == '/robot_description'
    assert model['Description Topic']['Reliability Policy'] == 'Reliable'
    assert model['Description Topic']['Durability Policy'] == 'Transient Local'
