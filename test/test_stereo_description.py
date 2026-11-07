"""Stereo camera description: frames, sensor, plugin, keep-out and non-interference."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro

from rover_sim.kinematics import modules_from_urdf
from rover_sim.stereo_geometry import (
    Keepout, OPTICAL_RPY, baseline_from_urdf, intrinsics, keepout_violations, link_transforms,
    stereo_visual_points)

PACKAGE = Path(__file__).resolve().parents[1]
XACRO = str(PACKAGE / 'urdf' / 'proxy_rover.urdf.xacro')
STEREO_LINKS = {'stereo_link', 'stereo_left_optical_frame', 'stereo_right_optical_frame'}


def expand(**mappings):
    return ET.fromstring(xacro.process_file(XACRO, mappings={k: str(v) for k, v in mappings.items()}).toxml())


@pytest.fixture(scope='module')
def combos():
    return {(stereo, lidar): expand(enable_stereo=str(stereo).lower(), enable_lidar=str(lidar).lower())
            for stereo in (True, False) for lidar in (True, False)}


@pytest.fixture(scope='module')
def robot(combos):
    return combos[(True, True)]


@pytest.fixture(scope='module')
def sensor(robot):
    sensors = robot.findall("gazebo[@reference='stereo_link']/sensor")
    assert len(sensors) == 1
    return sensors[0]


def xacro_defaults():
    root = ET.parse(XACRO).getroot()
    return {arg.get('name'): arg.get('default') for arg in root.iter('{http://ros.org/wiki/xacro}arg')}


def test_stereo_enabled_by_default_and_lidar_independent(combos):
    default = expand()
    assert {l.get('name') for l in default.findall('link')} >= STEREO_LINKS | {'lidar_link'}
    for (stereo, lidar), root in combos.items():
        links = {l.get('name') for l in root.findall('link')}
        assert (STEREO_LINKS <= links) == stereo
        if not stereo:
            assert not links & STEREO_LINKS
            assert root.find(".//plugin[@filename='libgazebo_ros_camera.so']") is None
        assert ('lidar_link' in links) == lidar
        types = sorted(s.get('type') for s in root.iter('sensor'))
        assert types == sorted((['multicamera'] if stereo else []) + (['ray'] if lidar else []))


def test_frames_are_fixed_with_ros_optical_axes(robot):
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    assert joints['stereo_link'].find('parent').get('link') == 'base_link'
    for frame in ('stereo_left_optical_frame', 'stereo_right_optical_frame'):
        joint = joints[frame]
        assert joint.get('type') == 'fixed' and joint.find('parent').get('link') == 'stereo_link'
        rpy = [float(v) for v in joint.find('origin').get('rpy').split()]
        assert rpy == pytest.approx(OPTICAL_RPY)
    assert all(joints[name].get('type') == 'fixed' for name in STEREO_LINKS)
    _, left = link_transforms(robot)['stereo_left_optical_frame']
    # optical z = base +x (forward), optical x = base -y (right), optical y = base -z (down)
    assert [row[2] for row in left] == pytest.approx([1, 0, 0], abs=1e-9)
    assert [row[0] for row in left] == pytest.approx([0, -1, 0], abs=1e-9)
    assert [row[1] for row in left] == pytest.approx([0, 0, -1], abs=1e-9)


@pytest.mark.parametrize('baseline', [0.08, 0.12, 0.25])
def test_baseline_arg_sets_frames_and_sensor_poses(baseline):
    root = expand(stereo_baseline=baseline)
    assert baseline_from_urdf(root) == pytest.approx(baseline)
    cameras = {c.get('name'): [float(v) for v in c.findtext('pose').split()]
               for c in root.findall("gazebo[@reference='stereo_link']/sensor/camera")}
    joints = {j.find('child').get('link'): [float(v) for v in j.find('origin').get('xyz').split()]
              for j in root.findall('joint')}
    assert cameras['left'][:3] == pytest.approx(joints['stereo_left_optical_frame'])
    assert cameras['right'][:3] == pytest.approx(joints['stereo_right_optical_frame'])
    assert cameras['left'][3:] == [0, 0, 0] and cameras['right'][3:] == [0, 0, 0]


def test_multicamera_sensor_configuration(sensor):
    assert sensor.get('type') == 'multicamera'
    assert sensor.findtext('always_on') == 'true' and float(sensor.findtext('update_rate')) == 15
    cameras = sensor.findall('camera')
    assert [c.get('name') for c in cameras] == ['left', 'right']
    for camera in cameras:
        assert float(camera.findtext('horizontal_fov')) == pytest.approx(math.pi / 2)
        assert (int(camera.findtext('image/width')), int(camera.findtext('image/height'))) == (480, 360)
        assert camera.findtext('image/format') == 'R8G8B8'
        assert float(camera.findtext('clip/near')) > 0
        assert camera.findtext('noise/type') == 'gaussian'


def test_plugin_namespace_frames_and_principal_point(sensor):
    plugins = sensor.findall('plugin')
    assert len(plugins) == 1 and plugins[0].get('filename') == 'libgazebo_ros_camera.so'
    plugin = plugins[0]
    assert plugin.findtext('camera_name') == 'stereo'
    assert plugin.findtext('ros/namespace') == '/'
    assert plugin.findtext('frame_name') == 'stereo_left_optical_frame'
    assert float(plugin.findtext('hack_baseline')) == 0
    remaps = sorted(r.text.strip() for r in plugin.findall('ros/remapping'))
    assert remaps == ['stereo/right/camera_info:=stereo/gazebo/right/camera_info',
                      'stereo/right/image_raw:=stereo/gazebo/right/image_raw']
    k = intrinsics(480, 360, math.pi / 2)
    assert (float(plugin.findtext('cx')), float(plugin.findtext('cy'))) == (k['cx'], k['cy'])


@pytest.mark.parametrize('args', [dict(stereo_width=320, stereo_height=240, stereo_rate=10, stereo_hfov=1.2)])
def test_resolution_rate_and_fov_args(args):
    sensor = expand(**args).find("gazebo[@reference='stereo_link']/sensor")
    assert float(sensor.findtext('update_rate')) == 10
    for camera in sensor.findall('camera'):
        assert (camera.findtext('image/width'), camera.findtext('image/height')) == ('320', '240')
        assert float(camera.findtext('horizontal_fov')) == pytest.approx(1.2)
    plugin = sensor.find('plugin')
    assert (float(plugin.findtext('cx')), float(plugin.findtext('cy'))) == (159.5, 119.5)


def test_stereo_links_are_visual_only_and_massless(robot):
    for link in robot.findall('link'):
        if link.get('name') in STEREO_LINKS:
            assert link.find('collision') is None
            assert link.find('inertial') is None
    assert robot.find("link[@name='stereo_link']/visual") is not None


def total_mass(root):
    return sum(float(link.find('inertial/mass').get('value')) for link in root.findall('link')
               if link.find('inertial') is not None)


def test_stereo_does_not_change_mass_geometry_joints_or_actuators(combos):
    for lidar in (True, False):
        on, off = combos[(True, lidar)], combos[(False, lidar)]
        assert total_mass(on) == total_mass(off)
        assert modules_from_urdf(ET.tostring(on)) == modules_from_urdf(ET.tostring(off))
        for joint in off.findall('joint'):
            twin = on.find(f"joint[@name='{joint.get('name')}']")
            assert ET.tostring(twin) == ET.tostring(joint)
        plugin = ".//plugin[@filename='librover_swerve_actuators.so']"
        assert ET.tostring(on.find(plugin)) == ET.tostring(off.find(plugin))


def test_lidar_description_is_byte_identical_with_stereo_on_or_off(combos):
    on, off = combos[(True, True)], combos[(False, True)]
    for path in ("link[@name='lidar_link']", "joint[@name='lidar_joint']", "gazebo[@reference='lidar_link']"):
        assert ET.tostring(on.find(path)) == ET.tostring(off.find(path))


def test_no_stereo_link_inside_default_lidar_keepout(robot):
    defaults = xacro_defaults()
    keepout = Keepout(origin=tuple(float(v) for v in defaults['lidar_keepout_origin'].split()),
                      vfov_min=float(defaults['lidar_keepout_vfov_min']),
                      vfov_max=float(defaults['lidar_keepout_vfov_max']),
                      margin=float(defaults['lidar_keepout_margin']),
                      radius=float(defaults['lidar_keepout_radius']))
    assert keepout == Keepout()   # Python defaults mirror the xacro defaults
    lidar = robot.find("joint[@name='lidar_joint']/origin").get('xyz').split()
    assert tuple(float(v) for v in lidar) == keepout.origin  # keep-out is centred on the actual LiDAR
    points = stereo_visual_points(robot)
    assert {name for name, _ in points} == STEREO_LINKS
    assert keepout_violations(robot, keepout) == []


@pytest.mark.parametrize('args', [
    dict(stereo_xyz='0.4 0 0.18'),                 # in the beam plane
    dict(stereo_xyz='0.05 0 0.2'),                 # inside the LiDAR housing zone
    dict(lidar_keepout_vfov_min=-30),              # wider 3D LiDAR band covers the default mount
    dict(stereo_xyz='0.52 0 0.19', stereo_rpy='0 0 0.4')])
def test_mount_inside_keepout_fails_expansion(args):
    with pytest.raises(xacro.XacroException, match='keep-out'):
        expand(**args)


def test_accepted_mounts_pass_the_exact_python_check():
    # The xacro corner check and the sampled-visual check must agree on accepted mounts.
    for args in (dict(), dict(stereo_baseline=0.3), dict(stereo_rpy='0 0.2 0'),
                 dict(stereo_xyz='0.52 0.1 -0.06')):
        root = expand(**args)
        assert keepout_violations(root, Keepout()) == []


def test_tf_tree_stays_connected_without_world_frames(combos):
    for root in combos.values():
        links = {link.get('name') for link in root.findall('link')}
        children = [joint.find('child').get('link') for joint in root.findall('joint')]
        assert links - set(children) == {'base_root'}
        assert len(children) == len(set(children))
        assert not links & {'odom', 'map', 'world'}
