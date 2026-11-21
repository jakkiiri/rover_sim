"""Real rover model (urdf/real_rover.urdf.xacro + generated config/meshes) and its launch geometry."""
import importlib.util
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml

from rover_sim.kinematics import inverse, modules_from_urdf

PACKAGE = Path(__file__).resolve().parents[1]
GEOMETRY = yaml.safe_load((PACKAGE / 'config' / 'real_rover.yaml').read_text())


def launch_module():
    spec = importlib.util.spec_from_file_location('rover_desert_launch', PACKAGE / 'launch' / 'rover_desert.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REAL = launch_module().real_rover(GEOMETRY)


def expand(**mappings):
    return xacro.process_file(str(PACKAGE / 'urdf' / 'real_rover.urdf.xacro'), mappings=mappings).toxml()


@pytest.fixture(scope='module')
def root():
    return ET.fromstring(expand(lidar_min_beam_deg=f"{REAL['min_beam_deg']:.3f}"))


def joint(root, name):
    found = root.find(f"joint[@name='{name}']")
    assert found is not None, name
    return found


def xyz(element):
    return [float(v) for v in element.find('origin').get('xyz').split()]


def test_total_mass_is_the_measured_42_kg(root):
    total = sum(float(m.get('value')) for m in root.iter('mass'))
    assert total == pytest.approx(42.0, abs=1e-6)


def test_every_link_with_geometry_has_positive_inertia(root):
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        inertia = inertial.find('inertia')
        assert all(float(inertia.get(k)) > 0 for k in ('ixx', 'iyy', 'izz')), link.get('name')


def test_swerve_modules_hang_on_the_rockers_at_the_cad_wheel_positions(root):
    modules = {m.steering: m for m in modules_from_urdf(ET.tostring(root, encoding='unicode'))}
    assert sorted(modules) == ['front_left_kingpin', 'front_right_kingpin', 'rear_left_kingpin', 'rear_right_kingpin']
    for name, m in modules.items():
        side = 'left' if 'left' in name else 'right'
        assert joint(root, name).find('parent').get('link') == side + '_rocker'
        assert m.x == pytest.approx(GEOMETRY['wheel_x_front' if name.startswith('front') else 'wheel_x_rear'])
        assert m.y == pytest.approx(GEOMETRY['wheel_y'] * (1 if side == 'left' else -1))
        assert m.radius == pytest.approx(GEOMETRY['wheel_radius'])
    # Straight driving: every wheel points ahead at the same rolling speed.
    angles, speeds = inverse(list(modules.values()), 0.5, 0.0, 0.0, [0.0] * 4)
    assert all(abs(a) < 1e-9 for a in angles)
    assert len({round(abs(s), 9) for s in speeds}) == 1


def test_rockers_are_passive_pivots_across_the_rover(root):
    for side, sign in (('left', 1), ('right', -1)):
        j = joint(root, side + '_rocker_joint')
        assert j.get('type') == 'revolute'
        assert j.find('parent').get('link') == 'base_link'
        assert j.find('axis').get('xyz') == '0 1 0'
        assert xyz(j)[1] == pytest.approx(sign * GEOMETRY['rocker_y'])
        assert float(j.find('limit').get('lower')) < 0 < float(j.find('limit').get('upper'))
    plugin = root.find("gazebo/plugin[@filename='librover_swerve_gz_system.so']")
    assert plugin.get('name') == 'rover_sim::SwerveGzSystem'
    assert float(plugin.find('differential_stiffness').text) > 0
    assert float(plugin.find('differential_damping').text) > 0


def test_ground_truth_comes_from_the_swerve_system_not_classic_p3d(root):
    assert root.find(".//plugin[@filename='libgazebo_ros_p3d.so']") is None
    plugin = root.find("gazebo/plugin[@filename='librover_swerve_gz_system.so']")
    assert plugin.findtext('ground_truth') in ('true', '1')
    assert plugin.findtext('body_name') == 'base_root'


def test_wheels_rest_on_the_ground_plane_below_base_link():
    assert GEOMETRY['wheel_z'] - GEOMETRY['wheel_radius'] == pytest.approx(GEOMETRY['ground_z'], abs=1e-3)
    # The proxy spawns at 0.36 m and settles 0.345 m above the ground; the real rover too.
    assert -GEOMETRY['ground_z'] == pytest.approx(0.345, abs=0.01)


def test_lidar_sits_on_the_one_metre_pole_without_a_mast(root):
    pole = joint(root, 'lidar_pole_joint')
    top = xyz(pole)[2] + GEOMETRY['pole']['length']
    assert GEOMETRY['pole']['length'] == 1.0
    assert xyz(pole)[2] == pytest.approx(GEOMETRY['top_plate_z'])
    lidar = xyz(joint(root, 'lidar_joint'))
    assert lidar == pytest.approx(GEOMETRY['lidar_xyz'])
    assert lidar[:2] == pytest.approx(xyz(pole)[:2])
    assert 0 < lidar[2] - top < 0.05
    assert len(root.find("link[@name='lidar_link']").findall('visual')) == 1   # housing only
    # Clear of the science box (CAD x -0.20..0.03 = sim y): pole radius 0.02 at y 0.08.
    assert xyz(pole)[1] - GEOMETRY['pole']['radius'] > 0.03


def test_dome_beams_down_to_the_launch_cut_off(root):
    vertical = root.find("gazebo[@reference='lidar_link']/sensor/ray/scan/vertical")
    low = math.degrees(float(vertical.find('min_angle').text))
    step = 90.0 / 47
    assert int(vertical.find('samples').text) == 48 - round((low + 90) / step)
    assert low >= REAL['min_beam_deg'] > low - step


def test_self_filter_box_holds_the_rover_and_the_lowest_beam_reaches_past_it():
    xmin, xmax, ymin, ymax, zmin, zmax = REAL['self_box']
    lx, ly, lz = GEOMETRY['lidar_xyz']
    extent = GEOMETRY['extent']
    reach = GEOMETRY['wheel_y'] + GEOMETRY['wheel_radius']          # wheel steered sideways
    assert xmin + lx < extent['rear'] and xmax + lx > extent['front']
    assert ymin + ly < -reach and ymax + ly > reach
    assert zmin + lz < GEOMETRY['ground_z'] and zmax == 0.0
    # Lowest simulated beam: flat ground leaves the box before it on the nearest side.
    low = math.radians(-90 + math.ceil((REAL['min_beam_deg'] + 90) / (90 / 47) - 1e-9) * 90 / 47)
    height = lz - GEOMETRY['ground_z']
    nearest = min(-xmin, xmax, -ymin, ymax)
    assert height / math.tan(-low) > nearest


def test_meshes_exist_and_stay_small(root):
    total = 0
    for mesh in root.iter('mesh'):
        name = Path(mesh.get('filename')).name
        assert mesh.get('filename').startswith('file://')
        path = PACKAGE / 'meshes' / 'real_rover' / name
        assert path.is_file(), name
        with open(path, 'rb') as f:
            f.seek(80)
            total += struct.unpack('<I', f.read(4))[0]
    assert 50_000 < total < 250_000           # simplified from 4.2 M CAD triangles


def test_stereo_camera_faces_forward_from_the_body_front(root):
    camera = xyz(joint(root, 'stereo_joint'))
    assert 0.2 < camera[0] < GEOMETRY['extent']['front']
    assert GEOMETRY['ground_z'] + 0.3 < camera[2] < GEOMETRY['top_plate_z']


def test_launch_selects_the_robot():
    module = launch_module()
    assert module.robot_config('proxy', str(PACKAGE))['xacro'] == 'proxy_rover.urdf.xacro'
    real = module.robot_config('real', str(PACKAGE))
    assert real['xacro'] == 'real_rover.urdf.xacro'
    assert real['lidar_xyz'] == tuple(GEOMETRY['lidar_xyz'])
    assert module.lidar_keepout('3d', 'airy', real['lidar_xyz'])['lidar_keepout_band'] == 'false'
