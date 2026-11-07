"""The stereo additions to desert.world must not disturb terrain, motion or LiDAR checks."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from rover_sim.stereo_truth import FEATURE_PREFIX, world_primitives

PACKAGE = Path(__file__).resolve().parents[1]
WORLD = PACKAGE / 'worlds' / 'desert.world'
# Original terrain, copied from the world before the stereo work.
HILLS = {'hill_west': ((-10.0, 3.0, -5.0), 6.0), 'hill_north': ((0.0, 13.0, -6.5), 8.0),
         'hill_east': ((12.0, 2.0, -8.0), 10.0)}
GROUND = ((0.0, 0.0, -0.25), (100.0, 100.0, 0.5))
LIDAR_PLANE_Z = 0.33 + 0.20        # base_link height on flat ground + LiDAR mount
MOTION_CLEARANCE = 3.5             # validators drive the rover centre up to 2.82 m from the origin
LIDAR_PROBES = [((5.25, 0.0, 1.0), (0.5, 2.0, 2.0)), ((0.0, 4.25, 1.0), (2.0, 0.5, 2.0))]


@pytest.fixture(scope='module')
def primitives():
    return world_primitives(WORLD)


@pytest.fixture(scope='module')
def features(primitives):
    found = [p for p in primitives if p[0].startswith(FEATURE_PREFIX)]
    assert len(found) >= 4
    return found


def footprint(kind, centre, dims):
    """Axis-aligned x/y bounds of a feature."""
    if kind == 'box':
        return [(centre[i] - dims[i] / 2, centre[i] + dims[i] / 2) for i in range(2)]
    return [(centre[i] - dims[0], centre[i] + dims[0]) for i in range(2)]


def height_range(kind, centre, dims):
    half = dims[2] / 2 if kind == 'box' else dims[1] / 2
    return centre[2] - half, centre[2] + half


def test_terrain_and_ground_collision_are_unchanged(primitives):
    by_name = {name: (kind, centre, dims) for name, kind, centre, dims in primitives}
    for name, (centre, radius) in HILLS.items():
        assert by_name[name] == ('sphere', centre, (radius,))
    assert by_name['sand_ground'] == ('box', *GROUND)
    root = ET.parse(WORLD).getroot()
    friction = root.find("world/model[@name='sand_ground']/link/collision/surface/friction/ode")
    assert (friction.findtext('mu'), friction.findtext('mu2')) == ('0.8', '0.8')
    assert root.findtext('world/physics/max_step_size') == '0.001'


def test_only_hills_are_spheres(primitives):
    # validate_lidar.world_spheres() treats every sphere collision as a hill.
    assert sorted(name for name, kind, _, _ in primitives if kind == 'sphere') == sorted(HILLS)


def test_features_rest_on_the_ground_below_the_lidar_plane(features):
    for name, kind, centre, dims in features:
        assert kind in ('box', 'cylinder'), name
        bottom, top = height_range(kind, centre, dims)
        assert bottom == pytest.approx(0.0), name
        assert top <= 0.35 < LIDAR_PLANE_Z - 0.15, name


def test_features_are_clear_of_spawn_and_validation_motion(features):
    for name, kind, centre, dims in features:
        (x0, x1), (y0, y1) = footprint(kind, centre, dims)
        nearest = math.hypot(min(max(0.0, x0), x1), min(max(0.0, y0), y1))
        assert nearest >= MOTION_CLEARANCE, name


def test_features_avoid_lidar_probe_volumes_and_hill_footprints(features):
    for name, kind, centre, dims in features:
        (x0, x1), (y0, y1) = footprint(kind, centre, dims)
        for probe_centre, size in LIDAR_PROBES:
            px0, px1 = probe_centre[0] - size[0] / 2, probe_centre[0] + size[0] / 2
            py0, py1 = probe_centre[1] - size[1] / 2, probe_centre[1] + size[1] / 2
            assert x1 < px0 or x0 > px1 or y1 < py0 or y0 > py1, name
        for hill, (hc, radius) in HILLS.items():
            ground_radius = math.sqrt(radius ** 2 - hc[2] ** 2)
            nearest = math.hypot(min(max(hc[0], x0), x1) - hc[0], min(max(hc[1], y0), y1) - hc[1])
            assert nearest > ground_radius + 0.1, (name, hill)


def test_features_are_in_front_of_the_rover_and_unique(features):
    names = [name for name, _, _, _ in features]
    assert len(names) == len(set(names))
    for name, kind, centre, dims in features:
        # In the default camera's 90 degree view from the spawn pose (camera ~0.55 m ahead).
        assert abs(centre[1]) < centre[0] - 0.55, name
        assert centre[0] <= 6.0, name


def test_ground_texture_is_visual_only():
    root = ET.parse(WORLD).getroot()
    ground = root.find("world/model[@name='sand_ground']/link")
    assert ground.find('visual/material/script/name').text == 'RoverSim/Sand'
    assert ground.find('collision/geometry/box/size').text == '100 100 0.5'
    for script in root.iter('script'):
        uri = script.findtext('uri')
        uris = [u.text for u in script.findall('uri')]
        assert uris in (['file://media/materials/scripts/gazebo.material'],
                        ['model://rover_sim_materials/materials/scripts',
                         'model://rover_sim_materials/materials/textures'])
    materials = PACKAGE / 'models' / 'rover_sim_materials' / 'materials'
    material = (materials / 'scripts' / 'rover_sim.material').read_text()
    assert 'material RoverSim/Sand' in material and 'material RoverSim/Rock' in material
    assert (materials / 'textures' / 'rover_sand.png').read_bytes()[:8] == b'\x89PNG\r\n\x1a\n'


def test_only_a_model_path_is_exported_for_gazebo():
    # A gazebo_media_path export makes gazebo_ros replace GAZEBO_RESOURCE_PATH,
    # which hid Gazebo's own materials and broke camera rendering.
    package = ET.parse(PACKAGE / 'package.xml').getroot()
    exports = package.findall('export/gazebo_ros')
    assert [dict(e.attrib) for e in exports] == [{'gazebo_model_path': '${prefix}/models'}]
