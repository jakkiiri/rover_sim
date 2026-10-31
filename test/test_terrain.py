"""Terrain generator regressions: determinism, conventions, rocks, worlds."""
import json
import math
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from rover_sim.terrain import heightmap as hm
from rover_sim.terrain import mesh_visual as mv
from rover_sim.terrain import rocks as rk
from rover_sim.terrain import scatter as sc
from rover_sim.terrain import world as wd
from rover_sim.terrain.noise import GradientNoise, fbm, ridged
from rover_sim.terrain.pipeline import generate_all, load_manifest, read_height_field


@pytest.fixture(scope='module')
def small_terrain():
    return hm.generate_heightmap(hm.TerrainParams(seed=3, resolution=129, size=120.0, relief=12.0,
                                                  crater_count=3))


def test_generation_is_deterministic_for_a_seed():
    a = hm.generate_heightmap(hm.TerrainParams(seed=11, resolution=65))
    b = hm.generate_heightmap(hm.TerrainParams(seed=11, resolution=65))
    c = hm.generate_heightmap(hm.TerrainParams(seed=12, resolution=65))
    assert np.array_equal(a['height'], b['height'])
    assert not np.array_equal(a['height'], c['height'])
    assert a['craters'] == b['craters']


@pytest.mark.parametrize('resolution', [64, 100, 0, 2])
def test_resolution_must_be_power_of_two_plus_one(resolution):
    with pytest.raises(ValueError, match='2\\^n'):
        hm.generate_heightmap(hm.TerrainParams(resolution=resolution))


def test_noise_is_bounded_and_seeded():
    noise = GradientNoise(np.random.default_rng(1))
    x, y = np.meshgrid(np.linspace(0, 20, 200), np.linspace(0, 20, 200))
    values = noise(x, y)
    assert -1.5 < values.min() < -0.3 and 0.3 < values.max() < 1.5
    assert np.array_equal(values, GradientNoise(np.random.default_rng(1))(x, y))
    assert 0.0 <= ridged(noise, x, y).min() and ridged(noise, x, y).max() <= 1.0
    assert abs(fbm(noise, x, y).mean()) < 0.2


def test_minimum_height_is_zero_and_relief_matches(small_terrain):
    height = small_terrain['height']
    assert height.min() == pytest.approx(0.0)
    assert 8.0 < np.ptp(height) <= 12.0 + 1e-6


def test_spawn_pad_is_flat_and_garden_is_flat(small_terrain):
    params = small_terrain['params']
    height = small_terrain['height']
    X, Y = hm.grid_coordinates(params)
    pad = height[np.hypot(X, Y) <= params.pad_radius - 0.5]
    assert np.ptp(pad) < 1e-6
    garden = height[(X > params.garden_x[0] + 0.5) & (X < params.garden_x[1] - 0.5) & (np.abs(Y) < 2.0)]
    assert np.ptp(garden) < 1e-6
    assert float(pad[0]) == pytest.approx(small_terrain['pad_level'])


def test_sample_height_uses_gazebo_image_convention():
    # Row 0 is +Y (north); column 0 is -X (west). Height = 2x + y.
    n, size = 65, 64.0
    xs = np.linspace(-32, 32, n)
    ys = np.linspace(32, -32, n)
    X, Y = np.meshgrid(xs, ys)
    height = 2 * X + Y + 100.0
    assert hm.sample_height(height, size, 10.0, 5.0) == pytest.approx(125.0)
    assert hm.sample_height(height, size, -3.5, -20.25) == pytest.approx(100 - 7 - 20.25)
    assert hm.sample_height(height, size, 0.0, 32.0) == pytest.approx(132.0)   # top row is +Y
    normal = hm.surface_normal(height, size, 0.0, 0.0)
    assert normal == pytest.approx(np.array([-2.0, -1.0, 1.0]) / math.sqrt(6.0), abs=1e-6)


def test_slope_statistics_cover_target_envelope(small_terrain):
    stats = hm.slope_statistics(small_terrain['slope_deg'], small_terrain['flat_weight'])
    for key in ('max_slope_deg', 'mean_slope_deg', 'percent_above_15_deg', 'percent_above_25_deg',
                'percent_above_35_deg', 'percent_below_20_deg', 'pad_max_slope_deg'):
        assert key in stats
    assert stats['percent_below_20_deg'] > 40.0
    assert stats['max_slope_deg'] > 30.0
    assert stats['pad_max_slope_deg'] < 3.0


def test_crater_profile_has_bowl_rim_and_falloff():
    r = np.array([0.0, 5.0, 10.0, 12.0, 30.0])
    profile = hm.crater_profile(r, radius=10.0, depth=1.5)
    assert profile[0] == pytest.approx(-1.5)
    assert profile[0] < profile[1] < 0
    assert profile[2] > 0 and profile[3] > 0
    assert abs(profile[4]) < 0.05


def test_geotiff_round_trip_and_georeference(tmp_path):
    height = np.linspace(0, 10, 33 * 33).reshape(33, 33)
    path = tmp_path / 'h.tif'
    hm.write_geotiff(path, height, 120.0)
    image = Image.open(path)
    assert image.mode == 'F'
    assert np.allclose(np.array(image), height, atol=1e-5)
    tags = image.tag_v2
    scale = tags[33550]
    assert scale[0] == pytest.approx(120.0 / hm.METRES_PER_DEGREE / 32)
    assert 33922 in tags and 34735 in tags
    assert tuple(tags[34735][4:8]) == (1024, 0, 1, 2)   # geographic model type


def test_power_law_diameters_are_bounded_and_heavy_tailed():
    rng = np.random.default_rng(5)
    d = sc.sample_diameters(rng, 20000, 0.08, 1.2, 2.1)
    assert d.min() >= 0.08 and d.max() <= 1.2
    fraction_big = np.mean(d > 0.3)
    expected = ((0.3 / 0.08) ** -2.1 - (1.2 / 0.08) ** -2.1) / (1 - (1.2 / 0.08) ** -2.1)
    assert fraction_big == pytest.approx(expected, abs=0.01)


def test_rock_meshes_are_normalised_with_small_hulls():
    directions, faces = rk.icosphere(2)
    assert len(directions) == 162 and len(faces) == 320
    rng = np.random.default_rng(2)
    for style in rk.STYLES:
        vertices, faces = rk.make_rock(rng, style)
        assert max(np.ptp(vertices[:, 0]), np.ptp(vertices[:, 1])) == pytest.approx(1.0)
        hull_vertices, hull_faces = rk.convex_hull(vertices)
        assert len(hull_faces) <= 64
        centroid = hull_vertices.mean(axis=0)
        for a, b, c in hull_faces:
            normal = np.cross(hull_vertices[b] - hull_vertices[a], hull_vertices[c] - hull_vertices[a])
            assert np.dot(normal, hull_vertices[a] - centroid) > 0   # outward winding


def test_rock_placement_respects_pad_slopes_and_surface(small_terrain, tmp_path):
    library = rk.build_rock_library(tmp_path, seed=3, count=6)
    instances = sc.place_rocks(small_terrain, library, 150, seed=3)
    params = small_terrain['params']
    assert 100 <= len(instances) <= 150
    for inst in instances:
        assert math.hypot(inst['x'], inst['y']) > params.pad_radius
        assert hm.sample_slope(small_terrain['slope_deg'], params.size, inst['x'], inst['y']) <= 28.0
        ground = hm.sample_height(small_terrain['height'], params.size, inst['x'], inst['y'])
        rock = library[inst['mesh']]
        lowest = inst['z'] + inst['scale'][2] * rock['z_min']
        extent = inst['scale'][2] * (rock['z_max'] - rock['z_min'])
        assert lowest == pytest.approx(ground - inst['burial'] * extent)
        assert 0.10 <= inst['burial'] <= 0.35
    sdf = sc.chunk_models(instances, library, 'model://x', params.size)
    assert sdf.count('<model name="rocks_') <= 36
    assert sdf.count('<visual') == len(instances)
    assert sc.collision_mode(0.05) == 'none' and sc.collision_mode(0.2) == 'sphere' and sc.collision_mode(0.5) == 'hull'


def test_labeled_boulders_expose_their_nominal_height(small_terrain, tmp_path):
    library = rk.build_rock_library(tmp_path, seed=3, count=6)
    boulders = sc.labeled_boulders(small_terrain, library)
    assert [b['height'] for b in boulders] == list(sc.BOULDER_HEIGHTS)
    for b in boulders:
        rock = library[b['mesh']]
        top = b['z'] + b['scale'][2] * rock['z_max']
        assert top - b['z_ground'] == pytest.approx(b['height'])
        assert abs(b['y']) < small_terrain['params'].garden_half_width
    sdf = sc.boulder_models(boulders, library, 'model://x')
    assert sdf.count('<model name="boulder_') == 6 and 'boulder_30cm' in sdf


def test_world_overrides_gravity_and_shadows():
    text = wd.flat_world('t', [], gravity=3.71, shadows=False)
    assert '<gravity>0 0 -3.71</gravity>' in text and '<shadows>false</shadows>' in text
    changed = wd.override_world(text, gravity=9.81, shadows=True)
    assert '<gravity>0 0 -9.81</gravity>' in changed
    assert '<shadows>true</shadows>' in changed and '<cast_shadows>true</cast_shadows>' in changed
    assert 'libgazebo_ros_state.so' in changed


def test_step_and_ramp_worlds_are_laid_out_as_documented():
    steps = wd.step_test_world()
    for h, _ in wd.step_lanes():
        assert f'<model name="step_{int(round(h * 100)):02d}cm">' in steps
    assert len(set(y for _, y in wd.step_lanes())) == len(wd.STEP_HEIGHTS)
    for angle in wd.RAMP_ANGLES:
        (cx, cz, length, _, thickness, theta), _, top_height = wd.ramp_geometry(angle)
        face_centre = (cx - thickness / 2 * math.sin(theta), cz + thickness / 2 * math.cos(theta))
        start = (face_centre[0] - wd.RAMP_LENGTH / 2, face_centre[1] - wd.RAMP_LENGTH * math.tan(theta) / 2)
        assert start == pytest.approx((wd.RAMP_RUNUP, 0.0), abs=1e-9)
        assert top_height == pytest.approx(wd.RAMP_LENGTH * math.tan(theta))
        assert length == pytest.approx(wd.RAMP_LENGTH / math.cos(theta))
    assert wd.slope_test_world().count('<model name="ramp_') == 2 * len(wd.RAMP_ANGLES)


def test_pipeline_writes_manifest_worlds_and_hashes(tmp_path):
    manifest = generate_all(tmp_path, seed=4, resolution=65, texture_size=32,
                            rock_counts=dict(low=20, medium=40, high=60), log=lambda *a: None)
    assert manifest['seed'] == 4 and manifest['resolution'] == 65
    for name in ('mars_low', 'mars_medium', 'mars_high', 'step_test', 'slope_test', 'boulder_garden'):
        assert (tmp_path / 'worlds' / f'{name}.world').exists()
    assert (tmp_path / 'models' / 'mars_terrain' / 'heightmap.tif').exists()
    assert all(len(v) == 64 for v in manifest['files'].values())
    assert load_manifest(tmp_path)['slope_stats'] == manifest['slope_stats']
    height = read_height_field(tmp_path)
    assert height.shape == (65, 65) and height.min() == pytest.approx(0.0, abs=1e-6)
    assert manifest['spawn']['z'] == pytest.approx(manifest['pad']['z'])
    assert 'Golombek' in manifest['rock_size_law']['reference']
    world = (tmp_path / 'worlds' / 'mars_medium.world').read_text()
    assert 'model://mars_terrain/heightmap.tif' in world and '<size>' not in world.split('<heightmap>')[1].split('</heightmap>')[0]


def test_terrain_mesh_visual_and_albedo(tmp_path, small_terrain):
    height = small_terrain['height']
    size = small_terrain['params'].size
    vertices, faces = mv.write_terrain_mesh(tmp_path / 'terrain_visual.obj', height, size, step=4)
    n = (height.shape[0] + 3) // 4
    assert vertices == n * n and faces == 2 * (n - 1) ** 2
    text = (tmp_path / 'terrain_visual.obj').read_text()
    assert text.count('\nv ') == vertices and text.count('\nvt ') == vertices and text.count('\nf ') == faces
    assert 'mtllib terrain_visual.mtl' in text and (tmp_path / 'terrain_visual.mtl').read_text().strip().endswith('map_Kd terrain_albedo.png')
    first = [float(v) for v in text.split('\nv ')[1].split('\n')[0].split()]
    assert first[:2] == pytest.approx([-size / 2, size / 2]) and first[2] == pytest.approx(height[0, 0], abs=1e-3)
    weights = mv.bake_albedo(tmp_path / 'terrain_albedo.png', height, small_terrain['slope_deg'], seed=3, resolution=128)
    assert sum(weights.values()) == pytest.approx(1.0, abs=0.02) and weights['sand'] > 0.2
    image = Image.open(tmp_path / 'terrain_albedo.png')
    assert image.size == (128, 128) and image.mode == 'RGB'
    assert np.array(image).mean() > 60           # not a black texture


def test_world_uses_mesh_visual_and_heightfield_collision():
    text = wd.terrain_model(4.0, (3.0, 8.0))
    assert 'meshes/terrain_visual.obj' in text and text.count('<heightmap>') == 1   # collision only
    legacy = wd.terrain_model(4.0, (3.0, 8.0), visual='heightmap')
    assert legacy.count('<heightmap>') == 2 and '<blend>' in legacy
