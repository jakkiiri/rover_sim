"""Ray casting and world parsing used for stereo ground truth."""
import math
from pathlib import Path
import random
import sys

import pytest

from rover_sim.stereo_truth import (
    cast, depth_tolerance, error_bins, expected_depth, optical_ray, ray_box, ray_cylinder,
    ray_sphere, world_primitives)

PACKAGE = Path(__file__).resolve().parents[1]
WORLD = PACKAGE / 'worlds' / 'desert.world'


def unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def test_sphere_and_box_match_the_lidar_validator():
    sys.path.insert(0, str(PACKAGE / 'scripts'))
    try:
        lidar = pytest.importorskip('validate_lidar')
    finally:
        sys.path.pop(0)
    rng = random.Random(3)
    for _ in range(300):
        origin = tuple(rng.uniform(-3, 3) for _ in range(3))
        direction = unit([rng.uniform(-1, 1) for _ in range(3)])
        centre = tuple(rng.uniform(-2, 2) for _ in range(3))
        assert ray_sphere(origin, direction, centre, 1.3) == lidar.ray_sphere(origin, direction, centre, 1.3)
        size = (0.5, 1.0, 1.5)
        assert ray_box(origin, direction, centre, size) == lidar.ray_box(origin, direction, centre, size)


def test_vertical_cylinder_side_cap_miss_and_inside():
    centre = (5.0, 0.0, 0.15)
    assert ray_cylinder((0, 0, 0.1), (1, 0, 0), centre, 0.2, 0.3) == pytest.approx(4.8)
    down = unit((0.0, 0.0, -1.0))
    assert ray_cylinder((5.0, 0.05, 2.0), down, centre, 0.2, 0.3) == pytest.approx(1.7)
    assert ray_cylinder((0, 0, 0.5), (1, 0, 0), centre, 0.2, 0.3) == math.inf   # passes above
    assert ray_cylinder((0, 1, 0.1), (1, 0, 0), centre, 0.2, 0.3) == math.inf   # passes beside
    assert ray_cylinder((5.0, 0.0, 0.1), (1, 0, 0), centre, 0.2, 0.3) == 0.0


def test_world_has_ground_hills_and_supported_features_only():
    primitives = world_primitives(WORLD)
    kinds = {name: kind for name, kind, _, _ in primitives}
    assert kinds['sand_ground'] == 'box'
    assert [n for n, k in kinds.items() if k == 'sphere'] == ['hill_west', 'hill_north', 'hill_east']


def test_unsupported_world_shapes_fail_closed(tmp_path):
    world = tmp_path / 'w.world'
    world.write_text('<sdf version="1.6"><world name="w"><model name="m"><static>true</static>'
                     '<pose>0 0 0 0 0 0.3</pose><link name="l"><collision name="c"><geometry>'
                     '<box><size>1 1 1</size></box></geometry></collision></link></model></world></sdf>')
    with pytest.raises(ValueError):
        world_primitives(world)
    world.write_text(world.read_text().replace('0 0 0 0 0 0.3', '0 0 0 0 0 0').replace(
        '<box><size>1 1 1</size></box>', '<mesh><uri>x.dae</uri></mesh>'))
    with pytest.raises(ValueError):
        world_primitives(world)


def test_optical_ray_axes():
    assert optical_ray(319.5, 239.5, 320, 320, 319.5, 239.5) == pytest.approx((0, 0, 1))
    x, y, z = optical_ray(639.5, 239.5, 320, 320, 319.5, 239.5)
    assert x > 0 and y == pytest.approx(0) and z > 0


def test_expected_depth_on_flat_ground():
    # Level camera 0.33 m above the ground looking along world +X.
    rotation = ((0, 0, 1), (-1, 0, 0), (0, -1, 0))   # optical -> world
    ground = [('sand_ground', 'box', (0.0, 0.0, -0.25), (100.0, 100.0, 0.5))]
    k = dict(fx=320.0, fy=320.0, cx=319.5, cy=239.5)
    v = 239.5 + 320.0 * 0.33 / 2.0   # pixel row that sees the ground 2 m ahead
    depth, name = expected_depth(319.5, v, k, (0.0, 0.0, 0.33), rotation, ground)
    assert name == 'sand_ground' and depth == pytest.approx(2.0)
    assert expected_depth(319.5, 100.0, k, (0.0, 0.0, 0.33), rotation, ground)[1] is None  # sky


def test_cast_picks_the_nearest_object():
    prims = [('far', 'box', (10, 0, 0), (1, 1, 1)), ('near', 'cylinder', (4, 0, 0), (0.3, 1))]
    assert cast((0, 0, 0), (1, 0, 0), prims) == (pytest.approx(3.7), 'near')


def test_error_bins_apply_the_preregistered_tolerance():
    assert depth_tolerance(2.0, 320, 0.12) == pytest.approx(4 / 38.4 * 0.5 + 0.02)
    samples = [(1.5, 1.51)] * 8 + [(1.5, None)] * 2 + [(5.5, 7.0)] * 10 + [(8.5, None)] * 5
    bins = error_bins(samples, [1, 2, 5, 6, 9], 320, 0.12)
    assert [b['passed'] for b in bins] == [True, False, False, False]
    assert bins[0]['valid_fraction'] == 0.8 and bins[0]['median_abs_error_m'] == pytest.approx(0.01)
    assert bins[1]['samples'] == 0
