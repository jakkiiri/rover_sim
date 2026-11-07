import math

import numpy as np
import pytest

from rover_planner.terrain import (ElevationMap, TerrainModel, directional_slopes, fit_planes,
                                   height_sigma, robust_planes)

RES = 0.1


def ramp_points(angle_deg, x_range=(-3, 3), y_range=(-3, 3), step=0.05, foot=0.0):
    """Dense ground points: flat for x < foot, then a ramp rising along +x."""
    xs, ys = np.meshgrid(np.arange(*x_range, step), np.arange(*y_range, step))
    x, y = xs.ravel(), ys.ravel()
    z = np.where(x > foot, (x - foot) * math.tan(math.radians(angle_deg)), 0.0)
    return np.column_stack([x, y, z])


def model(points, size=80, slope_window=1.0, ground_window=2.0, noise=0.0, seed=0):
    rng = np.random.default_rng(seed)
    pts = points.copy()
    pts[:, 2] += rng.normal(0, noise, len(pts)) if noise else 0.0
    emap = ElevationMap(RES, 120, max_weight=1e9)
    emap.recentre(0.0, 0.0, margin=10)
    emap.add(pts, np.full(len(pts), 0.02), min_points=1)
    half = size // 2
    return TerrainModel(emap, -half, -half, size, size, slope_window=slope_window,
                        ground_window=ground_window, min_fraction=0.3, outlier=0.08)


def cell(x, y, half=40):
    return int(math.floor(y / RES)) + half, int(math.floor(x / RES)) + half  # [gy, gx]


@pytest.mark.parametrize('angle', [0, 10, 20, 30])
def test_slope_of_a_plane_is_measured(angle):
    t = model(ramp_points(angle, foot=-5))
    gy, gx = cell(0.5, 0.5)
    assert t.slope_ok[gy, gx]
    assert math.degrees(t.slope[gy, gx]) == pytest.approx(angle, abs=0.3)
    assert t.grad[gy, gx, 0] == pytest.approx(math.tan(math.radians(angle)), abs=0.01)
    assert abs(t.grad[gy, gx, 1]) < 0.01


def test_slope_with_stereo_like_noise():
    t = model(ramp_points(15, foot=-5), noise=0.02)
    gy, gx = cell(0.0, 0.0)
    assert math.degrees(t.slope[gy, gx]) == pytest.approx(15, abs=1.0)


def test_ramp_foot_is_gentle_far_away_and_steep_on_the_ramp():
    t = model(ramp_points(25, foot=0.0))
    assert math.degrees(t.slope[cell(-1.5, 0)]) < 1
    assert math.degrees(t.slope[cell(1.5, 0)]) == pytest.approx(25, abs=0.5)


def test_rock_does_not_tilt_the_ground_and_stands_out():
    pts = ramp_points(10, foot=-5)
    hill = lambda x: (x + 5) * math.tan(math.radians(10))  # noqa: E731
    rock = (np.abs(pts[:, 0] - 0.5) < 0.2) & (np.abs(pts[:, 1]) < 0.2)
    pts[rock, 2] += 0.3                           # a 0.3 m tall, 0.4 m wide rock on a 10 deg slope
    t = model(pts)
    gy, gx = cell(0.5, 0.0)
    assert t.ground[gy, gx] == pytest.approx(hill(0.55), abs=0.03)
    above = t.height_above_ground(np.array([[0.5, 0.0, hill(0.5) + 0.3], [2.0, 0.0, hill(2.0)]]))
    assert above[0] == pytest.approx(0.3, abs=0.03)
    assert abs(above[1]) < 0.03
    # The slope next to the rock is still the hill's.
    assert math.degrees(t.slope[cell(1.3, 0.0)]) == pytest.approx(10, abs=1.5)


def test_unknown_ground_gives_no_slope_and_nan_height():
    pts = ramp_points(0, x_range=(-3, 0))         # nothing seen at x > 0
    t = model(pts)
    assert not t.slope_ok[cell(2.0, 0.0)]
    assert np.isnan(t.height_above_ground(np.array([[2.0, 0.0, 0.5]])))[0]


def test_robust_refit_ignores_a_tall_block():
    h = np.zeros((30, 30))
    h[10:14, 10:14] = 1.0
    known = np.ones_like(h, bool)
    value, a, b, ok = fit_planes(h, known, 11, 0.3)
    assert value[15, 15] > 0.05                     # the block lifts the plain fit
    value, a, b, ok = robust_planes(h, known, 11, 0.3, outlier=0.08)
    assert abs(value[15, 15]) < 1e-6 and abs(a[15, 15]) < 1e-6


def test_elevation_recentre_keeps_overlap_and_forgets_the_rest():
    emap = ElevationMap(RES, 50, max_weight=1e9)
    emap.recentre(0.0, 0.0, margin=10)
    emap.add(np.array([[0.05, 0.05, 1.0], [-2.0, 0.05, 2.0]]), np.array([0.02, 0.02]), 1)
    emap.recentre(2.0, 0.0, margin=24)              # moves the square 2 m east
    h, known = emap.window(0, 0, 1, 1)
    assert known[0, 0] and h[0, 0] == pytest.approx(1.0)
    h, known = emap.window(-20, 0, 1, 1)
    assert not known[0, 0]                          # fell off the west edge


def test_elevation_blends_by_weight():
    emap = ElevationMap(RES, 20, max_weight=1e9)
    emap.recentre(0.0, 0.0, margin=2)
    p = np.array([[0.05, 0.05, 0.0]])
    emap.add(p, np.array([0.02]), 1)
    emap.add(p + [0, 0, 1.0], np.array([0.02]), 1)   # equally good: halfway
    assert emap.window(0, 0, 1, 1)[0][0, 0] == pytest.approx(0.5)
    emap.add(p + [0, 0, 10.0], np.array([2.0]), 1)   # far worse measurement barely moves it
    assert emap.window(0, 0, 1, 1)[0][0, 0] == pytest.approx(0.5, abs=0.01)


def test_height_sigma_grows_with_range():
    sensor = (0.0, 0.0, 0.6)
    near, far = height_sigma(np.array([[1.0, 0, 0], [5.0, 0, 0]]), sensor, 0.0087, 0.02)
    assert near == pytest.approx(0.02, abs=0.002)
    assert far > near


def test_directional_slopes():
    grad = np.array([[math.tan(math.radians(20)), 0.0]])
    pitch, roll = directional_slopes(grad, [(1, 0), (0, 1), (-1, 0)])
    assert math.degrees(pitch[0, 0]) == pytest.approx(20)
    assert math.degrees(roll[1, 0]) == pytest.approx(20)
    assert math.degrees(pitch[2, 0]) == pytest.approx(-20)
    assert abs(roll[0, 0]) < 1e-9


def test_ground_looked_at_but_never_measured_is_a_drop_off():
    # Ground measured up to x = 1.0 m; beyond it the camera (at x = 0.2) sees nothing.
    pts = ramp_points(0, x_range=(-3, 1.0))
    emap = ElevationMap(RES, 120, max_weight=1e9)
    emap.recentre(0.0, 0.0, margin=10)
    emap.add(pts, np.full(len(pts), 0.02), min_points=1)
    for _ in range(3):
        emap.mark_viewed((0.2, 0.0), 0.0, 0.45, 1.6, math.radians(-40), math.radians(29))
    t = TerrainModel(emap, -40, -40, 80, 80, slope_window=1.0, ground_window=2.0,
                     min_fraction=0.3, outlier=0.08, unseen_views=3)
    assert t.unseen[cell(1.5, 0.0)]            # looked at, nothing there: drop-off
    assert not t.unseen[cell(0.8, 0.0)]        # measured ground
    assert not t.unseen[cell(1.5, 1.5)]        # outside the camera's view: just unknown
    assert not t.unseen[cell(2.5, 0.0)]        # beyond the checked range: just unknown


def test_drop_off_needs_enough_looks():
    pts = ramp_points(0, x_range=(-3, 1.0))
    emap = ElevationMap(RES, 120, max_weight=1e9)
    emap.recentre(0.0, 0.0, margin=10)
    emap.add(pts, np.full(len(pts), 0.02), min_points=1)
    emap.mark_viewed((0.2, 0.0), 0.0, 0.45, 1.6, -0.7, 0.5)
    t = TerrainModel(emap, -40, -40, 80, 80, slope_window=1.0, ground_window=2.0,
                     min_fraction=0.3, outlier=0.08, unseen_views=3)
    assert not t.unseen.any()
