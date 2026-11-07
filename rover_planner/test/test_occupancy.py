import math

import numpy as np
import pytest

from rover_planner.astar import OBSTACLE
from rover_planner.occupancy import (GridWindow, ObstacleMemory, filter_obstacle_points,
                                     quaternion_matrix, transform_points)

FILTER = dict(z_min=0.06, z_max=1.5, max_range=6.0, half_length=0.58, half_width=0.51)


def test_quaternion_matrix_yaw_90():
    s = math.sqrt(0.5)
    r = quaternion_matrix(0, 0, s, s)
    assert r @ np.array([1.0, 0, 0]) == pytest.approx([0, 1, 0])


def test_transform_optical_frame_to_rover_frame():
    # base_link -> stereo_left_optical_frame from the running sim: optical z
    # (forward) becomes base x, optical x (right) becomes base -y.
    q = (-0.5, 0.5, -0.5, 0.5)
    pts = transform_points(np.array([[0.0, 0.0, 2.0], [1.0, 0.0, 0.0]]), (0.55, 0.06, -0.03), q)
    assert pts[0] == pytest.approx([2.55, 0.06, -0.03])
    assert pts[1] == pytest.approx([0.55, -0.94, -0.03])


def test_filter_height_band_range_nan_and_self():
    pts = np.array([
        [3.0, 0.0, 0.03],    # ground
        [3.0, 0.0, 0.3],     # obstacle
        [3.0, 0.0, 2.0],     # overhead
        [9.0, 0.0, 0.3],     # too far
        [0.2, 0.1, 0.3],     # on the rover itself
        [np.nan, 0.0, 0.3],  # invalid
    ])
    kept = filter_obstacle_points(pts, (0.5, 0.0, 0.3), (0.0, 0.0, 0.0), **FILTER)
    assert kept.tolist() == [[3.0, 0.0, 0.3]]


def test_self_filter_follows_heading():
    pts = np.array([[0.0, 0.55, 0.3]])  # beside the rover at yaw 0, within its length at yaw 90
    assert len(filter_obstacle_points(pts, (0, 0, 0.3), (0, 0, 0.0), **FILTER)) == 1
    assert len(filter_obstacle_points(pts, (0, 0, 0.3), (0, 0, math.pi / 2), **FILTER)) == 0


def occupied(mem, size=200):
    """Set of occupied (ix, iy) in a window from -10 m to +10 m."""
    occ, height = mem.grid(-size // 2, -size // 2, size, size)
    gy, gx = np.nonzero(occ == OBSTACLE)
    return {(int(x) - size // 2, int(y) - size // 2) for x, y in zip(gx, gy)}, height


def test_memory_needs_min_points():
    mem = ObstacleMemory(0.1, unconfirmed_s=2.0, min_points=3)
    mem.add(np.array([[1.05, 2.05, 0.2]] * 2 + [[3.05, 3.05, 0.4]] * 3), stamp_s=100.0)
    assert occupied(mem)[0] == {(30, 30)}


def test_unconfirmed_cells_expire_confirmed_cells_do_not():
    mem = ObstacleMemory(0.1, unconfirmed_s=2.0, min_points=1, confirm_hits=2)
    mem.add(np.array([[1.05, 0.05, 0.3], [3.05, 0.05, 0.3]]), 0.0)
    mem.add(np.array([[3.05, 0.05, 0.3]]), 0.2)  # only this one is confirmed
    mem.prune(1.9)
    assert set(mem.cells) == {(10, 0), (30, 0)}
    mem.prune(2.1)
    assert set(mem.cells) == {(30, 0)}
    mem.prune(3600.0)
    assert occupied(mem)[0] == {(30, 0)}


def test_cell_counts_only_after_confirm_hits_clouds():
    mem = ObstacleMemory(0.1, unconfirmed_s=10.0, min_points=1, confirm_hits=3)
    point = np.array([[1.05, 0.05, 0.3]])
    mem.add(point, 1.0)
    mem.add(point, 1.2)
    assert occupied(mem)[0] == set()  # a mismatch seen twice is not enough
    mem.add(point, 1.4)
    assert occupied(mem)[0] == {(10, 0)}


def test_memory_keeps_tallest_point():
    mem = ObstacleMemory(0.1, 10.0, 1)
    mem.add(np.array([[0.05, 0.05, 0.2], [0.06, 0.04, 0.7]]), 0.0)
    mem.add(np.array([[0.05, 0.05, 0.3]]), 0.2)
    cells, height = occupied(mem)
    assert cells == {(0, 0)}
    assert height.max() == pytest.approx(0.7)


def test_window_is_centred_and_cells_round_trip():
    w = GridWindow(1.23, -4.56, 0.1, 120)
    gx, gy = w.world_to_grid(1.23, -4.56)
    assert (gx, gy) == (60, 60)
    x, y = w.grid_to_world(gx, gy)
    assert abs(x - 1.23) <= 0.05 + 1e-9 and abs(y + 4.56) <= 0.05 + 1e-9
    assert w.origin_xy == pytest.approx((1.2 - 6.0, -4.6 - 6.0))


def test_memory_grid_places_cells_in_window():
    mem = ObstacleMemory(0.1, 10.0, 1)
    mem.add(np.array([[2.05, 0.05, 0.3], [50.0, 0.0, 0.3]]), 0.0)
    w = GridWindow(0.0, 0.0, 0.1, 120)
    occ, height = mem.grid(w.origin_ix, w.origin_iy, 120, 120)
    gx, gy = w.world_to_grid(2.05, 0.05)
    assert occ[gy, gx] == OBSTACLE and height[gy, gx] == pytest.approx(0.3)
    assert (occ == OBSTACLE).sum() == 1  # the far point is outside the window


def test_cells_spread_along_the_camera_ray_by_range():
    mem = ObstacleMemory(0.1, 10.0, 1, depth_error_k=0.0094)
    # Cell centre 5.05 m ahead of the camera: sigma = 0.0094 * 5.05^2 = 0.24 m, along x only.
    mem.add(np.array([[5.05, 0.05, 0.3]]), 0.0, sensor_xy=(0.0, 0.05))
    cells, height = occupied(mem)
    assert {iy for _, iy in cells} == {0}
    assert sorted(ix for ix, _ in cells) == [48, 49, 50, 51, 52]  # 4.81 .. 5.29 m
    assert height.max() == pytest.approx(0.3)


def test_near_cells_spread_less_than_far_cells():
    near = ObstacleMemory(0.1, 10.0, 1, depth_error_k=0.0087)
    far = ObstacleMemory(0.1, 10.0, 1, depth_error_k=0.0087)
    near.add(np.array([[1.05, 0.05, 0.3]]), 0.0)
    far.add(np.array([[5.55, 0.05, 0.3]]), 0.0)
    assert len(occupied(near)[0]) == 1
    assert len(occupied(far)[0]) >= 5


def test_ground_seen_in_a_cell_clears_it_after_clear_misses_clouds():
    mem = ObstacleMemory(0.1, 2.0, min_points=1, clear_misses=3)
    obstacle = np.array([[2.05, 0.05, 0.3]])
    ground = np.array([[2.05, 0.05, 0.0], [2.06, 0.04, 0.01]])
    mem.add(obstacle, 0.0)
    for t in (0.2, 0.4):
        mem.add(np.empty((0, 3)), t, ground_xyz=ground)
    assert occupied(mem)[0] == {(20, 0)}
    mem.add(obstacle, 0.6, ground_xyz=ground)  # seen again: the miss count restarts
    for t in (0.8, 1.0):
        mem.add(np.empty((0, 3)), t, ground_xyz=ground)
    assert occupied(mem)[0] == {(20, 0)}
    mem.add(np.empty((0, 3)), 1.2, ground_xyz=ground)
    assert occupied(mem)[0] == set()


def test_cells_out_of_view_are_kept():
    # A parked rover must not forget the rock beside it, however long it waits.
    mem = ObstacleMemory(0.1, unconfirmed_s=2.0, min_points=1)
    mem.add(np.array([[2.05, 0.05, 0.3]]), 0.0)
    for t in np.arange(0.2, 30.0, 0.2):  # ground seen elsewhere only
        mem.add(np.empty((0, 3)), t, ground_xyz=np.array([[5.0, 5.0, 0.0]]))
    mem.prune(600.0)
    assert occupied(mem)[0] == {(20, 0)}


def test_prune_drops_cells_far_from_the_rover():
    mem = ObstacleMemory(0.1, 2.0, 1)
    mem.add(np.array([[1.05, 0.05, 0.3], [9.05, 0.05, 0.3]]), 0.0)
    mem.prune(1.0, centre_xy=(0.0, 0.0), keep_radius=5.0)
    assert occupied(mem)[0] == {(10, 0)}


def test_ground_points_band_and_range():
    from rover_planner.occupancy import ground_points
    pts = np.array([[1.0, 0, 0.01], [1.0, 0, -0.03], [1.0, 0, 0.2], [9.0, 0, 0.0]])
    kept = ground_points(pts, (0.0, 0.0, 0.3), z_min=0.06, max_range=6.0)
    assert kept.tolist() == [[1.0, 0, 0.01], [1.0, 0, -0.03]]


def test_lidar_cells_use_a_constant_range_error_and_stay_tight():
    stereo = ObstacleMemory(0.1, 10.0, 1, depth_error_k=0.0087)
    lidar = ObstacleMemory(0.1, 10.0, 1, depth_error_k=0.0087)
    stereo.add(np.array([[5.55, 0.05, 0.3]]), 0.0)
    lidar.add(np.array([[5.55, 0.05, 0.3]]), 0.0, range_sigma=0.03)
    assert len(occupied(stereo)[0]) >= 5
    assert len(occupied(lidar)[0]) == 1
