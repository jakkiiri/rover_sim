import math

import numpy as np
import pytest

from rover_planner.astar import (OBSTACLE, GridAStar, footprint_blocked, footprint_kernel,
                                 nearest_valid_cell)

RES = 0.1


def search(occupancy, half_length=0.0, half_width=0.0, yaw=0.0):
    kernel = footprint_kernel(half_length, half_width, yaw, RES)
    blocked = footprint_blocked(occupancy, kernel)
    return GridAStar(occupancy, np.zeros(occupancy.shape, np.float32), blocked), blocked


def empty(n=40):
    return np.zeros((n, n), np.int8)


def is_connected(path):
    return all(max(abs(a[0] - b[0]), abs(a[1] - b[1])) == 1 for a, b in zip(path, path[1:]))


def test_straight_line_on_empty_grid():
    s, _ = search(empty())
    path = s.a_star((5, 20), (30, 20))
    assert path[0] == (5, 20) and path[-1] == (30, 20)
    assert is_connected(path)
    assert all(cell[1] == 20 for cell in path)


def test_start_equals_goal():
    s, _ = search(empty())
    assert s.a_star((7, 7), (7, 7)) == [(7, 7)]


def test_path_goes_around_a_wall_through_the_gap():
    occ = empty()
    occ[0:30, 20] = OBSTACLE  # wall at gx = 20 with a gap at gy >= 30
    s, blocked = search(occ)
    path = s.a_star((10, 10), (30, 10))
    assert path and is_connected(path)
    assert not any(blocked[gy, gx] for gx, gy in path)
    assert max(gy for _, gy in path) >= 30


def test_footprint_too_wide_for_gap_finds_no_path():
    occ = empty()
    occ[:, 20] = OBSTACLE
    occ[18:22, 20] = 0  # 0.4 m gap
    s, _ = search(occ)
    assert s.a_star((10, 20), (30, 20))  # a point robot fits
    s, _ = search(occ, half_length=0.3, half_width=0.3)  # 0.6 m wide rover does not
    assert s.a_star((10, 20), (30, 20)) == []


def test_small_obstacle_between_footprint_corners_is_caught():
    # The original only tested four corners; a 0.1 m rock under the middle of
    # a 1.2 m rover would have been missed.
    occ = empty()
    occ[20, 20] = OBSTACLE
    _, blocked = search(occ, half_length=0.6, half_width=0.5)
    assert blocked[20, 20]
    assert blocked[20, 20 + 6] and not blocked[20, 20 + 8]   # along x: half length
    assert blocked[20 + 5, 20] and not blocked[20 + 7, 20]   # along y: half width


def test_kernel_rotates_with_heading():
    k0 = footprint_kernel(0.6, 0.2, 0.0, RES)
    k90 = footprint_kernel(0.6, 0.2, math.pi / 2, RES)
    assert np.array_equal(k0.T, k90)
    assert k0.sum() > 0


def test_goal_in_obstacle_returns_no_path():
    occ = empty()
    occ[20, 30] = OBSTACLE
    s, _ = search(occ)
    assert s.a_star((5, 20), (30, 20)) == []


def test_enclosed_goal_returns_no_path():
    occ = empty()
    occ[15:26, 25] = occ[15:26, 35] = OBSTACLE
    occ[15, 25:36] = occ[25, 25:36] = OBSTACLE
    s, _ = search(occ)
    assert s.a_star((5, 5), (30, 20)) == []


def test_height_cost_copied_from_original():
    occ = empty(5)
    height = np.zeros((5, 5), np.float32)
    height[1, 2] = 0.5
    occ[3, 3] = OBSTACLE
    s = GridAStar(occ, height, np.zeros((5, 5), bool))
    assert s.height_cost((1, 1), (2, 1)) == pytest.approx(1.5)
    assert s.height_cost((1, 1), (1, 2)) == 1
    assert s.height_cost((2, 3), (3, 3)) == math.inf
    assert s.height_cost((3, 3), (2, 3)) == math.inf


def test_nearest_valid_cell_outside_grid():
    blocked = np.zeros((10, 10), bool)
    assert nearest_valid_cell(blocked, (25, 4)) == (9, 4)
    blocked[:, 9] = True
    assert nearest_valid_cell(blocked, (25, 4)) == (8, 4)
    assert nearest_valid_cell(np.ones((3, 3), bool), (1, 1)) is None


def test_circle_kernel_is_round_and_covers_every_heading():
    from rover_planner.astar import circle_kernel
    # The planner's disc is the half diagonal plus inflation (0.15 m >= one cell).
    k = circle_kernel(math.hypot(0.58, 0.51) + RES, RES)
    assert np.array_equal(k, k.T) and np.array_equal(k, k[::-1])
    for yaw in np.linspace(0, math.pi, 7):
        rect = footprint_kernel(0.58, 0.51, yaw, RES)
        pad = (k.shape[0] - rect.shape[0]) // 2
        big = np.zeros_like(k)
        big[pad:pad + rect.shape[0], pad:pad + rect.shape[1]] = rect
        assert not np.any(big & ~k.astype(bool))  # the rectangle never pokes out


def margin_setup():
    from rover_planner.astar import circle_kernel
    occ = empty(60)
    occ[25:35, 30] = OBSTACLE                     # short wall at x = 3.0 m
    blocked = footprint_blocked(occ, circle_kernel(0.9, RES))
    body = footprint_blocked(occ, footprint_kernel(0.58, 0.51, 0.0, RES))
    return GridAStar(occ, np.zeros(occ.shape, np.float32), blocked), blocked, body


def test_plan_steps_out_of_the_margin_when_the_body_is_clear():
    from rover_planner.astar import plan_from
    search, blocked, body = margin_setup()
    start = (22, 30)                              # 0.8 m from the wall: inside the 0.9 m margin
    assert blocked[30, 22] and not body[30, 22]
    cells, problem = plan_from(search, start, (50, 30), body)
    assert problem is None
    assert cells[0] == start and cells[-1] == (50, 30) and is_connected(cells)
    assert not any(body[y, x] for x, y in cells)
    first_free = next(i for i, (x, y) in enumerate(cells) if not blocked[y, x])
    assert all(not blocked[y, x] for x, y in cells[first_free:])  # only the step-out is in the margin


def test_plan_refuses_when_the_body_touches_an_obstacle():
    from rover_planner.astar import plan_from
    search, _, body = margin_setup()
    cells, problem = plan_from(search, (25, 30), (50, 30), body)   # nose 0.08 m from the wall
    assert cells == [] and 'touching' in problem


def test_plan_from_free_start_is_plain_a_star():
    from rover_planner.astar import plan_from
    search, _, body = margin_setup()
    cells, problem = plan_from(search, (10, 10), (50, 10), body)
    assert problem is None and cells == search.a_star((10, 10), (50, 10))


def test_line_cells():
    from rover_planner.astar import line_cells
    assert line_cells((0, 0), (3, 0)) == [(0, 0), (1, 0), (2, 0), (3, 0)]
    assert line_cells((0, 0), (2, 2)) == [(0, 0), (1, 1), (2, 2)]
    assert line_cells((4, 4), (4, 4)) == [(4, 4)]



def test_goal_beyond_grid_is_replaced_by_a_reachable_cell():
    from rover_planner.astar import nearest_valid_cell
    blocked = np.zeros((30, 30), bool)
    blocked[18:30, 8:22] = True           # a mesa's steep flanks, out to the grid edge ...
    blocked[20:28, 11:19] = False         # ... around a flat top nearer the goal
    goal = (15, 40)                       # beyond the top edge, straight past the mesa
    top = nearest_valid_cell(blocked, goal)
    assert 20 <= top[1] < 28 and 11 <= top[0] < 19   # old choice: on the unreachable top
    cell = nearest_valid_cell(blocked, goal, reachable_from=(15, 2))
    assert cell[1] == 29 and not (8 <= cell[0] < 22)    # top row, beside the mesa
    # A rover inside the margin still finds its own region.
    margin = blocked.copy()
    margin[0:4, 13:17] = True
    assert nearest_valid_cell(margin, goal, reachable_from=(15, 2))[1] == 29


def test_step_out_goes_straight_ahead_when_turning_would_touch():
    from rover_planner.astar import plan_from
    occ = np.zeros((40, 40), np.int8)
    blocked = np.zeros((40, 40), bool)
    blocked[15:26, 15:26] = True          # margin around the start
    body = np.zeros_like(blocked)
    turn = np.zeros_like(blocked)
    turn[18:23, 18:23] = True             # turning at the start would sweep a rock
    search = GridAStar(occ, np.zeros(occ.shape, np.float32), blocked)
    start, goal = (20, 20), (20, 2)       # goal is behind (south of) a rover facing north
    cells, problem = plan_from(search, start, goal, body)
    assert problem is None
    cells, problem = plan_from(search, start, goal, body, turn, heading=math.pi / 2)
    assert problem is None
    exit_cell = next(c for c in cells if not blocked[c[1], c[0]])
    assert exit_cell[1] > 20 and abs(exit_cell[0] - 20) <= 6   # stepped out northwards first
    blocked_ahead = blocked.copy()
    blocked_ahead[20:, :] = True
    search = GridAStar(occ, np.zeros(occ.shape, np.float32), blocked_ahead)
    cells, problem = plan_from(search, start, goal, body, turn, heading=math.pi / 2)
    assert cells == [] and 'ahead' in problem
