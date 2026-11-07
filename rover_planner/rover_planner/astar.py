"""A* grid search, copied from the rsx-rover local planner.

Source: rsx-rover/rover/autonomy/scripts/local_astar.py, class
AstarObstacleAvoidance, methods height_cost, heuristic, reconstruct_path,
a_star, get_neighbors, is_pose_valid and transform_corners.

Kept as in the original:
  - 8-connected moves in the original neighbour order.
  - Step cost: 1.0 straight, 1.414 diagonal, plus height_cost (|dh| + 1, or
    infinity when either cell holds an obstacle).
  - Euclidean heuristic measured in cells.
  - The goal pre-check: a goal the rover cannot fit on returns no path.

Changed, and why:
  - No ROS here, so the search can be unit tested and reused by any node.
  - is_pose_valid tested the rover's four footprint corners for every
    neighbour. Corners alone miss obstacles smaller than the rover that sit
    between them, so the same "does the rover fit here at this heading" test
    is computed once per plan for the whole grid (footprint_blocked) and each
    neighbour check becomes a single lookup.
  - The abort flag and logging are replaced by return values.
  - Terrain (slope-aware planning): `height` is now the measured ground height,
    so height_cost charges the real climb of each step; cells whose height is
    unknown (NaN) add no climb. An optional per-cell `cost` (slope cost)
    lengthens every step into a cell: step cost x (1 + cost).
"""

import heapq
import math

import cv2
import numpy as np

# Occupancy value of a cell holding an obstacle (the original's obstacle_threshold).
OBSTACLE = 100

# Same order as the original get_neighbors.
NEIGHBOURS = ((-1, 0), (1, 0), (0, 1), (0, -1), (-1, -1), (-1, 1), (1, -1), (1, 1))


def footprint_kernel(half_length, half_width, yaw, resolution):
    """Grid mask of the cells the rover's rectangle covers at heading `yaw`.

    The rectangle is centred on the kernel's middle cell. Each half extent is
    grown by half a cell so that any obstacle cell overlapping the footprint
    counts, not only cells whose centre lies inside it.
    """
    hl = half_length + resolution / 2.0
    hw = half_width + resolution / 2.0
    n = int(math.ceil(math.hypot(hl, hw) / resolution))
    rows, cols = np.mgrid[-n:n + 1, -n:n + 1]
    x = cols * resolution
    y = rows * resolution
    c, s = math.cos(yaw), math.sin(yaw)
    # Rotate each world offset into the rover's own frame, then test the box.
    local_x = x * c + y * s
    local_y = -x * s + y * c
    inside = (np.abs(local_x) <= hl + 1e-9) & (np.abs(local_y) <= hw + 1e-9)
    return inside.astype(np.uint8)


def circle_kernel(radius, resolution):
    """Grid mask of a disc: the footprint of a rover that may face any way.

    The rover cannot strafe, so following a path means turning to face it; a
    disc around its corners (the rectangle's half diagonal) fits every heading.
    Grown by half a cell like footprint_kernel.
    """
    r = radius + resolution / 2.0
    n = int(math.ceil(r / resolution))
    rows, cols = np.mgrid[-n:n + 1, -n:n + 1]
    return (np.hypot(cols, rows) * resolution <= r + 1e-9).astype(np.uint8)


def footprint_blocked(occupancy, kernel):
    """True where the rover's footprint, centred on that cell, touches an obstacle.

    Dilating the obstacle mask by the footprint gives exactly that answer for
    every cell at once. Cells beyond the grid edge count as free, like the
    original, which skipped corners that fell outside its grid.
    """
    obstacles = (occupancy >= OBSTACLE).astype(np.uint8)
    return cv2.dilate(obstacles, kernel, borderType=cv2.BORDER_CONSTANT, borderValue=0) > 0


class GridAStar:
    """A* over one grid snapshot. Cells are (gx, gy); arrays are indexed [gy, gx]."""

    def __init__(self, occupancy, height, blocked, cost=None):
        self.occupancy = occupancy
        self.height = height
        self.blocked = blocked
        self.cost = cost
        self.h, self.w = occupancy.shape

    def in_grid(self, cell):
        return 0 <= cell[0] < self.w and 0 <= cell[1] < self.h

    def is_pose_valid(self, cell):
        """The rover fits on this cell (replaces the original corner test)."""
        return self.in_grid(cell) and not self.blocked[cell[1], cell[0]]

    def height_cost(self, current, neighbor):
        """Copied: infinite into or out of an obstacle cell, else height change + 1."""
        current_gx, current_gy = current
        neighbor_gx, neighbor_gy = neighbor
        if self.occupancy[current_gy, current_gx] >= OBSTACLE:
            return float('inf')
        if self.occupancy[neighbor_gy, neighbor_gx] >= OBSTACLE:
            return float('inf')
        height_diff = abs(float(self.height[current_gy, current_gx])
                          - float(self.height[neighbor_gy, neighbor_gx]))
        if math.isnan(height_diff):  # ground height not measured at one of the cells
            height_diff = 0.0
        return height_diff + 1

    @staticmethod
    def heuristic(node, goal):
        """Copied: Euclidean distance in cells."""
        return math.hypot(node[0] - goal[0], node[1] - goal[1])

    @staticmethod
    def reconstruct_path(came_from, current):
        """Copied: walk back from the goal, then reverse to get start -> goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def get_neighbors(self, node):
        """Copied: in-grid 8-neighbours where the rover fits."""
        gx, gy = node
        neighbors = []
        for dx, dy in NEIGHBOURS:
            cell = (gx + dx, gy + dy)
            if self.is_pose_valid(cell):
                neighbors.append(cell)
        return neighbors

    def a_star(self, start, goal):
        """Copied search loop. Returns a list of cells from start to goal, or []."""
        # Pre-check from the original: an invalid goal would make A* search the
        # whole grid before giving up.
        if not self.is_pose_valid(goal):
            return []

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        closed = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                if neighbor in closed:
                    continue
                is_diagonal = (abs(current[0] - neighbor[0]) == 1
                               and abs(current[1] - neighbor[1]) == 1)
                step_cost = 1.414 if is_diagonal else 1.0
                if self.cost is not None:  # slope cost: a step on a slope counts as longer
                    step_cost *= 1.0 + float(self.cost[neighbor[1], neighbor[0]])
                tentative_g = g_score[current] + self.height_cost(current, neighbor) + step_cost
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))
        return []


def nearest_valid_cell(blocked, cell, reachable_from=None, allowed=None):
    """The free cell closest to `cell` (which may lie outside the grid), or None.

    reachable_from: only cells connected to this cell through free cells count
    (if it is blocked itself, the region of the free cell nearest to it). A
    goal beyond the grid must not be replaced by a cell the rover cannot reach,
    such as the flat top of a mesa ringed by steep flanks.
    allowed: optional mask of cells that may be chosen.
    """
    free_mask = ~blocked if allowed is None else (~blocked & allowed)
    if reachable_from is not None:
        _, labels = cv2.connectedComponents((~blocked).astype(np.uint8), connectivity=8)
        seed = reachable_from
        if blocked[seed[1], seed[0]]:
            seed = nearest_valid_cell(blocked, seed)
            if seed is None:
                return None
        free_mask = free_mask & (labels == labels[seed[1], seed[0]])
    free = np.argwhere(free_mask)  # rows of (gy, gx)
    if free.size == 0:
        return None
    d2 = (free[:, 1] - cell[0]) ** 2 + (free[:, 0] - cell[1]) ** 2
    gy, gx = free[int(np.argmin(d2))]
    return int(gx), int(gy)


def line_cells(a, b):
    """Grid cells on the straight segment from cell a to cell b, both included."""
    n = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
    cells = []
    for i in range(n + 1):
        t = i / n if n else 0.0
        cell = (int(round(a[0] + (b[0] - a[0]) * t)), int(round(a[1] + (b[1] - a[1]) * t)))
        if not cells or cells[-1] != cell:
            cells.append(cell)
    return cells


def plan_from(search, start, goal, body_blocked, turn_blocked=None, heading=None, max_turn=0.5):
    """A* from the rover's cell, stepping out of the safety margin first if needed.

    Following a path that hugs the margin, or a newly seen obstacle, can leave
    the rover's centre just inside the margin (search.blocked), where the
    original search could never start. If the rover's real body
    (body_blocked: footprint without inflation) still touches nothing, the
    path begins with a straight step to the nearest free cell, every cell of
    which must keep the real body clear. The follower turns in place to face a
    step more than max_turn off the nose, sweeping the disc around the body's
    corners; where that disc touches something (turn_blocked at the start),
    only free cells within max_turn of the heading are used.
    Returns (cells, problem): problem is None or a reason there is no path.
    """
    if search.blocked[start[1], start[0]]:
        if body_blocked[start[1], start[0]]:
            return [], 'the rover is touching an obstacle'
        ahead = None
        if turn_blocked is not None and heading is not None and turn_blocked[start[1], start[0]]:
            rows, cols = np.mgrid[0:search.blocked.shape[0], 0:search.blocked.shape[1]]
            bearing = np.arctan2(rows - start[1], cols - start[0]) - heading
            ahead = np.abs(np.arctan2(np.sin(bearing), np.cos(bearing))) <= max_turn
        exit_cell = nearest_valid_cell(search.blocked, start, allowed=ahead)
        if exit_cell is None:
            return [], ('no free cell ahead to step out to (turning here would touch an obstacle)'
                        if ahead is not None else 'no free cell to step out to')
        step = line_cells(start, exit_cell)
        if any(body_blocked[y, x] for x, y in step):
            return [], 'the way out of the safety margin is blocked'
        rest = search.a_star(exit_cell, goal)
        return (step[:-1] + rest, None) if rest else ([], 'A* found no path to the goal')
    cells = search.a_star(start, goal)
    return cells, None if cells else 'A* found no path to the goal'
