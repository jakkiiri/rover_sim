"""Tolerance rule for the Fortress gpu_lidar (rover_sim/gpu_lidar_check.py)."""
import json
import math

import numpy as np
import pytest

from rover_sim import gpu_lidar_check as g
from rover_sim.sim_services import description_sensors


def test_head_on_bounds_are_three_sigma_and_the_mesh_gap_only_reads_long():
    assert g.error_bounds(10.0, 1.0, 0.02, 0.01) == pytest.approx((-0.06, 0.06))
    assert g.error_bounds(10.0, 1.0, 0.02, 0.01, gap=0.05) == pytest.approx((-0.06, 0.11))


def test_bounds_grow_towards_grazing_by_the_angle_error():
    s = math.sin(math.radians(2))
    spread = 0.06 + 20 * math.radians(0.8) / math.tan(math.radians(2))
    assert g.error_bounds(20.0, s, 0.02, math.radians(0.8)) == pytest.approx((-spread, spread))
    assert all(math.isfinite(b) for b in g.error_bounds(20.0, 0.0, 0.02, 0.01))


def test_sphere_gap_of_a_16_segment_mesh():
    assert g.sphere_gap(10.0) == pytest.approx(10 * (1 - math.cos(math.pi / 16)))
    assert 0.19 < g.sphere_gap(10.0) < 0.2


def test_turned_moves_azimuth_and_elevation():
    assert g.turned((1.0, 0.0, 0.0), math.pi / 2, 0.0) == pytest.approx((0.0, 1.0, 0.0), abs=1e-12)
    assert g.turned((1.0, 0.0, 0.0), 0.0, math.radians(30)) == pytest.approx(
        (math.cos(math.radians(30)), 0.0, 0.5))


def test_edge_points_are_where_a_small_turn_changes_the_object():
    wall = lambda d: 'wall' if d[1] > 0 else 'ground'          # boundary at azimuth 0
    assert g.is_edge(g.turned((1, 0, 0), 0.001, 0), 0.01, wall)
    assert not g.is_edge(g.turned((1, 0, 0), 0.5, 0), 0.01, wall)


def test_box_normal_picks_the_nearest_face():
    assert g.box_normal((4.25, 0.1, 0.2), (4.5, 0, 0.175), (0.5, 0.5, 0.35)) == [-1.0, 0.0, 0.0]
    assert g.box_normal((4.5, 0.0, 0.35), (4.5, 0, 0.175), (0.5, 0.5, 0.35)) == [0.0, 0.0, 1.0]


def test_summary_needs_95_percent_within_and_a_bias_inside_the_range():
    good = g.summarize([0.01] * 100, [True] * 96 + [False] * 4, edges=2)
    assert good['passed'] and good['within_fraction'] == 0.96 and good['edge_points'] == 2
    assert not g.summarize([0.01] * 100, [True] * 94 + [False] * 6)['passed']
    assert not g.summarize([0.05] * 100, [True] * 100)['passed']             # 5 cm long on a plane
    assert g.summarize([0.05] * 100, [True] * 100, gap=0.02)['passed']       # allowed on a sphere
    assert not g.summarize([-0.05] * 100, [True] * 100, gap=0.2)['passed']   # never short
    assert not g.summarize([0.0] * 4, [True] * 4)['passed']                  # too few points
    assert not g.summarize([], [])['passed']


def test_summary_is_plain_json_with_numpy_inputs():
    json.dumps(g.summarize(list(np.array([0.01] * 10)), list(np.array([0.01] * 10) < 0.06)))


def test_description_gives_the_lidar_resolution_and_noise():
    urdf = ('<robot name="r"><gazebo><sensor name="l" type="gpu_lidar"><update_rate>5</update_rate>'
            '<topic>rover/lidar3d</topic><ignition_frame_id>lidar_link</ignition_frame_id><lidar><scan>'
            '<horizontal><samples>450</samples><min_angle>-3.141592653589793</min_angle>'
            '<max_angle>3.141592653589793</max_angle></horizontal></scan>'
            '<noise><type>gaussian</type><stddev>0.02</stddev></noise></lidar></sensor></gazebo></robot>')
    sensor = description_sensors(urdf)['l']
    assert sensor['samples'] == 450 and sensor['noise_stddev'] == 0.02
    assert sensor['horizontal_step'] == pytest.approx(2 * math.pi / 450)
