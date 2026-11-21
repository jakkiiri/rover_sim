"""Survey reconstruction and collision-consistent spawn regression tests."""
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from rover_sim.survey_terrain import GroundGrid, SurveySurface, grid_height, grid_mesh, spawn_height

PACKAGE = Path(__file__).resolve().parents[1]


def test_streaming_plane_fits_node_not_biased_cell_mean():
    grid = GroundGrid(500000., 4250000., 4., (3, 3))
    # Deliberately asymmetric observations around each node.
    xx, yy = np.meshgrid(np.arange(3)*4., np.arange(3)*4.)
    x = (xx.ravel()[:, None]+[.1, .2, .4, .7, .3, .5, .6, .8]).ravel()
    y = (yy.ravel()[:, None]+[.2, .5, .1, .6, .7, .4, .3, .8]).ravel()
    z = 1400.+.1*x-.2*y
    for sl in (slice(0, 13), slice(13, None)):
        grid.add(500000.+x[sl], 4250000.+y[sl], z[sl], np.ones(len(x[sl]), bool))
    h, valid, stats = grid.finish()
    assert valid.all() and stats['filled_nodes'] == 0
    assert np.allclose(h, 1400.+.1*xx-.2*yy, atol=1e-7)


def test_ill_conditioned_sparse_plane_cannot_create_height_spikes():
    grid = GroundGrid(0, 0, 1., (2, 2))
    x = np.linspace(.1, .4, 8)
    y = x + np.array([0, 1e-7, 0, 0, 1e-7, 0, 0, 0])
    z = 1400. + np.array([0, .01, .01, 0, 0, .01, 0, .01])
    grid.add(x, y, z, np.ones(8, bool))
    h, _, _ = grid.finish()
    assert h[0, 0] == pytest.approx(z.mean())


def test_non_ground_is_coverage_but_never_ground_elevation():
    grid = GroundGrid(0, 0, 1., (3, 3))
    x, y = np.meshgrid(np.arange(3), np.arange(3))
    grid.add(np.r_[x.ravel(), 1], np.r_[y.ravel(), 1], np.r_[np.zeros(9), 99], np.r_[np.ones(9, bool), False])
    h, _, _ = grid.finish()
    assert np.all(h == 0)


def test_fill_small_ground_gap_but_never_unsurveyed_exterior():
    grid = GroundGrid(0, 0, 1., (7, 7))
    x, y = np.meshgrid(np.arange(1, 6), np.arange(1, 6))
    ground = ~((x.ravel() == 3) & (y.ravel() == 3))
    grid.add(x.ravel(), y.ravel(), np.full(25, 10.), ground)
    h, valid, stats = grid.finish(maximum_hole_metres=1.)
    assert h[3, 3] == 10 and stats['filled_nodes'] == 1
    assert not valid[0].any() and not valid[:, 0].any()
    assert np.isnan(h[0]).all()


def test_large_unsupported_ground_hole_is_not_bridged():
    grid = GroundGrid(0, 0, 1., (11, 11))
    x, y = np.meshgrid(np.arange(11), np.arange(11))
    ground = (x.ravel() == 0) | (y.ravel() == 0) | (x.ravel() == 10) | (y.ravel() == 10)
    grid.add(x.ravel(), y.ravel(), np.zeros(121), ground)
    _, valid, _ = grid.finish(maximum_hole_metres=2.)
    assert not valid[5, 5] and valid[1, 1]


def test_empty_ground_is_rejected():
    with pytest.raises(ValueError, match='ground'):
        GroundGrid(0, 0, 1, (2, 2)).finish()


@pytest.mark.parametrize('spacing', [0, -1, math.nan, math.inf])
def test_invalid_spacing_is_rejected(spacing):
    with pytest.raises(ValueError):
        GroundGrid(0, 0, spacing, (3, 3))


def test_exact_triangle_lookup_not_bilinear_saddle():
    h = np.array([[0., 0.], [0., 1.]])
    assert grid_height(h, (0, 0), 1., .25, .25) == 0
    assert grid_height(h, (0, 0), 1., .75, .75) == .5
    assert grid_height(h, (10, 20), 2., 12., 22.) == 1


@pytest.mark.parametrize('x,y', [(-1, 0), (0, 2), (math.inf, 0), (0, math.nan)])
def test_height_rejects_outside_and_nonfinite(x, y):
    with pytest.raises(ValueError):
        grid_height(np.zeros((2, 2)), (0, 0), 1., x, y)


def test_triangle_hole_cannot_provide_spawn_floor():
    h = np.array([[0., 0.], [math.nan, 1.]])
    with pytest.raises(ValueError, match='supported'):
        grid_height(h, (0, 0), 1., .3, .3)
    assert grid_height(h, (0, 0), 1., .5, 0) == 0


def test_mesh_winding_support_and_metres():
    h = np.array([[0., 0., math.nan], [0., 0., math.nan]])
    vertices, faces, _ = grid_mesh(h, 4.)
    assert len(faces) == 2 and len(vertices) == 4
    assert np.ptp(vertices[:, 0]) == 4 and np.ptp(vertices[:, 1]) == 4
    normals = np.cross(vertices[faces[:, 1]]-vertices[faces[:, 0]], vertices[faces[:, 2]]-vertices[faces[:, 0]])
    assert np.all(normals[:, 2] > 0)


@pytest.fixture(scope='module')
def survey():
    return SurveySurface(PACKAGE/'worlds/usgs.terrain.json')


def test_generated_data_includes_all_six_original_tiles(survey):
    inputs = survey.metadata['inputs']
    assert len(inputs) == 6
    assert sum(r['point_count'] for r in inputs) == 47893699
    assert len({r['filename'] for r in inputs}) == 6
    assert all(len(r['sha256']) == 64 and r['ground_points'] > 0 for r in inputs)
    assert survey.metadata['horizontal_epsg'] == 6341
    assert survey.metadata['source_bounds_m'][2]-survey.metadata['source_bounds_m'][0] == 3000


def test_default_spawn_preserves_real_terrain_height(survey):
    assert abs(survey.height(0, 0)) < 1e-4
    assert spawn_height('usgs', PACKAGE, 0, 0) == pytest.approx(.36, abs=1e-4)
    assert survey.metadata['processing']['vertical_exaggeration'] == 1
    assert survey.metadata['spawn']['maximum_nearby_coarse_slope_deg'] < 8


def test_fine_tile_boundary_matches_coarse_surface(survey):
    fine, coarse = survey.grids
    fs, fo, fh = fine
    cs, co, ch = coarse
    for j in range(fh.shape[0]):
        for i in (0, fh.shape[1]-1):
            if np.isfinite(fh[j, i]):
                assert fh[j, i] == pytest.approx(grid_height(ch, co, cs, fo[0]+i*fs, fo[1]+j*fs), abs=1e-4)
    for i in range(fh.shape[1]):
        for j in (0, fh.shape[0]-1):
            if np.isfinite(fh[j, i]):
                assert fh[j, i] == pytest.approx(grid_height(ch, co, cs, fo[0]+i*fs, fo[1]+j*fs), abs=1e-4)


def test_old_worlds_keep_existing_spawn_and_explicit_z(survey):
    assert spawn_height('mars', PACKAGE, 0, 0) == .36
    assert spawn_height('desert', PACKAGE, 0, 0) == .36
    assert spawn_height('usgs', PACKAGE, 99999, 99999, 4.) == 4
    with pytest.raises(ValueError):
        spawn_height('usgs', PACKAGE, 99999, 99999)
    with pytest.raises(ValueError):
        spawn_height('usgs', PACKAGE, 0, 0, 'nan')


def test_package_world_path_finds_same_spawn(survey):
    assert spawn_height(str(PACKAGE/'worlds/usgs.world'), PACKAGE, 0, 0) == pytest.approx(.36, abs=1e-4)


def test_exported_meshes_and_world_have_real_collisions(survey):
    import xml.etree.ElementTree as ET
    root = ET.parse(PACKAGE/'worlds/usgs.world').getroot()
    assert root.find('world').get('name') == 'usgs'
    assert root.findall('.//heightmap') == []
    # Tiles only; the synthetic rock fields (rover_sim/usgs_sites.py) follow them.
    models = [m for m in root.findall('world/model') if m.get('name').startswith('terrain_')]
    assert len(models) == len(survey.metadata['tiles'])
    assert len(models) > 10
    for model, tile in zip(models, survey.metadata['tiles']):
        assert model.get('name') == tile['name']
        assert model.findtext('static') == 'true'
        collision = model.findtext('link/collision/geometry/mesh/uri')
        assert collision == model.findtext('link/visual/geometry/mesh/uri')
        assert (PACKAGE/'models/usgs_terrain/meshes'/tile['mesh']).is_file()
    assert root.find("world/model[@name='sand_ground']") is None


def test_launch_defaults_to_imported_world_with_auto_height(monkeypatch):
    path = PACKAGE/'launch/rover_desert.launch.py'
    spec = importlib.util.spec_from_file_location('survey_rover_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name: str(PACKAGE))
    context = LaunchContext()
    description = module.generate_launch_description()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = entity.default_value[0].perform(context)
    assert context.launch_configurations['world'] == 'usgs'
    assert context.launch_configurations['z'] == 'auto'
    opaque = next(e for e in description.entities if isinstance(e, OpaqueFunction))
    assert opaque.execute(context)
