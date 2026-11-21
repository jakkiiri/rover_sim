"""USGS spawn sites, the hills rock field and the GUI camera pose."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from rover_sim import usgs_sites
from rover_sim.survey_terrain import SurveySurface

PACKAGE = Path(__file__).resolve().parents[1]
SITES = usgs_sites.load_sites(PACKAGE / 'config/usgs_sites.yaml')


@pytest.fixture(scope='module')
def surface():
    return SurveySurface(PACKAGE / 'worlds/usgs.terrain.json')


def test_survey_world_defaults_to_the_hills_site():
    assert SITES['default_site'] == 'hills'
    hills = SITES['sites']['hills']
    assert usgs_sites.resolve_xy('usgs', '', '', '', SITES) == (hills['x'], hills['y'])
    assert usgs_sites.resolve_xy('usgs', 'flat', '', '', SITES) == (0.0, 0.0)


def test_explicit_coordinates_override_the_site_and_other_worlds_default_to_zero():
    assert usgs_sites.resolve_xy('usgs', 'hills', '5', '', SITES) == (5.0, SITES['sites']['hills']['y'])
    assert usgs_sites.resolve_xy('mars', '', '', '', SITES) == (0.0, 0.0)
    assert usgs_sites.resolve_xy('desert', 'hills', '1.5', '-2', SITES) == (1.5, -2.0)
    assert usgs_sites.resolve_xy('/tmp/x/usgs.world', 'flat', '', '', SITES) == (0.0, 0.0)


def test_unknown_site_and_non_finite_coordinates_are_rejected():
    with pytest.raises(ValueError, match='Unknown site'):
        usgs_sites.resolve_xy('usgs', 'moon', '', '', SITES)
    with pytest.raises(ValueError, match='finite'):
        usgs_sites.resolve_xy('usgs', '', 'nan', '', SITES)


@pytest.mark.parametrize('name', sorted(SITES['sites']))
def test_every_site_is_gentle_under_the_rover(surface, name):
    site = SITES['sites'][name]
    # Rover footprint is about 1 m x 1 m; sample its corners and centre.
    for dx, dy in [(0, 0), (-0.6, -0.6), (0.6, -0.6), (-0.6, 0.6), (0.6, 0.6)]:
        grade = usgs_sites.ground_slope(surface, site['x'] + dx, site['y'] + dy, 1.0)
        assert math.degrees(math.atan(grade)) < 6.0


def test_hills_site_has_driveable_and_lethal_slopes_nearby(surface):
    site = SITES['sites']['hills']
    angles = []
    for r in np.arange(4.0, 24.0, 2.0):
        for a in np.linspace(0, 2 * math.pi, 36, endpoint=False):
            grade = usgs_sites.ground_slope(surface, site['x'] + r * math.cos(a), site['y'] + r * math.sin(a), 2.0)
            angles.append(math.degrees(math.atan(grade)))
    angles = np.array(angles)
    assert np.mean((angles >= 8) & (angles < 25)) > 0.25
    assert np.any(angles >= 28)


def recorded_rocks():
    return json.loads((PACKAGE / 'worlds/usgs_rocks.json').read_text())['fields']


def test_world_contains_exactly_the_recorded_rocks():
    root = ET.parse(PACKAGE / 'worlds/usgs.world').getroot()
    names = [m.get('name') for m in root.iter('model') if not m.get('name').startswith('terrain_')]
    expected = [f"{f['site']}_{r['kind']}_{i:03d}" for f in recorded_rocks() for i, r in enumerate(f['rocks'])]
    assert names == expected and len(names) == len(expected) > 70


def test_rock_field_matches_its_config_and_regenerates_identically(surface):
    tops = [usgs_sites.mesh_top(PACKAGE / f'models/mars_rocks/meshes/rock_{k}.dae')
            for k in range(usgs_sites.ROCK_MESHES)]
    for field, recorded in zip(SITES['rock_fields'], recorded_rocks()):
        site = SITES['sites'][field['site']]
        rocks = usgs_sites.place_rocks(field, (site['x'], site['y']), surface, tops)
        assert rocks == recorded['rocks']
        counts = {kind: sum(r['kind'] == kind for r in rocks) for kind in field['groups']}
        assert counts == {kind: g[0] for kind, g in field['groups'].items()}
        for i, r in enumerate(rocks):
            d = math.hypot(r['x'] - site['x'], r['y'] - site['y'])
            assert field['clear'] + r['radius'] <= d <= field['radius']
            assert r['z'] < surface.height(r['x'], r['y'])          # base is buried
            assert 0 < r['top_above_ground'] < 1.0
            for o in rocks[:i]:
                assert math.hypot(r['x'] - o['x'], r['y'] - o['y']) >= r['radius'] + o['radius'] + 0.4


def test_gui_camera_looks_at_the_rover_from_behind():
    for yaw in (0.0, 1.0, -2.5):
        (px, py, pz), (w, x, y, z) = usgs_sites.camera_pose(10.0, -3.0, 2.0, yaw)
        assert math.isclose(w * w + x * x + y * y + z * z, 1.0)
        # Camera forward axis (body x) rotated into the world.
        forward = np.array([1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)])
        target = np.array([10.0 - px, -3.0 - py, 2.0 - pz])
        assert np.allclose(forward, target / np.linalg.norm(target), atol=1e-9)
