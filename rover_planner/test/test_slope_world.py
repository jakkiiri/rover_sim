"""The slope test's ramp world is strict XML with a named Fortress world."""
import xml.etree.ElementTree as ET

from rover_planner.slope_test import DEFAULT_ANGLES, world_sdf


def test_ramp_world_parses_as_strict_xml():
    root = ET.fromstring(world_sdf(DEFAULT_ANGLES))
    world = root.find('world')
    assert world.get('name') == 'slope_test'
    assert world.find('physics').get('type') == 'dart'
    names = {m.get('name') for m in world.findall('model')}
    assert names == {'ground'} | {f'ramp_{i}_{a:g}deg' for i, a in enumerate(DEFAULT_ANGLES)}
