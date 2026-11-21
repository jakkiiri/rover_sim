"""Every rover_sim world is a Gazebo Fortress world built from rover_sim/fortress_world.py."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from rover_sim import fortress_world

PACKAGE = Path(__file__).resolve().parents[1]
WORLDS = sorted((PACKAGE / 'worlds').glob('*.world'))
REQUIRED_SYSTEMS = {'ignition::gazebo::systems::Physics', 'ignition::gazebo::systems::UserCommands',
                    'ignition::gazebo::systems::SceneBroadcaster', 'ignition::gazebo::systems::Sensors',
                    'ignition::gazebo::systems::Imu'}


@pytest.fixture(params=WORLDS, ids=lambda p: p.name)
def world(request):
    return request.param


def test_worlds_exist():
    assert {p.name for p in WORLDS} >= {'desert.world', 'mars.world'}


def test_world_uses_shared_physics_and_systems(world):
    text = world.read_text()
    assert fortress_world.physics_block() in text
    assert fortress_world.SYSTEMS in text
    root = ET.parse(world).getroot()
    assert root.get('version') == '1.8'
    physics = root.find('world/physics')
    assert physics.get('type') == 'dart'
    assert physics.findtext('max_step_size') == '0.001'
    systems = {p.get('name') for p in root.findall('world/plugin')}
    assert systems == REQUIRED_SYSTEMS


def test_world_has_no_classic_only_elements(world):
    root = ET.parse(world).getroot()
    assert not list(root.iter('script')), 'OGRE material scripts do not render in ogre2'
    assert root.find('world/physics/ode') is None
    assert not [p for p in root.iter('plugin') if (p.get('filename') or '').startswith('libgazebo_ros')]
    assert not [i for i in root.iter('include') if i.findtext('uri') == 'model://sun']
    assert root.find('world/gui/camera') is None, 'Classic GUI camera; Fortress uses MinimalScene camera_pose'


def test_gui_has_scene_and_teleop(world):
    root = ET.parse(world).getroot()
    names = {p.get('filename') for p in root.findall('world/gui/plugin')}
    assert {'MinimalScene', 'WorldControl', 'WorldStats', 'CameraTracking', 'Teleop'} <= names
    assert root.findtext("world/gui/plugin[@filename='Teleop']/topic") == '/cmd_vel'


def test_mesh_and_texture_uris_resolve_in_the_package(world):
    root = ET.parse(world).getroot()
    models = PACKAGE / 'models'
    uris = [u.text for u in root.iter('uri')] + [a.text for a in root.iter('albedo_map')]
    assert uris
    for uri in uris:
        assert uri.startswith('model://'), uri
        assert (models / uri[len('model://'):]).is_file(), uri


def test_pbr_helper_writes_an_albedo_map():
    xml = ET.fromstring(fortress_world.pbr('model://x/t.png'))
    assert xml.findtext('pbr/metal/albedo_map') == 'model://x/t.png'
    assert xml.findtext('pbr/metal/metalness') == '0'
