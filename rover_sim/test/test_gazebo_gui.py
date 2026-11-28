"""rover_sim.gazebo_gui helpers (no Gazebo or X server needed)."""
from pathlib import Path

import pytest

from rover_sim import gazebo_gui as gg

WORLDS = Path(__file__).resolve().parents[1] / 'worlds'


def test_workarea_parsed_from_xprop():
    text = '_NET_WORKAREA(CARDINAL) = 140, 54, 2420, 1546, 140, 54, 2420, 1546\n'
    assert gg.parse_workarea(text) == (140, 54, 2420, 1546)
    assert gg.parse_workarea('_NET_WORKAREA:  not found.\n') is None
    assert gg.parse_workarea('') is None


def test_screen_halves_split_the_workarea():
    left = gg.screen_half((140, 54, 2420, 1546), 'left')
    right = gg.screen_half((140, 54, 2420, 1546), 'right')
    assert left == (140, 54, 1210, 1546 - gg.TITLE_BAR)
    assert right == (1350, 54, 1210, 1546 - gg.TITLE_BAR)
    with pytest.raises(ValueError):
        gg.screen_half((0, 0, 10, 10), 'top')


def test_gui_config_keeps_the_worlds_plugins_except_dropped_ones():
    text = gg.gui_config_text(WORLDS / 'usgs.world')
    assert 'filename="MinimalScene"' in text and 'filename="Teleop"' in text
    assert all(f'filename="{name}"' not in text for name in gg.DROP_PLUGINS)
    assert '<window>' not in text


def test_gui_config_is_none_for_a_world_without_gui(tmp_path):
    world = tmp_path / 'bare.world'
    world.write_text('<sdf version="1.9"><world name="bare"/></sdf>')
    assert gg.gui_config_text(world) is None


def test_running_world_from_service_list():
    services = '/gazebo/resource_paths/get\n/world/usgs/control\n/world/usgs/control/state\n'
    assert gg.running_world(services) == 'usgs'
    assert gg.running_world('/gazebo/worlds\n') is None


def test_model_pose_from_ign_model_output():
    text = ('Requesting state for world [usgs]...\n\nModel: [1494]\n  - Name: proxy_rover\n'
            '  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [-395.997000 51.995200 -15.390700]\n'
            '    [-0.016501 0.004668 -2.618050]\n')
    assert gg.parse_model_pose(text) == (-395.997, 51.9952, -15.3907, -2.61805)
    assert gg.parse_model_pose('No model named [proxy_rover] was found.\n') is None


def test_find_window_matches_class_and_pid(monkeypatch):
    tree = ('     0x4200010 "Gazebo GUI": ()  1x1+0+0  +0+0\n'
            '        0x420000e "Gazebo": ("ign-gazebo-gui" "Gazebo GUI")  2420x1472+0+74  +140+128\n'
            '        0x5100003 "Gazebo": ("ign-gazebo-gui" "Gazebo GUI")  2420x1472+0+74  +140+128\n')
    pids = {'0x420000e': '_NET_WM_PID(CARDINAL) = 777\n', '0x5100003': '_NET_WM_PID(CARDINAL) = 4242\n'}
    monkeypatch.setattr(gg, 'run', lambda command, timeout=10: pids[command[2]])
    assert gg.find_window(4242, tree) == '0x5100003'
    assert gg.find_window(77, tree) is None
