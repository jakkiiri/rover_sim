"""Fortress launch plumbing and sim_services helpers (no simulator started)."""
import importlib.util
import math
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node

from rover_sim import sim_services

PACKAGE = Path(__file__).resolve().parents[1]


def load(name):
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), PACKAGE / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


WORLD = load('desert.launch.py')
ROVER = load('rover_desert.launch.py')


def test_world_names_match_the_files():
    for name in ('desert', 'mars'):
        assert WORLD.world_name(PACKAGE / 'worlds' / f'{name}.world') == name


def test_world_path_rejects_missing_files(tmp_path):
    with pytest.raises(FileNotFoundError):
        WORLD.world_path(str(tmp_path / 'nope.world'))


def test_world_without_name_is_rejected(tmp_path):
    path = tmp_path / 'w.sdf'
    path.write_text('<sdf version="1.8"><world/></sdf>')
    with pytest.raises(ValueError):
        WORLD.world_name(path)


def test_services_are_bridged_under_fixed_ros_names():
    arguments, remappings = WORLD.sim_services('mars')
    assert arguments == ['/world/mars/create@ros_gz_interfaces/srv/SpawnEntity',
                         '/world/mars/remove@ros_gz_interfaces/srv/DeleteEntity',
                         '/world/mars/set_pose@ros_gz_interfaces/srv/SetEntityPose',
                         '/world/mars/control@ros_gz_interfaces/srv/ControlWorld']
    assert dict(remappings) == {'/world/mars/create': sim_services.SPAWN,
                                '/world/mars/remove': sim_services.DELETE,
                                '/world/mars/set_pose': sim_services.SET_POSE,
                                '/world/mars/control': sim_services.CONTROL}


def test_headless_server_keeps_rendering_for_sensors():
    assert WORLD.gazebo_command('/w.world', True) == ['ign', 'gazebo', '-r', '-v', '2', '/w.world']
    assert WORLD.gazebo_command('/w.world', False) == ['ign', 'gazebo', '-r', '-v', '2', '-s',
                                                       '--headless-rendering', '/w.world']


def test_environment_adds_models_and_plugin_library(monkeypatch):
    monkeypatch.setenv('IGN_GAZEBO_RESOURCE_PATH', '/other/models')
    monkeypatch.delenv('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', raising=False)
    env = WORLD.gazebo_environment('/ws/install/rover_sim/share/rover_sim')
    assert env['IGN_GAZEBO_RESOURCE_PATH'] == '/ws/install/rover_sim/share/rover_sim/models:/other/models'
    assert env['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] == '/ws/install/rover_sim/lib'


def evaluate(module, **overrides):
    description = module.generate_launch_description()
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = entity.default_value[0].perform(context)
    context.launch_configurations.update(overrides)
    opaque = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    return [action for function in opaque for action in function.execute(context)]


def test_world_launch_starts_gazebo_and_the_bridge_only():
    actions = evaluate(WORLD, world='desert', gui='false')
    gazebo = [a for a in actions if isinstance(a, ExecuteProcess) and not isinstance(a, Node)]
    nodes = [a for a in actions if isinstance(a, Node)]
    assert len(gazebo) == 1 and len(nodes) == 1
    assert nodes[0]._Node__node_executable == 'parameter_bridge'


def test_rover_is_spawned_with_ros_gz_sim_create_at_the_launch_pose():
    nodes = evaluate(ROVER, world='desert', x='1.5', yaw='0.25')
    create = [n for n in nodes if isinstance(n, Node) and n._Node__node_executable == 'create']
    assert len(create) == 1
    args = [a.perform(LaunchContext()) if hasattr(a, 'perform') else a
            for a in create[0]._Node__arguments]
    flat = [''.join(x.perform(LaunchContext()) for x in a) if isinstance(a, list) else a for a in args]
    pairs = dict(zip(flat[::2], flat[1::2]))
    assert pairs['-world'] == 'desert' and pairs['-name'] == sim_services.ROVER
    assert pairs['-topic'] == 'robot_description'
    assert float(pairs['-x']) == 1.5 and float(pairs['-Y']) == 0.25 and float(pairs['-z']) == 0.36


def test_nonfinite_spawn_pose_is_rejected():
    with pytest.raises(ValueError):
        evaluate(ROVER, world='desert', z='nan')


def test_request_builders():
    spawn = sim_services.spawn_request('box', '<sdf/>', (1, 2, 3))
    factory = spawn.entity_factory
    assert (factory.name, factory.sdf, factory.allow_renaming) == ('box', '<sdf/>', False)
    assert (factory.pose.position.x, factory.pose.position.z, factory.pose.orientation.w) == (1, 3, 1)
    delete = sim_services.delete_request('box')
    assert delete.entity.name == 'box' and delete.entity.type == delete.entity.MODEL
    move = sim_services.set_pose_request('proxy_rover', (0, 0, .36), (0, 0, 0, 1))
    assert move.entity.name == 'proxy_rover' and move.pose.orientation.z == 1


def test_yaw_quaternion_and_odometry_round_trip():
    from nav_msgs.msg import Odometry
    msg = Odometry()
    w, x, y, z = sim_services.yaw_quaternion(0.1, -0.2, 2.5)
    q = msg.pose.pose.orientation
    q.w, q.x, q.y, q.z = w, x, y, z
    msg.pose.pose.position.x = 4.0
    msg.header.stamp.sec, msg.header.stamp.nanosec = 12, 500000000
    pose, stamp = sim_services.odometry_pose(msg)
    assert pose[0] == 4.0 and stamp == 12.5
    assert pose[3:] == pytest.approx([0.1, -0.2, 2.5])


def test_isolation_requires_domain_and_partition(monkeypatch):
    errors = []
    parser = type('P', (), {'error': lambda self, text: errors.append(text)})()
    monkeypatch.setenv('ROS_DOMAIN_ID', '73')
    monkeypatch.delenv('IGN_PARTITION', raising=False)
    sim_services.require_isolation(parser)
    assert errors and 'IGN_PARTITION' in errors[0]
    monkeypatch.setenv('IGN_PARTITION', 'x')
    errors.clear()
    sim_services.require_isolation(parser)
    assert not errors


def bridge_topics(sensors):
    import yaml
    entries = yaml.safe_load((PACKAGE / 'config' / 'ros_gz_bridge.yaml').read_text())
    return {e['ros_topic_name'] for e in WORLD.bridge_entries(entries, sensors)}


def test_bridge_carries_only_enabled_sensors():
    always = {'/clock', '/cmd_vel'}
    stereo = {f'/stereo/gazebo/{side}/{kind}' for side in ('left', 'right') for kind in ('image_raw', 'camera_info')}
    assert bridge_topics(set()) == always
    assert bridge_topics({'lidar3d'}) == always | {'/points_raw'}
    assert bridge_topics({'lidar2d', 'stereo'}) == always | {'/scan'} | stereo
    assert bridge_topics(set(WORLD.SENSOR_TOPICS)) == always | {'/points_raw', '/scan'} | stereo


@pytest.mark.parametrize('lidar, mode, stereo, expected', [
    ('true', '3d', 'true', {'lidar3d', 'stereo'}), ('true', '2d', 'false', {'lidar2d'}),
    ('false', '3d', 'true', {'stereo'}), ('False', '2D', 'FALSE', set())])
def test_enabled_sensors(lidar, mode, stereo, expected):
    assert WORLD.enabled_sensors(lidar, mode, stereo) == expected


@pytest.mark.parametrize('args', [('yes', '3d', 'true'), ('true', '4d', 'true'), ('true', '3d', '1')])
def test_enabled_sensors_rejects_bad_values(args):
    with pytest.raises(ValueError):
        WORLD.enabled_sensors(*args)


def test_rover_launch_passes_its_sensor_choice_to_the_world_launch():
    from launch.actions import IncludeLaunchDescription
    include = [e for e in ROVER.generate_launch_description().entities if isinstance(e, IncludeLaunchDescription)]
    assert len(include) == 1
    names = {''.join(p.perform(LaunchContext()) if hasattr(p, 'perform') else p for p in k) if isinstance(k, (list, tuple)) else k
             for k, _ in include[0].launch_arguments}
    assert {'gui', 'world', 'lidar', 'lidar_mode', 'stereo'} <= names


def create_pairs(nodes):
    create = [n for n in nodes if isinstance(n, Node) and n._Node__node_executable == 'create']
    assert len(create) == 1
    flat = [a if isinstance(a, str) else ''.join(x.perform(LaunchContext()) for x in a)
            for a in create[0]._Node__arguments]
    return dict(zip(flat[::2], flat[1::2]))


def test_usgs_spawns_at_the_hills_site_on_the_ground_by_default():
    pairs = create_pairs(evaluate(ROVER))
    assert pairs['-world'] == 'usgs'
    assert (float(pairs['-x']), float(pairs['-y'])) == (-444.0, 64.0)
    from rover_sim.survey_terrain import SurveySurface
    ground = SurveySurface(PACKAGE / 'worlds/usgs.terrain.json').height(-444.0, 64.0)
    assert math.isclose(float(pairs['-z']), ground + 0.36, abs_tol=1e-9)


def test_site_flat_and_explicit_coordinates_select_the_spawn():
    pairs = create_pairs(evaluate(ROVER, site='flat'))
    assert (float(pairs['-x']), float(pairs['-y']), float(pairs['-z'])) == (0.0, 0.0, 0.36)
    pairs = create_pairs(evaluate(ROVER, site='hills', x='10', y='-5'))
    assert (float(pairs['-x']), float(pairs['-y'])) == (10.0, -5.0)


def test_gui_camera_moves_behind_the_rover_only_with_the_gui():
    def camera(nodes):
        return [n for n in nodes if isinstance(n, ExecuteProcess) and not isinstance(n, Node)]
    with_gui = camera(evaluate(ROVER, world='desert', x='2', yaw='0'))
    assert len(with_gui) == 1
    command = ' '.join(''.join(s.perform(LaunchContext()) for s in part) for part in with_gui[0].cmd)
    assert '/gui/move_to/pose' in command and 'x: -5.000' in command
    assert with_gui[0].condition.evaluate(context_with(gui='true'))
    assert not with_gui[0].condition.evaluate(context_with(gui='false'))


def context_with(**values):
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context
