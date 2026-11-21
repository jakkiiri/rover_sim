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
