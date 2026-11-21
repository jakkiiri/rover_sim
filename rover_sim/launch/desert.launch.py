"""Launch a rover_sim world in Gazebo Fortress (gz-sim 6) with its ROS bridge.

world: a name in this package's worlds/ folder (mars, desert) or a path to a
.world/.sdf file. Default: mars.
gui:=false runs the server only, with EGL headless rendering so the GPU
LiDAR and cameras still work.

Started here:
- ``ign gazebo`` with this package's models on IGN_GAZEBO_RESOURCE_PATH and
  its lib folder (librover_swerve_gz_system.so) on IGN_GAZEBO_SYSTEM_PLUGIN_PATH;
  when it exits the whole launch shuts down.
- ros_gz_bridge: the topics in config/ros_gz_bridge.yaml, plus the world's
  services under fixed ROS names (``sim_services``) so tools do not need the
  world name: /sim/spawn_entity, /sim/delete_entity, /sim/set_entity_pose,
  /sim/control (ros_gz_interfaces/srv/{SpawnEntity,DeleteEntity,SetEntityPose,ControlWorld}).
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ROS name -> (gz service under /world/<name>/, ros_gz_interfaces type)
SIM_SERVICES = {
    '/sim/spawn_entity': ('create', 'SpawnEntity'),
    '/sim/delete_entity': ('remove', 'DeleteEntity'),
    '/sim/set_entity_pose': ('set_pose', 'SetEntityPose'),
    '/sim/control': ('control', 'ControlWorld'),
}


def world_path(name):
    """Resolve a world name or path to an existing world file."""
    name = name.strip()
    if os.sep in name or name.endswith(('.world', '.sdf')):
        path = os.path.abspath(os.path.expanduser(name))
    else:
        path = os.path.join(get_package_share_directory('rover_sim'), 'worlds', name + '.world')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'world not found: {path}')
    return path


def world_name(path):
    """The <world name="..."> of an SDF file (gz service names contain it)."""
    world = ET.parse(path).getroot().find('world')
    if world is None or not world.get('name'):
        raise ValueError(f'{path} has no named <world>')
    return world.get('name')


def sim_services(name):
    """parameter_bridge arguments and remappings for the world's services."""
    arguments, remappings = [], []
    for ros_name, (service, kind) in SIM_SERVICES.items():
        gz_name = f'/world/{name}/{service}'
        arguments.append(f'{gz_name}@ros_gz_interfaces/srv/{kind}')
        remappings.append((gz_name, ros_name))
    return arguments, remappings


def gazebo_environment(share):
    """IGN_GAZEBO_* search paths with this package's models and plugin library added."""
    lib = os.path.normpath(os.path.join(share, '..', '..', 'lib'))

    def extend(key, value):
        current = os.environ.get(key, '')
        return os.pathsep.join([value] + ([current] if current else []))
    return {'IGN_GAZEBO_RESOURCE_PATH': extend('IGN_GAZEBO_RESOURCE_PATH', os.path.join(share, 'models')),
            'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': extend('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', lib)}


def gazebo_command(path, gui):
    command = ['ign', 'gazebo', '-r', '-v', '2']
    if not gui:
        command += ['-s', '--headless-rendering']
    return command + [path]


def gazebo(context):
    share = get_package_share_directory('rover_sim')
    path = world_path(LaunchConfiguration('world').perform(context))
    gui = IfCondition(LaunchConfiguration('gui')).evaluate(context)
    arguments, remappings = sim_services(world_name(path))
    return [
        ExecuteProcess(cmd=gazebo_command(path, gui), additional_env=gazebo_environment(share),
                       output='screen', on_exit=Shutdown(reason='Gazebo exited')),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
             arguments=arguments, remappings=remappings, output='screen',
             parameters=[{'config_file': os.path.join(share, 'config', 'ros_gz_bridge.yaml'),
                          'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo Fortress GUI.'),
        DeclareLaunchArgument(
            'world', default_value='mars',
            description='World name in rover_sim/worlds (mars, desert) or a world file path.'),
        OpaqueFunction(function=gazebo),
    ])
