"""Launch a rover_sim world in Gazebo Fortress (gz-sim 6) with its ROS bridge.

world: a name in this package's worlds/ folder (usgs, mars, desert) or a path
to a .world/.sdf file. Default: usgs, the imported USGS Utah survey terrain.
The server always runs headless (EGL rendering, so the GPU LiDAR and cameras
work); gui:=true (default) adds the GUI as a separate process through
rover_sim.gazebo_gui, which loads a lighter plugin set, puts the window on the
left half of the screen and points the camera at the rover. Closing the GUI
window ends the launch, as before.

Started here:
- ``ign gazebo -s`` with this package's models on IGN_GAZEBO_RESOURCE_PATH and
  its lib folder (librover_swerve_gz_system.so) on IGN_GAZEBO_SYSTEM_PLUGIN_PATH;
  when it exits the whole launch shuts down.
- ros_gz_bridge: the topics in config/ros_gz_bridge.yaml for the sensors that
  are enabled (lidar, lidar_mode, stereo; set by rover_desert.launch.py, all on
  when this file runs alone), plus the world's
  services under fixed ROS names (``sim_services``) so tools do not need the
  world name: /sim/spawn_entity, /sim/delete_entity, /sim/set_entity_pose,
  /sim/control (ros_gz_interfaces/srv/{SpawnEntity,DeleteEntity,SetEntityPose,ControlWorld}).
"""

import os
import sys
import tempfile
import xml.etree.ElementTree as ET

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_sim.gazebo_gui import gazebo_environment

# ROS name -> (gz service under /world/<name>/, ros_gz_interfaces type)
SIM_SERVICES = {
    '/sim/spawn_entity': ('create', 'SpawnEntity'),
    '/sim/delete_entity': ('remove', 'DeleteEntity'),
    '/sim/set_entity_pose': ('set_pose', 'SetEntityPose'),
    '/sim/control': ('control', 'ControlWorld'),
}


# Sensor -> gz topic prefix of its bridge entries; entries matching none are always bridged.
SENSOR_TOPICS = {'lidar3d': '/rover/lidar3d/', 'lidar2d': '/rover/scan', 'stereo': '/stereo/gazebo/'}


def enabled_sensors(lidar, lidar_mode, stereo):
    """Sensor groups present on the robot, from the launch arguments' text."""
    def flag(value, name):
        value = value.strip().lower()
        if value not in ('true', 'false'):
            raise ValueError(f'{name} must be true or false, got {value!r}')
        return value == 'true'
    mode = lidar_mode.strip().lower()
    if mode not in ('2d', '3d'):
        raise ValueError(f'lidar_mode must be 2d or 3d, got {lidar_mode!r}')
    sensors = {'stereo'} if flag(stereo, 'stereo') else set()
    if flag(lidar, 'lidar'):
        sensors.add('lidar' + mode)
    return sensors


def bridge_entries(entries, sensors):
    """Bridge config entries without the topics of sensors that are not on the robot.

    Bridging a missing sensor would still advertise its ROS topic (e.g. /scan
    with lidar:=false), which tools take as the sensor being present.
    """
    def wanted(entry):
        groups = [g for g, prefix in SENSOR_TOPICS.items() if entry['gz_topic_name'].startswith(prefix)]
        return not groups or groups[0] in sensors
    return [e for e in entries if wanted(e)]


def bridge_config_file(share, sensors):
    """Write the filtered bridge config to a temporary file and return its path."""
    with open(os.path.join(share, 'config', 'ros_gz_bridge.yaml')) as f:
        entries = bridge_entries(yaml.safe_load(f), sensors)
    with tempfile.NamedTemporaryFile('w', prefix='rover_sim_bridge_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(entries, f, sort_keys=False)
        return f.name


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


def gazebo_command(path):
    return ['ign', 'gazebo', '-r', '-v', '2', '-s', '--headless-rendering', path]


def gui_command(path):
    """The Gazebo GUI on the server started here (rover_sim.gazebo_gui).

    Run directly, not through ``ros2 run``: that wrapper swallows the launch's
    SIGINT instead of passing it on, which left the window open after shutdown.
    """
    return [sys.executable, '-m', 'rover_sim.gazebo_gui', '--world-file', path]


def gazebo(context):
    share = get_package_share_directory('rover_sim')
    path = world_path(LaunchConfiguration('world').perform(context))
    gui = IfCondition(LaunchConfiguration('gui')).evaluate(context)
    arguments, remappings = sim_services(world_name(path))
    sensors = enabled_sensors(*(LaunchConfiguration(n).perform(context) for n in ('lidar', 'lidar_mode', 'stereo')))
    return [
        ExecuteProcess(cmd=gazebo_command(path), additional_env=gazebo_environment(share),
                       output='screen', on_exit=Shutdown(reason='Gazebo exited')),
        *([ExecuteProcess(cmd=gui_command(path), output='screen',
                          on_exit=Shutdown(reason='Gazebo GUI closed'))] if gui else []),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='ros_gz_bridge',
             arguments=arguments, remappings=remappings, output='screen',
             parameters=[{'config_file': bridge_config_file(share, sensors),
                          'use_sim_time': True}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo Fortress GUI.'),
        DeclareLaunchArgument(
            'world', default_value='usgs',
            description='World name in rover_sim/worlds (usgs, mars, desert) or a world file path.'),
        # Declared again (same names) by rover_desert.launch.py, whose values win.
        DeclareLaunchArgument('lidar', default_value='true', description='Bridge the LiDAR topics.'),
        DeclareLaunchArgument('lidar_mode', default_value='3d', description='Which LiDAR to bridge: 3d or 2d.'),
        DeclareLaunchArgument('stereo', default_value='true', description='Bridge the stereo camera topics.'),
        OpaqueFunction(function=gazebo),
    ])
