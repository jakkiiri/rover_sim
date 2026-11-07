"""Launch a rover_sim world in Gazebo Classic using gazebo_ros.

world: a name in this package's worlds/ folder (mars, desert) or a path to a
.world file. Default: mars.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def world_path(name):
    """Resolve a world name or path to an existing .world file."""
    name = name.strip()
    if os.sep in name or name.endswith('.world'):
        path = os.path.abspath(os.path.expanduser(name))
    else:
        path = os.path.join(get_package_share_directory('rover_sim'), 'worlds', name + '.world')
    if not os.path.isfile(path):
        raise FileNotFoundError(f'world not found: {path}')
    return path


def gazebo(context):
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            'world': world_path(LaunchConfiguration('world').perform(context)),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true', description='Open the Gazebo Classic GUI.'
        ),
        DeclareLaunchArgument(
            'world', default_value='mars',
            description='World name in rover_sim/worlds (mars, desert) or a .world file path.'
        ),
        OpaqueFunction(function=gazebo),
    ])
