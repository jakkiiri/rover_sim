"""Launch the desert environment in Gazebo Classic using gazebo_ros."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = os.path.join(
        get_package_share_directory('rover_sim'), 'worlds', 'desert.world'
    )
    gazebo_launch = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui', default_value='true', description='Open the Gazebo Classic GUI.'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
            launch_arguments={
                'world': world,
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),
    ])
