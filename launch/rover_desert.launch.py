"""Compatibility wrapper: the former desert launch now forwards to rover_mars.launch.py.

The legacy three-hill desert world is kept as ``world:=desert``; every other
argument is forwarded unchanged.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('rover_sim')
    forwarded = ['gui', 'rviz', 'lidar', 'actuator_watchdog', 'gravity', 'x', 'y', 'z', 'roll', 'pitch', 'yaw']
    defaults = {'gui': 'true', 'rviz': 'false', 'lidar': 'true', 'actuator_watchdog': '0.3',
                'gravity': '9.81', 'x': '0', 'y': '0', 'z': '0.36', 'roll': '0', 'pitch': '0', 'yaw': '0'}
    args = [DeclareLaunchArgument('world', default_value='desert')]
    args += [DeclareLaunchArgument(name, default_value=defaults[name]) for name in forwarded]
    launch_arguments = {name: LaunchConfiguration(name) for name in forwarded + ['world']}
    return LaunchDescription(args + [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rover_mars.launch.py')),
        launch_arguments=launch_arguments.items())])
