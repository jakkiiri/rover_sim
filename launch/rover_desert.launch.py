"""Spawn the swerve-controlled proxy rover in the Gazebo Classic desert."""
import os
import math

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('rover_sim')
    args = [DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('lidar', default_value='true', description='Enable the Classic 2D ray LiDAR'),
            DeclareLaunchArgument('rviz', default_value='false', description='Open local LiDAR visualization'),
            DeclareLaunchArgument('actuator_watchdog', default_value='0.3',
                                  description='Positive actuator watchdog in wall seconds')]
    for name, value in [('x', '0'), ('y', '0'), ('z', '0.36'),
                        ('roll', '0'), ('pitch', '0'), ('yaw', '0')]:
        args.append(DeclareLaunchArgument(name, default_value=value))
    spawn_args = ['-entity', 'proxy_rover', '-topic', 'robot_description',
                  '-timeout', '60']
    for option, name in [('x', 'x'), ('y', 'y'), ('z', 'z'),
                         ('R', 'roll'), ('P', 'pitch'), ('Y', 'yaw')]:
        spawn_args.extend(['-' + option, LaunchConfiguration(name)])
    def robot_nodes(context):
        watchdog = float(LaunchConfiguration('actuator_watchdog').perform(context))
        if not math.isfinite(watchdog) or watchdog <= 0:
            raise ValueError('actuator_watchdog must be finite and positive')
        description = xacro.process_file(
            os.path.join(share, 'urdf', 'proxy_rover.urdf.xacro'),
            mappings={'actuator_watchdog': str(watchdog),
                      'enable_lidar': LaunchConfiguration('lidar').perform(context)}).toxml()
        return [
            Node(package='rover_sim', executable='swerve_controller',
                 parameters=[os.path.join(share, 'config', 'swerve.yaml'),
                             {'robot_description': description, 'use_sim_time': True}],
                 output='screen'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description, 'use_sim_time': True}],
                 output='screen'),
            # spawn_entity waits for the factory service; no fixed startup delay.
            Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=spawn_args, output='screen'),
        ]
    return LaunchDescription(args + [
        OpaqueFunction(function=robot_nodes),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'desert.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui')}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'rover_lidar.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
