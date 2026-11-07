"""Start the A* planner next to an already running rover_sim simulation.

    ros2 launch rover_sim rover_desert.launch.py           # terminal 1
    ros2 launch rover_planner astar_planner.launch.py      # terminal 2

Arguments: rviz (default true) opens the planner view; follow (default false)
starts the path follower, which drives the rover along the path on /cmd_vel
(close teleop first); goal_x and goal_y set a start-up goal in the odom frame
(default: none, use RViz "2D Goal Pose").
"""

import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def planner_nodes(context):
    share = get_package_share_directory('rover_planner')
    goal = {}
    for name in ('goal_x', 'goal_y'):
        text = LaunchConfiguration(name).perform(context).strip()
        if text:
            value = float(text)
            if not math.isfinite(value):
                raise ValueError(f'{name} must be a finite number')
            goal[name] = value
    if len(goal) == 1:
        raise ValueError('Give both goal_x and goal_y, or neither')
    return [
        Node(package='rover_planner', executable='ground_truth_odom',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rover_planner', executable='astar_planner',
             parameters=[os.path.join(share, 'config', 'astar_planner.yaml'),
                         {'use_sim_time': True, **goal}],
             output='screen'),
        Node(package='rover_planner', executable='path_follower',
             parameters=[os.path.join(share, 'config', 'path_follower.yaml'), {'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('follow')),
             output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('rover_planner')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Open the planner RViz view'),
        DeclareLaunchArgument('follow', default_value='false',
                              description='Drive the rover along the path (close teleop first)'),
        DeclareLaunchArgument('goal_x', default_value='', description='Start-up goal x in odom (m)'),
        DeclareLaunchArgument('goal_y', default_value='', description='Start-up goal y in odom (m)'),
        OpaqueFunction(function=planner_nodes),
        Node(package='rviz2', executable='rviz2', name='planner_rviz',
             arguments=['-d', os.path.join(share, 'rviz', 'astar_planner.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz')),
             output='screen'),
    ])
