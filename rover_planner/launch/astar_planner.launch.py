"""Start the A* planner next to an already running rover_sim simulation.

    ros2 launch rover_sim rover_desert.launch.py           # terminal 1
    ros2 launch rover_planner astar_planner.launch.py      # terminal 2

Arguments: robot (default real; match rover_sim's robot argument) sets the
rover footprint; rviz (default true) opens the planner view; follow (default false)
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

from rover_planner.robot_geometry import ROBOTS, rover_geometry


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
    rover = rover_geometry(LaunchConfiguration('robot').perform(context))
    footprint = {'footprint_half_length': rover['half_length'], 'footprint_half_width': rover['half_width'],
                 'self_shadow_box': [float(v) for v in rover['shadow_box']]}
    return [
        Node(package='rover_planner', executable='ground_truth_odom',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='rover_planner', executable='astar_planner',
             parameters=[os.path.join(share, 'config', 'astar_planner.yaml'),
                         {'use_sim_time': True, **footprint, **goal}],
             output='screen'),
        Node(package='rover_planner', executable='path_follower',
             parameters=[os.path.join(share, 'config', 'path_follower.yaml'), {'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('follow')),
             output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('rover_planner')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='real', choices=list(ROBOTS),
                              description="Rover model, as given to rover_sim's launch"),
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
