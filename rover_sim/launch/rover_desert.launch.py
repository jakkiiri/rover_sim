"""Spawn the swerve-controlled proxy rover in a Gazebo Classic world.

world:=mars (default) or world:=desert, or a path to a .world file.
lidar_mode:=3d (default, 3D LiDAR on /points) or 2d (/scan);
lidar_profile:=airy (default: RoboSense Airy-style 48-beam hemispherical dome
mounted upside down, horizon to straight down) or lite|standard|high|dense
(16-64 beam spinning LiDARs centred on the horizon);
lidar_rate (Hz) and lidar_range (m) set its sweep rate and maximum range;
lidar_samples overrides the profile's samples per beam per sweep (fewer is
faster: the 3D LiDAR is the sim's biggest cost).
"""
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

from rover_sim import stereo_launch

# 3D LiDAR profiles: (channels, lowest beam deg, highest beam deg, minimum
# range m). Must match urdf/upstream/mars_lidar3d.urdf.xacro.
LIDAR3D_PROFILES = {'lite': (16, -15.0, 15.0, 0.5), 'standard': (16, -15.0, 15.0, 0.5),
                    'high': (32, -22.5, 22.5, 0.5), 'dense': (64, -22.5, 22.5, 0.5),
                    'airy': (48, -90.0, 0.0, 0.1)}
LIDAR3D_HEIGHT = 0.40  # m above base_link, urdf/sensors/lidar3d.urdf.xacro
# The rover's own body in the lidar_link frame (xmin xmax ymin ymax zmin zmax):
# chassis x +-0.50, wheels (r 0.18 at x +-0.38, y +-0.45) to x +-0.56, and to
# y +-0.63 when steered sideways, camera to x 0.58, ground 0.73 m below the
# LiDAR. Only the downward dome can see it.
LIDAR3D_SELF_BOX = [-0.62, 0.64, -0.66, 0.66, -0.80, 0.0]


def lidar_keepout(mode, profile):
    """Xacro args that keep the stereo camera out of the active LiDAR's beams."""
    if mode != '3d':
        return {}
    _, low, high, _ = LIDAR3D_PROFILES[profile]
    # A dome looking down over the rover sees the camera whatever its mount;
    # only the housing clearance is checked then.
    dome = low <= -60.0
    return {'lidar_keepout_origin': f'0 0 {LIDAR3D_HEIGHT}',
            'lidar_keepout_vfov_min': str(low), 'lidar_keepout_vfov_max': str(high),
            'lidar_keepout_band': str(not dome).lower()}


def lidar3d_rate_and_range(rate, max_range):
    """Validated 3D LiDAR sweep rate (Hz) and maximum range (m)."""
    rate, max_range = float(rate), float(max_range)
    if not (math.isfinite(rate) and 0 < rate <= 50):
        raise ValueError('lidar_rate must be in (0, 50] Hz')
    if not (math.isfinite(max_range) and 1.0 < max_range <= 200):
        raise ValueError('lidar_range must be in (1, 200] m (minimum range is 0.5 m)')
    return rate, max_range


def lidar3d_samples(samples):
    """Validated samples per beam ('' keeps the profile's), as a xacro string."""
    if not samples.strip():
        return ''
    value = int(samples)
    if not 36 <= value <= 4096:
        raise ValueError('lidar_samples must be between 36 and 4096')
    return str(value)


def rviz_config(stereo, mode):
    if stereo:
        return 'rover_sensors_3d.rviz' if mode == '3d' else 'rover_sensors.rviz'
    return 'rover_lidar3d.rviz' if mode == '3d' else 'rover_lidar.rviz'


def generate_launch_description():
    share = get_package_share_directory('rover_sim')
    args = [DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('world', default_value='mars',
                                  description='mars, desert, or a path to a .world file'),
            DeclareLaunchArgument('lidar', default_value='true', description='Enable the Classic ray LiDAR'),
            DeclareLaunchArgument('lidar_mode', default_value='3d', choices=['2d', '3d']),
            DeclareLaunchArgument('lidar_profile', default_value='airy', choices=list(LIDAR3D_PROFILES)),
            DeclareLaunchArgument('lidar_rate', default_value='10', description='3D LiDAR sweeps per second'),
            DeclareLaunchArgument('lidar_range', default_value='100.0', description='3D LiDAR maximum range (m)'),
            DeclareLaunchArgument('lidar_samples', default_value='',
                                  description="3D LiDAR samples per beam per sweep ('' = the profile's)"),
            DeclareLaunchArgument('rviz', default_value='false', description='Open local sensor visualization'),
            DeclareLaunchArgument('actuator_watchdog', default_value='0.3',
                                  description='Positive actuator watchdog in wall seconds')]
    # Stereo camera system, independent of the LiDAR switch (docs/stereo.md).
    args.append(DeclareLaunchArgument('stereo', default_value='true', description='Enable the stereo camera'))
    for name, value in stereo_launch.LAUNCH_ARGS.items():
        args.append(DeclareLaunchArgument(name, default_value=value))
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
        stereo = IfCondition(LaunchConfiguration('stereo')).evaluate(context)
        stereo_config = stereo_launch.parse_stereo_args(
            {name: LaunchConfiguration(name).perform(context) for name in stereo_launch.LAUNCH_ARGS}) if stereo else None
        mode = LaunchConfiguration('lidar_mode').perform(context)
        profile = LaunchConfiguration('lidar_profile').perform(context)
        enabled = IfCondition(LaunchConfiguration('lidar')).evaluate(context)
        rate, max_range = lidar3d_rate_and_range(LaunchConfiguration('lidar_rate').perform(context),
                                                 LaunchConfiguration('lidar_range').perform(context))
        description = xacro.process_file(
            os.path.join(share, 'urdf', 'proxy_rover.urdf.xacro'),
            mappings={'actuator_watchdog': str(watchdog),
                      'enable_lidar': LaunchConfiguration('lidar').perform(context),
                      'lidar_mode': mode, 'lidar_profile': profile,
                      'lidar_rate': str(rate), 'lidar_range': str(max_range),
                      'lidar_samples': lidar3d_samples(LaunchConfiguration('lidar_samples').perform(context)),
                      **lidar_keepout(mode, profile),
                      **stereo_launch.xacro_mappings(stereo, stereo_config)}).toxml()
        nodes = (stereo_launch.stereo_nodes(share, description, stereo_config) if stereo else []) + [
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
            # With stereo on, RViz opens the combined LiDAR + stereo view.
            Node(package='rviz2', executable='rviz2', name='rviz2',
                 arguments=['-d', os.path.join(share, 'rviz', rviz_config(stereo, mode))],
                 parameters=[{'use_sim_time': True}],
                 condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
        ]
        if enabled and mode == '3d':
            channels, low, high, min_range = LIDAR3D_PROFILES[profile]
            nodes.append(Node(package='rover_sim', executable='lidar_postprocess',
                              parameters=[{'use_sim_time': True, 'channels': channels,
                                           'vertical_fov_deg': high - low, 'vertical_min_deg': low,
                                           'rate': rate, 'min_range': min_range,
                                           'max_range': max_range,
                                           'self_filter_box': LIDAR3D_SELF_BOX,
                                           'vertical_shift_steps': 0.0}],
                              output='screen'))
        return nodes
    return LaunchDescription(args + [
        OpaqueFunction(function=robot_nodes),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'desert.launch.py')),
            launch_arguments={'gui': LaunchConfiguration('gui'),
                              'world': LaunchConfiguration('world')}.items()),
    ])
