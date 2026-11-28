"""Spawn the swerve-controlled rover in a Gazebo Fortress world.

robot:=real (default: the real rover from its Onshape CAD, rocker suspension,
stowed arm, 3D LiDAR on a 1 m pole) or robot:=proxy (the original box rover).
world:=usgs (default), world:=mars or world:=desert, or a .world file path.
site:=hills (default) or site:=flat picks the spawn on the usgs world
(config/usgs_sites.yaml; hills has a rock field); x:=/y:=/yaw:= override it,
and on other worlds they default to 0. z:=auto puts the rover 0.36 m above the ground.
With the GUI on (rover_sim.gazebo_gui), its camera moves to just behind the
rover once it is spawned, and its window takes the left half of the screen.
lidar_mode:=3d (default, 3D LiDAR on /points) or 2d (/scan);
lidar_profile:=airy (default: RoboSense Airy-style 48-beam hemispherical dome
mounted upside down, horizon to straight down) or lite|standard|high|dense
(16-64 beam spinning LiDARs centred on the horizon);
lidar_rate (Hz) and lidar_range (m) set its sweep rate and maximum range;
lidar_samples overrides the profile's samples per beam per sweep (on
Fortress fewer samples also means less accurate ranges: docs/fortress.md).
gui:=false runs Gazebo headless (sensors still render).
On the usgs world, chunks:=true (default) streams the terrain in 16 m chunks
within chunk_radius (default 8) chunks of the rover, like Minecraft's render
distance (rover_sim/terrain_chunks.py); chunks:=false loads all of it.
"""
import os
import math

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            SetLaunchConfiguration)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rover_sim import stereo_launch, terrain_chunks, usgs_sites
from rover_sim.survey_terrain import spawn_height


def _load_world_launch():
    """The sibling desert.launch.py (world resolution, Gazebo and bridge)."""
    import importlib.util
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'desert.launch.py')
    spec = importlib.util.spec_from_file_location('rover_sim_world_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


world_launch = _load_world_launch()

# 3D LiDAR profiles: (channels, lowest beam deg, highest beam deg, minimum
# range m). Must match urdf/upstream/mars_lidar3d.urdf.xacro.
LIDAR3D_PROFILES = {'lite': (16, -15.0, 15.0, 0.5), 'standard': (16, -15.0, 15.0, 0.5),
                    'high': (32, -22.5, 22.5, 0.5), 'dense': (64, -22.5, 22.5, 0.5),
                    'airy': (48, -90.0, 0.0, 0.1)}
# Proxy rover: 3D LiDAR 0.40 m above base_link (urdf/sensors/lidar3d.urdf.xacro).
# Its own body in the lidar_link frame (xmin xmax ymin ymax zmin zmax):
# chassis x +-0.50, wheels (r 0.18 at x +-0.38, y +-0.45) to x +-0.56, and to
# y +-0.63 when steered sideways, camera to x 0.58, ground 0.73 m below the
# LiDAR. Only the downward dome can see it.
PROXY = {'xacro': 'proxy_rover.urdf.xacro', 'lidar_xyz': (0.0, 0.0, 0.40),
         'self_box': [-0.62, 0.64, -0.66, 0.66, -0.80, 0.0], 'min_beam_deg': None}
LIDAR3D_HEIGHT = PROXY['lidar_xyz'][2]
LIDAR3D_SELF_BOX = PROXY['self_box']
SELF_BOX_MARGIN = 0.03   # m around the real rover's body in the self-filter box


def real_rover(geometry):
    """LiDAR mount, self-filter box and lowest useful dome beam of the real rover.

    geometry is config/real_rover.yaml (scripts/generate_real_rover.py). The
    box holds the body, stowed arm and wheels (steered sideways they reach
    wheel_y + radius) from the ground up to the LiDAR. Beams steeper than the
    angle at which flat ground leaves the box on its nearest side hit only the
    rover or ground under it, so they are not simulated.
    """
    lx, ly, lz = geometry['lidar_xyz']
    extent, m = geometry['extent'], SELF_BOX_MARGIN
    half_y = max(extent['half_width'], geometry['wheel_y'] + geometry['wheel_radius']) + m
    box = [extent['rear'] - m - lx, extent['front'] + m - lx, -half_y - ly, half_y - ly,
           geometry['ground_z'] - 0.08 - lz, 0.0]
    nearest = min(-box[0], box[1], -box[2], box[3])
    height = lz - geometry['ground_z']
    return {'xacro': 'real_rover.urdf.xacro', 'lidar_xyz': (lx, ly, lz),
            'self_box': [round(v, 3) for v in box],
            'min_beam_deg': -math.degrees(math.atan2(height, nearest))}


def robot_config(robot, share):
    if robot == 'proxy':
        return PROXY
    with open(os.path.join(share, 'config', 'real_rover.yaml')) as f:
        return real_rover(yaml.safe_load(f))


def lidar_keepout(mode, profile, lidar_xyz=PROXY['lidar_xyz']):
    """Xacro args that keep the stereo camera out of the active LiDAR's beams."""
    if mode != '3d':
        return {}
    _, low, high, _ = LIDAR3D_PROFILES[profile]
    # A dome looking down over the rover sees the camera whatever its mount;
    # only the housing clearance is checked then.
    dome = low <= -60.0
    return {'lidar_keepout_origin': ' '.join(f'{v:g}' for v in lidar_xyz),
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


# Entity name kept as proxy_rover for both robots: validators move it by name.
ENTITY = 'proxy_rover'


def spawn_arguments(world, context, x, y, z, yaw):
    """ros_gz_sim create arguments: the robot_description topic at the launch pose.

    x/y/yaw come from the site or x:=/y:=/yaw:= (usgs_sites.resolve_pose); z is the resolved
    height (survey_terrain.spawn_height turns z:=auto into the terrain height
    under x/y plus 0.36 m).
    """
    if not all(math.isfinite(float(v)) for v in (x, y, z, yaw)):
        raise ValueError('x, y, z and yaw must be finite')
    arguments = ['-world', world, '-name', ENTITY, '-topic', 'robot_description',
                 '-x', repr(float(x)), '-y', repr(float(y)), '-z', repr(float(z)), '-Y', repr(float(yaw))]
    for flag, name in [('R', 'roll'), ('P', 'pitch')]:
        value = float(LaunchConfiguration(name).perform(context))
        if not math.isfinite(value):
            raise ValueError(f'{name} must be finite')
        arguments += ['-' + flag, repr(value)]
    return arguments


def rviz_config(stereo, mode):
    if stereo:
        return 'rover_sensors_3d.rviz' if mode == '3d' else 'rover_sensors.rviz'
    return 'rover_lidar3d.rviz' if mode == '3d' else 'rover_lidar.rviz'


def generate_launch_description():
    share = get_package_share_directory('rover_sim')
    args = [DeclareLaunchArgument('robot', default_value='real', choices=['real', 'proxy'],
                                  description='real (CAD model, rockers, LiDAR pole) or proxy (box rover)'),
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('world', default_value='usgs',
                                  description='usgs survey terrain, mars, desert, or a .world file'),
            DeclareLaunchArgument('lidar', default_value='true', description='Enable the LiDAR (gpu_lidar)'),
            DeclareLaunchArgument('lidar_mode', default_value='3d', choices=['2d', '3d']),
            DeclareLaunchArgument('lidar_profile', default_value='airy', choices=list(LIDAR3D_PROFILES)),
            DeclareLaunchArgument('lidar_rate', default_value='5',
                                  description='3D LiDAR sweeps per second'),
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
    args.append(DeclareLaunchArgument('site', default_value='',
                                      description='usgs spawn site from config/usgs_sites.yaml '
                                                  "(hills, flat; '' = its default_site)"))
    for name, value in [('x', ''), ('y', ''), ('z', 'auto'),
                        ('roll', '0'), ('pitch', '0'), ('yaw', '')]:
        args.append(DeclareLaunchArgument(name, default_value=value))

    args.append(DeclareLaunchArgument('chunks', default_value='true',
                                      description='usgs: stream 16 m terrain chunks around the rover'))
    args.append(DeclareLaunchArgument('chunk_radius', default_value='8',
                                      description='usgs: render distance in chunks (square radius)'))

    def spawn_pose(context):
        world_arg = LaunchConfiguration('world').perform(context)
        return (world_arg, *usgs_sites.resolve_pose(
            world_arg, LaunchConfiguration('site').perform(context),
            *(LaunchConfiguration(name).perform(context) for name in ('x', 'y', 'yaw')),
            usgs_sites.load_sites(os.path.join(share, 'config', 'usgs_sites.yaml'))))

    def terrain(context):
        """World file for Gazebo: on usgs with chunks, only the chunks around the spawn."""
        world_arg, x, y, _ = spawn_pose(context)
        if world_arg != 'usgs' or not IfCondition(LaunchConfiguration('chunks')).evaluate(context):
            return [SetLaunchConfiguration('gazebo_world', world_arg)]
        radius = int(LaunchConfiguration('chunk_radius').perform(context))
        if radius < 1:
            raise ValueError('chunk_radius must be at least 1')
        manifest = terrain_chunks.build_chunks(os.path.join(share, 'worlds', 'usgs.terrain.json'),
                                               os.path.join(share, 'models', 'usgs_terrain'))
        index = terrain_chunks.ChunkIndex(manifest)
        text, _ = terrain_chunks.chunk_world(os.path.join(share, 'worlds', 'usgs_chunked.world'),
                                             index, x, y, radius)
        path = manifest.parent / f'usgs_r{radius}_{x:g}_{y:g}.world'
        path.write_text(text)
        return [SetLaunchConfiguration('gazebo_world', str(path)),
                Node(package='rover_sim', executable='terrain_chunks',
                     parameters=[{'manifest': str(manifest), 'radius': radius,
                                  'spawn_x': float(x), 'spawn_y': float(y), 'use_sim_time': True}],
                     output='screen')]

    def robot_nodes(context):
        # The survey is shifted so its default spawn has ground z=0. Other
        # x/y locations need their actual triangle elevation, not a flat z.
        world_arg, x, y, yaw = spawn_pose(context)
        z = spawn_height(world_arg, share, x, y, LaunchConfiguration('z').perform(context))
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
        robot = robot_config(LaunchConfiguration('robot').perform(context), share)
        world = world_launch.world_name(world_launch.world_path(LaunchConfiguration('world').perform(context)))
        extra = {}
        if robot['min_beam_deg'] is not None:
            extra['lidar_min_beam_deg'] = f"{robot['min_beam_deg']:.3f}"
        description = xacro.process_file(
            os.path.join(share, 'urdf', robot['xacro']),
            mappings={'actuator_watchdog': str(watchdog),
                      'enable_lidar': LaunchConfiguration('lidar').perform(context),
                      'lidar_mode': mode, 'lidar_profile': profile,
                      'lidar_rate': str(rate), 'lidar_range': str(max_range),
                      'lidar_samples': lidar3d_samples(LaunchConfiguration('lidar_samples').perform(context)),
                      **lidar_keepout(mode, profile, robot['lidar_xyz']), **extra,
                      **stereo_launch.xacro_mappings(stereo, stereo_config)}).toxml()
        nodes = (stereo_launch.stereo_nodes(share, description, stereo_config) if stereo else []) + [
            Node(package='rover_sim', executable='swerve_controller',
                 parameters=[os.path.join(share, 'config', 'swerve.yaml'),
                             {'robot_description': description, 'use_sim_time': True}],
                 output='screen'),
            Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'robot_description': description, 'use_sim_time': True}],
                 output='screen'),
            # create waits for robot_description (latched) and the world's create
            # service; no fixed startup delay.
            Node(package='ros_gz_sim', executable='create',
                 arguments=spawn_arguments(world, context, x, y, z, yaw), output='screen'),
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
                                           'self_filter_box': robot['self_box'],
                                           'vertical_shift_steps': 0.0}],
                              output='screen'))
        return nodes
    world_args = {name: LaunchConfiguration(name) for name in ('gui', 'lidar', 'lidar_mode', 'stereo')}
    world_args['world'] = LaunchConfiguration('gazebo_world')   # set by terrain()
    return LaunchDescription(args + [
        OpaqueFunction(function=lambda context: terrain(context) + robot_nodes(context)),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'desert.launch.py')),
            launch_arguments=world_args.items()),
    ])
