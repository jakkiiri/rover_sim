"""Launch the Mars rover simulation in Gazebo Classic.

Worlds: ``mars`` (generated heightmap + rocks), ``flat``, ``step_test``,
``slope_test``, ``boulder_garden`` and the legacy ``desert``. Terrain assets
are generated at build time for the default seed; another ``seed`` is
generated on first use into ``~/.cache/rover_sim``. Gravity and shadows are
substituted into a per-launch copy of the world file.
"""
import hashlib
import math
import os
from pathlib import Path
import sys

import xacro
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLDS = ('mars', 'flat', 'step_test', 'slope_test', 'boulder_garden', 'desert')
DENSITIES = ('low', 'medium', 'high')
# base_link height above flat ground when the rover rests (URDF: wheel radius
# 0.20 + ground clearance 0.30 - body half height 0.11 = 0.41 at Mars preload).
REST_HEIGHT = 0.41
DEFAULT_SEED = 7


def _truthy(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _cache_dir():
    return Path(os.environ.get('XDG_CACHE_HOME', Path.home() / '.cache')) / 'rover_sim'


def _terrain_directory(share, seed, resolution, regenerate, override):
    """Return the directory holding terrain_manifest.json for this seed."""
    sys.path.insert(0, share)
    from rover_sim.terrain.pipeline import generate_all
    if override:
        return Path(override)
    installed = Path(share) / 'generated'
    if seed == DEFAULT_SEED and resolution == 513 and not regenerate and (installed / 'terrain_manifest.json').exists():
        return installed
    target = _cache_dir() / 'terrain' / f'seed_{seed}_r{resolution}'
    if regenerate or not (target / 'terrain_manifest.json').exists():
        print(f'[rover_mars.launch] generating terrain seed={seed} resolution={resolution} -> {target}', flush=True)
        generate_all(target, seed=seed, resolution=resolution)
    return target


def _world_file(share, terrain_dir, world, density):
    if world == 'mars':
        return terrain_dir / 'worlds' / f'mars_{density}.world'
    if world == 'desert':
        return Path(share) / 'worlds' / 'desert.world'
    if world == 'flat':
        return Path(share) / 'worlds' / 'flat.world'
    generated = terrain_dir / 'worlds' / f'{world}.world'
    return generated if generated.exists() else Path(share) / 'worlds' / 'test' / f'{world}.world'


def _prepared_world(share, path, gravity, shadows):
    sys.path.insert(0, share)
    from rover_sim.terrain.world import override_world
    text = override_world(path.read_text(), gravity=gravity, shadows=shadows)
    directory = _cache_dir() / 'worlds'
    directory.mkdir(parents=True, exist_ok=True)
    digest = hashlib.sha256(text.encode()).hexdigest()[:12]
    target = directory / f'{path.stem}_{digest}.world'
    if not target.exists():
        target.write_text(text)
    return target


def _spawn_height(terrain_dir, world, x, y):
    if world != 'mars':
        return 0.0
    from rover_sim.terrain.heightmap import sample_height
    from rover_sim.terrain.pipeline import load_manifest, read_height_field
    manifest = load_manifest(terrain_dir)
    return float(sample_height(read_height_field(terrain_dir), manifest['size'], x, y))


def setup(context):
    share = get_package_share_directory('rover_sim')
    cfg = {name: LaunchConfiguration(name).perform(context) for name in (
        'world', 'seed', 'rock_density', 'gravity', 'shadows', 'gui', 'rviz', 'lidar', 'lidar_profile',
        'lidar_gpu', 'scan_2d', 'drive_mode', 'actuator_watchdog', 'terrain_dir', 'regenerate',
        'resolution', 'x', 'y', 'z', 'roll', 'pitch', 'yaw', 'wheel_collision', 'wheel_stall_torque', 'lidar_prefer_gpu',
        'lidar_channels', 'lidar_samples', 'lidar_vfov_deg', 'lidar_max_range',
        'wheel_no_load_speed', 'wheel_mu', 'mast_height', 'ground_truth_tf', 'robot', 'suspension_mode',
        'strut_inertia_size')}
    if cfg['world'] not in WORLDS:
        raise ValueError(f"world must be one of {WORLDS}, not {cfg['world']!r}")
    if cfg['rock_density'] not in DENSITIES:
        raise ValueError(f"rock_density must be one of {DENSITIES}")
    gravity = float(cfg['gravity'])
    if not math.isfinite(gravity) or gravity <= 0:
        raise ValueError('gravity must be a positive magnitude in m/s^2')
    watchdog = float(cfg['actuator_watchdog'])
    if not math.isfinite(watchdog) or watchdog <= 0:
        raise ValueError('actuator_watchdog must be finite and positive')
    seed = int(cfg['seed'])
    resolution = int(cfg['resolution'])

    terrain_dir = _terrain_directory(share, seed, resolution, _truthy(cfg['regenerate']), cfg['terrain_dir'])
    world_path = _world_file(share, terrain_dir, cfg['world'], cfg['rock_density'])
    if not world_path.exists():
        raise FileNotFoundError(f'world file missing: {world_path}')
    prepared = _prepared_world(share, world_path, gravity, _truthy(cfg['shadows']))
    # Gazebo resolves model://mars_terrain/... through GAZEBO_MODEL_PATH.
    models = str(terrain_dir / 'models')
    os.environ['GAZEBO_MODEL_PATH'] = models + (':' + os.environ['GAZEBO_MODEL_PATH']
                                                if os.environ.get('GAZEBO_MODEL_PATH') else '')

    lidar_profile = cfg['lidar_profile']
    if lidar_profile not in ('auto', 'lite', 'standard', 'high', 'dense'):
        raise ValueError('lidar_profile must be auto, lite, standard, high or dense')
    lidar_gpu = cfg['lidar_gpu'].lower()
    lidar_enabled = _truthy(cfg['lidar'])
    heightmap_world = cfg['world'] == 'mars'
    if lidar_gpu == 'auto':
        # The CPU ray sensor hangs gzserver on the heightmap world (measured,
        # docs/lidar.md), so the Mars world needs gpu_ray, which needs an
        # OpenGL context (a display). Flat/test worlds use the exact CPU sensor
        # unless lidar_prefer_gpu is set.
        if heightmap_world:
            if os.environ.get('DISPLAY'):
                lidar_gpu = 'true'
            else:
                print('[rover_mars.launch] no DISPLAY for gpu_ray and the CPU ray sensor cannot be used on the '
                      'heightmap world: disabling the LiDAR', flush=True)
                lidar_enabled = False
                lidar_gpu = 'false'
        else:
            lidar_gpu = 'true' if os.environ.get('DISPLAY') and _truthy(cfg['lidar_prefer_gpu']) else 'false'
    elif lidar_gpu == 'false' and heightmap_world and lidar_enabled:
        print('[rover_mars.launch] WARNING: the CPU ray sensor is known to hang on the heightmap world; '
              'use lidar_gpu:=true or lidar:=false', flush=True)
    if lidar_profile == 'auto':
        # dense is free on the GPU sensor (measured RTF 0.998 on the Mars world);
        # lite is the only CPU profile that holds RTF >= 0.9.
        lidar_profile = 'dense' if lidar_gpu == 'true' else 'lite'
    channels, vertical_fov = {'high': (32, 45.0), 'dense': (64, 45.0)}.get(lidar_profile, (16, 30.0))
    print(f'[rover_mars.launch] lidar: {"gpu_ray" if lidar_gpu == "true" else "ray"} profile {lidar_profile}'
          + ('' if lidar_enabled else ' (disabled)'), flush=True)
    if cfg['lidar_channels']:
        channels = int(cfg['lidar_channels'])
    if cfg['lidar_vfov_deg']:
        vertical_fov = float(cfg['lidar_vfov_deg'])
    if cfg['drive_mode'] not in ('ackermann', 'crab', 'point_turn', 'auto'):
        raise ValueError('drive_mode must be ackermann, crab, point_turn or auto')

    x, y = float(cfg['x']), float(cfg['y'])
    z = _spawn_height(terrain_dir, cfg['world'], x, y) + REST_HEIGHT + 0.02 if cfg['z'] == 'auto' else float(cfg['z'])
    print(f'[rover_mars.launch] world={prepared} spawn=({x:.2f}, {y:.2f}, {z:.3f}) gravity={gravity}', flush=True)

    description = xacro.process_file(
        os.path.join(share, 'urdf', cfg['robot']),
        mappings={'actuator_watchdog': str(watchdog), 'enable_lidar': 'true' if lidar_enabled else 'false',
                  'lidar_profile': lidar_profile, 'lidar_gpu': lidar_gpu, 'lidar_channels': cfg['lidar_channels'],
                  'lidar_samples': cfg['lidar_samples'], 'lidar_vfov_deg': cfg['lidar_vfov_deg'],
                  'lidar_max_range': cfg['lidar_max_range'],
                  'wheel_collision': cfg['wheel_collision'], 'wheel_stall_torque': cfg['wheel_stall_torque'],
                  'wheel_no_load_speed': cfg['wheel_no_load_speed'], 'wheel_mu': cfg['wheel_mu'],
                  'mast_height': cfg['mast_height'], 'ground_truth_tf': cfg['ground_truth_tf'],
                  'suspension_mode': cfg['suspension_mode'], 'strut_inertia_size': cfg['strut_inertia_size']}).toxml()
    gazebo_launch_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    spawn_args = ['-entity', 'mars_rover', '-topic', 'robot_description', '-timeout', '120',
                  '-x', str(x), '-y', str(y), '-z', str(z),
                  '-R', cfg['roll'], '-P', cfg['pitch'], '-Y', cfg['yaw']]
    actions = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'gzserver.launch.py')),
                                 launch_arguments={'world': str(prepared), 'verbose': 'false'}.items()),
        Node(package='rover_sim', executable='swerve_controller',
             parameters=[os.path.join(share, 'config', 'swerve.yaml'),
                         {'robot_description': description, 'use_sim_time': True,
                          'drive_mode': cfg['drive_mode']}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        Node(package='gazebo_ros', executable='spawn_entity.py', arguments=spawn_args, output='screen'),
    ]
    if lidar_enabled:
        scan_from_postprocess = False
        if _truthy(cfg['scan_2d']):
            try:
                get_package_share_directory('pointcloud_to_laserscan')
                actions.append(Node(package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
                                    name='pointcloud_to_laserscan', remappings=[('cloud_in', '/points'), ('scan', '/scan')],
                                    parameters=[{'use_sim_time': True, 'target_frame': 'lidar_link', 'min_height': -0.3,
                                                 'max_height': 0.3, 'angle_min': -math.pi, 'angle_max': math.pi,
                                                 'angle_increment': math.pi / 360.0, 'range_min': 0.5, 'range_max': 100.0,
                                                 'use_inf': True}], output='screen'))
            except PackageNotFoundError:
                scan_from_postprocess = True   # fallback documented in docs/lidar.md
        actions.append(Node(package='rover_sim', executable='lidar_postprocess', name='lidar_postprocess',
                            parameters=[{'use_sim_time': True, 'channels': channels, 'vertical_fov_deg': vertical_fov,
                                         'publish_scan': scan_from_postprocess, 'seed': seed,
                                         'vertical_shift_steps': 1.0 if lidar_gpu == 'true' else 0.0}], output='screen'))
    if _truthy(cfg['gui']):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_launch_dir, 'gzclient.launch.py'))))
    if _truthy(cfg['rviz']):
        actions.append(Node(package='rviz2', executable='rviz2', name='rviz2',
                            arguments=['-d', os.path.join(share, 'rviz', 'rover_mars.rviz')],
                            parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world', default_value='mars', description='|'.join(WORLDS)),
        DeclareLaunchArgument('seed', default_value=str(DEFAULT_SEED), description='terrain seed (integer)'),
        DeclareLaunchArgument('resolution', default_value='513', description='heightmap pixels per side (513|1025)'),
        DeclareLaunchArgument('rock_density', default_value='medium', description='|'.join(DENSITIES)),
        DeclareLaunchArgument('gravity', default_value='3.71', description='gravity magnitude m/s^2 (9.81 for Earth)'),
        DeclareLaunchArgument('shadows', default_value='false', description='cast shadows (GPU cost)'),
        DeclareLaunchArgument('gui', default_value='true', description='open the Gazebo client'),
        DeclareLaunchArgument('rviz', default_value='false', description='open RViz'),
        DeclareLaunchArgument('lidar', default_value='true', description='enable the LiDAR'),
        DeclareLaunchArgument('lidar_profile', default_value='auto', description='auto|lite|standard|high|dense (auto: dense on gpu_ray, lite on the CPU sensor)'),
        DeclareLaunchArgument('lidar_gpu', default_value='auto', description='auto|true|false (gpu_ray needs OpenGL)'),
        DeclareLaunchArgument('lidar_prefer_gpu', default_value='false', description='let auto choose gpu_ray with a display'),
        DeclareLaunchArgument('lidar_channels', default_value='', description='override the profile channel count'),
        DeclareLaunchArgument('lidar_samples', default_value='', description='override the profile samples per revolution'),
        DeclareLaunchArgument('lidar_vfov_deg', default_value='', description='override the vertical field of view (deg)'),
        DeclareLaunchArgument('lidar_max_range', default_value='100.0', description='sensor maximum range (m)'),
        DeclareLaunchArgument('scan_2d', default_value='false', description='also publish a 2D /scan'),
        DeclareLaunchArgument('drive_mode', default_value='ackermann', description='initial drive mode'),
        DeclareLaunchArgument('actuator_watchdog', default_value='0.3', description='actuator watchdog wall seconds'),
        DeclareLaunchArgument('terrain_dir', default_value='', description='override generated terrain directory'),
        DeclareLaunchArgument('regenerate', default_value='false', description='force terrain regeneration'),
        DeclareLaunchArgument('robot', default_value='mars_rover.urdf.xacro', description='xacro under urdf/'),
        DeclareLaunchArgument('wheel_collision', default_value='sphere', description='sphere|multisphere|cylinder|band'),
        DeclareLaunchArgument('wheel_stall_torque', default_value='8.0', description='per-wheel stall torque N m'),
        DeclareLaunchArgument('wheel_no_load_speed', default_value='10.0', description='per-wheel no-load speed rad/s'),
        DeclareLaunchArgument('wheel_mu', default_value='0.8', description='tyre friction coefficient'),
        DeclareLaunchArgument('mast_height', default_value='0.60', description='LiDAR mast height above the deck'),
        DeclareLaunchArgument('ground_truth_tf', default_value='false', description='plugin publishes world->base_link TF'),
        DeclareLaunchArgument('suspension_mode', default_value='spring', description='spring|plain|rigid'),
        DeclareLaunchArgument('strut_inertia_size', default_value='0.10', description='strut inertia cube edge (m)'),
    ]
    for name, value in [('x', '0'), ('y', '0'), ('z', 'auto'), ('roll', '0'), ('pitch', '0'), ('yaw', '0')]:
        args.append(DeclareLaunchArgument(name, default_value=value))
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
