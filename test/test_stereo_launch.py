"""Evaluate rover_desert.launch.py for every stereo x LiDAR combination (no processes started)."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

from rover_sim.stereo_launch import LAUNCH_ARGS, parse_stereo_args

PACKAGE = Path(__file__).resolve().parents[1]


def load_description():
    spec = importlib.util.spec_from_file_location('rover_desert_launch', PACKAGE / 'launch' / 'rover_desert.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def evaluate(**overrides):
    """Return (context, robot node actions, top-level actions) for the given launch arguments."""
    description = load_description()
    context = LaunchContext()
    for entity in description.entities:
        if isinstance(entity, DeclareLaunchArgument):
            context.launch_configurations[entity.name] = entity.default_value[0].perform(context)
    context.launch_configurations.update({k: str(v) for k, v in overrides.items()})
    opaque = [e for e in description.entities if isinstance(e, OpaqueFunction)]
    assert len(opaque) == 1
    return context, opaque[0].execute(context), description.entities


def executable(node):
    return node._Node__node_executable


def perform(value):
    """launch_ros keeps parameter values as substitutions holding YAML text."""
    if isinstance(value, (list, tuple)):
        return yaml.safe_load(''.join(v.perform(LaunchContext()) for v in value))
    return value


def parameters(node):
    found = {}
    for entry in node._Node__parameters:
        if isinstance(entry, dict):
            found.update({''.join(k.perform(LaunchContext()) for k in key) if isinstance(key, tuple) else key:
                          perform(value) for key, value in entry.items()})
    return found


def robot_description(nodes):
    for node in nodes:
        if executable(node) == 'robot_state_publisher':
            return ET.fromstring(parameters(node)['robot_description'])
    raise AssertionError('robot_state_publisher has no robot_description')


def test_default_launch_enables_stereo_and_lidar():
    defaults = {e.name: e.default_value[0].perform(LaunchContext())
                for e in load_description().entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['stereo'] == 'true' and defaults['lidar'] == 'true'
    assert {k: defaults[k] for k in LAUNCH_ARGS} == LAUNCH_ARGS


def active_rviz(nodes, context):
    rviz = [n for n in nodes if isinstance(n, Node) and executable(n) == 'rviz2']
    assert len(rviz) == 1
    return rviz[0] if rviz[0].condition.evaluate(context) else None


def launch_module():
    spec = importlib.util.spec_from_file_location('rover_desert_launch', PACKAGE / 'launch' / 'rover_desert.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('stereo', [True, False])
@pytest.mark.parametrize('lidar', [True, False])
@pytest.mark.parametrize('mode', ['3d', '2d'])
def test_every_combination_produces_the_expected_nodes_and_sensors(stereo, lidar, mode):
    context, nodes, _ = evaluate(stereo=str(stereo).lower(), lidar=str(lidar).lower(), lidar_mode=mode, rviz='true')
    names = sorted(executable(n) for n in nodes if isinstance(n, Node))
    expected = ['robot_state_publisher', 'rviz2', 'spawn_entity.py', 'swerve_controller']
    if stereo:
        expected += ['stereo_camera_relay', 'stereo_processor']
    if lidar and mode == '3d':
        expected += ['lidar_postprocess']
    assert names == sorted(expected)
    robot = robot_description(nodes)
    sensors = sorted(s.get('type') for s in robot.iter('sensor'))
    assert sensors == sorted((['multicamera'] if stereo else []) + (['ray'] if lidar else []))
    links = {l.get('name') for l in robot.findall('link')}
    assert ('stereo_link' in links) == stereo and ('lidar_link' in links) == lidar
    if lidar:
        names = {s.get('name') for s in robot.iter('sensor')}
        assert ('lidar3d' in names) == (mode == '3d')
    config = active_rviz(nodes, context)._Node__arguments[1]
    expected_config = {(True, '3d'): 'rover_sensors_3d.rviz', (True, '2d'): 'rover_sensors.rviz',
                       (False, '3d'): 'rover_lidar3d.rviz', (False, '2d'): 'rover_lidar.rviz'}[(stereo, mode)]
    assert config.endswith(expected_config)


def test_default_lidar_is_the_3d_airy_dome():
    defaults = {e.name: e.default_value[0].perform(LaunchContext())
                for e in load_description().entities if isinstance(e, DeclareLaunchArgument)}
    assert defaults['lidar_mode'] == '3d' and defaults['lidar_profile'] == 'airy'


@pytest.mark.parametrize('profile, low, high, band', [
    ('lite', -15.0, 15.0, 'true'), ('standard', -15.0, 15.0, 'true'),
    ('high', -22.5, 22.5, 'true'), ('dense', -22.5, 22.5, 'true'), ('airy', -90.0, 0.0, 'false')])
def test_stereo_keepout_follows_the_3d_lidar(profile, low, high, band):
    module = launch_module()
    keepout = module.lidar_keepout('3d', profile)
    assert keepout['lidar_keepout_origin'] == '0 0 0.4'
    assert float(keepout['lidar_keepout_vfov_min']) == low
    assert float(keepout['lidar_keepout_vfov_max']) == high
    assert keepout['lidar_keepout_band'] == band    # the dome sees the camera wherever it is
    assert module.lidar_keepout('2d', profile) == {}
    # The default camera mount builds with every profile (the xacro rejects a camera in the beams).
    _, nodes, _ = evaluate(lidar_profile=profile)
    assert robot_description(nodes).find("link[@name='stereo_link']") is not None


def test_rviz_off_starts_no_rviz():
    context, nodes, _ = evaluate(rviz='false')
    assert active_rviz(nodes, context) is None


def test_stereo_nodes_share_the_exact_robot_description():
    _, nodes, _ = evaluate()
    descriptions = []
    for node in nodes:
        if executable(node) in ('stereo_camera_relay', 'stereo_processor', 'robot_state_publisher'):
            descriptions.append(parameters(node)['robot_description'])
    assert len(descriptions) == 3 and len(set(descriptions)) == 1


def test_stereo_launch_args_reach_the_sensor():
    _, nodes, _ = evaluate(stereo_width=320, stereo_height=240, stereo_rate=10, stereo_baseline=0.2,
                           stereo_hfov=1.2)
    sensor = robot_description(nodes).find("gazebo[@reference='stereo_link']/sensor")
    assert float(sensor.findtext('update_rate')) == 10
    camera = sensor.find('camera')
    assert camera.findtext('image/width') == '320' and float(camera.findtext('horizontal_fov')) == pytest.approx(1.2)
    poses = [float(c.findtext('pose').split()[1]) for c in sensor.findall('camera')]
    assert poses == pytest.approx([0.1, -0.1])


@pytest.mark.parametrize('bad', [dict(stereo_width='abc'), dict(stereo_rate='0'), dict(stereo_baseline='nan'),
                                 dict(stereo_hfov='3.5'), dict(stereo_height='100.5')])
def test_invalid_stereo_args_stop_the_launch(bad):
    with pytest.raises(ValueError):
        evaluate(**bad)
    with pytest.raises(ValueError):
        parse_stereo_args(dict(LAUNCH_ARGS, **bad))


def test_invalid_stereo_args_are_ignored_when_stereo_is_off():
    _, nodes, _ = evaluate(stereo='false', stereo_rate='0')
    assert 'stereo_processor' not in [executable(n) for n in nodes]


def rviz_displays(name):
    return yaml.safe_load((PACKAGE / 'rviz' / name).read_text())['Visualization Manager']


def test_combined_rviz_shows_robot_lidar_and_stereo_in_the_robot_frame():
    rviz = rviz_displays('rover_sensors.rviz')
    assert rviz['Global Options']['Fixed Frame'] == 'base_link'
    by_topic = {d['Topic']['Value']: d for d in rviz['Displays'] if 'Topic' in d}
    assert by_topic['/scan']['Class'] == 'rviz_default_plugins/LaserScan'
    assert by_topic['/stereo/points2']['Class'] == 'rviz_default_plugins/PointCloud2'
    assert by_topic['/stereo/points2']['Color Transformer'] == 'RGB8'
    for topic in ('/stereo/left/image_raw', '/stereo/right/image_raw', '/stereo/depth/image_raw'):
        assert by_topic[topic]['Class'] == 'rviz_default_plugins/Image'
    for topic, display in by_topic.items():
        # Viewing must never back-pressure a sensor publisher.
        assert display['Topic']['Reliability Policy'] == 'Best Effort', topic
        assert display['Topic']['Durability Policy'] == 'Volatile', topic
        assert display.get('Decay Time', 0) == 0, topic
    models = [d for d in rviz['Displays'] if d['Class'] == 'rviz_default_plugins/RobotModel']
    assert models[0]['Description Topic']['Durability Policy'] == 'Transient Local'


def test_combined_3d_rviz_swaps_the_scan_for_the_3d_cloud_and_keeps_stereo():
    sensors3d = rviz_displays('rover_sensors_3d.rviz')
    sensors = rviz_displays('rover_sensors.rviz')
    by_topic = {d['Topic']['Value']: d for d in sensors3d['Displays'] if 'Topic' in d}
    assert '/scan' not in by_topic
    assert by_topic['/points']['Class'] == 'rviz_default_plugins/PointCloud2'
    # Same settings as the 3D LiDAR's own view (reliable: fragmented clouds are dropped otherwise).
    lidar3d = [d for d in rviz_displays('rover_lidar3d.rviz')['Displays'] if d.get('Topic', {}).get('Value') == '/points']
    assert {k: v for k, v in by_topic['/points'].items() if k != 'Name'} == \
        {k: v for k, v in lidar3d[0].items() if k != 'Name'}
    rest = [d for d in sensors['Displays'] if d['Class'] != 'rviz_default_plugins/LaserScan']
    assert rest == [d for d in sensors3d['Displays'] if d.get('Topic', {}).get('Value') != '/points']
    assert sensors3d['Global Options'] == sensors['Global Options']


def test_lidar_only_rviz_is_unchanged_and_matches_the_combined_lidar_display():
    lidar = rviz_displays('rover_lidar.rviz')
    sensors = rviz_displays('rover_sensors.rviz')
    assert not [d for d in lidar['Displays'] if 'stereo' in str(d.get('Topic', {}).get('Value', ''))]
    scan = [d for d in lidar['Displays'] if d['Class'] == 'rviz_default_plugins/LaserScan']
    assert scan == [d for d in sensors['Displays'] if d['Class'] == 'rviz_default_plugins/LaserScan']


def test_default_frames_fit_default_shared_memory_and_large_frames_warn():
    from rover_sim.stereo_launch import SHM_SEGMENT_BYTES, frame_size_warning
    defaults = parse_stereo_args(LAUNCH_ARGS)
    assert defaults['width'] * defaults['height'] * 3 <= SHM_SEGMENT_BYTES - 1024
    assert frame_size_warning(defaults) is None
    assert 'shared-memory' in frame_size_warning(parse_stereo_args(dict(LAUNCH_ARGS, stereo_width='640',
                                                                        stereo_height='480')))
    _, actions, _ = evaluate(stereo_width='640', stereo_height='480')
    assert any(type(a).__name__ == 'LogInfo' for a in actions)
    _, actions, _ = evaluate()
    assert not any(type(a).__name__ == 'LogInfo' for a in actions)


def test_lidar_samples_override_reaches_the_sensor_and_is_validated():
    _, nodes, _ = evaluate(lidar_samples='450')
    scan = robot_description(nodes).find("gazebo[@reference='lidar_link']/sensor/ray/scan/horizontal")
    assert int(scan.findtext('samples')) == 450
    with pytest.raises(ValueError):
        evaluate(lidar_samples='5')


def test_dome_self_box_covers_wheels_steered_any_way():
    # Wheels of radius 0.18 m at x +-0.38, y +-0.45 under a LiDAR 0.40 m above base_link.
    xmin, xmax, ymin, ymax, zmin, zmax = launch_module().LIDAR3D_SELF_BOX
    reach = 0.18
    assert xmax >= 0.38 + reach and -xmin >= 0.38 + reach
    assert ymax >= 0.45 + reach and -ymin >= 0.45 + reach
    assert zmin <= -(0.40 + 0.33) and zmax <= 0.0          # down to the ground, never above the sensor
