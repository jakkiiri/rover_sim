"""3D sensor contracts, geometry and deterministic point-cloud regressions."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro
import yaml
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from rover_sim.kinematics import modules_from_urdf
from rover_sim.lidar_postprocess import cloud_to_array, make_cloud, make_scan, process_cloud


PACKAGE = Path(__file__).resolve().parents[1]


def expand(**mappings):
    return ET.fromstring(xacro.process_file(
        str(PACKAGE / 'urdf/proxy_rover.urdf.xacro'),
        mappings={'lidar_mode': '3d', **mappings}).toxml())


def sensor(root):
    sensors = root.findall("gazebo[@reference='lidar_link']/sensor")
    assert len(sensors) == 1
    return sensors[0]


@pytest.mark.parametrize('profile,channels,samples,low,high,min_range', [
    ('lite', 16, 450, -15, 15, .5), ('standard', 16, 900, -15, 15, .5),
    ('high', 32, 1024, -22.5, 22.5, .5), ('dense', 64, 1024, -22.5, 22.5, .5),
    # Airy: 48 beams from -90 to 0 deg; the lowest 21 only see the rover and are not simulated.
    # The top beam sits 0.1 deg below the horizon (Gazebo's horizontal rays hit the Mars mesh falsely).
    ('airy', 27, 900, -90 + 21 * 90 / 47, -0.1, .1)])
def test_profile_beams(profile, channels, samples, low, high, min_range):
    s = sensor(expand(lidar_profile=profile))
    assert s.get('type') == 'ray'
    assert float(s.findtext('update_rate')) == 10
    h, v = s.find('ray/scan/horizontal'), s.find('ray/scan/vertical')
    assert int(h.findtext('samples')) == samples
    assert int(v.findtext('samples')) == channels
    assert [float(h.findtext(k)) for k in ('min_angle', 'max_angle')] == pytest.approx([-math.pi, math.pi])
    assert [float(v.findtext(k)) for k in ('min_angle', 'max_angle')] == pytest.approx(
        [math.radians(low), math.radians(high)])
    assert float(s.findtext('ray/range/min')) == min_range


def test_operator_sensor_is_cpu_only_with_classic_interface():
    s = sensor(expand())
    assert s.get('type') == 'ray'
    v = s.find('ray/scan/vertical')
    assert int(v.findtext('samples')) == 16
    assert math.degrees(float(v.findtext('min_angle'))) == pytest.approx(-15)
    plugin = s.find('plugin')
    assert plugin.get('filename') == 'libgazebo_ros_ray_sensor.so'
    assert plugin.findtext('output_type') == 'sensor_msgs/PointCloud2'
    assert plugin.findtext('frame_name') == 'lidar_link'
    assert plugin.findtext('ros/remapping') == '~/out:=/points_raw'
    assert expand().find('.//ignition_frame_id') is None
    wrapper = ET.parse(PACKAGE / 'urdf/sensors/lidar3d.urdf.xacro')
    call = wrapper.find('.//{http://ros.org/wiki/xacro}rover_lidar3d')
    assert call.get('gpu') == 'false'


def test_3d_range_and_sensor_noise():
    s = sensor(expand())
    assert float(s.findtext('ray/range/min')) == .5
    assert float(s.findtext('ray/range/max')) == 100
    assert s.findtext('ray/noise/type') == 'gaussian'
    assert float(s.findtext('ray/noise/stddev')) == .02


def test_mount_clears_chassis_and_wheels_at_widest_fov():
    root = expand(lidar_profile='dense')
    joint = root.find("joint[@name='lidar_joint']")
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').get('link') == 'base_link'
    assert joint.find('origin').get('xyz') == '0 0 0.40'
    # Even a -22.5 degree beam clears the far chassis corner and tire top.
    assert .4 - math.hypot(.5, .25) * math.tan(math.radians(22.5)) > .1
    assert .4 - math.hypot(.38, .51) * math.tan(math.radians(22.5)) > .03
    mass = float(root.find("link[@name='lidar_link']/inertial/mass").get('value'))
    assert mass == .8
    links = {e.get('name') for e in root.findall('link')}
    children = {e.find('child').get('link') for e in root.findall('joint')}
    assert links - children == {'base_root'}
    assert not {'odom', 'map', 'imu_link'} & links


def test_modes_do_not_change_actuators_or_drive_geometry():
    roots = [expand(enable_lidar='false'), expand(lidar_mode='2d'), expand()]
    assert roots[0].find("link[@name='lidar_link']") is None
    expected = modules_from_urdf(ET.tostring(roots[0]))
    for root in roots[1:]:
        assert modules_from_urdf(ET.tostring(root)) == expected
        for j in roots[0].findall('joint'):
            assert ET.tostring(root.find(f"joint[@name='{j.get('name')}']")) == ET.tostring(j)
        path = ".//plugin[@filename='librover_swerve_actuators.so']"
        assert ET.tostring(root.find(path)) == ET.tostring(roots[0].find(path))


def points(xyz):
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    result = np.zeros(len(xyz), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    result['x'], result['y'], result['z'] = xyz.T
    return result


def process(p, **kwargs):
    args = dict(channels=16, vertical_fov=math.radians(30), rate=10., sigma0=0.,
                sigma_per_metre=0., p0=0., p_per_metre=0., min_range=.5, max_range=100.)
    args.update(kwargs)
    return process_cloud(p, np.random.default_rng(3), **args)


def test_seeded_dropout_and_range_dependent_noise():
    p = points([[r, 0, 0] for r in np.linspace(1, 90, 10000)])
    out = process(p, p0=.1, sigma0=.01, sigma_per_metre=.001)
    again = process(p, p0=.1, sigma0=.01, sigma_per_metre=.001)
    assert 8800 < len(out[0]) < 9200
    for a, b in zip(out, again):
        assert np.array_equal(a, b)
    r = out[0][:, 0]
    assert np.all(np.isfinite(r)) and np.all((r >= .5) & (r <= 100))


def test_noise_scales_with_range():
    p = points([[2, 0, 0]] * 10000 + [[50, 0, 0]] * 10000)
    xyz, *_ = process(p, sigma0=.01, sigma_per_metre=.001)
    assert np.std(xyz[:10000, 0]) == pytest.approx(.012, rel=.1)
    assert np.std(xyz[10000:, 0]) == pytest.approx(.06, rel=.1)


def test_nonfinite_out_of_range_and_empty_returns():
    p = points([[math.nan, 0, 0], [math.inf, 0, 0], [.1, 0, 0], [101, 0, 0], [2, 0, 0]])
    xyz, *_ = process(p)
    assert np.array_equal(xyz, [[2, 0, 0]])
    xyz, intensity, ring, timing = process(points([]))
    msg = make_cloud(Header(frame_id='lidar_link'), xyz, intensity, ring, timing)
    assert msg.width == 0 and len(cloud_to_array(msg)) == 0


def test_extreme_noise_cannot_flip_rays_or_exceed_bounds():
    xyz, *_ = process(points([[1, 0, 0]] * 1000), sigma0=100.)
    assert len(xyz) > 0
    assert np.all((xyz[:, 0] >= .5) & (xyz[:, 0] <= 100))


def test_ring_time_and_little_endian_cloud_round_trip():
    xyz = []
    for k in range(16):
        e = math.radians(-15 + k * 2)
        for a in (-math.pi, -.4, .4, math.pi):
            xyz.append([5 * math.cos(e) * math.cos(a), 5 * math.cos(e) * math.sin(a), 5 * math.sin(e)])
    output = process(points(xyz))
    msg = make_cloud(Header(frame_id='lidar_link'), *output)
    array = cloud_to_array(msg)
    assert msg.header.frame_id == 'lidar_link' and msg.point_step == 22
    assert msg.is_bigendian is False and msg.is_dense
    assert np.allclose(array['x'], output[0][:, 0])
    assert set(array['ring']) == set(range(16))
    assert np.all((array['time'] >= 0) & (array['time'] <= .100001))


def test_gpu_label_correction_restores_ground_height():
    height = .73
    xyz = []
    for k in range(8):
        label = math.radians(-17 + k * 2)
        true_angle = label + math.radians(2)
        r = height / math.sin(-true_angle)
        xyz.append([r * math.cos(label), 0, r * math.sin(label)])
    result, _, ring, _ = process(points(xyz), vertical_shift_steps=1.)
    assert np.allclose(result[:, 2], -height, atol=1e-6)
    assert set(ring) == set(range(8))


def test_gpu_correction_discards_extra_top_beam():
    e = math.radians(15)
    result, *_ = process(points([[5 * math.cos(e), 0, 5 * math.sin(e)]]), vertical_shift_steps=1.)
    assert len(result) == 0


def test_single_channel_ring_is_zero():
    assert np.array_equal(process(points([[2, 0, 0]]), channels=1)[2], [0])


def test_big_endian_organized_cloud_with_padded_rows():
    fields = [PointField(name=name, offset=i*4, datatype=PointField.FLOAT32, count=1)
              for i, name in enumerate(('x', 'y', 'z'))]
    rows = [np.array([[1, 2, 3], [4, 5, 6]], dtype='>f4').tobytes() + b'pad!',
            np.array([[7, 8, 9], [10, 11, 12]], dtype='>f4').tobytes() + b'pad!']
    msg = PointCloud2(height=2, width=2, fields=fields, is_bigendian=True,
                      point_step=12, row_step=28, data=b''.join(rows))
    assert cloud_to_array(msg)['x'].tolist() == [1, 4, 7, 10]


@pytest.mark.parametrize('malformation', ['truncated', 'duplicate', 'missing', 'offset', 'count', 'stride'])
def test_malformed_cloud_rejected(malformation):
    msg = make_cloud(Header(), *process(points([[2, 0, 0]])))
    if malformation == 'truncated': msg.data = msg.data[:-1]
    if malformation == 'duplicate': msg.fields[1].name = 'x'
    if malformation == 'missing': msg.fields[1].name = 'other'
    if malformation == 'offset': msg.fields[0].offset = msg.point_step
    if malformation == 'count': msg.fields[0].count = 2
    if malformation == 'stride': msg.row_step = 1
    with pytest.raises(ValueError):
        cloud_to_array(msg)


def test_optional_scan_nearest_bin_wrap_and_metadata():
    xyz = np.array([[2, 0, 0], [3, 0, 0], [-4, 0, 0], [5, 0, 0]], dtype=float)
    scan = make_scan(Header(frame_id='lidar_link'), xyz, np.array([8, 8, 8, 7]), 8, 720, .5, 100.)
    assert scan.angle_max == pytest.approx(scan.angle_min + 719 * scan.angle_increment)
    assert scan.ranges[360] == 2 and scan.ranges[0] == 4
    assert sum(math.isfinite(r) for r in scan.ranges) == 2
    assert scan.time_increment == 0


def test_rviz_local_current_cloud_without_odometry():
    vm = yaml.safe_load((PACKAGE / 'rviz/rover_lidar3d.rviz').read_text())['Visualization Manager']
    assert vm['Global Options']['Fixed Frame'] == 'base_link'
    cloud = next(d for d in vm['Displays'] if d['Class'] == 'rviz_default_plugins/PointCloud2')
    assert cloud['Enabled'] and cloud['Topic']['Value'] == '/points'
    assert cloud['Topic']['Reliability Policy'] == 'Reliable'
    assert cloud['Decay Time'] == 0
    assert not any(d['Class'] == 'rviz_default_plugins/Odometry' for d in vm['Displays'])


def test_postprocessor_repeated_signals_destroy_once(monkeypatch):
    import signal
    import rover_sim.lidar_postprocess as module
    handlers, calls = {}, []
    monkeypatch.setattr(module.signal, 'signal', lambda sig, handler: handlers.update({sig: handler}))
    monkeypatch.setattr(module.rclpy, 'init', lambda **kw: calls.append('init'))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(module.rclpy, 'try_shutdown', lambda: calls.append('shutdown'))

    def signals():
        for sig in (signal.SIGINT, signal.SIGINT, signal.SIGTERM):
            handlers[sig](sig, None)

    class Node:
        def destroy_node(self):
            signals()
            calls.append('destroy')

    monkeypatch.setattr(module, 'LidarPostprocess', Node)
    monkeypatch.setattr(module.rclpy, 'spin_once', lambda *a, **kw: signals())
    module.main()
    assert calls == ['init', 'destroy', 'shutdown']


def _dome_points(elevations_deg, r=5.0):
    e = np.radians(np.asarray(elevations_deg, dtype=float))
    pts = np.zeros(len(e), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    pts['x'], pts['z'] = r * np.cos(e), r * np.sin(e)
    return pts


def test_dome_rings_count_up_from_the_lowest_beam():
    # Airy dome: 48 beams from -90 to 0 deg.
    fov, low = math.radians(90), math.radians(-90)
    pts = _dome_points([-90, -45, 0])
    _, _, ring, _ = process_cloud(pts, np.random.default_rng(0), 48, fov, 10.0, 0, 0, 0, 0, .1, 100,
                                  vertical_min=low)
    assert ring.tolist() == [0, 24, 47]


def test_self_filter_box_drops_the_rovers_own_body():
    pts = np.zeros(3, dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])
    pts['x'], pts['z'] = [0.3, 0.3, 3.0], [-0.4, -0.4, -0.73]    # chassis, chassis, ground ahead
    pts['y'] = [0.0, 0.2, 0.0]
    box = (-0.6, 0.62, -0.55, 0.55, -0.8, 0.0)
    xyz, *_ = process_cloud(pts, np.random.default_rng(0), 48, math.radians(90), 10.0, 0, 0, 0, 0, .1, 100,
                            vertical_min=math.radians(-90), self_box=box)
    assert len(xyz) == 1 and xyz[0, 0] == pytest.approx(3.0, abs=.01)
    kept, *_ = process_cloud(pts, np.random.default_rng(0), 48, math.radians(90), 10.0, 0, 0, 0, 0, .1, 100,
                             self_box=(0, 0, 0, 0, 0, 0))   # zero box: off
    assert len(kept) == 3
