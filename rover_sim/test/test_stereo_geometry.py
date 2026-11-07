"""Stereo intrinsics, camera_info, baseline and LiDAR keep-out math (no simulator)."""
import math

import pytest

from rover_sim.stereo_geometry import (
    Keepout, baseline_from_urdf, camera_info_fields, intrinsics, keepout_violations,
    link_transforms, point_in_keepout, right_projection, validate_stereo_config)


def urdf(left_xyz='0.03 0.06 0', right_xyz='0.03 -0.06 0', rpy='-1.5707963267948966 0 -1.5707963267948966',
         stereo_xyz='0.52 0 0'):
    return f"""<robot name="r">
      <link name="base_link"/><link name="stereo_link">
        <visual><geometry><box size="0.04 0.18 0.03"/></geometry></visual></link>
      <link name="stereo_left_optical_frame"/><link name="stereo_right_optical_frame"/>
      <joint name="stereo_joint" type="fixed"><parent link="base_link"/><child link="stereo_link"/>
        <origin xyz="{stereo_xyz}" rpy="0 0 0"/></joint>
      <joint name="l" type="fixed"><parent link="stereo_link"/><child link="stereo_left_optical_frame"/>
        <origin xyz="{left_xyz}" rpy="{rpy}"/></joint>
      <joint name="r" type="fixed"><parent link="stereo_link"/><child link="stereo_right_optical_frame"/>
        <origin xyz="{right_xyz}" rpy="{rpy}"/></joint>
    </robot>"""


def test_default_intrinsics_follow_gazebo_symmetric_pinhole():
    k = intrinsics(640, 480, math.pi / 2)
    assert k['fx'] == pytest.approx(320.0)
    assert k['fy'] == pytest.approx(320.0)
    assert (k['cx'], k['cy']) == (319.5, 239.5)


def test_camera_info_tx_is_zero_left_and_minus_fx_baseline_right():
    left = camera_info_fields(640, 480, math.pi / 2, 0.12, 'left')
    right = camera_info_fields(640, 480, math.pi / 2, 0.12, 'right')
    assert left['p'][3] == 0.0
    assert right['p'][3] == pytest.approx(-320.0 * 0.12)
    assert left['k'] == right['k']
    assert left['frame_id'] == 'stereo_left_optical_frame'
    assert right['frame_id'] == 'stereo_right_optical_frame'
    assert left['d'] == [0.0] * 5 and left['r'] == [1, 0, 0, 0, 1, 0, 0, 0, 1]


def test_right_projection_sets_only_tx():
    p = [300.0, 0, 319.5, 0, 0, 300.0, 239.5, 0, 0, 0, 1, 0]
    corrected = right_projection(p, 0.2)
    assert corrected[3] == pytest.approx(-60.0)
    assert corrected[:3] + corrected[4:] == p[:3] + p[4:]
    assert p[3] == 0  # input untouched


@pytest.mark.parametrize('p, baseline', [
    ([0.0] * 12, 0.12), ([300.0] + [math.nan] * 11, 0.12), ([300.0] * 11, 0.12),
    ([300.0] + [0.0] * 11, 0.0), ([300.0] + [0.0] * 11, math.inf)])
def test_right_projection_fails_closed(p, baseline):
    with pytest.raises(ValueError):
        right_projection(p, baseline)


@pytest.mark.parametrize('kwargs', [
    dict(width=640.0), dict(width=True), dict(width=32), dict(height=5000), dict(rate=0),
    dict(rate=math.nan), dict(rate=100), dict(baseline=0), dict(baseline=2), dict(hfov=3.0),
    dict(hfov=math.inf)])
def test_invalid_stereo_config_rejected(kwargs):
    values = dict(width=640, height=480, rate=15.0, baseline=0.12, hfov=1.5708)
    values.update(kwargs)
    with pytest.raises(ValueError):
        validate_stereo_config(**values)


def test_baseline_read_from_optical_joints():
    assert baseline_from_urdf(urdf()) == pytest.approx(0.12)


@pytest.mark.parametrize('kwargs', [
    dict(rpy='0 0 0'), dict(left_xyz='0.03 -0.06 0', right_xyz='0.03 0.06 0'),
    dict(right_xyz='0.04 -0.06 0'), dict(left_xyz='0.03 0.06 0.01')])
def test_baseline_rejects_unrectified_layouts(kwargs):
    with pytest.raises(ValueError):
        baseline_from_urdf(urdf(**kwargs))


def test_baseline_requires_both_frames():
    with pytest.raises(ValueError):
        baseline_from_urdf(urdf().replace('stereo_right_optical_frame', 'other'))


def test_link_transforms_compose_fixed_joints():
    poses = link_transforms(urdf())
    translation, matrix = poses['stereo_left_optical_frame']
    assert translation == pytest.approx((0.55, 0.06, 0.0))
    # Optical z (forward) is base x; optical x (right) is base -y; optical y (down) is base -z.
    assert [row[2] for row in matrix] == pytest.approx([1, 0, 0], abs=1e-12)
    assert [row[0] for row in matrix] == pytest.approx([0, -1, 0], abs=1e-12)
    assert [row[1] for row in matrix] == pytest.approx([0, 0, -1], abs=1e-12)


def test_keepout_housing_band_and_outside_points():
    k = Keepout()
    assert point_in_keepout((0.0, 0.0, 0.20), k)            # LiDAR origin
    assert point_in_keepout((0.1, 0.0, 0.25), k)            # housing zone
    assert point_in_keepout((5.0, 3.0, 0.20), k)            # in the beam plane, far away
    assert point_in_keepout((1.0, 0.0, 0.20 - math.tan(math.radians(16.5))), k)  # inside the margin
    assert not point_in_keepout((1.0, 0.0, 0.20 - math.tan(math.radians(17.5))), k)
    assert not point_in_keepout((0.52, 0.0, 0.0), k)        # default stereo mount, about -21 degrees


@pytest.mark.parametrize('kwargs', [
    dict(vfov_min=20, vfov_max=10), dict(margin=-1), dict(radius=-0.1),
    dict(origin=(0, 0)), dict(vfov_max=math.nan)])
def test_invalid_keepout_rejected(kwargs):
    with pytest.raises(ValueError):
        Keepout(**kwargs)


def test_keepout_violations_report_stereo_links_in_the_band():
    assert keepout_violations(urdf(), Keepout()) == []
    bad = keepout_violations(urdf(stereo_xyz='0.4 0 0.18'), Keepout())
    assert bad and {name for name, _ in bad} >= {'stereo_link'}
    wide = keepout_violations(urdf(), Keepout(vfov_min=-30))
    assert wide
