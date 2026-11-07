"""Image conversion, pair checks, SGBM output handling, depth and point clouds."""
import math

import numpy as np
import pytest

from rover_sim.stereo_matching import (
    INVALID_DISPARITY, MATCHER_DEFAULTS, check_pair, compute_disparity, create_matcher,
    depth_to_points, disparity_from_sgbm, disparity_to_depth, erode, image_array, local_std, pack_xyzrgb,
    to_gray, unpack_xyz, valid_window, validate_matcher_config)

FX, B = 320.0, 0.12


def side(frame, tx, stamp=(5, 100), **changes):
    values = dict(stamp=stamp, info_stamp=stamp, image_frame=frame, info_frame=frame,
                  width=640, height=480, info_width=640, info_height=480, d=[0.0] * 5,
                  k=[FX, 0, 319.5, 0, FX, 239.5, 0, 0, 1], r=[1, 0, 0, 0, 1, 0, 0, 0, 1],
                  p=[FX, 0, 319.5, tx, 0, FX, 239.5, 0, 0, 0, 1, 0])
    values.update(changes)
    return values


def pair(**right_changes):
    return side('stereo_left_optical_frame', 0.0), side('stereo_right_optical_frame', -FX * B, **right_changes)


def test_defaults_are_valid():
    assert validate_matcher_config({}) == MATCHER_DEFAULTS


@pytest.mark.parametrize('change', [
    dict(num_disparities=40), dict(num_disparities=0), dict(block_size=4), dict(block_size=13),
    dict(min_disparity=-1), dict(uniqueness_ratio=101), dict(min_valid_disparity=0),
    dict(min_valid_disparity=math.nan), dict(cloud_stride=0), dict(block_size=5.0), dict(unknown=1),
    dict(min_texture=-1.0), dict(min_texture=math.inf), dict(texture_window=8), dict(texture_window=1),
    dict(edge_erosion=-1), dict(edge_erosion=6), dict(edge_erosion=1.0)])
def test_invalid_matcher_config_rejected(change):
    with pytest.raises(ValueError):
        validate_matcher_config(change)


def test_image_array_respects_row_padding():
    rows = [bytes([r, r, r]) * 4 + b'\xff\xff' for r in range(3)]  # width 4, step 14
    array = image_array('rgb8', 4, 3, 14, b''.join(rows))
    assert array.shape == (3, 4, 3)
    assert array[2, 3].tolist() == [2, 2, 2]


@pytest.mark.parametrize('args', [
    ('rgb8', 4, 3, 11, bytes(33)), ('rgb8', 4, 3, 12, bytes(35)), ('yuv422', 4, 3, 8, bytes(24)),
    ('mono8', 0, 3, 4, bytes(0))])
def test_bad_image_fields_fail_closed(args):
    with pytest.raises(ValueError):
        image_array(*args)


def test_rgb_and_bgr_give_the_same_gray():
    rgb = np.random.default_rng(1).integers(0, 256, (6, 5, 3), dtype=np.uint8)
    assert np.array_equal(to_gray(rgb, 'rgb8'), to_gray(rgb[:, :, ::-1].copy(), 'bgr8'))
    assert to_gray(rgb, 'rgb8').dtype == np.uint8


def test_consistent_pair_returns_geometry():
    geometry = check_pair(*pair(), expected_baseline=B)
    assert geometry.baseline == pytest.approx(B)
    assert (geometry.fx, geometry.cx, geometry.cy) == (FX, 319.5, 239.5)


@pytest.mark.parametrize('right_changes', [
    dict(stamp=(5, 101), info_stamp=(5, 101)), dict(info_stamp=(5, 99)),
    dict(image_frame='stereo_left_optical_frame', info_frame='stereo_left_optical_frame'),
    dict(info_frame='other'), dict(p=[FX, 0, 319.5, 0.0, 0, FX, 239.5, 0, 0, 0, 1, 0]),
    dict(p=[FX, 0, 319.5, FX * B, 0, FX, 239.5, 0, 0, 0, 1, 0]),
    dict(p=[FX + 1, 0, 319.5, -FX * B, 0, FX, 239.5, 0, 0, 0, 1, 0]),
    dict(d=[0.1, 0, 0, 0, 0]), dict(width=320, info_width=320), dict(info_height=240),
    dict(r=[0, 1, 0, 1, 0, 0, 0, 0, 1]), dict(k=[math.nan] * 9)])
def test_inconsistent_pairs_fail_closed(right_changes):
    with pytest.raises(ValueError):
        check_pair(*pair(**right_changes), expected_baseline=B)


def test_left_tx_must_be_zero_and_baseline_must_match():
    left, right = pair()
    left['p'] = list(left['p'])
    left['p'][3] = -1.0
    with pytest.raises(ValueError):
        check_pair(left, right)
    with pytest.raises(ValueError):
        check_pair(*pair(), expected_baseline=0.2)


def test_sgbm_fixed_point_conversion_and_valid_window():
    config = validate_matcher_config({})
    raw = np.full((480, 640), 16 * 10, dtype=np.int16)
    raw[300, 400] = -16          # SGBM invalid marker
    raw[301, 400] = 16 * 1       # below min_valid_disparity (2 px)
    disparity = disparity_from_sgbm(raw, config)
    x, y, w, h = valid_window(640, 480, config)
    assert (x, y, w, h) == (81, 2, 557, 476)   # 80 disparities, 5 px block
    assert disparity[240, 320] == 10.0
    assert disparity[300, 400] == INVALID_DISPARITY and disparity[301, 400] == INVALID_DISPARITY
    assert np.all(disparity[:, :x] == INVALID_DISPARITY)
    assert np.all(disparity[:y, :] == INVALID_DISPARITY)


def test_local_std_matches_a_direct_window_computation():
    gray = np.random.default_rng(5).integers(0, 256, (20, 30)).astype(np.uint8)
    std = local_std(gray, 5)
    assert std[10, 12] == pytest.approx(gray[8:13, 10:15].astype(float).std())
    assert local_std(np.full((10, 10), 77, np.uint8), 3).max() == pytest.approx(0.0, abs=1e-6)


def test_erosion_shrinks_regions_without_wrapping_around_the_image():
    mask = np.zeros((7, 9), bool)
    mask[1:6, 2:8] = True
    eroded = erode(mask, 1)
    assert eroded.sum() == 3 * 4 and eroded[2:5, 3:7].all()
    edge = np.ones((4, 4), bool)
    assert not erode(edge, 1)[0].any() and not erode(edge, 1)[:, -1].any()   # image border is not "valid"
    assert np.array_equal(erode(mask, 0), mask)


def test_textureless_regions_are_invalid_not_guessed():
    cv2 = pytest.importorskip('cv2')
    config = validate_matcher_config(dict(num_disparities=32))
    rng = np.random.default_rng(9)
    texture = rng.integers(0, 256, (120, 200), dtype=np.uint8)
    texture[:60] = 128 + rng.integers(-2, 3, (60, 200))   # flat "sky": only camera-like noise
    left, right = np.ascontiguousarray(texture[:, :160]), np.ascontiguousarray(texture[:, 6:166])
    disparity = compute_disparity(create_matcher(cv2, config), left, right, config)
    assert (disparity[:55] == INVALID_DISPARITY).all()
    assert (disparity[65:] > 0).mean() > 0.5
    off = validate_matcher_config(dict(num_disparities=32, min_texture=0.0))
    assert (compute_disparity(create_matcher(cv2, off), left, right, off)[:55] > 0).any()


def test_depth_is_fx_b_over_d_and_invalid_is_nan():
    depth = disparity_to_depth(np.array([[10.0, -1.0, 0.0], [np.nan, 1.0, 38.4]], np.float32), FX, B, 2.0)
    assert depth[0, 0] == pytest.approx(3.84)
    assert depth[1, 2] == pytest.approx(1.0)
    assert np.isnan(depth[0, 1]) and np.isnan(depth[0, 2]) and np.isnan(depth[1, 0]) and np.isnan(depth[1, 1])


@pytest.mark.parametrize('args', [(0.0, B, 2.0), (FX, -B, 2.0), (FX, B, 0.0), (math.nan, B, 2.0)])
def test_depth_parameters_fail_closed(args):
    with pytest.raises(ValueError):
        disparity_to_depth(np.ones((2, 2), np.float32), *args)


def test_back_projection_uses_optical_axes():
    depth = np.full((480, 640), np.nan, np.float32)
    depth[239, 639] = 2.0     # right of centre, near cy
    depth[0, 319] = 4.0       # top row
    points, rows, cols = depth_to_points(depth, FX, FX, 319.5, 239.5)
    by_pixel = {(r, c): p for r, c, p in zip(rows, cols, points)}
    assert by_pixel[(239, 639)] == pytest.approx([2.0 * 319.5 / FX, -0.5 * 2.0 / FX, 2.0])
    assert by_pixel[(0, 319)] == pytest.approx([-0.5 * 4.0 / FX, -239.5 * 4.0 / FX, 4.0])
    assert len(depth_to_points(depth, FX, FX, 319.5, 239.5, stride=2)[0]) == 0


def test_point_cloud_packing_round_trip():
    points = np.array([[1.0, -2.0, 3.5], [0.25, 0.5, 9.0]], np.float32)
    colors = np.array([[255, 0, 1], [10, 20, 30]], np.uint8)
    data = pack_xyzrgb(points, colors)
    assert len(data) == 32
    assert unpack_xyz(data, 2) == [pytest.approx(p) for p in points.tolist()]
    rgb = np.frombuffer(data, dtype='<u4').reshape(2, 4)[:, 3]
    assert rgb.tolist() == [0xFF0001, 0x0A141E]
    with pytest.raises(ValueError):
        pack_xyzrgb(points, colors[:1])


def test_sgbm_recovers_a_known_shift_on_random_texture():
    cv2 = pytest.importorskip('cv2')
    config = validate_matcher_config({})
    texture = np.random.default_rng(7).integers(0, 256, (240, 400), dtype=np.uint8)
    texture = cv2.GaussianBlur(texture, (3, 3), 0)
    shift = 12
    # A point at left column u appears at right column u - shift.
    left = np.ascontiguousarray(texture[:, :320])
    right = np.ascontiguousarray(texture[:, shift:shift + 320])
    disparity = compute_disparity(create_matcher(cv2, config), left, right, config)
    valid = disparity[disparity != INVALID_DISPARITY]
    assert valid.size > 0.6 * disparity.size * 0.7
    assert float(np.median(valid)) == pytest.approx(shift, abs=0.1)
    depth = disparity_to_depth(disparity, FX, B, config['min_valid_disparity'])
    assert float(np.nanmedian(depth)) == pytest.approx(FX * B / shift, rel=0.01)
