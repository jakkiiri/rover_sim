"""Stereo relay and processor callbacks, without ROS discovery or Gazebo."""
from types import SimpleNamespace

import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo, Image, PointField
from stereo_msgs.msg import DisparityImage

from rover_sim.stereo_geometry import RIGHT_FRAME
from rover_sim.stereo_matching import create_matcher, unpack_xyz, validate_matcher_config
from rover_sim.stereo_processor import StereoProcessor, side_fields
from rover_sim.stereo_relay import StereoCameraRelay

FX, B, W, H = 160.0, 0.12, 320, 240


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def logger():
    warnings = []
    return SimpleNamespace(warning=lambda text, **kw: warnings.append(text), info=lambda text: None), warnings


def floats(*values):
    return [float(v) for v in values]


def projection(tx):
    return floats(FX, 0, (W - 1) / 2, tx, 0, FX, (H - 1) / 2, 0, 0, 0, 1, 0)


def info(frame, tx, stamp=(3, 5)):
    msg = CameraInfo()
    msg.header.frame_id = frame
    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
    msg.width, msg.height = W, H
    msg.distortion_model = 'plumb_bob'
    msg.d = [0.0] * 5
    msg.k = floats(FX, 0, (W - 1) / 2, 0, FX, (H - 1) / 2, 0, 0, 1)
    msg.r = floats(1, 0, 0, 0, 1, 0, 0, 0, 1)
    msg.p = projection(tx)
    return msg


def image(frame, array, stamp=(3, 5)):
    msg = Image()
    msg.header.frame_id = frame
    msg.header.stamp.sec, msg.header.stamp.nanosec = stamp
    msg.height, msg.width = array.shape[:2]
    msg.encoding, msg.step = 'rgb8', 3 * msg.width
    msg.data = np.ascontiguousarray(array).tobytes()
    return msg


@pytest.fixture
def relay():
    log, warnings = logger()
    fake = SimpleNamespace(baseline=B, image_pub=Recorder(), info_pub=Recorder(), get_logger=lambda: log)
    return fake, warnings


def test_relay_relabels_right_image_without_touching_stamp_or_pixels(relay):
    fake, _ = relay
    msg = image('stereo_left_optical_frame', np.zeros((H, W, 3), np.uint8))
    StereoCameraRelay.on_image(fake, msg)
    out = fake.image_pub.messages[-1]
    assert out.header.frame_id == RIGHT_FRAME
    assert (out.header.stamp.sec, out.header.stamp.nanosec) == (3, 5)
    assert bytes(out.data) == bytes(msg.data)


def test_relay_sets_right_tx_and_frame(relay):
    fake, _ = relay
    StereoCameraRelay.on_info(fake, info('stereo_left_optical_frame', 0.0))
    out = fake.info_pub.messages[-1]
    assert out.header.frame_id == RIGHT_FRAME
    assert out.p[3] == pytest.approx(-FX * B)
    assert list(out.k) == list(info('x', 0.0).k)


def test_relay_drops_invalid_camera_info(relay):
    fake, warnings = relay
    bad = info('stereo_left_optical_frame', 0.0)
    bad.p[0] = float('nan')
    StereoCameraRelay.on_info(fake, bad)
    assert fake.info_pub.messages == [] and warnings


@pytest.fixture
def processor():
    cv2 = pytest.importorskip('cv2')
    config = validate_matcher_config(dict(num_disparities=32))
    log, warnings = logger()
    fake = SimpleNamespace(config=config, matcher=create_matcher(cv2, config), expected_baseline=B,
                           baseline_tolerance=0.002, stats=dict(processed=0, rejected=0, seconds=0.0),
                           disparity_pub=Recorder(), depth_pub=Recorder(), depth_info_pub=Recorder(),
                           cloud_pub=Recorder(), get_logger=lambda: log)
    fake.publish = lambda *args: StereoProcessor.publish(fake, *args)
    fake.reject = lambda reason: StereoProcessor.reject(fake, reason)
    return fake, warnings


def textured_pair(shift):
    rng = np.random.default_rng(11)
    texture = rng.integers(0, 256, (H, W + shift, 3), dtype=np.uint8)
    return texture[:, :W], texture[:, shift:shift + W]   # left column u == right column u - shift


def test_processor_publishes_consistent_disparity_depth_and_cloud(processor):
    fake, warnings = processor
    left, right = textured_pair(8)
    StereoProcessor.on_pair(fake, image('stereo_left_optical_frame', left), info('stereo_left_optical_frame', 0.0),
                            image(RIGHT_FRAME, right), info(RIGHT_FRAME, -FX * B))
    assert warnings == [] and fake.stats['processed'] == 1
    disparity = fake.disparity_pub.messages[-1]
    assert isinstance(disparity, DisparityImage)
    assert disparity.header.frame_id == 'stereo_left_optical_frame'
    assert (disparity.f, disparity.t) == pytest.approx((FX, B))
    assert disparity.image.encoding == '32FC1' and disparity.max_disparity == 32
    d = np.frombuffer(bytes(disparity.image.data), '<f4').reshape(H, W)
    assert float(np.median(d[d > 0])) == pytest.approx(8, abs=0.1)
    depth = np.frombuffer(bytes(fake.depth_pub.messages[-1].data), '<f4').reshape(H, W)
    assert float(np.nanmedian(depth)) == pytest.approx(FX * B / 8, rel=0.01)
    assert np.isnan(depth[:, :10]).all()   # outside the valid window: invalid, not guessed
    assert fake.depth_info_pub.messages[-1].header.frame_id == 'stereo_left_optical_frame'
    cloud = fake.cloud_pub.messages[-1]
    assert cloud.header.frame_id == 'stereo_left_optical_frame' and cloud.height == 1
    assert [f.name for f in cloud.fields] == ['x', 'y', 'z', 'rgb']
    assert cloud.fields[3].datatype == PointField.UINT32 and cloud.is_dense
    z = [p[2] for p in unpack_xyz(bytes(cloud.data), cloud.width)]
    assert cloud.width > 1000 and np.median(z) == pytest.approx(FX * B / 8, rel=0.01)


@pytest.mark.parametrize('break_input', [
    lambda l, li, r, ri: setattr(r.header.stamp, 'nanosec', 6),           # unsynchronised
    lambda l, li, r, ri: setattr(ri, 'p', projection(0.0)),
    lambda l, li, r, ri: setattr(ri, 'p', projection(-FX * 0.2)),
    lambda l, li, r, ri: setattr(r.header, 'frame_id', 'stereo_left_optical_frame'),
    lambda l, li, r, ri: setattr(r, 'encoding', 'bgr8'),
    lambda l, li, r, ri: setattr(l, 'encoding', '16UC1'),
    lambda l, li, r, ri: setattr(l, 'data', bytes(10)),
    lambda l, li, r, ri: setattr(li, 'd', floats(0.1, 0, 0, 0, 0))])
def test_processor_fails_closed_on_inconsistent_input(processor, break_input):
    fake, warnings = processor
    left, right = textured_pair(8)
    msgs = [image('stereo_left_optical_frame', left), info('stereo_left_optical_frame', 0.0),
            image(RIGHT_FRAME, right), info(RIGHT_FRAME, -FX * B)]
    break_input(*msgs)
    StereoProcessor.on_pair(fake, *msgs)
    assert warnings and fake.stats['rejected'] == 1
    assert not (fake.disparity_pub.messages or fake.depth_pub.messages or fake.cloud_pub.messages
                or fake.depth_info_pub.messages)


def test_side_fields_reads_headers_and_matrices():
    fields = side_fields(image('a', np.zeros((H, W, 3), np.uint8)), info('a', 0.0))
    assert fields['stamp'] == fields['info_stamp'] == (3, 5)
    assert (fields['width'], fields['info_height']) == (W, H)
    assert len(fields['p']) == 12


def test_runner_survives_repeated_signals_and_destroys_once(monkeypatch):
    import os
    import signal
    import rover_sim.node_runner as runner
    calls = dict(spins=0, destroyed=0, shutdown=0, init_options=None)

    class FakeNode:
        def destroy_node(self):
            calls['destroyed'] += 1
            os.kill(os.getpid(), signal.SIGINT)    # second Ctrl+C lands inside teardown
            os.kill(os.getpid(), signal.SIGTERM)

    def spin_once(node, timeout_sec):
        calls['spins'] += 1
        if calls['spins'] == 3:
            os.kill(os.getpid(), signal.SIGINT)

    def init(args=None, signal_handler_options=None):
        calls['init_options'] = signal_handler_options

    monkeypatch.setattr(runner.rclpy, 'init', init)
    monkeypatch.setattr(runner.rclpy, 'ok', lambda: calls['shutdown'] == 0)
    monkeypatch.setattr(runner.rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(runner.rclpy, 'try_shutdown', lambda: calls.__setitem__('shutdown', calls['shutdown'] + 1))
    original = {sig: signal.getsignal(sig) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        runner.run_until_signal(FakeNode)      # must return normally, no KeyboardInterrupt
    finally:
        for sig, handler in original.items():
            signal.signal(sig, handler)
    assert calls['spins'] == 3 and calls['destroyed'] == 1 and calls['shutdown'] == 1
    assert calls['init_options'] == runner.SignalHandlerOptions.NO
