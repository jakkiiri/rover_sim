"""Deterministic controller-core regressions, without ROS discovery or Gazebo."""
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import signal

import pytest
import xacro
from std_msgs.msg import String, UInt64
from geometry_msgs.msg import Twist

from rover_sim.control_state import ControlState, DEFAULTS, DRIVE_MODES, apply_mode, validate_parameters
from rover_sim.controller import SwerveController
import rover_sim.controller as controller
from rover_sim.kinematics import inverse, modules_from_urdf


@pytest.fixture(scope='module')
def modules():
    description = xacro.process_file(str(Path(__file__).resolve().parents[1] / 'urdf/mars_rover.urdf.xacro'),
                                     mappings={'enable_lidar': 'false'}).toxml()
    return modules_from_urdf(description)


@pytest.fixture
def state(modules):
    return ControlState(modules)


def feedback(state, sim=0.0, positions=None, velocities=None):
    names = [m.steering for m in state.modules] + [m.drive for m in state.modules]
    positions = ([0.0]*4 if positions is None else list(positions)) + [0.0]*4
    velocities = [0.0]*4 + ([0.0]*4 if velocities is None else list(velocities))
    return state.on_feedback(names, positions, sim, sim, velocities)


def moving(state, speed=0.4):
    feedback(state)
    state.on_command((speed, 0.0, 0.0), 0.0)
    state.step(0.0, 0.0)
    _, speeds = state.step(0.02, 0.02)
    assert all(v < 0 for v in speeds)


def drive(state, command, seconds, start=0.0, dt=0.02, feedback_positions=None, imu=None):
    """Tick the controller with fresh command/feedback for ``seconds``."""
    t = start
    result = None
    while t < start + seconds - 1e-9:
        t += dt
        if feedback_positions is not None:
            feedback(state, sim=t, positions=feedback_positions)
        else:
            feedback(state, sim=t, positions=list(state.angles))
        if imu is not None:
            state.on_imu(*imu, t)
        state.on_command(command, t)
        result = state.step(t, t)
    return result, t


# --- startup, feedback and timeouts ---------------------------------------------

def test_startup_waits_for_complete_feedback(state):
    state.on_command((.4, 0., 0.), 0.)
    assert state.step(.02, .02) is None
    assert state.speeds == [0.]*4


def test_complete_feedback_initializes_actual_multiturn_angles(state):
    positions = [8*math.pi+.2, -6*math.pi-.4, 1., -1.2]
    assert feedback(state, positions=positions)
    assert state.step(0., 0.) == (positions, [0.]*4)


@pytest.mark.parametrize('kind', ['partial', 'empty', 'duplicate', 'nan', 'infinity',
                                  'positions_length', 'velocity_length', 'effort_nan',
                                  'missing_name', 'future_stamp', 'negative_stamp', 'old_stamp'])
def test_complete_then_invalid_feedback_inhibits_drive_immediately(state, kind):
    moving(state)
    names = [m.steering for m in state.modules]
    positions, velocities, efforts, stamp = [0.]*4, [], [], .02
    if kind == 'partial': names, positions = names[:3], positions[:3]
    if kind == 'empty': names, positions = [], []
    if kind == 'duplicate': names[-1] = names[0]
    if kind == 'nan': positions[0] = math.nan
    if kind == 'infinity': positions[0] = math.inf
    if kind == 'positions_length': positions.pop()
    if kind == 'velocity_length': velocities = [0.]
    if kind == 'effort_nan': efforts = [math.nan]*4
    if kind == 'missing_name': names[-1] = ''
    if kind == 'future_stamp': stamp = 100.
    if kind == 'negative_stamp': stamp = -1.
    if kind == 'old_stamp':
        feedback(state, sim=.02)
        stamp = .01
    assert not state.on_feedback(names, positions, stamp, .02, velocities, efforts)
    assert state.step(.03, .03)[1] == [0.]*4
    assert not state.feedback_valid
    assert feedback(state, sim=.04)
    assert all(v < 0 for v in state.step(.04, .04)[1])


def test_feedback_timeout_uses_simulation_time(state):
    moving(state)
    state.on_command((.4, 0., 0.), 20.)
    assert all(v < 0 for v in state.step(.1, 20.)[1])
    state.on_command((.4, 0., 0.), 20.1)
    assert state.step(.5, 20.1)[1] == [0.]*4


def test_command_timeout_expires_during_pause(state):
    moving(state)
    assert state.step(.02, .5)[1] == [0.]*4


def test_pause_does_not_integrate_acceleration(state):
    moving(state)
    before = list(state.speeds)
    state.on_command((.4, 0., 0.), .1)
    assert state.step(.02, .1)[1] == before


def test_reset_clears_command_and_reinitializes_from_feedback(state):
    moving(state)
    state.reset()
    assert state.step(0., .04) is None
    assert feedback(state, positions=[.5]*4)
    assert state.step(.02, .05) == ([.5]*4, [0.]*4)
    assert state.command_at == -math.inf
    assert state.odometry.pose() == (0., 0., 0.)


def test_backward_clock_jump_requires_new_feedback_and_command(state):
    moving(state)
    assert state.step(0., .03) is None
    assert feedback(state)
    assert state.step(.02, .04)[1] == [0.]*4


def test_stale_command_stops_immediately_without_deceleration_ramp(state):
    (angles, speeds), t = drive(state, (.6, 0., 0.), 3.0)
    assert all(v < -1.0 for v in speeds)
    assert state.step(t + .02, t + 10.)[1] == [0.]*4        # wall clock ran away: fail closed
    assert state.twist == [0., 0., 0.]


# --- parameters ---------------------------------------------------------------

@pytest.mark.parametrize('name', [n for n in DEFAULTS if n not in ('twist_scaling', 'odometry_use_imu')])
@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf, -1.])
def test_invalid_configuration_rejected_atomically(state, name, bad):
    before = dict(state.parameters)
    with pytest.raises(ValueError, match=name):
        state.configure({name: bad})
    assert state.parameters == before


@pytest.mark.parametrize('changes', [dict(max_wheel_speed=0.), dict(max_accel=0.), dict(max_wheel_speed=21.),
                                    dict(steer_error_cutoff=2.0), dict(tilt_cutoff_deg=10.0),
                                    dict(tilt_cutoff_deg=95.0), dict(slip_min_scale=1.5), dict(twist_scaling=1)])
def test_physically_invalid_limits_rejected(state, changes):
    with pytest.raises(ValueError):
        state.configure(changes)


def test_missing_parameter_rejected(modules):
    values = dict(DEFAULTS)
    del values['max_jerk']
    with pytest.raises(ValueError, match='max_jerk'):
        validate_parameters(values, modules)


@pytest.mark.parametrize('name', ['command_timeout', 'feedback_timeout'])
def test_zero_timeout_is_valid_and_inhibits_drive(state, name):
    moving(state)
    state.configure({name: 0.})
    assert state.step(.03, .03)[1] == [0.]*4


def test_atomic_ros_parameter_callback_has_no_side_effects(state):
    fake = SimpleNamespace(state=state, get_parameter=lambda n: SimpleNamespace(value=state.parameters[n]))
    result = SwerveController.validate_update(fake, [SimpleNamespace(name='command_timeout', value=.8),
                                                   SimpleNamespace(name='max_accel', value=-1.)])
    assert not result.successful and 'max_accel' in result.reason
    assert state.parameters == DEFAULTS
    assert SwerveController.validate_update(fake, [SimpleNamespace(name='max_accel', value=0.4)]).successful
    assert not SwerveController.validate_update(fake, [SimpleNamespace(name='drive_mode', value='sideways')]).successful
    applied = []
    fake.apply_drive_mode = applied.append
    assert SwerveController.validate_update(fake, [SimpleNamespace(name='drive_mode', value='crab')]).successful
    assert applied == ['crab']


def test_extreme_finite_command_fails_closed_without_changing_kinematics(state):
    feedback(state)
    state.on_command((1.7e308, 0., 0.), 0.)
    state.step(0., 0.)
    speeds = state.step(.02, .02)[1]
    assert all(math.isfinite(v) for v in speeds)
    assert max(abs(v) for v in speeds) <= DEFAULTS['max_accel'] * .02 / .2 + 1e-9    # clamped to max_speed and ramped


@pytest.mark.parametrize('lateral', [math.nan, math.inf, -math.inf])
def test_nonfinite_input_stops_every_axis(state, lateral):
    moving(state)
    state.on_command((.4, lateral, .5), .03)
    assert state.command == (0., 0., 0.)
    assert state.step(.03, .03)[1] == [0.]*4


# --- drive modes ----------------------------------------------------------------

@pytest.mark.parametrize('mode,expected', [('ackermann', (.3, 0., .2)), ('crab', (.3, .1, 0.)),
                                           ('point_turn', (0., 0., .2)), ('auto', (.3, .1, .2))])
def test_modes_clamp_the_twist(mode, expected):
    assert apply_mode(mode, (.3, .1, .2)) == expected


def test_default_mode_is_ackermann_and_lateral_is_ignored(state):
    assert state.drive_mode == 'ackermann'
    reference = ControlState(state.modules)
    (_, speeds_a), _ = drive(state, (.4, .35, 0.), 1.0)
    (_, speeds_b), _ = drive(reference, (.4, 0., 0.), 1.0)
    assert speeds_a == pytest.approx(speeds_b)
    assert all(abs(a) < 1e-9 for a in state.angles)


def test_pure_lateral_command_in_ackermann_holds_steering_and_stops(state):
    moving(state)
    old_angles = list(state.angles)
    (_, speeds), _ = drive(state, (0., .35, 0.), 2.0, start=.02)
    assert state.angles == old_angles and speeds == [0.]*4


def test_crab_mode_strafes_with_parallel_wheels(state):
    state.set_drive_mode('crab')
    drive(state, (0., .3, 0.), 3.0)
    assert [abs(a) for a in state.angles] == pytest.approx([math.pi/2]*4)
    assert state.twist[1] == pytest.approx(.3) and state.twist[2] == 0.


def test_point_turn_mode_spins_in_place(state):
    state.set_drive_mode('point_turn')
    drive(state, (.5, .2, .6), 3.0)
    assert state.twist[0] == 0. and state.twist[1] == 0. and state.twist[2] == pytest.approx(.6)
    expected, _ = inverse(state.modules, 0., 0., .6, [0.]*4)
    assert state.angles == pytest.approx(expected, abs=1e-6)


def test_invalid_mode_rejected(state):
    with pytest.raises(ValueError):
        state.set_drive_mode('diagonal')
    with pytest.raises(ValueError):
        ControlState(state.modules, drive_mode='x')
    assert set(DRIVE_MODES) == {'ackermann', 'crab', 'point_turn', 'auto'}


# --- body-level limits and steer-error scaling ------------------------------------

def test_body_acceleration_and_jerk_limits(state):
    p = state.parameters
    feedback(state)
    state.step(0., 0.)
    previous_v, previous_a, t = 0., 0., 0.
    while t < 2.0:
        t += .02
        feedback(state, sim=t, positions=list(state.angles))
        state.on_command((1.0, 0., 0.), t)
        state.step(t, t)
        v = state.twist[0]
        a = (v - previous_v) / .02
        assert a <= p['max_accel'] + 1e-6
        assert abs(a - previous_a) <= p['max_jerk'] * .02 + 1e-6
        previous_v, previous_a = v, a
    assert state.twist[0] == pytest.approx(1.0, abs=1e-6)
    assert max(abs(s) for s in state.speeds) == pytest.approx(1.0 / .2)


def test_speed_and_yaw_limits_preserve_direction(state):
    drive(state, (3.0, 0., 5.0), 4.0)
    assert state.twist[0] == pytest.approx(state.parameters['max_speed'])
    assert state.twist[2] == pytest.approx(state.parameters['max_yaw_rate'])


def test_steer_error_scales_speed_continuously(state):
    feedback(state)
    state.on_command((.4, 0., .0), 0.)
    state.step(0., 0.)
    # Feedback says the wheels still point sideways: speed is scaled by cos(e)^p, not gated to zero.
    error = 0.6
    feedback(state, sim=.02, positions=[error]*4)
    state.on_command((.4, 0., 0.), .02)
    _, speeds = state.step(.02, .02)
    reference = ControlState(state.modules)
    feedback(reference)
    reference.on_command((.4, 0., 0.), 0.)
    reference.step(0., 0.)
    feedback(reference, sim=.02)
    reference.on_command((.4, 0., 0.), .02)
    _, aligned = reference.step(.02, .02)
    assert speeds == pytest.approx([v * math.cos(error) ** 2 for v in aligned])
    feedback(state, sim=.04, positions=[1.3]*4)          # beyond the cutoff: hard stop
    state.on_command((.4, 0., 0.), .04)
    assert state.step(.04, .04)[1] == [0.]*4


def test_steering_slews_at_the_urdf_rate(state):
    feedback(state)
    state.on_command((0., 0., 1.0), 0.)
    state.step(0., 0.)
    state.on_command((0., 0., 1.0), .02)
    state.step(.02, .02)
    assert max(abs(a) for a in state.angles) == pytest.approx(3.0 * .02)


# --- tilt limits, traction control, stall -----------------------------------------

def test_tilt_reduces_speed_and_cuts_off(state):
    drive(state, (1.0, 0., 0.), 4.0, imu=(0., 0., 0.))
    assert state.twist[0] == pytest.approx(1.0)
    drive(state, (1.0, 0., 0.), 6.0, start=4.0, imu=(math.radians(23.5), 0., 0.))
    assert state.twist[0] == pytest.approx(0.5, abs=1e-3)      # halfway between 15 and 32 deg
    assert state.diagnostics['tilt_scale'] == pytest.approx(0.5)
    drive(state, (1.0, 0., 0.), 6.0, start=10.0, imu=(0., math.radians(40.), 0.))
    assert state.twist[0] == pytest.approx(0.0, abs=1e-6)
    assert state.speeds == [0.]*4


def test_traction_control_reduces_a_slipping_wheel(state):
    feedback(state)
    state.step(0., 0.)
    t = 0.
    angles, rates = inverse(state.modules, .5, 0., 0., [0.]*4)
    while t < 3.0:                         # consistent wheels: no slip
        t += .02
        feedback(state, sim=t, positions=angles, velocities=rates)
        state.on_command((.5, 0., 0.), t)
        state.step(t, t)
    assert state.traction_scale == [1.]*4 and max(state.slip) < .05
    spinning = list(rates)
    spinning[0] *= 2.0                     # front-left spins at twice the rolling speed
    while t < 5.0:
        t += .02
        feedback(state, sim=t, positions=angles, velocities=spinning)
        state.on_command((.5, 0., 0.), t)
        _, speeds = state.step(t, t)
    assert state.slip[0] > state.parameters['slip_threshold']
    assert state.traction_scale[0] == pytest.approx(state.parameters['slip_min_scale'])
    assert state.traction_scale[1:] == pytest.approx([1.]*3, abs=1e-6)
    assert abs(speeds[0]) < abs(speeds[1])
    while t < 9.0:                         # recovery once the wheel rolls again
        t += .02
        feedback(state, sim=t, positions=angles, velocities=rates)
        state.on_command((.5, 0., 0.), t)
        state.step(t, t)
    assert state.traction_scale[0] == pytest.approx(1.0)


def test_stall_detector_warns_instead_of_winding_up(state):
    feedback(state)
    state.step(0., 0.)
    t = 0.
    while t < 4.0:                         # commanded forward, wheels report zero rate
        t += .02
        feedback(state, sim=t, positions=[0.]*4, velocities=[0.]*4)
        state.on_command((.5, 0., 0.), t)
        _, speeds = state.step(t, t)
    assert state.stalled and state.diagnostics['stalled']
    assert max(abs(s) for s in speeds) == pytest.approx(.5 / .2 * state.parameters['stall_scale'])
    angles, rates = inverse(state.modules, .5, 0., 0., [0.]*4)
    while t < 6.0:
        t += .02
        feedback(state, sim=t, positions=angles, velocities=rates)
        state.on_command((.5, 0., 0.), t)
        state.step(t, t)
    assert not state.stalled


def test_odometry_integrates_feedback(state):
    feedback(state)
    state.step(0., 0.)
    angles, rates = inverse(state.modules, .5, 0., 0., [0.]*4)
    t = 0.
    while t < 2.0:
        t += .02
        feedback(state, sim=t, positions=angles, velocities=rates)
        state.on_imu(0., 0., 0., t)
        state.on_command((.5, 0., 0.), t)
        state.step(t, t)
    x, y, yaw = state.odometry.pose()
    assert x == pytest.approx(1.0, abs=1e-6) and abs(y) < 1e-9 and yaw == 0.
    assert state.diagnostics['measured_twist'] == pytest.approx((.5, 0., 0.))


# --- ROS wrapper behaviour (no graph) ----------------------------------------------

def test_reset_notification_clears_state_and_acknowledges(state):
    moving(state)
    acknowledgements = []
    fake = SimpleNamespace(state=state, epoch=1,
                           renew_command_subscription=Mock(),
                           reset_ack=SimpleNamespace(publish=acknowledgements.append),
                           get_logger=lambda: SimpleNamespace(info=lambda text: None))
    SwerveController.on_reset(fake, UInt64(data=2))
    assert not state.initialized and state.command == (0., 0., 0.)
    assert acknowledgements[-1].data == 2
    assert feedback(state)
    state.on_command((.4, 0., 0.), .1)
    SwerveController.on_reset(fake, UInt64(data=2))
    assert state.initialized and state.command == (.4, 0., 0.)
    assert len(acknowledgements) == 2
    fake.renew_command_subscription.assert_called_once()


def test_ros_twist_is_stored_verbatim_and_mode_applies_at_step(state):
    feedback(state)
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = .4, .35, .1
    SwerveController.on_command(SimpleNamespace(state=state), msg)
    assert state.command == (.4, .35, .1)
    state.step(0., state.command_at)
    assert state.diagnostics['twist'][1] == 0.                 # ackermann clamps vy


def test_drive_mode_topic_switches_and_rejects_unknown(state):
    published, logs = [], []
    fake = SimpleNamespace(_closing=False, state=state, mode_pub=SimpleNamespace(publish=published.append),
                           get_logger=lambda: SimpleNamespace(info=logs.append, warning=logs.append))
    fake.apply_drive_mode = lambda mode: SwerveController.apply_drive_mode(fake, mode)
    SwerveController.on_drive_mode(fake, String(data='Crab '))
    assert state.drive_mode == 'crab' and published[-1].data == 'crab'
    SwerveController.on_drive_mode(fake, String(data='sideways'))
    assert state.drive_mode == 'crab' and len(published) == 1 and 'Ignoring' in logs[-1]


def test_new_epoch_discards_reader_before_acknowledgement(state):
    moving(state)
    events = []
    old_reader = object()
    readers = []
    def create_subscription(kind, topic, callback, qos):
        assert topic == '/cmd_vel' and qos == 1
        events.append('new_reader')
        readers.append(callback)
        return callback
    fake = SimpleNamespace(state=state, epoch=1, command_sub=old_reader,
                           destroy_subscription=lambda reader: events.append(('destroy', reader)),
                           create_subscription=create_subscription,
                           reset_ack=SimpleNamespace(publish=lambda msg: events.append(('ack', msg.data))),
                           get_logger=lambda: SimpleNamespace(info=lambda text: None))
    fake.on_command = lambda msg, generation: SwerveController.on_command(fake, msg, generation)
    fake.renew_command_subscription = lambda: SwerveController.renew_command_subscription(fake)
    old_callback = lambda msg: SwerveController.on_command(fake, msg, 1)
    SwerveController.on_reset(fake, UInt64(data=2))
    assert events == [('destroy', old_reader), 'new_reader', ('ack', 2)]
    msg = Twist()
    msg.linear.x, msg.linear.y = .4, .3
    old_callback(msg)
    assert state.command == (0., 0., 0.) and state.command_at == -math.inf
    assert state.step(.1, .1) is None
    assert feedback(state, sim=.1)
    assert state.step(.1, .1)[1] == [0.]*4
    readers[-1](msg)
    assert state.command == (.4, .3, 0.)
    assert all(v < 0 for v in state.step(.12, state.command_at)[1])


def test_first_epoch_creates_reader_without_destroying_one(state):
    fake = SimpleNamespace(command_sub=None, epoch=10, create_subscription=Mock(return_value='new'),
                           destroy_subscription=Mock(), on_command=Mock())
    SwerveController.renew_command_subscription(fake)
    assert fake.command_sub == 'new'
    fake.destroy_subscription.assert_not_called()


@pytest.fixture
def closing_controller(state):
    feedback(state, positions=[.1, .2, .3, .4])
    return SimpleNamespace(_closed=False, _closing=False, state=state,
                           timer=SimpleNamespace(cancel=Mock()),
                           context=SimpleNamespace(ok=lambda: True),
                           publisher=SimpleNamespace(wait_for_all_acked=Mock(return_value=True)),
                           publish_targets=Mock(), get_logger=lambda: SimpleNamespace(warning=Mock()))


def test_close_stops_and_destroys_at_most_once(closing_controller, monkeypatch):
    fake = closing_controller
    fake.state.on_command((.4, 0., 0.), 0.)
    destroy = Mock(side_effect=lambda node: SwerveController.close(node))
    monkeypatch.setattr(controller.Node, 'destroy_node', destroy)
    SwerveController.close(fake)
    SwerveController.close(fake)
    fake.timer.cancel.assert_called_once()
    fake.publish_targets.assert_called_once_with([.1, .2, .3, .4], [0.]*4)
    fake.publisher.wait_for_all_acked.assert_called_once()
    assert fake.state.command == (0., 0., 0.) and fake.state.speeds == [0.]*4
    destroy.assert_called_once_with(fake)


@pytest.mark.parametrize('unavailable', ['feedback', 'context'])
def test_close_without_feedback_or_context_does_not_fabricate_targets(closing_controller, monkeypatch,
                                                                    unavailable):
    fake = closing_controller
    if unavailable == 'feedback': fake.state.reset()
    else: fake.context.ok = lambda: False
    destroy = Mock()
    monkeypatch.setattr(controller.Node, 'destroy_node', destroy)
    SwerveController.close(fake)
    fake.publish_targets.assert_not_called()
    fake.publisher.wait_for_all_acked.assert_not_called()
    destroy.assert_called_once_with(fake)


def test_close_still_destroys_if_stop_publish_fails(closing_controller, monkeypatch):
    fake = closing_controller
    fake.publish_targets.side_effect = RuntimeError('transport failed')
    destroy = Mock()
    monkeypatch.setattr(controller.Node, 'destroy_node', destroy)
    with pytest.raises(RuntimeError, match='transport failed'): SwerveController.close(fake)
    SwerveController.close(fake)
    destroy.assert_called_once_with(fake)


def test_close_ack_timeout_preserves_cleanup(closing_controller, monkeypatch):
    fake = closing_controller
    fake.publisher.wait_for_all_acked.return_value = False
    logger = SimpleNamespace(warning=Mock())
    fake.get_logger = lambda: logger
    destroy = Mock()
    monkeypatch.setattr(controller.Node, 'destroy_node', destroy)
    SwerveController.close(fake)
    logger.warning.assert_called_once()
    destroy.assert_called_once_with(fake)


@pytest.mark.parametrize('callback', ['command', 'feedback', 'imu', 'reset', 'update', 'mode', 'diagnostics'])
def test_callbacks_cannot_restart_motion_during_close(state, callback):
    moving(state)
    state.on_command((0., 0., 0.), .03)
    before = dict(state.__dict__)
    fake = SimpleNamespace(_closing=True, state=state, epoch=1)
    if callback == 'command': SwerveController.on_command(fake, Twist())
    if callback == 'feedback': SwerveController.on_state(fake, None)
    if callback == 'imu': SwerveController.on_imu(fake, None)
    if callback == 'reset': SwerveController.on_reset(fake, UInt64(data=2))
    if callback == 'update': SwerveController.update(fake)
    if callback == 'mode': SwerveController.on_drive_mode(fake, String(data='crab'))
    if callback == 'diagnostics': SwerveController.publish_diagnostics(fake)
    assert state.__dict__ == before and fake.epoch == 1


def test_publish_during_close_forces_zero_wheel_drive(state):
    published = []
    fake = SimpleNamespace(_closing=True, state=state,
                           get_clock=lambda: SimpleNamespace(now=lambda: controller.rclpy.time.Time()),
                           publisher=SimpleNamespace(publish=published.append))
    SwerveController.publish_targets(fake, [.1]*4, [5.]*4)
    assert list(published[-1].velocity) == [0.]*8
    assert list(published[-1].position[:4]) == [.1]*4


def test_odometry_message_and_tf_are_published(state):
    feedback(state)
    state.odometry.x, state.odometry.y, state.odometry.yaw = 1.0, 2.0, math.pi/2
    odoms, tfs = [], []
    fake = SimpleNamespace(state=state, odom_frame='odom', base_frame='base_link', publish_tf=True,
                           odom_pub=SimpleNamespace(publish=odoms.append), tf_pub=SimpleNamespace(publish=tfs.append))
    SwerveController.publish_odometry(fake, controller.rclpy.time.Time())
    assert odoms[-1].header.frame_id == 'odom' and odoms[-1].child_frame_id == 'base_link'
    assert odoms[-1].pose.pose.position.x == 1.0 and odoms[-1].pose.pose.orientation.w == pytest.approx(math.cos(math.pi/4))
    assert tfs[-1].transforms[0].child_frame_id == 'base_link' and tfs[-1].transforms[0].transform.translation.y == 2.0
    fake.publish_tf = False
    SwerveController.publish_odometry(fake, controller.rclpy.time.Time())
    assert len(tfs) == 1


@pytest.mark.parametrize('failure', [False, True])
def test_repeated_signals_during_spin_and_cleanup_are_idempotent(monkeypatch, failure):
    handlers = {}
    monkeypatch.setattr(controller.signal, 'signal', lambda sig, handler: handlers.update({sig: handler}))
    init = Mock()
    shutdown = Mock()
    node = SimpleNamespace(_closing=False)
    def close():
        for sig in (signal.SIGINT, signal.SIGINT, signal.SIGTERM): handlers[sig](sig, None)
        if failure: raise RuntimeError('cleanup failed')
    node.close = Mock(side_effect=close)
    def spin_once(actual, timeout_sec):
        assert actual is node and timeout_sec == .02
        for sig in (signal.SIGINT, signal.SIGINT, signal.SIGTERM): handlers[sig](sig, None)
    monkeypatch.setattr(controller.rclpy, 'init', init)
    monkeypatch.setattr(controller.rclpy, 'ok', lambda: True)
    monkeypatch.setattr(controller.rclpy, 'try_shutdown', shutdown)
    monkeypatch.setattr(controller.rclpy, 'spin_once', spin_once)
    monkeypatch.setattr(controller, 'SwerveController', lambda: node)
    if failure:
        with pytest.raises(RuntimeError, match='cleanup failed'): controller.main(args=[])
    else: controller.main(args=[])
    init.assert_called_once_with(args=[], signal_handler_options=controller.SignalHandlerOptions.NO)
    node.close.assert_called_once()
    shutdown.assert_called_once()
    assert node._closing


def test_imu_fusion_detects_a_wall_stall_with_spinning_wheels(state):
    """All four wheels spin at the rolling speed but the IMU saw the rover stop."""
    feedback(state)
    state.step(0., 0.)
    angles, rates = inverse(state.modules, .5, 0., 0., [0.]*4)
    t = 0.
    g = 3.71
    # 3 s of consistent motion: IMU reads only gravity (rest frame of a constant velocity).
    while t < 3.0:
        t += .02
        feedback(state, sim=t, positions=angles, velocities=rates)
        state.on_imu(0., 0., 0., t, (0., 0., g))
        state.on_command((.5, 0., 0.), t)
        state.step(t, t)
    assert not state.stalled and abs(state.velocity_correction[0]) < 0.02
    # Wall impact: a 0.25 s deceleration of 2 m/s^2 (0.5 m/s lost) while wheels keep spinning.
    for _ in range(13):
        t += .02
        feedback(state, sim=t, positions=angles, velocities=rates)
        state.on_imu(0., 0., 0., t, (-2.0, 0., g))
        state.on_command((.5, 0., 0.), t)
        state.step(t, t)
    assert state.velocity_correction[0] < -0.4
    slipped = False
    while t < 7.0:
        t += .02
        feedback(state, sim=t, positions=angles, velocities=rates)
        state.on_imu(0., 0., 0., t, (0., 0., g))
        state.on_command((.5, 0., 0.), t)
        state.step(t, t)
        slipped = slipped or all(sl > state.parameters['slip_threshold'] for sl in state.slip)
    assert slipped and state.stalled
    assert all(scale == pytest.approx(state.parameters['slip_min_scale']) for scale in state.traction_scale)
    assert state.diagnostics['measured_twist'][0] < 0.3


def test_gravity_compensation_on_a_slope(state):
    """Resting on a 20 deg pitch, the specific force is tilted gravity: no spurious velocity."""
    feedback(state)
    state.step(0., 0.)
    pitch, g = math.radians(20.0), 3.71
    t = 0.
    while t < 4.0:
        t += .01
        state.on_imu(0., pitch, 0., t, (-g * math.sin(pitch), 0., g * math.cos(pitch)))
    assert abs(state.velocity_correction[0]) < 1e-6 and abs(state.velocity_correction[1]) < 1e-6
    assert state.gravity_estimate == pytest.approx(g)


def test_odometry_is_published_with_the_feedback_stamp(state):
    from sensor_msgs.msg import JointState
    published = []
    fake = SimpleNamespace(_closing=False, state=state, odom_frame='odom', base_frame='base_link', publish_tf=True,
                           odom_pub=SimpleNamespace(publish=published.append), tf_pub=SimpleNamespace(publish=lambda m: None),
                           get_clock=lambda: SimpleNamespace(now=lambda: controller.rclpy.time.Time(seconds=5.0)))
    fake.publish_odometry = lambda now: SwerveController.publish_odometry(fake, now)
    msg = JointState()
    msg.header.stamp.sec, msg.header.stamp.nanosec = 4, 940000000
    msg.name = [m.steering for m in state.modules] + [m.drive for m in state.modules]
    msg.position = [0.0] * 8
    msg.velocity = [0.0] * 8
    SwerveController.on_state(fake, msg)
    assert published and published[-1].header.stamp.sec == 4 and published[-1].header.stamp.nanosec == 940000000
    SwerveController.update(SimpleNamespace(_closing=False, state=state, get_parameter=lambda n: SimpleNamespace(value=state.parameters[n]),
                                            get_clock=lambda: SimpleNamespace(now=lambda: controller.rclpy.time.Time(seconds=5.0)),
                                            publish_targets=lambda a, s: None))
    assert len(published) == 1          # the wall tick no longer publishes odometry
