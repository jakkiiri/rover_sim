"""Deterministic state regressions, without ROS discovery or Gazebo."""
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import signal

import pytest
import xacro
from std_msgs.msg import UInt64
from geometry_msgs.msg import Twist

from rover_sim.control_state import ControlState, DEFAULTS
from rover_sim.controller import SwerveController
import rover_sim.controller as controller
from rover_sim.kinematics import modules_from_urdf


@pytest.fixture
def state():
    description = xacro.process_file(str(Path(__file__).resolve().parents[1] /
                                         'urdf/proxy_rover.urdf.xacro')).toxml()
    return ControlState(modules_from_urdf(description))


def feedback(state, sim=0.0, positions=None):
    return state.on_feedback([m.steering for m in state.modules],
                             [0.0]*4 if positions is None else positions, sim, sim)


def moving(state):
    feedback(state)
    state.on_command((0.4, 0.0, 0.0), 0.0)
    state.step(0.0, 0.0)
    _, speeds = state.step(0.02, 0.02)
    assert all(v < 0 for v in speeds)


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


def test_backward_clock_jump_requires_new_feedback_and_command(state):
    moving(state)
    assert state.step(0., .03) is None
    assert feedback(state)
    assert state.step(.02, .04)[1] == [0.]*4


@pytest.mark.parametrize('name', list(DEFAULTS))
@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf, -1.])
def test_invalid_configuration_rejected_atomically(state, name, bad):
    before = dict(state.parameters)
    with pytest.raises(ValueError, match=name):
        state.configure(dict(wheel_acceleration=4., **{name: bad}) if name != 'wheel_acceleration'
                        else {name: bad, 'command_timeout': .8})
    assert state.parameters == before


@pytest.mark.parametrize('changes', [dict(max_wheel_speed=0.), dict(wheel_acceleration=0.),
                                    dict(max_wheel_speed=21.), dict(alignment_tolerance=math.pi/2)])
def test_physically_invalid_limits_rejected(state, changes):
    with pytest.raises(ValueError): state.configure(changes)


@pytest.mark.parametrize('name', ['command_timeout', 'feedback_timeout'])
def test_zero_timeout_is_valid_and_inhibits_drive(state, name):
    moving(state)
    state.configure({name: 0.})
    assert state.step(.03, .03)[1] == [0.]*4


def test_zero_tolerance_accepts_exact_alignment(state):
    state.configure({'alignment_tolerance': 0.})
    moving(state)


def test_atomic_ros_parameter_callback_has_no_side_effects(state):
    fake = SimpleNamespace(state=state, get_parameter=lambda n: SimpleNamespace(value=state.parameters[n]))
    result = SwerveController.validate_update(fake, [SimpleNamespace(name='command_timeout', value=.8),
                                                   SimpleNamespace(name='wheel_acceleration', value=-1.)])
    assert not result.successful and 'wheel_acceleration' in result.reason
    assert state.parameters == DEFAULTS
    assert SwerveController.validate_update(fake, [SimpleNamespace(name='wheel_acceleration', value=4.)]).successful


def test_extreme_finite_command_fails_closed_without_changing_kinematics(state):
    feedback(state)
    state.on_command((1.7e308, 0., 0.), 0.)
    state.step(0., 0.)
    assert state.step(.02, .02)[1] == [0.]*4


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


@pytest.mark.parametrize('command', [(0., .35, 0.), (0., -.35, 0.), (.4, .35, 0.),
                                   (-.4, -.35, 0.), (.3, -.35, .3), (0., 1.7e308, .5)])
def test_ros_twist_lateral_input_has_same_output_as_zero_lateral_input(state, command):
    reference = ControlState(state.modules)
    assert feedback(state) and feedback(reference)
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = command
    SwerveController.on_command(SimpleNamespace(state=state), msg)
    reference.on_command((command[0], 0., command[2]), state.command_at)
    assert state.command == reference.command
    for sim in (0., .02, .04):
        assert state.step(sim, state.command_at+sim) == reference.step(sim, state.command_at+sim)


def test_pure_lateral_command_stops_existing_drive_and_holds_steering(state):
    moving(state)
    old_angles = list(state.angles)
    state.on_command((0., .35, 0.), .03)
    assert state.step(.03, .03) == (old_angles, [0.]*4)


@pytest.mark.parametrize('lateral', [math.nan, math.inf, -math.inf])
def test_nonfinite_lateral_input_still_stops_every_axis(state, lateral):
    moving(state)
    state.on_command((.4, lateral, .5), .03)
    assert state.command == (0., 0., 0.)
    assert state.step(.03, .03)[1] == [0.]*4


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
    # Keep an already-taken callback from the old reader, as an executor could.
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
    assert state.command == (.4, 0., 0.)  # new commands retain the no-strafe constraint
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


@pytest.mark.parametrize('callback', ['command', 'feedback', 'reset', 'update'])
def test_callbacks_cannot_restart_motion_during_close(state, callback):
    moving(state)
    state.on_command((0., 0., 0.), .03)
    before = dict(state.__dict__)
    fake = SimpleNamespace(_closing=True, state=state, epoch=1)
    if callback == 'command': SwerveController.on_command(fake, Twist())
    if callback == 'feedback': SwerveController.on_state(fake, None)
    if callback == 'reset': SwerveController.on_reset(fake, UInt64(data=2))
    if callback == 'update': SwerveController.update(fake)
    assert state.__dict__ == before and fake.epoch == 1


def test_publish_during_close_forces_zero_wheel_drive(state):
    published = []
    fake = SimpleNamespace(_closing=True, state=state,
                           get_clock=lambda: SimpleNamespace(now=lambda: controller.rclpy.time.Time()),
                           publisher=SimpleNamespace(publish=published.append))
    SwerveController.publish_targets(fake, [.1]*4, [5.]*4)
    assert list(published[-1].velocity) == [0.]*8
    assert list(published[-1].position[:4]) == [.1]*4


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
