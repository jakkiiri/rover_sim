"""Keyboard and gamepad command mapping, stop boundaries and terminal handling."""
import math
import os
import pty
import termios
from types import SimpleNamespace

import pytest

from rover_sim.teleop import KeyboardCommand, RoverTeleop, ZERO, keyboard_terminal
from rover_sim.teleop_joy import JoyCommand


@pytest.mark.parametrize('key,expected', [
    ('w', (0.4, 0.0, 0.0)), ('s', (-0.4, 0.0, 0.0)),
    ('a', (0.0, 0.0, 0.5)), ('d', (0.0, 0.0, -0.5)),
])
def test_key_mapping(key, expected):
    for variant in (key, key.upper()):
        state = KeyboardCommand()
        state.feed(variant.encode(), 1.0)
        assert state.command == expected


def test_deadman_only_motion_keys_refresh_and_boundary_stops():
    state = KeyboardCommand(key_timeout=0.5)
    state.feed(b'w', 0.0)
    state.expire(0.499)
    assert state.command != ZERO
    state.feed(b'w', 0.49)
    state.expire(0.989)
    assert state.command != ZERO
    state.expire(0.99)
    assert state.command == ZERO
    state.feed(b'a', 1.0)
    state.feed(b'?', 1.1)
    assert state.command == ZERO


def test_space_and_ctrl_c_override_buffered_motion():
    state = KeyboardCommand()
    state.feed(b'w', 0.0)
    assert not state.feed(b'w sw', 0.1)
    assert state.command == ZERO
    assert state.feed(b'w\x03w', 0.2)
    assert state.command == ZERO


def test_escape_sequences_never_turn_into_motion():
    state = KeyboardCommand()
    state.feed(b'w', 0.0)
    for chunk in (b'\x1b', b'[', b'A', b'\x1bOD', b'\x1b[1;5D'):
        state.feed(chunk, 0.1)
        assert state.command == ZERO
    state.feed(b'd', 0.2)
    assert state.command == (0.0, 0.0, -0.5)


def test_configured_speeds_and_idle():
    state = KeyboardCommand(linear_speed=0.2, angular_speed=0.3)
    state.expire(1e6)
    assert state.command == ZERO
    for key, expected in [(b's', (-0.2, 0.0, 0.0)),
                          (b'a', (0.0, 0.0, 0.3)), (b'd', (0.0, 0.0, -0.3))]:
        state.feed(key, 1.0)
        assert state.command == expected


@pytest.mark.parametrize('config', [
    dict(linear_speed=-0.1), dict(linear_speed=math.nan),
    dict(angular_speed=math.inf), dict(key_timeout=0.0), dict(publish_rate=0.0),
    dict(speed_step=1.0), dict(min_scale=3.0, max_scale=2.0), dict(drive_mode='sideways'),
])
def test_invalid_parameters(config):
    with pytest.raises(ValueError):
        KeyboardCommand(**config)


def test_terminal_restored_even_on_exception():
    master, slave = pty.openpty()
    try:
        with os.fdopen(os.dup(slave)) as stream:
            original = termios.tcgetattr(slave)
            with pytest.raises(RuntimeError, match='test failure'):
                with keyboard_terminal(stream):
                    active = termios.tcgetattr(slave)
                    assert not active[3] & termios.ICANON
                    assert not active[3] & termios.ECHO
                    assert active[3] & termios.ISIG
                    raise RuntimeError('test failure')
            assert termios.tcgetattr(slave) == original
    finally:
        os.close(master)
        os.close(slave)


def test_non_terminal_rejected():
    with open(os.devnull) as stream:
        with pytest.raises(RuntimeError, match='interactive terminal'):
            with keyboard_terminal(stream):
                pass


@pytest.mark.parametrize('key', [b'q', b'Q', b'e', b'E'])
def test_strafe_keys_are_inactive_in_ackermann_mode(key):
    state = KeyboardCommand()
    state.feed(key, 0.)
    assert state.command == ZERO and state.last_key == -math.inf
    state.feed(b'w', .1)
    state.feed(key, .2)
    assert state.command == ZERO and state.last_key == .1


def test_strafe_keys_work_in_crab_and_auto_modes():
    for mode in ('crab', 'auto'):
        state = KeyboardCommand(drive_mode=mode)
        state.feed(b'q', 0.)
        assert state.command == (0.0, 0.3, 0.0)
        state.feed(b'e', 0.1)
        assert state.command == (0.0, -0.3, 0.0)


def test_mode_key_cycles_modes_and_stops():
    state = KeyboardCommand()
    state.feed(b'w', 0.)
    state.feed(b'm', .1)
    assert state.mode == 'crab' and state.mode_changed and state.command == ZERO
    for expected in ('point_turn', 'auto', 'ackermann'):
        state.feed(b'M', .2)
        assert state.mode == expected


def test_speed_keys_scale_within_bounds():
    state = KeyboardCommand(speed_step=2.0, min_scale=0.5, max_scale=2.0)
    state.feed(b'+', 0.)
    state.feed(b'w', 0.1)
    assert state.command == (0.8, 0.0, 0.0)
    state.feed(b'+', 0.2)
    assert state.scale == 2.0
    for _ in range(5):
        state.feed(b'-', 0.3)
    assert state.scale == 0.5
    state.feed(b'w', 0.4)
    assert state.command == (0.2, 0.0, 0.0)
    assert 'speed x0.50' in state.status()


def test_estop_latches_until_toggled():
    state = KeyboardCommand()
    state.feed(b'w', 0.)
    state.feed(b'x', .1)
    assert state.estop and state.command == ZERO
    state.feed(b'w', .2)
    assert state.command == ZERO and 'E-STOP' in state.status()
    state.feed(b'x', .3)
    assert not state.estop
    state.feed(b'w', .4)
    assert state.command == (0.4, 0.0, 0.0)


def test_every_input_byte_is_safe_in_ackermann_mode():
    for value in range(256):
        state = KeyboardCommand()
        state.feed(bytes([value]), 0.)
        assert state.command[1] == 0.


def test_publication_forces_lateral_zero_outside_strafe_modes():
    messages = []
    fake = SimpleNamespace(input=SimpleNamespace(command=(.2, .8, -.3), label='test', estop=False,
                                                 mode='ackermann', mode_changed=False, status=lambda: 's'),
                           publisher=SimpleNamespace(publish=messages.append), displayed=None)
    RoverTeleop.publish_command(fake)
    msg = messages[-1]
    assert (msg.linear.x, msg.linear.y, msg.angular.z) == (.2, 0., -.3)
    fake.input.mode = 'crab'
    RoverTeleop.publish_command(fake)
    assert messages[-1].linear.y == .8
    fake.input.estop = True
    RoverTeleop.publish_command(fake)
    assert (messages[-1].linear.x, messages[-1].linear.y, messages[-1].angular.z) == (0., 0., 0.)


def test_mode_change_is_published_once():
    published = []
    fake = SimpleNamespace(input=SimpleNamespace(command=ZERO, label='l', estop=False, mode='crab',
                                                 mode_changed=True, status=lambda: 's'),
                           publisher=SimpleNamespace(publish=lambda m: None),
                           mode_publisher=SimpleNamespace(publish=published.append), displayed=None)
    RoverTeleop.publish_command(fake)
    RoverTeleop.publish_command(fake)
    assert [m.data for m in published] == ['crab']


# --- gamepad -----------------------------------------------------------------

def test_joy_requires_deadman_and_applies_deadzone():
    joy = JoyCommand(deadzone=0.2)
    assert joy.feed([0, 1.0, 0, 0.5], [0, 0, 0, 0, 0], 0.0) == ZERO
    vx, vy, wz = joy.feed([0.5, 1.0, 0, -1.0], [0, 0, 0, 0, 1], 0.1)
    assert vx == pytest.approx(0.6) and vy == 0.0 and wz == pytest.approx(-0.8)
    assert joy.feed([0.1, 0.1, 0, 0.1], [0, 0, 0, 0, 1], 0.2) == ZERO


def test_joy_strafe_mode_estop_and_timeout():
    joy = JoyCommand(drive_mode='crab', joy_timeout=0.5)
    vx, vy, wz = joy.feed([1.0, 0.0, 0, 0], [0, 0, 0, 0, 1], 0.0)
    assert vy == pytest.approx(0.4)
    joy.feed([1.0, 0, 0, 0], [1, 0, 0, 0, 1], 0.1)      # mode button edge -> point_turn
    assert joy.mode == 'point_turn' and joy.mode_changed
    joy.feed([1.0, 0, 0, 0], [1, 0, 0, 0, 1], 0.2)      # held: no second change
    assert joy.mode == 'point_turn'
    joy.feed([0, 0, 0, 0], [0, 1, 0, 0, 1], 0.3)
    assert joy.estop and joy.command == ZERO
    joy.feed([0, 0, 0, 0], [0, 0, 0, 0, 1], 0.4)
    joy.feed([0, 0, 0, 0], [0, 1, 0, 0, 1], 0.5)
    assert not joy.estop
    joy.feed([0, 1.0, 0, 0], [0, 0, 0, 0, 1], 0.6)
    assert joy.expire(1.0) != ZERO
    assert joy.expire(1.2) == ZERO


@pytest.mark.parametrize('config', [dict(linear_speed=-1), dict(deadzone=1.0), dict(joy_timeout=0), dict(drive_mode='x')])
def test_joy_invalid_parameters(config):
    with pytest.raises(ValueError):
        JoyCommand(**config)
