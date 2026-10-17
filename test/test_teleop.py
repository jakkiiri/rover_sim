"""Keyboard commands and stop boundaries, without a running ROS graph."""
import math
import os
import pty
import termios
from types import SimpleNamespace

import pytest

from rover_sim.teleop import KeyboardCommand, RoverTeleop, ZERO, keyboard_terminal


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
def test_removed_keys_never_start_or_continue_motion(key):
    state = KeyboardCommand()
    state.feed(key, 0.)
    assert state.command == ZERO and state.last_key == -math.inf
    state.feed(b'w', .1)
    state.feed(key, .2)
    assert state.command == ZERO and state.last_key == .1


def test_every_input_byte_keeps_lateral_command_zero():
    for value in range(256):
        state = KeyboardCommand()
        state.feed(bytes([value]), 0.)
        assert state.command[1] == 0.


def test_publication_forces_lateral_zero_even_for_an_invalid_internal_command():
    messages = []
    fake = SimpleNamespace(input=SimpleNamespace(command=(.2, .8, -.3), label='test'),
                           publisher=SimpleNamespace(publish=messages.append), displayed=None)
    RoverTeleop.publish_command(fake)
    msg = messages[-1]
    assert (msg.linear.x, msg.linear.y, msg.angular.z) == (.2, 0., -.3)
    assert (msg.linear.z, msg.angular.x, msg.angular.y) == (0., 0., 0.)


def test_lateral_speed_configuration_is_removed():
    with pytest.raises(TypeError, match='lateral_speed'):
        KeyboardCommand(lateral_speed=.35)
