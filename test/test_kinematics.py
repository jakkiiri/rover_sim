"""Validate inverse kinematics against reconstructed chassis velocity vectors."""
from dataclasses import replace
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
from rover_sim.kinematics import inverse, modules_from_urdf


@pytest.fixture(scope='module')
def description():
    return xacro.process_file(str(Path(__file__).resolve().parents[1] / 'urdf/proxy_rover.urdf.xacro')).toxml()


@pytest.fixture
def modules(description):
    return modules_from_urdf(description)


@pytest.mark.parametrize('vx,vy,wz', [
    (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0),
    (0, 0, 1), (0.6, 0, 0.8), (0.5, 0.5, 0), (0, 0, 0),
], ids=['forward', 'backward', 'left', 'right', 'rotate', 'forward_yaw', 'diagonal', 'zero'])
def test_requested_vectors(modules, vx, vy, wz):
    angles, speeds = inverse(modules, vx, vy, wz, [0]*4)
    for m, angle, speed in zip(modules, angles, speeds):
        rolling = speed*m.drive_sign*m.radius
        assert rolling*math.cos(angle) == pytest.approx(vx-wz*m.y, abs=1e-9)
        assert rolling*math.sin(angle) == pytest.approx(vy+wz*m.x, abs=1e-9)
        assert m.lower <= angle <= m.upper


def test_geometry_and_axis_sign(modules):
    assert [(m.x, m.y) for m in modules] == [(0.38, 0.45), (0.38, -0.45), (-0.38, 0.45), (-0.38, -0.45)]
    assert all(m.radius == 0.18 and m.drive_sign == -1 for m in modules)
    _, forward = inverse(modules, 1, 0, 0, [0]*4)
    _, backward = inverse(modules, -1, 0, 0, [0]*4)
    assert all(v < 0 for v in forward)
    assert all(v > 0 for v in backward)


def test_geometry_is_read_from_urdf(description):
    root = ET.fromstring(description)
    root.find("joint[@name='front_left_kingpin']/origin").set('xyz', '0.52 0.63 -0.15')
    root.find("link[@name='front_left_wheel']/collision/geometry/cylinder").set('radius', '0.24')
    m = modules_from_urdf(ET.tostring(root))[0]
    assert (m.x, m.y, m.radius) == (0.52, 0.63, 0.24)


def test_common_speed_normalization(modules):
    angles, speeds = inverse(modules, 5, 2, 3, [0]*4, 2)
    assert max(abs(v) for v in speeds) == pytest.approx(2)
    ratios = []
    for m, a, s in zip(modules, angles, speeds):
        u, v = 5-3*m.y, 2+3*m.x
        rolling = s*m.drive_sign*m.radius
        ratios.append(rolling*math.cos(a)/u)
        assert rolling*math.sin(a)/v == pytest.approx(ratios[-1])
    assert max(ratios)-min(ratios) < 1e-9


def test_joint_speed_limit_also_applies(modules):
    modules[0] = replace(modules[0], wheel_limit=0.5)
    _, speeds = inverse(modules, 1, 0, 0, [0]*4, 12)
    assert all(abs(v) == pytest.approx(0.5) for v in speeds)


def test_reverse_without_steering_flip(modules):
    angles, speeds = inverse(modules, -1, 0.001, 0, [0]*4)
    assert all(abs(a) < 0.01 for a in angles)
    assert all(s > 0 for s in speeds)


def test_wrap_boundary_chooses_nearby_equivalent(modules):
    old = [math.pi-0.02]*4
    angles, _ = inverse(modules, -1, -0.01, 0, old)
    assert all(abs(a-o) < math.pi/2 for a, o in zip(angles, old))


def test_zero_holds_each_angle(modules):
    previous = [0.2, -0.4, 1.0, -1.2]
    angles, speeds = inverse(modules, 0, 0, 0, previous)
    assert angles == previous
    assert speeds == [0]*4


def test_stationary_module_holds_angle(modules):
    m = modules[0]
    a, s = inverse(modules, m.y, -m.x, 1, [0.3]*4)
    assert a[0] == 0.3 and s[0] == 0


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf])
def test_nonfinite_rejected(modules, bad):
    with pytest.raises(ValueError):
        inverse(modules, bad, 0, 0, [0]*4)


def test_mirrored_strafe_tie_reduces_reaction(modules):
    angles, _ = inverse(modules, 0, 1, 0, [0]*4)
    assert angles == pytest.approx([math.pi/2, -math.pi/2, math.pi/2, -math.pi/2])


def test_continuous_steering_tracks_multiple_turns(modules):
    old = [8*math.pi+.1]*4
    angles, _ = inverse(modules, 1, 0, 0, old)
    assert angles == pytest.approx([8*math.pi]*4)


def test_missing_modules_rejected(description):
    root = ET.fromstring(description)
    root.remove(root.find("joint[@name='front_left_kingpin']"))
    with pytest.raises(ValueError, match='four'):
        modules_from_urdf(ET.tostring(root))
