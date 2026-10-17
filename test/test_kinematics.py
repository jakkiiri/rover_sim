"""Inverse/forward kinematics and odometry against reconstructed module velocities."""
from dataclasses import replace
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
from rover_sim.kinematics import forward, inverse, modules_from_urdf, wrap_angle
from rover_sim.odometry import WheelOdometry


@pytest.fixture(scope='module')
def description():
    return xacro.process_file(str(Path(__file__).resolve().parents[1] / 'urdf/mars_rover.urdf.xacro'),
                              mappings={'enable_lidar': 'false'}).toxml()


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
    assert [(m.x, m.y) for m in modules] == [(0.4, 0.5), (0.4, -0.5), (-0.4, 0.5), (-0.4, -0.5)]
    assert all(m.radius == 0.2 and m.drive_sign == -1 for m in modules)
    assert all(m.wheel_limit == 10.0 and m.steering_rate == 3.0 for m in modules)
    _, forward_rates = inverse(modules, 1, 0, 0, [0]*4)
    _, backward = inverse(modules, -1, 0, 0, [0]*4)
    assert all(v < 0 for v in forward_rates)
    assert all(v > 0 for v in backward)


def test_geometry_is_read_through_the_suspension_chain(description):
    root = ET.fromstring(description)
    root.find("joint[@name='front_left_suspension']/origin").set('xyz', '0.52 0.63 -0.11')
    root.find("link[@name='front_left_wheel']/collision/geometry/sphere").set('radius', '0.24')
    m = modules_from_urdf(ET.tostring(root))[0]
    assert (m.x, m.y, m.radius) == (0.52, 0.63, 0.24)


@pytest.mark.parametrize('mutation', ['rotated_suspension', 'tilted_axis', 'offset_wheel', 'missing_drive'])
def test_unsupported_layouts_are_rejected(description, mutation):
    root = ET.fromstring(description)
    if mutation == 'rotated_suspension':
        root.find("joint[@name='front_left_suspension']/origin").set('rpy', '0 0.1 0')
    if mutation == 'tilted_axis':
        root.find("joint[@name='front_left_suspension']/axis").set('xyz', '0 1 0')
    if mutation == 'offset_wheel':
        root.find("joint[@name='front_left_wheel_joint']/origin").set('xyz', '0.02 0 -0.06')
    if mutation == 'missing_drive':
        root.remove(root.find("joint[@name='front_left_wheel_joint']"))
    with pytest.raises(ValueError):
        modules_from_urdf(ET.tostring(root))


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


def test_flip_optimisation_minimises_steering_travel(modules):
    # Requesting a module velocity 170 deg away picks theta+pi with reversed speed.
    old = [0.0]*4
    angles, speeds = inverse(modules, -math.cos(math.radians(10)), math.sin(math.radians(10)), 0, old)
    assert all(abs(a - math.radians(-10)) < 1e-9 for a in angles)
    assert all(s > 0 for s in speeds)       # reversed wheel instead of a 170 deg swing


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


# --- forward kinematics and odometry ---------------------------------------

@pytest.mark.parametrize('twist', [(0.5, 0, 0), (0, 0.3, 0), (0, 0, 0.8), (0.4, -0.2, 0.6), (-0.3, 0.1, -0.5)])
def test_forward_inverts_inverse(modules, twist):
    angles, rates = inverse(modules, *twist, [0.1, -0.2, 0.3, -0.4])
    estimate, residuals = forward(modules, angles, rates)
    assert estimate == pytest.approx(twist, abs=1e-9)
    assert max(residuals) < 1e-9


def test_forward_least_squares_isolates_a_slipping_wheel(modules):
    angles, rates = inverse(modules, 0.5, 0, 0, [0]*4)
    rates[0] *= 1.6                           # front-left spins 60 % faster than the body moves
    estimate, residuals = forward(modules, angles, rates)
    assert 0.5 < estimate[0] < 0.6            # the outlier pulls the fit only a little
    assert residuals[0] == max(residuals) and residuals[0] > 1.4 * max(residuals[1:])


def test_forward_rejects_bad_input(modules):
    with pytest.raises(ValueError):
        forward(modules, [0]*3, [0]*4)
    with pytest.raises(ValueError):
        forward(modules, [math.nan]*4, [0]*4)


def test_odometry_follows_a_circle_and_uses_the_gyro(modules):
    odometry = WheelOdometry(modules)
    vx, wz = 0.5, 0.5
    angles, rates = inverse(modules, vx, 0, wz, [0]*4)
    dt = 0.02
    for _ in range(int(2*math.pi/wz/dt)):
        odometry.update(angles, rates, dt)
    x, y, yaw = odometry.pose()
    assert math.hypot(x, y) < 0.01 and abs(wrap_angle(yaw)) < 0.01   # closed circle
    assert odometry.distance == pytest.approx(2*math.pi*vx/wz, rel=1e-3)
    odometry.reset()
    for _ in range(50):
        odometry.update(angles, rates, dt, imu_yaw_rate=0.0)
    assert odometry.pose()[2] == pytest.approx(0.0)                  # gyro overrides the wheel yaw
    assert odometry.wheel_twist[2] == pytest.approx(wz)
    assert odometry.predicted_module_speeds() == pytest.approx([abs(r)*0.2 for r in rates])
    assert WheelOdometry(modules).predicted_module_speeds() == [0.0]*4


def test_odometry_rejects_negative_dt(modules):
    with pytest.raises(ValueError):
        WheelOdometry(modules).update([0]*4, [0]*4, -1.0)
