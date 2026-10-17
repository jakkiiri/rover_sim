"""URDF-derived planar swerve inverse kinematics (no ROS dependencies)."""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET


@dataclass(frozen=True)
class Module:
    steering: str
    drive: str
    x: float
    y: float
    radius: float
    drive_sign: float
    lower: float
    upper: float
    steering_rate: float
    wheel_limit: float


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def _xyz(element, key='xyz'):
    return [float(v) for v in (element.get(key, '0 0 0') if element is not None else '0 0 0').split()]


def modules_from_urdf(xml):
    """Read swerve modules whose kingpin hangs below base_link through fixed or
    prismatic (suspension) joints with zero rotation; reject other layouts.

    The module position is the kingpin axis location in base_link at the
    nominal (zero) suspension position; wheels must be centred on that axis.
    """
    root = ET.fromstring(xml)
    joints = {j.attrib['name']: j for j in root.findall('joint')}
    links = {j.attrib['name']: j for j in root.findall('link')}
    by_child = {j.find('child').get('link'): j for j in joints.values()}
    result = []
    for name in sorted(n for n in joints if n.endswith('_kingpin')):
        j = joints[name]
        drive_name = name[:-len('_kingpin')] + '_wheel_joint'
        if drive_name not in joints:
            raise ValueError(f'Missing drive joint {drive_name}')
        d = joints[drive_name]
        # Walk from the kingpin up to base_link, composing translations only.
        xyz = _xyz(j.find('origin'))
        if any(abs(v) > 1e-9 for v in _xyz(j.find('origin'), 'rpy')):
            raise ValueError('Kingpin origin must not be rotated')
        parent = j.find('parent').get('link')
        depth = 0
        while parent != 'base_link':
            if parent not in by_child or depth > 4:
                raise ValueError('Kingpin must hang below base_link through fixed/prismatic joints')
            up = by_child[parent]
            if up.get('type') not in ('fixed', 'prismatic') or any(abs(v) > 1e-9 for v in _xyz(up.find('origin'), 'rpy')):
                raise ValueError('Only unrotated fixed or prismatic (suspension) joints may carry a module')
            if up.get('type') == 'prismatic' and up.find('axis').get('xyz').split() != ['0', '0', '1']:
                raise ValueError('Suspension joints must travel vertically')
            xyz = [a + b for a, b in zip(xyz, _xyz(up.find('origin')))]
            parent = up.find('parent').get('link')
            depth += 1
        offset = _xyz(d.find('origin'))
        if (any(abs(v) > 1e-9 for v in offset[:2]) or j.find('axis').get('xyz').split() != ['0', '0', '1']
                or d.find('parent').get('link') != j.find('child').get('link')):
            raise ValueError('Expected wheel-centred, vertical steering joints')
        spin = list(map(float, d.find('axis').get('xyz').split()))
        mat = rotation(_xyz(d.find('origin'), 'rpy'))
        axis = [sum(mat[i][k]*spin[k] for k in range(3)) for i in range(3)]
        # Rolling direction = spin_axis cross +Z. At steering=0 it must be +/-X.
        if abs(axis[0]) > 1e-6 or abs(axis[2]) > 1e-6 or abs(abs(axis[1])-1) > 1e-6:
            raise ValueError('Wheel axis must be transverse at zero steering')
        wheel = links[d.find('child').get('link')]
        radius = None
        for element in wheel.findall('collision') + wheel.findall('visual'):
            cylinder = element.find('geometry/cylinder')
            sphere = element.find('geometry/sphere')
            if cylinder is not None or sphere is not None:
                radius = float((cylinder if cylinder is not None else sphere).get('radius'))
                break
        if radius is None:
            raise ValueError('Expected a cylindrical or spherical wheel geometry')
        limit, dl = j.find('limit'), d.find('limit')
        module = Module(name, drive_name, xyz[0], xyz[1], radius,
                        axis[1],
                        -math.inf if j.get('type') == 'continuous' else float(limit.get('lower')),
                        math.inf if j.get('type') == 'continuous' else float(limit.get('upper')),
                        float(limit.get('velocity')), float(dl.get('velocity')))
        if module.radius <= 0 or module.wheel_limit <= 0 or module.steering_rate <= 0:
            raise ValueError('Non-positive wheel radius or joint velocity limit')
        result.append(module)
    if len(result) != 4:
        raise ValueError('Expected four steering/drive modules')
    return result


def inverse(modules, vx, vy, yaw_rate, previous, max_wheel_speed=None):
    """Return steering angles and signed joint rad/s, preserving motion ratios."""
    if not all(math.isfinite(v) for v in (vx, vy, yaw_rate, *previous)):
        raise ValueError('Commands and steering angles must be finite')
    if len(previous) != len(modules):
        raise ValueError('One previous steering angle required per module')
    if max_wheel_speed is not None and (not math.isfinite(max_wheel_speed) or max_wheel_speed <= 0):
        raise ValueError('Maximum wheel speed must be positive and finite')
    angles, speeds, scale = [], [], 1.0
    for m, old in zip(modules, previous):
        u, v = vx-yaw_rate*m.y, vy+yaw_rate*m.x
        magnitude = math.hypot(u, v)
        if magnitude < 1e-9:
            angles.append(old)
            speeds.append(0.0)
            continue
        theta = math.atan2(v, u)
        candidates = []
        if math.isinf(m.lower) and math.isinf(m.upper):
            nearest = round((old-theta)/math.pi)
            choices = range(nearest-1, nearest+2)
        else:
            choices = range(math.ceil((m.lower-theta)/math.pi), math.floor((m.upper-theta)/math.pi)+1)
        for k in choices:
            angle = theta+k*math.pi
            # Mirror exactly tied left/right steering choices to reduce chassis reaction.
            candidates.append((round(abs(angle-old), 12), (angle-old)*m.y < 0,
                               angle, (-1 if k % 2 else 1)))
        if not candidates:
            raise ValueError('Steering range cannot realize requested module velocity')
        _, _, angle, direction = min(candidates)
        speed = direction*m.drive_sign*magnitude/m.radius
        limit = min(m.wheel_limit, max_wheel_speed or m.wheel_limit)
        scale = min(scale, limit/abs(speed))
        angles.append(angle)
        speeds.append(speed)
    return angles, [s*scale for s in speeds]


def forward(modules, angles, wheel_rates):
    """Least-squares body twist (vx, vy, wz) from measured steering and wheel rates.

    Each module contributes two equations
    ``rho_i cos(theta_i) = vx - wz*y_i`` and ``rho_i sin(theta_i) = vy + wz*x_i``
    with ``rho_i = rate_i * drive_sign_i * R_i`` the signed rolling speed.
    Returns ``(twist, residuals)`` where residuals are per-module speed
    mismatches (m/s) between the measured rolling velocity and the fitted
    twist, used for slip detection.
    """
    if len(angles) != len(modules) or len(wheel_rates) != len(modules):
        raise ValueError('One steering angle and one wheel rate required per module')
    if not all(math.isfinite(v) for v in list(angles) + list(wheel_rates)):
        raise ValueError('Measurements must be finite')
    # Normal equations A^T A x = A^T b for the 2N x 3 system.
    ata = [[0.0] * 3 for _ in range(3)]
    atb = [0.0] * 3
    rows = []
    for m, theta, rate in zip(modules, angles, wheel_rates):
        rho = rate * m.drive_sign * m.radius
        for coefficients, value in (((1.0, 0.0, -m.y), rho * math.cos(theta)),
                                    ((0.0, 1.0, m.x), rho * math.sin(theta))):
            rows.append((coefficients, value))
            for i in range(3):
                atb[i] += coefficients[i] * value
                for j in range(3):
                    ata[i][j] += coefficients[i] * coefficients[j]
    twist = _solve3(ata, atb)
    residuals = []
    for m, theta, rate in zip(modules, angles, wheel_rates):
        rho = rate * m.drive_sign * m.radius
        px = twist[0] - twist[2] * m.y
        py = twist[1] + twist[2] * m.x
        residuals.append(math.hypot(rho * math.cos(theta) - px, rho * math.sin(theta) - py))
    return twist, residuals


def _solve3(a, b):
    """Cramer's rule for a symmetric positive 3x3 system; zero on singularity."""
    det = (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
           - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
           + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    if abs(det) < 1e-12:
        return (0.0, 0.0, 0.0)
    result = []
    for column in range(3):
        m = [list(row) for row in a]
        for row in range(3):
            m[row][column] = b[row]
        d = (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
             - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
             + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
        result.append(d / det)
    return tuple(result)


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))
