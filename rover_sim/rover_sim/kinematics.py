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


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _apply(m, v):
    return [sum(m[i][k]*v[k] for k in range(3)) for i in range(3)]


def _pose_in_base(joints_by_child, joint):
    """Pose (rotation, translation) of a joint's frame in base_link at zero joint angles.

    The chain from base_link may pass through fixed joints and passive
    revolute/continuous joints (rocker suspension); prismatic joints are rejected.
    """
    chain = [joint]
    while chain[-1].find('parent').get('link') != 'base_link':
        parent = joints_by_child.get(chain[-1].find('parent').get('link'))
        if parent is None or parent.get('type') not in ('fixed', 'revolute', 'continuous') or len(chain) > 8:
            raise ValueError('Expected steering joints on base_link or on links hinged to it')
        chain.append(parent)
    rot, pos = ((1, 0, 0), (0, 1, 0), (0, 0, 1)), [0.0, 0.0, 0.0]
    for j in reversed(chain):
        origin = j.find('origin')
        xyz = list(map(float, origin.get('xyz', '0 0 0').split())) if origin is not None else [0.0]*3
        rpy = list(map(float, origin.get('rpy', '0 0 0').split())) if origin is not None else [0.0]*3
        pos = [p + q for p, q in zip(pos, _apply(rot, xyz))]
        rot = _matmul(rot, rotation(rpy))
    return rot, pos


def modules_from_urdf(xml):
    """Read wheel-centred modules with vertical steering joints; reject other layouts.

    Steering joints may hang directly on base_link (proxy rover) or on passive
    rockers (real rover); module positions are taken at zero rocker angles.
    """
    root = ET.fromstring(xml)
    joints = {j.attrib['name']: j for j in root.findall('joint')}
    joints_by_child = {j.find('child').get('link'): j for j in joints.values()}
    links = {j.attrib['name']: j for j in root.findall('link')}
    result = []
    for name in sorted(n for n in joints if n.endswith('_kingpin')):
        j = joints[name]
        drive_name = name[:-len('_kingpin')] + '_wheel_joint'
        d = joints[drive_name]
        base_rot, xyz = _pose_in_base(joints_by_child, j)
        steer_axis = _apply(base_rot, list(map(float, j.find('axis').get('xyz').split())))
        wheel_origin = d.find('origin')
        offset = list(map(float, wheel_origin.get('xyz', '0 0 0').split()))
        if (any(abs(a - b) > 1e-6 for a, b in zip(steer_axis, (0, 0, 1)))
                or any(abs(v) > 1e-9 for v in offset)
                or d.find('parent').get('link') != j.find('child').get('link')):
            raise ValueError('Expected wheel-centered, vertical steering joints')
        spin = list(map(float, d.find('axis').get('xyz').split()))
        mat = _matmul(base_rot, rotation(list(map(float, wheel_origin.get('rpy', '0 0 0').split()))))
        axis = _apply(mat, spin)
        # Rolling direction = spin_axis cross +Z. At steering=0 it must be +/-X.
        if abs(axis[0]) > 1e-6 or abs(axis[2]) > 1e-6 or abs(abs(axis[1])-1) > 1e-6:
            raise ValueError('Wheel axis must be transverse at zero steering')
        wheel = links[d.find('child').get('link')]
        collision = wheel.find('collision')
        geometry = collision.find('geometry/cylinder')
        if geometry is None:
            raise ValueError('Expected a cylindrical wheel collision')
        co = collision.find('origin')
        if co is not None and any(abs(float(v)) > 1e-9 for key in ('xyz', 'rpy') for v in co.get(key, '0 0 0').split()):
            raise ValueError('Wheel collision must be centered and aligned with wheel link')
        limit, dl = j.find('limit'), d.find('limit')
        module = Module(name, drive_name, xyz[0], xyz[1], float(geometry.get('radius')),
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
