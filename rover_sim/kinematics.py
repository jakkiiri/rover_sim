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


def modules_from_urdf(xml):
    """Read supported direct-to-base, wheel-centered modules; reject other layouts."""
    root = ET.fromstring(xml)
    joints = {j.attrib['name']: j for j in root.findall('joint')}
    links = {j.attrib['name']: j for j in root.findall('link')}
    result = []
    for name in sorted(n for n in joints if n.endswith('_kingpin')):
        j = joints[name]
        drive_name = name[:-len('_kingpin')] + '_wheel_joint'
        d = joints[drive_name]
        origin = j.find('origin')
        xyz = list(map(float, origin.get('xyz', '0 0 0').split()))
        rpy = list(map(float, origin.get('rpy', '0 0 0').split()))
        wheel_origin = d.find('origin')
        offset = list(map(float, wheel_origin.get('xyz', '0 0 0').split()))
        if (j.find('parent').get('link') != 'base_link' or any(abs(v) > 1e-9 for v in rpy + offset)
                or j.find('axis').get('xyz').split() != ['0', '0', '1']
                or d.find('parent').get('link') != j.find('child').get('link')):
            raise ValueError('Expected wheel-centered, vertical steering joints directly on base_link')
        spin = list(map(float, d.find('axis').get('xyz').split()))
        mat = rotation(list(map(float, wheel_origin.get('rpy', '0 0 0').split())))
        axis = [sum(mat[i][k]*spin[k] for k in range(3)) for i in range(3)]
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
