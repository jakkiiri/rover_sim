"""Stereo camera geometry, camera_info math and LiDAR keep-out checks (no ROS).

Everything here takes explicit inputs and raises ValueError on anything it
cannot verify, so callers can fail closed.
"""
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

# ROS optical frame (x right, y down, z forward) relative to a body frame
# (x forward, y left, z up).
OPTICAL_RPY = (-math.pi / 2, 0.0, -math.pi / 2)
LEFT_FRAME = 'stereo_left_optical_frame'
RIGHT_FRAME = 'stereo_right_optical_frame'
BODY_FRAME = 'stereo_link'
STEREO_DEFAULTS = dict(width=480, height=360, rate=15.0, baseline=0.12, hfov=math.pi / 2)


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be a finite number')
    return float(value)


def validate_stereo_config(width, height, rate, baseline, hfov):
    """Reject camera settings that the sensor or the matcher cannot use."""
    for name, value in (('width', width), ('height', height)):
        if isinstance(value, bool) or not isinstance(value, int) or not 64 <= value <= 4096:
            raise ValueError(f'stereo_{name} must be an integer from 64 to 4096 pixels')
    rate = _finite(rate, 'stereo_rate')
    if not 0 < rate <= 60:
        raise ValueError('stereo_rate must be in (0, 60] Hz')
    baseline = _finite(baseline, 'stereo_baseline')
    if not 0.01 <= baseline <= 1.0:
        raise ValueError('stereo_baseline must be from 0.01 to 1.0 m')
    hfov = _finite(hfov, 'stereo_hfov')
    if not 0.1 <= hfov <= 2.8:
        raise ValueError('stereo_hfov must be from 0.1 to 2.8 rad')
    return dict(width=width, height=height, rate=rate, baseline=baseline, hfov=hfov)


def intrinsics(width, height, hfov):
    """Pinhole intrinsics of a symmetric Gazebo Classic camera.

    Gazebo renders a symmetric frustum, so the optical axis passes through the
    image centre. In the ROS convention (pixel (0, 0) is the centre of the first
    pixel) that centre is ((w-1)/2, (h-1)/2). The gazebo_ros plugin's default,
    (w+1)/2, is one pixel off, so the xacro sets cx and cy explicitly.
    """
    validate_stereo_config(width, height, 1.0, 0.1, hfov)
    fx = width / (2.0 * math.tan(hfov / 2.0))
    return dict(fx=fx, fy=fx, cx=(width - 1) / 2.0, cy=(height - 1) / 2.0)


def camera_info_fields(width, height, hfov, baseline, side):
    """Expected rectified, distortion-free camera_info for 'left' or 'right'."""
    if side not in ('left', 'right'):
        raise ValueError("side must be 'left' or 'right'")
    validate_stereo_config(width, height, 1.0, baseline, hfov)
    k = intrinsics(width, height, hfov)
    tx = 0.0 if side == 'left' else -k['fx'] * baseline
    return dict(
        width=width, height=height, distortion_model='plumb_bob', d=[0.0] * 5,
        k=[k['fx'], 0.0, k['cx'], 0.0, k['fy'], k['cy'], 0.0, 0.0, 1.0],
        r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        p=[k['fx'], 0.0, k['cx'], tx, 0.0, k['fy'], k['cy'], 0.0, 0.0, 0.0, 1.0, 0.0],
        frame_id=LEFT_FRAME if side == 'left' else RIGHT_FRAME)


def right_projection(p, baseline):
    """Return a copy of a 3x4 projection with the right-camera Tx = -fx * baseline."""
    p = [float(v) for v in p]
    if len(p) != 12 or not all(math.isfinite(v) for v in p):
        raise ValueError('Projection must contain 12 finite values')
    if p[0] <= 0:
        raise ValueError('Projection fx must be positive')
    baseline = _finite(baseline, 'baseline')
    if baseline <= 0:
        raise ValueError('baseline must be positive')
    p[3] = -p[0] * baseline
    return p


def rotation(rpy):
    """Fixed-axis roll-pitch-yaw rotation matrix (child to parent), as in URDF."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
            (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr),
            (-sp, cp*sr, cp*cr))


def _matmul(a, b):
    return tuple(tuple(sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _apply(matrix, vector):
    return tuple(sum(matrix[i][k]*vector[k] for k in range(3)) for i in range(3))


def _origin(element):
    origin = element.find('origin') if element is not None else None
    if origin is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    xyz = tuple(float(v) for v in origin.get('xyz', '0 0 0').split())
    rpy = tuple(float(v) for v in origin.get('rpy', '0 0 0').split())
    if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz + rpy):
        raise ValueError('Origins must contain three finite xyz and rpy values')
    return xyz, rpy


def link_transforms(xml, root_link='base_link'):
    """Pose (translation, rotation) of every link reachable from root_link through fixed joints."""
    robot = ET.fromstring(xml) if isinstance(xml, (str, bytes)) else xml
    children = {}
    for joint in robot.findall('joint'):
        children.setdefault(joint.find('parent').get('link'), []).append(joint)
    poses = {root_link: ((0.0, 0.0, 0.0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))}
    pending = [root_link]
    while pending:
        parent = pending.pop()
        translation, matrix = poses[parent]
        for joint in children.get(parent, []):
            if joint.get('type') != 'fixed':
                continue
            xyz, rpy = _origin(joint)
            offset = _apply(matrix, xyz)
            child = joint.find('child').get('link')
            poses[child] = (tuple(translation[i]+offset[i] for i in range(3)),
                            _matmul(matrix, rotation(rpy)))
            pending.append(child)
    return poses


def baseline_from_urdf(xml):
    """Baseline from the two optical-frame joints; reject any layout that is not a rectified pair."""
    robot = ET.fromstring(xml) if isinstance(xml, (str, bytes)) else xml
    joints = {}
    for joint in robot.findall('joint'):
        child = joint.find('child').get('link')
        if child in (LEFT_FRAME, RIGHT_FRAME):
            if child in joints:
                raise ValueError(f'{child} has more than one parent joint')
            joints[child] = joint
    if set(joints) != {LEFT_FRAME, RIGHT_FRAME}:
        raise ValueError('Both stereo optical frames must exist')
    origins = {}
    for child, joint in joints.items():
        if joint.get('type') != 'fixed' or joint.find('parent').get('link') != BODY_FRAME:
            raise ValueError('Optical frames must be fixed children of stereo_link')
        xyz, rpy = _origin(joint)
        if any(abs(a - b) > 1e-6 for a, b in zip(rpy, OPTICAL_RPY)):
            raise ValueError('Optical frames must use the ROS optical rotation (-pi/2, 0, -pi/2)')
        origins[child] = xyz
    left, right = origins[LEFT_FRAME], origins[RIGHT_FRAME]
    if abs(left[0] - right[0]) > 1e-9 or abs(left[2] - right[2]) > 1e-9:
        raise ValueError('Optical centres must differ only along stereo_link Y')
    baseline = left[1] - right[1]
    if not baseline > 0:
        raise ValueError('Left optical centre must be on +Y of the right one')
    return baseline


@dataclass(frozen=True)
class Keepout:
    """LiDAR keep-out region in base_link coordinates.

    A point is inside if it is within `radius` of the LiDAR origin horizontally
    and vertically (the housing zone), or if its elevation seen from the LiDAR
    origin lies in [vfov_min - margin, vfov_max + margin] degrees (the beam band,
    all azimuths, any range).
    """
    origin: tuple = (0.0, 0.0, 0.20)
    vfov_min: float = -15.0
    vfov_max: float = 15.0
    margin: float = 2.0
    radius: float = 0.15

    def __post_init__(self):
        values = tuple(self.origin) + (self.vfov_min, self.vfov_max, self.margin, self.radius)
        if len(self.origin) != 3 or not all(math.isfinite(v) for v in values):
            raise ValueError('Keep-out values must be finite')
        if not -90 <= self.vfov_min <= self.vfov_max <= 90 or self.margin < 0 or self.radius < 0:
            raise ValueError('Keep-out needs -90 <= vfov_min <= vfov_max <= 90, margin >= 0 and radius >= 0')


def point_in_keepout(point, keepout):
    dx, dy, dz = (point[i] - keepout.origin[i] for i in range(3))
    horizontal = math.hypot(dx, dy)
    if horizontal <= keepout.radius and abs(dz) <= keepout.radius:
        return True
    elevation = math.degrees(math.atan2(dz, horizontal))
    return keepout.vfov_min - keepout.margin <= elevation <= keepout.vfov_max + keepout.margin


def _geometry_points(geometry):
    """Sample the surface of a URDF primitive in its own frame: corners/rims plus centre."""
    box, cylinder, sphere = geometry.find('box'), geometry.find('cylinder'), geometry.find('sphere')
    if box is not None:
        sx, sy, sz = (float(v) / 2 for v in box.get('size').split())
        return [(x*sx, y*sy, z*sz) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)]
    if cylinder is not None:
        r, half = float(cylinder.get('radius')), float(cylinder.get('length')) / 2
        return [(r*math.cos(a), r*math.sin(a), z) for a in [i*math.pi/8 for i in range(16)]
                for z in (-half, half)] + [(0.0, 0.0, -half), (0.0, 0.0, half)]
    if sphere is not None:
        r = float(sphere.get('radius'))
        return [(r*math.cos(a)*math.cos(e), r*math.sin(a)*math.cos(e), r*math.sin(e))
                for a in [i*math.pi/8 for i in range(16)] for e in (-math.pi/2, -math.pi/4, 0, math.pi/4, math.pi/2)]
    raise ValueError('Stereo visuals must be boxes, cylinders or spheres so clearance can be checked')


def stereo_visual_points(xml, prefix='stereo_'):
    """Base_link coordinates of sampled visual points on every link named prefix*."""
    robot = ET.fromstring(xml) if isinstance(xml, (str, bytes)) else xml
    poses = link_transforms(robot)
    points = []
    for link in robot.findall('link'):
        name = link.get('name')
        if not name.startswith(prefix):
            continue
        if name not in poses:
            raise ValueError(f'{name} is not attached to base_link by fixed joints')
        translation, matrix = poses[name]
        points.append((name, translation))  # frame origin, e.g. an optical centre
        for visual in link.findall('visual'):
            xyz, rpy = _origin(visual)
            local = rotation(rpy)
            for sample in _geometry_points(visual.find('geometry')):
                in_link = tuple(a + b for a, b in zip(_apply(local, sample), xyz))
                offset = _apply(matrix, in_link)
                points.append((name, tuple(translation[i] + offset[i] for i in range(3))))
    return points


def keepout_violations(xml, keepout, prefix='stereo_'):
    """Return (link, point) for every sampled stereo visual point inside the keep-out region."""
    return [(name, point) for name, point in stereo_visual_points(xml, prefix)
            if point_in_keepout(point, keepout)]
