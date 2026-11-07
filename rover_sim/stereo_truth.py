"""Analytic ground truth for stereo validation (no ROS).

Reads the primitive collision shapes of worlds/desert.world and casts camera
rays against them. Unsupported shapes, rotations or offsets raise ValueError
instead of being approximated.
"""
import math
import xml.etree.ElementTree as ET

FEATURE_PREFIX = 'stereo_feature_'


def _pose(element):
    values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Poses must have six finite values')
    return values


def world_primitives(world_path):
    """Return [(model, kind, centre, dims)] for every static collision in the world.

    kind is 'box' (dims = size xyz), 'sphere' (dims = (radius,)) or 'cylinder'
    (dims = (radius, length), vertical axis). Model poses may translate only;
    link and collision poses must be identity.
    """
    primitives = []
    for model in ET.parse(world_path).getroot().findall('world/model'):
        name = model.attrib['name']
        if model.findtext('static', 'false').strip().lower() not in ('true', '1'):
            raise ValueError(f'{name}: only static models have analytic ground truth')
        pose = _pose(model)
        if any(abs(v) > 1e-12 for v in pose[3:]):
            raise ValueError(f'{name}: rotated models are not supported')
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                if any(abs(v) > 1e-12 for element in (link, collision) for v in _pose(element)):
                    raise ValueError(f'{name}: link/collision offsets are not supported')
                geometry = collision.find('geometry')
                centre = tuple(pose[:3])
                if geometry.find('box') is not None:
                    size = tuple(float(v) for v in geometry.findtext('box/size').split())
                    primitives.append((name, 'box', centre, size))
                elif geometry.find('sphere') is not None:
                    primitives.append((name, 'sphere', centre, (float(geometry.findtext('sphere/radius')),)))
                elif geometry.find('cylinder') is not None:
                    primitives.append((name, 'cylinder', centre,
                                       (float(geometry.findtext('cylinder/radius')),
                                        float(geometry.findtext('cylinder/length')))))
                else:
                    raise ValueError(f'{name}: unsupported collision geometry')
    return primitives


def ray_sphere(origin, direction, centre, radius):
    """Nearest non-negative ray-sphere distance; direction must be a unit vector."""
    delta = [origin[i] - centre[i] for i in range(3)]
    b = sum(delta[i] * direction[i] for i in range(3))
    discriminant = b * b - sum(v * v for v in delta) + radius * radius
    if discriminant < 0:
        return math.inf
    root = math.sqrt(discriminant)
    return min((t for t in (-b - root, -b + root) if t >= 0), default=math.inf)


def ray_box(origin, direction, centre, size):
    """Nearest non-negative ray-distance to an axis-aligned box (slab method)."""
    lower, upper = -math.inf, math.inf
    for p, d, c, length in zip(origin, direction, centre, size):
        a, b = c - length / 2, c + length / 2
        if abs(d) < 1e-12:
            if p < a or p > b:
                return math.inf
            continue
        t0, t1 = sorted(((a - p) / d, (b - p) / d))
        lower, upper = max(lower, t0), min(upper, t1)
        if lower > upper:
            return math.inf
    if upper < 0:
        return math.inf
    return lower if lower >= 0 else upper


def ray_cylinder(origin, direction, centre, radius, length):
    """Nearest non-negative ray-distance to a capped cylinder with a vertical axis."""
    ox, oy = origin[0] - centre[0], origin[1] - centre[1]
    dx, dy = direction[0], direction[1]
    bottom, top = centre[2] - length / 2, centre[2] + length / 2
    hits = []
    a = dx * dx + dy * dy
    if a > 1e-12:
        b = ox * dx + oy * dy
        discriminant = b * b - a * (ox * ox + oy * oy - radius * radius)
        if discriminant >= 0:
            root = math.sqrt(discriminant)
            for t in ((-b - root) / a, (-b + root) / a):
                if t >= 0 and bottom <= origin[2] + t * direction[2] <= top:
                    hits.append(t)
    if abs(direction[2]) > 1e-12:
        for z in (bottom, top):
            t = (z - origin[2]) / direction[2]
            if t >= 0 and (ox + t * dx) ** 2 + (oy + t * dy) ** 2 <= radius * radius:
                hits.append(t)
    if ox * ox + oy * oy <= radius * radius and bottom <= origin[2] <= top:
        return 0.0  # origin inside
    return min(hits, default=math.inf)


def cast(origin, direction, primitives):
    """(distance, model) of the nearest primitive hit, or (inf, None)."""
    best = (math.inf, None)
    for name, kind, centre, dims in primitives:
        if kind == 'box':
            t = ray_box(origin, direction, centre, dims)
        elif kind == 'sphere':
            t = ray_sphere(origin, direction, centre, dims[0])
        else:
            t = ray_cylinder(origin, direction, centre, *dims)
        if t < best[0]:
            best = (t, name)
    return best


def optical_ray(u, v, fx, fy, cx, cy):
    """Unit ray in the optical frame (x right, y down, z forward) through pixel (u, v)."""
    x, y = (u - cx) / fx, (v - cy) / fy
    norm = math.sqrt(x * x + y * y + 1.0)
    return (x / norm, y / norm, 1.0 / norm)


def expected_depth(u, v, intrinsics, camera_origin, camera_rotation, primitives):
    """Ground-truth optical-frame depth Z for pixel (u, v) and the model it hits.

    camera_rotation maps optical-frame vectors into world coordinates.
    """
    ray = optical_ray(u, v, intrinsics['fx'], intrinsics['fy'], intrinsics['cx'], intrinsics['cy'])
    world = tuple(sum(camera_rotation[i][k] * ray[k] for k in range(3)) for i in range(3))
    distance, name = cast(camera_origin, world, primitives)
    return distance * ray[2], name


def depth_tolerance(depth, fx, baseline, disparity_error=0.5, floor=0.02):
    """Pre-registered pass threshold: error from a 0.5 px disparity error, plus 2 cm."""
    return depth * depth / (fx * baseline) * disparity_error + floor


def error_bins(samples, edges, fx, baseline):
    """Summarise (expected, measured_or_None) samples per distance bin."""
    bins = []
    for low, high in zip(edges, edges[1:]):
        inside = [(e, m) for e, m in samples if low <= e < high]
        errors = sorted(abs(m - e) for e, m in inside if m is not None and math.isfinite(m))
        centre = (low + high) / 2
        median = errors[len(errors) // 2] if errors else None
        p90 = errors[min(len(errors) - 1, int(0.9 * len(errors)))] if errors else None
        valid_fraction = len(errors) / len(inside) if inside else None
        tolerance = depth_tolerance(centre, fx, baseline)
        bins.append(dict(range_m=[low, high], samples=len(inside), valid=len(errors),
                         valid_fraction=valid_fraction, median_abs_error_m=median, p90_abs_error_m=p90,
                         tolerance_m=tolerance,
                         passed=bool(inside) and valid_fraction >= 0.5 and median <= tolerance
                         if errors else False))
    return bins
