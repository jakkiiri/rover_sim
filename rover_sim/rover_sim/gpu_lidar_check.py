"""Accuracy check for Gazebo Fortress's gpu_lidar against analytic geometry.

Fortress has only the GPU LiDAR. Its ranges differ from the exact CPU ray
sensor of Gazebo Classic in two ways (docs/fortress.md, step 5), each modelled
from captures at 450, 900 and 1800 samples per beam:

- direction quantisation: ranges come from a rendered cube map, so a beam's
  direction is off by up to about half a horizontal sample spacing (99th
  percentile 0.43-0.47 spacings). An angle error d_theta moves the hit by
  R * d_theta / tan(grazing angle). One full spacing is allowed.
- tessellation: spheres are drawn as a mesh inside the true surface, so ranges
  to them read long by 0.7-0.8% of the radius whatever the sample count,
  consistent with a 16-segment mesh (gap up to r * (1 - cos(pi/16))).
  The allowance is one-sided: a sphere may read long by up to that gap, never
  short beyond noise.

A point whose error is outside tolerance but whose beam lands on another
object when turned by one spacing is an edge point (box against ground
behind it) and is excluded and counted. An object passes when at least 95%
of its points are within tolerance and its median signed error lies within
[-4 cm, 4 cm + mesh gap]. On the desert world this passes the real sensor at
all three sample counts (>= 99.6% within) and fails a mount 3 cm low, 5 cm
high or yawed 3 deg at every one (validated offline, 2026-11-21).
"""
import math

SPHERE_SEGMENTS = 16
WITHIN_FRACTION = 0.95
BIAS_LIMIT = 0.04          # metres, the Classic median-error limit, kept for bias


def sphere_gap(radius, segments=SPHERE_SEGMENTS):
    """Largest gap between a sphere and its tessellated mesh (m)."""
    return radius * (1.0 - math.cos(math.pi / segments))


def error_bounds(expected_range, sin_incidence, sigma, angle, gap=0.0):
    """(low, high) allowed for measured - expected range of one point (m).

    sin_incidence: |ray . surface normal| (1 = head on, 0 = grazing).
    angle: direction resolution in radians (one horizontal sample spacing).
    gap: mesh gap of a tessellated sphere (the surface is inside, ranges long).
    """
    s = min(max(abs(sin_incidence), 1e-3), 1.0)
    cos_g = math.sqrt(max(0.0, 1.0 - s * s))
    symmetric = 3.0 * sigma + expected_range * angle * cos_g / s
    return -symmetric, symmetric + gap / max(s, 0.1)


def turned(direction, d_azimuth, d_elevation):
    """Unit direction turned by small azimuth/elevation steps (rad)."""
    x, y, z = direction
    azimuth = math.atan2(y, x) + d_azimuth
    elevation = math.asin(max(-1.0, min(1.0, z))) + d_elevation
    return (math.cos(elevation) * math.cos(azimuth), math.cos(elevation) * math.sin(azimuth), math.sin(elevation))


def is_edge(direction, angle, object_hit):
    """True when turning the beam by one step in any direction hits another object.

    object_hit(direction) -> name of the object the beam meets first.
    """
    name = object_hit(direction)
    return any(object_hit(turned(direction, a, e)) != name
               for a, e in ((angle, 0.0), (-angle, 0.0), (0.0, angle), (0.0, -angle)))


def box_normal(hit, center, size):
    """Outward normal of the box face nearest to a hit point."""
    offsets = [(hit[i] - center[i]) / (size[i] / 2.0) for i in range(3)]
    axis = max(range(3), key=lambda i: abs(offsets[i]))
    normal = [0.0, 0.0, 0.0]
    normal[axis] = 1.0 if offsets[axis] >= 0 else -1.0
    return normal


def summarize(errors, within, edges=0, gap=0.0):
    """Per-object result: signed errors and within-tolerance flags of the scored points."""
    errors = [float(e) for e in errors]   # plain JSON types
    n = len(errors)
    low, high = -BIAS_LIMIT, BIAS_LIMIT + gap
    if n == 0:
        return dict(count=0, edge_points=edges, within_fraction=0.0, bias_m=None, median_abs_error_m=None,
                    p95_error_m=None, bias_range_m=[low, high], passed=False)
    ordered = sorted(errors)
    bias = ordered[n // 2] if n % 2 else 0.5 * (ordered[n // 2 - 1] + ordered[n // 2])
    absolute = sorted(abs(e) for e in errors)
    fraction = sum(bool(w) for w in within) / n
    return dict(count=n, edge_points=edges, within_fraction=fraction, bias_m=bias,
                median_abs_error_m=absolute[n // 2], p95_error_m=absolute[min(n - 1, int(0.95 * n))],
                bias_range_m=[low, high],
                passed=bool(n >= 5 and fraction >= WITHIN_FRACTION and low <= bias <= high))
