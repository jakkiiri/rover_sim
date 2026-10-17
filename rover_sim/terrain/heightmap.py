"""Layered Mars heightmap generation, diagnostics and GeoTIFF export.

Layers (all parameterised, see :class:`TerrainParams`):

1. ridged multifractal hills/ridges,
2. fBm rolling undulation,
3. cm-scale rocky-floor micro-relief,
4. impact craters (bowl, raised rim, ejecta falloff) with power-law sizes,
5. optional domain warping of the large-scale coordinates,
6. optional wind ripples confined to low areas,
7. a flat spawn pad blended smoothly into the surroundings, extended by a
   flat "boulder garden" strip.

Gazebo Classic conventions, verified empirically on this machine
(``docs/terrain.md``): image row 0 is +Y (north), column 0 is -X (west);
8-bit PNG quantises relief to size_z/255 and 16-bit PNG is misread, so the
collision/visual heightmap is a float32 GeoTIFF DEM read through GDAL, whose
extent is encoded in the georeference (no ``<size>`` element).
"""
from dataclasses import asdict, dataclass, field
import json
import math

import numpy as np
from PIL import Image, TiffImagePlugin

from rover_sim.terrain.noise import GradientNoise, domain_warp, fbm, ridged

# Gazebo's SphericalCoordinates::Distance uses a 6371 km sphere.
GAZEBO_EARTH_RADIUS = 6371000.0
METRES_PER_DEGREE = GAZEBO_EARTH_RADIUS * math.pi / 180.0


@dataclass
class TerrainParams:
    seed: int = 7
    resolution: int = 513            # 2^n + 1 pixels per side
    size: float = 120.0              # metres per side
    relief: float = 15.0             # target max - min height in metres
    hill_frequency: float = 1.0 / 120.0   # cycles per metre of the ridged layer
    hill_octaves: int = 4
    hill_gain: float = 0.40
    hill_exponent: float = 1.8     # >1 broadens valleys, sharpens ridges
    undulation_amplitude: float = 0.10   # fraction of relief
    undulation_frequency: float = 1.0 / 30.0
    undulation_octaves: int = 5
    micro_amplitude: float = 0.012       # metres, rocky-floor roughness
    micro_frequency: float = 1.0 / 1.2
    micro_octaves: int = 2
    crater_count: int = 7
    crater_radius_min: float = 3.0
    crater_radius_max: float = 14.0
    crater_depth_ratio: float = 0.12     # depth / radius (degraded craters)
    warp_strength: float = 0.25          # lattice units of the hill layer
    ripple_amplitude: float = 0.06
    ripple_wavelength: float = 1.6
    pad_radius: float = 8.0
    pad_blend: float = 12.0
    garden_x: tuple = (6.0, 17.0)        # flat strip extending the pad (+x)
    garden_half_width: float = 9.5
    garden_blend: float = 8.0

    def validate(self):
        n = self.resolution - 1
        if n < 8 or n & (n - 1):
            raise ValueError('resolution must be 2^n + 1 with n >= 3')
        if self.size <= 0 or self.relief <= 0:
            raise ValueError('size and relief must be positive')
        if not 0 < self.pad_radius < self.size / 2:
            raise ValueError('pad_radius must lie inside the terrain')


def smoothstep(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def grid_coordinates(params):
    """World X/Y (metres) for every pixel; row 0 is +Y, column 0 is -X."""
    half = params.size / 2.0
    xs = np.linspace(-half, half, params.resolution)
    ys = np.linspace(half, -half, params.resolution)
    return np.meshgrid(xs, ys)


def crater_profile(r, radius, depth):
    """Bowl with raised rim and ejecta falloff; returns height change (m)."""
    rim_height = 0.35 * depth
    rim_width = 0.18 * radius
    inside = r < radius
    bowl = -depth * (1.0 - (r / radius) ** 2) + rim_height * (r / radius) ** 6
    rim = rim_height * np.exp(-((r - radius) / rim_width) ** 2)
    ejecta = rim_height * 0.6 * np.clip(radius / np.maximum(r, radius), 0, 1) ** 3 * (r >= radius)
    return np.where(inside, bowl, rim * 0.6 + ejecta)


def generate_heightmap(params):
    """Return a dict with the height field and every intermediate layer."""
    params.validate()
    rng = np.random.default_rng(params.seed)
    noise = GradientNoise(rng)
    X, Y = grid_coordinates(params)

    # 1. Large-scale hills and ridges (ridged multifractal, warped).
    hx, hy = X * params.hill_frequency, Y * params.hill_frequency
    if params.warp_strength > 0:
        hx, hy = domain_warp(noise, hx, hy, params.warp_strength)
    hills = ridged(noise, hx, hy, octaves=params.hill_octaves, gain=params.hill_gain)
    hills = (hills - hills.min()) / max(hills.max() - hills.min(), 1e-9)
    # Shape the histogram: wide valleys, steep ridge flanks.
    hills = hills ** params.hill_exponent

    # 2. Rolling undulation (fBm).
    undulation = fbm(noise, X * params.undulation_frequency + 100.0,
                     Y * params.undulation_frequency - 100.0,
                     octaves=params.undulation_octaves)

    # 3. Rocky-floor micro relief.
    micro = fbm(noise, X * params.micro_frequency + 300.0, Y * params.micro_frequency + 300.0,
                octaves=params.micro_octaves, gain=0.6)

    height = params.relief * (0.78 * hills + params.undulation_amplitude * undulation)
    height += params.micro_amplitude * micro

    # 4. Impact craters: power-law radii, N(>R) ~ R^-2, kept away from the pad.
    craters = []
    for _ in range(params.crater_count):
        u = rng.random()
        rmin, rmax = params.crater_radius_min, params.crater_radius_max
        radius = rmin * (1.0 - u * (1.0 - (rmin / rmax) ** 2)) ** (-0.5)
        for _attempt in range(50):
            cx, cy = rng.uniform(-0.42, 0.42, size=2) * params.size
            if math.hypot(cx, cy) > params.pad_radius + params.pad_blend + radius * 1.4 and \
                    not (params.garden_x[0] - radius < cx < params.garden_x[1] + radius and
                         abs(cy) < params.garden_half_width + radius):
                break
        depth = params.crater_depth_ratio * radius
        r = np.hypot(X - cx, Y - cy)
        height += crater_profile(r, radius, depth)
        craters.append(dict(x=float(cx), y=float(cy), radius=float(radius), depth=float(depth)))

    # 5. Wind ripples in low areas only.
    ripples = np.zeros_like(height)
    if params.ripple_amplitude > 0:
        low = 1.0 - smoothstep(0.18, 0.40, (height - height.min()) / max(np.ptp(height), 1e-9))
        direction = fbm(noise, X / 60.0 + 700.0, Y / 60.0 + 700.0, octaves=2) * 0.8
        phase = (X * np.cos(direction) + Y * np.sin(direction)) * (2.0 * math.pi / params.ripple_wavelength)
        ripples = params.ripple_amplitude * low * (0.5 + 0.5 * np.sin(phase)) ** 1.5
        height += ripples

    # Rescale so the pre-pad relief equals the requested value exactly.
    height = (height - height.min()) * (params.relief / max(np.ptp(height), 1e-9))

    # 6. Flat spawn pad and garden strip, blended with smoothstep.
    r = np.hypot(X, Y)
    pad_weight = 1.0 - smoothstep(params.pad_radius, params.pad_radius + params.pad_blend, r)
    gx0, gx1 = params.garden_x
    dx = np.maximum(np.maximum(gx0 - X, X - gx1), 0.0)
    dy = np.maximum(np.abs(Y) - params.garden_half_width, 0.0)
    garden_weight = 1.0 - smoothstep(0.0, params.garden_blend, np.hypot(dx, dy))
    flat_weight = np.maximum(pad_weight, garden_weight)
    pad_mask = r <= params.pad_radius
    pad_level = float(np.mean(height[pad_mask]))
    height = flat_weight * pad_level + (1.0 - flat_weight) * height

    # Normalise so that the minimum is exactly zero (Gazebo DEM origin).
    offset = float(height.min())
    height -= offset
    pad_level -= offset
    pixel = params.size / (params.resolution - 1)
    slope_deg, roughness = diagnostics(height, pixel)
    return dict(height=height.astype(np.float64), slope_deg=slope_deg, roughness=roughness,
                craters=craters, pad_level=pad_level, pixel=pixel, params=params,
                hills=hills, undulation=undulation, micro=micro, ripples=ripples,
                flat_weight=flat_weight)


def diagnostics(height, pixel):
    gy, gx = np.gradient(height, pixel)
    slope_deg = np.degrees(np.arctan(np.hypot(gx, gy)))
    # Roughness: RMS deviation from a 7x7 (about 1.6 m at 513 px) local mean.
    k = 7
    padded = np.pad(height, k // 2, mode='edge')
    local = np.zeros_like(height)
    for dy in range(k):
        for dx in range(k):
            local += padded[dy:dy + height.shape[0], dx:dx + height.shape[1]]
    local /= k * k
    roughness = np.abs(height - local)
    return slope_deg, roughness


def slope_statistics(slope_deg, flat_weight=None):
    stats = dict(max_slope_deg=float(slope_deg.max()), mean_slope_deg=float(slope_deg.mean()),
                 median_slope_deg=float(np.median(slope_deg)))
    for threshold in (10, 15, 20, 25, 30, 35):
        stats[f'percent_above_{threshold}_deg'] = float(100.0 * np.mean(slope_deg > threshold))
    stats['percent_below_20_deg'] = float(100.0 * np.mean(slope_deg < 20))
    if flat_weight is not None:
        # Erode the flat mask by one pixel so gradient stencils touching the
        # blend ring are excluded.
        mask = flat_weight > 0.999
        core = mask.copy()
        core[1:, :] &= mask[:-1, :]
        core[:-1, :] &= mask[1:, :]
        core[:, 1:] &= mask[:, :-1]
        core[:, :-1] &= mask[:, 1:]
        stats['pad_max_slope_deg'] = float(slope_deg[core].max()) if core.any() else None
    return stats


def sample_height(height, size, x, y):
    """Bilinear height at world (x, y) using the Gazebo image convention."""
    n = height.shape[0]
    half = size / 2.0
    fx = (np.asarray(x, dtype=np.float64) + half) / size * (n - 1)
    fy = (half - np.asarray(y, dtype=np.float64)) / size * (n - 1)
    fx = np.clip(fx, 0, n - 1 - 1e-9)
    fy = np.clip(fy, 0, n - 1 - 1e-9)
    x0 = np.floor(fx).astype(int)
    y0 = np.floor(fy).astype(int)
    x1 = np.minimum(x0 + 1, n - 1)
    y1 = np.minimum(y0 + 1, n - 1)
    tx, ty = fx - x0, fy - y0
    top = height[y0, x0] * (1 - tx) + height[y0, x1] * tx
    bottom = height[y1, x0] * (1 - tx) + height[y1, x1] * tx
    return top * (1 - ty) + bottom * ty


def sample_slope(slope_deg, size, x, y):
    return sample_height(slope_deg, size, x, y)


def surface_normal(height, size, x, y, delta=0.25):
    """Unit normal of the terrain at (x, y) by central differences."""
    hx = (sample_height(height, size, x + delta, y) - sample_height(height, size, x - delta, y)) / (2 * delta)
    hy = (sample_height(height, size, x, y + delta) - sample_height(height, size, x, y - delta)) / (2 * delta)
    n = np.array([-hx, -hy, 1.0])
    return n / np.linalg.norm(n)


def write_geotiff(path, height, size):
    """Write a float32 GeoTIFF whose georeference spans ``size`` metres.

    Gazebo computes the world extent from the corner coordinates with a
    6371 km spherical Earth, so pixel spacing is given in degrees of that
    sphere. The raster origin is placed at the equator so longitude degrees
    and latitude degrees have the same length.
    """
    n = height.shape[0]
    degrees_per_pixel = size / METRES_PER_DEGREE / (n - 1)
    ifd = TiffImagePlugin.ImageFileDirectory_v2()
    ifd[33550] = (degrees_per_pixel, degrees_per_pixel, 0.0)   # ModelPixelScaleTag
    ifd.tagtype[33550] = 12
    # ModelTiepointTag: raster (0,0) -> (lon, lat) of the north-west corner.
    ifd[33922] = (0.0, 0.0, 0.0, -size / 2.0 / METRES_PER_DEGREE, size / 2.0 / METRES_PER_DEGREE, 0.0)
    ifd.tagtype[33922] = 12
    # GeoKeyDirectoryTag: geographic model, pixel-is-area, WGS84 (4326).
    ifd[34735] = (1, 1, 0, 3, 1024, 0, 1, 2, 1025, 0, 1, 1, 2048, 0, 1, 4326)
    ifd.tagtype[34735] = 3
    Image.fromarray(height.astype(np.float32), mode='F').save(str(path), tiffinfo=ifd)


def write_preview_png(path, array, vmin=None, vmax=None):
    vmin = float(array.min()) if vmin is None else vmin
    vmax = float(array.max()) if vmax is None else vmax
    scaled = np.clip((array - vmin) / max(vmax - vmin, 1e-9) * 255.0, 0, 255).astype(np.uint8)
    Image.fromarray(scaled, mode='L').save(str(path))


def params_to_json(params):
    data = asdict(params)
    data['garden_x'] = list(params.garden_x)
    return data


def params_from_json(data):
    data = dict(data)
    if 'garden_x' in data:
        data['garden_x'] = tuple(data['garden_x'])
    return TerrainParams(**data)
