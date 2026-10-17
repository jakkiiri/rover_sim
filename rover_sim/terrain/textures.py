"""Procedural, tileable albedo and normal textures for the Mars heightmap.

Three materials are produced (fine sand, gravel, basalt) with a butterscotch /
rust / dark-basalt palette. Gazebo Classic blends heightmap textures by
height only (``<blend>`` has ``min_height``/``fade_dist``), so the look is
height-banded: sand in the lows, gravel on the flanks, basalt on the ridges.
"""
import numpy as np
from PIL import Image

from rover_sim.terrain.noise import GradientNoise, fbm


PALETTE = {
    # Linear-ish RGB in [0, 1]; butterscotch / rust / basalt.
    'sand': ((0.80, 0.56, 0.36), (0.70, 0.46, 0.28)),
    'gravel': ((0.62, 0.38, 0.24), (0.46, 0.27, 0.17)),
    'basalt': ((0.30, 0.24, 0.20), (0.18, 0.15, 0.13)),
}


def _tile_grid(size, cells):
    coords = np.arange(size) / size * cells
    return np.meshgrid(coords, coords)


def _normal_map(height, strength):
    gy, gx = np.gradient(height)
    nx, ny, nz = -gx * strength, -gy * strength, np.ones_like(height)
    norm = np.sqrt(nx * nx + ny * ny + nz * nz)
    rgb = np.stack([(nx / norm) * 0.5 + 0.5, (ny / norm) * 0.5 + 0.5, (nz / norm) * 0.5 + 0.5], axis=-1)
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


def _albedo(base, dark, variation, grain):
    base, dark = np.array(base), np.array(dark)
    mix = np.clip(0.5 + 0.5 * variation, 0, 1)[..., None]
    rgb = base * mix + dark * (1 - mix)
    rgb *= (1.0 + 0.10 * grain)[..., None]
    return (np.clip(rgb, 0, 1) * 255).astype(np.uint8)


def make_material(name, rng, size=1024):
    noise = GradientNoise(rng)
    base, dark = PALETTE[name]
    if name == 'sand':
        x, y = _tile_grid(size, 8)
        variation = fbm(noise, x, y, octaves=5)
        xg, yg = _tile_grid(size, 128)
        grain = fbm(noise, xg + 50, yg + 50, octaves=2)
        relief = 0.35 * variation + 0.15 * grain
        normal = _normal_map(relief, strength=6.0)
    elif name == 'gravel':
        x, y = _tile_grid(size, 32)
        pebbles = np.abs(fbm(noise, x + 9, y + 3, octaves=3))
        pebbles = 1.0 - np.clip(pebbles * 2.5, 0, 1)
        xl, yl = _tile_grid(size, 4)
        variation = 0.6 * fbm(noise, xl, yl, octaves=4) + 0.8 * (pebbles - 0.5)
        grain = fbm(noise, x * 4 + 77, y * 4 + 77, octaves=2)
        normal = _normal_map(pebbles, strength=10.0)
    else:  # basalt
        x, y = _tile_grid(size, 16)
        cracks = np.abs(fbm(noise, x + 5, y + 5, octaves=4))
        cracks = np.clip(cracks * 3.0, 0, 1)
        xl, yl = _tile_grid(size, 4)
        variation = 0.5 * fbm(noise, xl + 1, yl + 1, octaves=4) - 0.6 * (1 - cracks)
        grain = fbm(noise, x * 8 + 13, y * 8 + 13, octaves=2)
        normal = _normal_map(cracks + 0.2 * grain, strength=12.0)
    return _albedo(base, dark, variation, grain), normal


def write_textures(directory, seed, size=1024):
    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed + 1000)
    written = []
    for name in ('sand', 'gravel', 'basalt'):
        albedo, normal = make_material(name, rng, size)
        for suffix, data in (('albedo', albedo), ('normal', normal)):
            path = directory / f'{name}_{suffix}.png'
            Image.fromarray(data, mode='RGB').save(str(path), optimize=False, compress_level=6)
            written.append(path)
    return written
