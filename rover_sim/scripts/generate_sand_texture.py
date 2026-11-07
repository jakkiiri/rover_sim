#!/usr/bin/env python3
"""Generate models/rover_sim_materials/materials/textures/rover_sand.png, a seamless sand texture.

Stereo matching needs texture; the desert ground was a flat colour. This makes
a deterministic (fixed seed), tileable, non-periodic-within-a-tile noise image
with numpy only and writes it as PNG with zlib, so no image library is needed.
Re-running it reproduces the committed file byte for byte.
"""
import argparse
from pathlib import Path
import struct
import zlib

import numpy as np


def periodic_noise(rng, size, exponent):
    """Tileable noise with a 1/f^exponent spectrum (FFT noise wraps around seamlessly)."""
    fx = np.fft.fftfreq(size)[:, None]
    fy = np.fft.fftfreq(size)[None, :]
    radius = np.sqrt(fx * fx + fy * fy)
    radius[0, 0] = 1.0
    spectrum = (rng.standard_normal((size, size)) + 1j * rng.standard_normal((size, size))) / radius ** exponent
    spectrum[0, 0] = 0.0
    field = np.fft.ifft2(spectrum).real
    return (field - field.mean()) / field.std()


def sand(size=1024, seed=20261017):
    rng = np.random.default_rng(seed)
    dunes = periodic_noise(rng, size, 1.6)        # large soft variations
    ripples = periodic_noise(rng, size, 1.0)      # mid-scale structure for matching
    grain = rng.standard_normal((size, size))     # per-texel grain
    shade = 0.10 * dunes + 0.14 * ripples + 0.10 * grain
    pebbles = np.clip(periodic_noise(rng, size, 0.6) - 2.2, 0, None) * -0.35  # dark specks
    value = np.clip(0.62 + shade + pebbles, 0.0, 1.0)
    base = np.array([0.93, 0.78, 0.55])           # sand colour
    rgb = np.clip(value[:, :, None] * base[None, None, :] * 1.15, 0, 1)
    return (rgb * 255 + 0.5).astype(np.uint8)


def write_png(path, rgb):
    height, width, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[row].tobytes() for row in range(height))

    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff)

    png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_bytes(png)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    default = (Path(__file__).resolve().parents[1] / 'models' / 'rover_sim_materials' / 'materials'
               / 'textures' / 'rover_sand.png')
    parser.add_argument('--output', type=Path, default=default)
    parser.add_argument('--size', type=int, default=1024)
    args = parser.parse_args()
    write_png(args.output, sand(args.size))
    print(f'Wrote {args.output}')


if __name__ == '__main__':
    main()
