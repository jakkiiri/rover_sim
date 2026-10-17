"""Seeded 2-D gradient noise, fBm, ridged multifractal and domain warping.

Everything is pure numpy and deterministic from the ``numpy.random.Generator``
that seeds the permutation table. Coordinates are in lattice units; callers
scale metres by a base frequency before evaluating.
"""
import math

import numpy as np

GRADIENT_COUNT = 16
_ANGLES = np.arange(GRADIENT_COUNT) * (2.0 * math.pi / GRADIENT_COUNT)
_GRAD_X = np.cos(_ANGLES)
_GRAD_Y = np.sin(_ANGLES)


class GradientNoise:
    """Classic Perlin-style gradient noise with a seeded permutation table."""

    def __init__(self, rng):
        permutation = rng.permutation(256)
        self.perm = np.concatenate([permutation, permutation]).astype(np.int64)

    def _hash(self, ix, iy):
        return self.perm[(self.perm[ix & 255] + (iy & 255)) & 511] % GRADIENT_COUNT

    def __call__(self, x, y):
        x = np.asarray(x, dtype=np.float64)
        y = np.asarray(y, dtype=np.float64)
        xi = np.floor(x).astype(np.int64)
        yi = np.floor(y).astype(np.int64)
        xf = x - xi
        yf = y - yi
        u = xf * xf * xf * (xf * (xf * 6.0 - 15.0) + 10.0)
        v = yf * yf * yf * (yf * (yf * 6.0 - 15.0) + 10.0)

        def corner(dx, dy):
            h = self._hash(xi + dx, yi + dy)
            return _GRAD_X[h] * (xf - dx) + _GRAD_Y[h] * (yf - dy)

        n00, n10, n01, n11 = corner(0, 0), corner(1, 0), corner(0, 1), corner(1, 1)
        top = n00 + u * (n10 - n00)
        bottom = n01 + u * (n11 - n01)
        # Scale so the output comfortably spans about [-1, 1].
        return (top + v * (bottom - top)) * 1.9


def fbm(noise, x, y, octaves=6, gain=0.5, lacunarity=2.0):
    """Fractional Brownian motion: sum_k gain^k * n(lacunarity^k * x)."""
    total = np.zeros(np.broadcast(x, y).shape)
    amplitude, frequency, norm = 1.0, 1.0, 0.0
    for _ in range(octaves):
        total += amplitude * noise(x * frequency + 17.3 * frequency, y * frequency - 11.1 * frequency)
        norm += amplitude
        amplitude *= gain
        frequency *= lacunarity
    return total / norm


def ridged(noise, x, y, octaves=6, gain=0.5, lacunarity=2.0, offset=1.0, sharpness=2.0):
    """Musgrave-style ridged multifractal; returns values in roughly [0, 1]."""
    total = np.zeros(np.broadcast(x, y).shape)
    amplitude, frequency, weight, norm = 1.0, 1.0, np.ones_like(total), 0.0
    for _ in range(octaves):
        signal = offset - np.abs(noise(x * frequency - 5.5 * frequency, y * frequency + 23.7 * frequency))
        signal = np.clip(signal, 0.0, None) ** sharpness
        signal *= weight
        weight = np.clip(signal * 2.0, 0.0, 1.0)
        total += signal * amplitude
        norm += amplitude
        amplitude *= gain
        frequency *= lacunarity
    return total / norm


def domain_warp(noise, x, y, strength, octaves=3):
    """Displace coordinates by two low-frequency fBm fields."""
    wx = fbm(noise, x + 31.7, y - 8.9, octaves=octaves)
    wy = fbm(noise, x - 19.3, y + 47.1, octaves=octaves)
    return x + strength * wx, y + strength * wy


def plane_wave_bumps(rng, directions, count=12, amplitude=0.1, frequency_range=(1.5, 4.0)):
    """Smooth deterministic 3-D bump field on unit directions (N x 3).

    Used for rock displacement: a sum of random cosine plane waves is cheap,
    smooth and needs no 3-D lattice noise.
    """
    directions = np.asarray(directions, dtype=np.float64)
    result = np.zeros(len(directions))
    for _ in range(count):
        k = rng.normal(size=3)
        k /= np.linalg.norm(k)
        freq = rng.uniform(*frequency_range)
        phase = rng.uniform(0.0, 2.0 * math.pi)
        result += np.cos(freq * directions @ k + phase)
    return amplitude * result / math.sqrt(count)
