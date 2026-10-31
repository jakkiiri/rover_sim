"""Textured mesh visual for the terrain (robust alternative to the heightmap shader).

Gazebo Classic's heightmap material shader fails to compile on some Mesa
drivers (measured here: the terrain renders black). A plain OBJ mesh with a
baked albedo texture uses the standard material path and always renders.
The mesh is a decimated grid of the DEM; the albedo is blended by **slope and
height** (sand in low flat areas, gravel on flanks, basalt on steep/high
ground) with detail noise, so it also removes the heightmap's height-only
blending limitation. Collision stays on the exact heightfield.
"""
import numpy as np
from PIL import Image

from rover_sim.terrain.noise import GradientNoise, fbm
from rover_sim.terrain.textures import PALETTE


def decimate(height, step):
    return height[::step, ::step]


def write_terrain_mesh(path, height, size, step=2):
    """Write an OBJ grid mesh with UVs and smooth normals; returns (vertices, faces) counts."""
    grid = decimate(height, step)
    n = grid.shape[0]
    half = size / 2.0
    xs = np.linspace(-half, half, n)
    ys = np.linspace(half, -half, n)
    X, Y = np.meshgrid(xs, ys)
    # Normals by central differences (image row 0 is +Y).
    spacing = size / (n - 1)
    gy, gx = np.gradient(grid, spacing)
    normals = np.stack([-gx, gy, np.ones_like(grid)], axis=-1)   # dh/d(row) points to -Y
    normals /= np.linalg.norm(normals, axis=-1, keepdims=True)
    vertices = np.stack([X.ravel(), Y.ravel(), grid.ravel()], axis=1)
    us = (X.ravel() + half) / size
    vs = (Y.ravel() + half) / size
    index = np.arange(n * n).reshape(n, n)
    a, b, c, d = index[:-1, :-1], index[:-1, 1:], index[1:, :-1], index[1:, 1:]
    # Counter-clockwise seen from above (+Z): a(top-left) -> c(bottom-left) -> b(top-right).
    faces = np.concatenate([np.stack([a, c, b], axis=-1).reshape(-1, 3),
                            np.stack([b, c, d], axis=-1).reshape(-1, 3)])
    lines = ['# rover_sim terrain visual mesh\n', f'mtllib {path.stem}.mtl\n', 'o terrain\n']
    lines += [f'v {v[0]:.3f} {v[1]:.3f} {v[2]:.3f}\n' for v in vertices]
    lines += [f'vt {u:.5f} {v:.5f}\n' for u, v in zip(us, vs)]
    lines += [f'vn {nv[0]:.4f} {nv[1]:.4f} {nv[2]:.4f}\n' for nv in normals.reshape(-1, 3)]
    lines.append('usemtl terrain\n')
    lines += [f'f {f[0]+1}/{f[0]+1}/{f[0]+1} {f[1]+1}/{f[1]+1}/{f[1]+1} {f[2]+1}/{f[2]+1}/{f[2]+1}\n' for f in faces]
    path.write_text(''.join(lines))
    path.with_suffix('.mtl').write_text(
        'newmtl terrain\nKa 0.55 0.50 0.45\nKd 1.0 1.0 1.0\nKs 0.02 0.02 0.02\nNs 4\nmap_Kd terrain_albedo.png\n')
    return len(vertices), len(faces)


def bake_albedo(path, height, slope_deg, seed, resolution=2048):
    """Slope/height-blended albedo over the whole terrain (one texel ~ 6 cm)."""
    rng = np.random.default_rng(seed + 4000)
    noise = GradientNoise(rng)
    n = height.shape[0]
    # Resample height/slope to the texture grid (nearest is fine at 4x).
    rows = (np.arange(resolution) * (n - 1) / (resolution - 1)).round().astype(int)
    h = height[rows][:, rows]
    s = slope_deg[rows][:, rows]
    h01 = (h - h.min()) / max(np.ptp(h), 1e-9)
    coords = np.linspace(0, 48, resolution)
    X, Y = np.meshgrid(coords, coords)
    detail = fbm(noise, X, Y, octaves=4)
    grain = fbm(noise, X * 6 + 300, Y * 6 + 300, octaves=2)
    # Material weights: basalt on steep ground and ridge tops, gravel on flanks, sand in low flats.
    w_basalt = np.clip((s - 18.0) / 12.0, 0, 1) * 0.8 + np.clip((h01 - 0.7) / 0.3, 0, 1) * 0.5
    w_basalt = np.clip(w_basalt + 0.15 * detail, 0, 1)
    w_gravel = np.clip((s - 6.0) / 10.0, 0, 1) * (1 - w_basalt)
    w_gravel = np.clip(w_gravel + 0.15 * detail, 0, 1) * (1 - w_basalt)
    w_sand = np.clip(1 - w_basalt - w_gravel, 0, 1)

    def colour(name):
        base, dark = (np.array(c) for c in PALETTE[name])
        mix = np.clip(0.55 + 0.45 * detail, 0, 1)[..., None]
        return base * mix + dark * (1 - mix)

    rgb = w_sand[..., None] * colour('sand') + w_gravel[..., None] * colour('gravel') + w_basalt[..., None] * colour('basalt')
    rgb *= (1.0 + 0.08 * grain)[..., None]
    # Gentle shading by slope so relief reads even under flat lighting.
    rgb *= (1.0 - 0.25 * np.clip(s / 45.0, 0, 1))[..., None]
    image = (np.clip(rgb, 0, 1) * 255).astype(np.uint8)
    # OBJ UV v=0 is the bottom (south, -Y) row; our arrays have row 0 = north.
    Image.fromarray(image[::-1], mode='RGB').save(str(path), compress_level=6)
    return dict(sand=float(w_sand.mean()), gravel=float(w_gravel.mean()), basalt=float(w_basalt.mean()))
