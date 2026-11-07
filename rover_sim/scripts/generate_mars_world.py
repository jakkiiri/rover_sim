#!/usr/bin/env python3
"""Generate the Mars test world: terrain, rocks, shrubs and their layout.

Writes (all deterministic, fixed seed; re-running reproduces them):
  models/mars_terrain/   terrain mesh (COLLADA), its heights (.npy), regolith texture
  models/mars_rocks/     rock meshes (COLLADA) and texture
  models/mars_shrubs/    dry shrub meshes (COLLADA) and texture
  worlds/mars.world      the world: terrain, rocks and shrubs placed on it
  worlds/mars_layout.json  every rock and shrub (position, size) and the terrain
                           features, for validation scripts

Terrain (64 x 64 m mesh, 0.25 m grid spacing, z = 0 at the spawn point):
  - flat spawn area (3 m) blending into gently rolling ground with fine roughness;
  - a smooth hill east of the spawn, steepest flank HILL_SLOPE_DEG (climbable);
  - a steep-sided mesa to the north, flanks MESA_SLOPE_DEG (not climbable);
  - a crater to the south-west with steep walls (CRATER_WALL_DEG) except one
    gentle breach on its east side (CRATER_BREACH_DEG);
  - a dune field to the south.
Rocks follow a size distribution like Mars rock fields: many pebbles the
rover can drive over, fewer rocks it must avoid, a few boulders. There are no
shrubs on Mars; they are here because they are useful test obstacles (thin,
irregular, hard for stereo).

Usage: python3 scripts/generate_mars_world.py [--hill-slope 15 --mesa-slope 35 ...]
"""
import argparse
import json
import math
from pathlib import Path
import struct
import zlib

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SEED = 20261031

TERRAIN_SIZE = 64.0          # m, square, centred on the origin
TERRAIN_PX = 257             # heightmap samples per side (Gazebo needs 2^n + 1)
SPACING = TERRAIN_SIZE / (TERRAIN_PX - 1)
TEXTURE_TILE = 8.0           # m covered by one repeat of the ground texture
FRICTION = 0.8               # same as desert.world's ground (slope tests used it)

SPAWN_CLEAR = 3.0            # m around the origin kept flat and free of rocks
HILL = dict(x=12.0, y=3.0, sigma=3.5)
MESA = dict(x=3.0, y=15.0, radius=4.0, height=1.4)
CRATER = dict(x=-13.0, y=-7.0, radius=7.0, depth=1.5, rim=0.4, gully_half_width=1.6, gully_edge=2.0)
DUNES = dict(y_max=-12.0, amplitude=0.2, wavelength=5.0, angle_deg=25.0)


# --------------------------------------------------------------------------- terrain

def periodic_noise(rng, size, exponent):
    """Tileable noise with a 1/f^exponent spectrum, zero mean and unit deviation."""
    fx = np.fft.fftfreq(size)[:, None]
    fy = np.fft.fftfreq(size)[None, :]
    radius = np.sqrt(fx * fx + fy * fy)
    radius[0, 0] = 1.0
    spectrum = (rng.standard_normal((size, size)) + 1j * rng.standard_normal((size, size))) / radius ** exponent
    spectrum[0, 0] = 0.0
    field = np.fft.ifft2(spectrum).real
    return (field - field.mean()) / field.std()


def smoothstep(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def cosine_ramp(t):
    """0 -> 1 over t in [0, 1] with zero slope at both ends; max slope pi/2."""
    t = np.clip(t, 0.0, 1.0)
    return 0.5 - 0.5 * np.cos(np.pi * t)


def terrain(hill_slope_deg, mesa_slope_deg, crater_wall_deg, crater_breach_deg, rng):
    """Heights [row, col] with row = y index (south to north), col = x index (west to east)."""
    coords = (np.arange(TERRAIN_PX) - (TERRAIN_PX - 1) / 2) * SPACING
    x, y = np.meshgrid(coords, coords)            # x[row, col], y[row, col]
    r0 = np.hypot(x, y)

    # Rolling ground and fine roughness, faded out over the spawn area.
    rolling = 0.22 * periodic_noise(rng, TERRAIN_PX, 2.6)
    rough = 0.025 * periodic_noise(rng, TERRAIN_PX, 1.2)
    h = (rolling + rough) * smoothstep(SPAWN_CLEAR, SPAWN_CLEAR + 4.0, r0)

    # Hill: Gaussian with steepest slope H / sigma * exp(-1/2).
    hs = HILL['sigma']
    hill_height = math.tan(math.radians(hill_slope_deg)) * hs * math.exp(0.5)
    h += hill_height * np.exp(-((x - HILL['x']) ** 2 + (y - HILL['y']) ** 2) / (2 * hs * hs))

    # Mesa: flat top, flanks with a cosine profile whose steepest slope is mesa_slope.
    width = (math.pi / 2) * MESA['height'] / math.tan(math.radians(mesa_slope_deg))
    rm = np.hypot(x - MESA['x'], (y - MESA['y']) * 1.3)   # slightly oval
    h += MESA['height'] * cosine_ramp((MESA['radius'] + width - rm) / width)

    # Crater: flat floor, steep wall, raised rim; a gentle gully cut through the east wall.
    cx, cy, cr, depth = CRATER['x'], CRATER['y'], CRATER['radius'], CRATER['depth']
    rc = np.hypot(x - cx, y - cy)
    wall = (math.pi / 2) * depth / math.tan(math.radians(crater_wall_deg))   # wall run (m)
    floor = cr - wall
    bowl = -depth * (1 - cosine_ramp((rc - floor) / wall))
    run = (math.pi / 2) * depth / math.tan(math.radians(crater_breach_deg))  # gully run (m)
    gully_floor = -depth * (1 - cosine_ramp((x - cx - floor) / run))
    half, edge = CRATER['gully_half_width'], CRATER['gully_edge']
    across = 1 - cosine_ramp((np.abs(y - cy) - half) / edge)                # 1 in the gully
    across *= (x > cx)
    crater = np.minimum(bowl, gully_floor * across)
    rim = CRATER['rim'] * np.exp(-((rc - cr) / 1.5) ** 2) * (1 - across)
    h += crater + rim

    # Dunes: ripples fading in south of y_max.
    a = math.radians(DUNES['angle_deg'])
    phase = (x * math.cos(a) + y * math.sin(a)) * 2 * math.pi / DUNES['wavelength']
    h += DUNES['amplitude'] * np.sin(phase) * smoothstep(DUNES['y_max'], DUNES['y_max'] - 5.0, y)

    h -= h[(TERRAIN_PX - 1) // 2, (TERRAIN_PX - 1) // 2]   # z = 0 at the spawn point
    info = {'hill_height_m': round(hill_height, 3), 'mesa_flank_width_m': round(width, 3),
            'crater_floor_radius_m': round(floor, 3), 'gully_length_m': round(run, 3)}
    return x, y, h, info


def sample(h, px, py):
    """Bilinear terrain height at world (px, py) (arrays or floats)."""
    c = np.asarray(px) / SPACING + (TERRAIN_PX - 1) / 2
    r = np.asarray(py) / SPACING + (TERRAIN_PX - 1) / 2
    c0 = np.clip(np.floor(c).astype(int), 0, TERRAIN_PX - 2)
    r0 = np.clip(np.floor(r).astype(int), 0, TERRAIN_PX - 2)
    fc, fr = c - c0, r - r0
    return ((1 - fr) * ((1 - fc) * h[r0, c0] + fc * h[r0, c0 + 1])
            + fr * ((1 - fc) * h[r0 + 1, c0] + fc * h[r0 + 1, c0 + 1]))


def slope_deg(h):
    gy, gx = np.gradient(h, SPACING)
    return np.degrees(np.arctan(np.hypot(gx, gy)))


# --------------------------------------------------------------------------- images

def write_png(path, pixels):
    """8-bit RGB PNG from an H x W x 3 uint8 array."""
    height, width, _ = pixels.shape
    rows = pixels
    header = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    raw = b''.join(b'\x00' + rows[row].tobytes() for row in range(height))

    def chunk(kind, data):
        return struct.pack('>I', len(data)) + kind + data + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff)

    png = b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', header) + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b'')
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def to_rgb(value, base):
    return (np.clip(value[:, :, None] * np.asarray(base)[None, None, :], 0, 1) * 255 + 0.5).astype(np.uint8)


def regolith_texture(rng, size=1024):
    """Rusty regolith with the desert sand's contrast (stereo matching needs it):
    soft patches, mid-scale structure, grain and dark grit."""
    patches = periodic_noise(rng, size, 1.6)
    mid = periodic_noise(rng, size, 1.0)
    grain = rng.standard_normal((size, size))
    grit = np.clip(periodic_noise(rng, size, 0.6) - 2.2, 0, None) * -0.35
    value = np.clip(0.62 + 0.10 * patches + 0.14 * mid + 0.10 * grain + grit, 0, 1)
    tint = 0.05 * periodic_noise(rng, size, 2.0)
    base = np.stack([0.93 + tint, 0.62 + 0.6 * tint, 0.44 + 0.3 * tint], -1)
    return (np.clip(value[:, :, None] * base * 1.15, 0, 1) * 255 + 0.5).astype(np.uint8)


def rock_texture(rng, size=512):
    speckle = rng.standard_normal((size, size))
    veins = periodic_noise(rng, size, 1.3)
    value = np.clip(0.55 + 0.10 * speckle + 0.12 * veins, 0, 1)
    return to_rgb(value, (0.72, 0.50, 0.40))


def shrub_texture(rng, size=256):
    """Left half: dry wood. Right half: dusty grey-olive foliage."""
    n = rng.standard_normal((size, size))
    wood = to_rgb(np.clip(0.5 + 0.12 * n, 0, 1), (0.55, 0.40, 0.28))
    leaf = to_rgb(np.clip(0.55 + 0.18 * n, 0, 1), (0.58, 0.56, 0.38))
    img = wood.copy()
    img[:, size // 2:] = leaf[:, size // 2:]
    return img


# --------------------------------------------------------------------------- meshes

def icosphere(subdivisions):
    t = (1 + 5 ** 0.5) / 2
    verts = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0), (0, -1, t), (0, 1, t),
             (0, -1, -t), (0, 1, -t), (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11), (1, 5, 9), (5, 11, 4),
             (11, 10, 2), (10, 7, 6), (7, 1, 8), (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8),
             (3, 8, 9), (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    verts = [np.array(v, float) / np.linalg.norm(v) for v in verts]
    for _ in range(subdivisions):
        cache, new_faces = {}, []

        def mid(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = verts[a] + verts[b]
                verts.append(m / np.linalg.norm(m))
                cache[key] = len(verts) - 1
            return cache[key]

        for a, b, c in faces:
            ab, bc, ca = mid(a, b), mid(b, c), mid(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    return np.array(verts), np.array(faces)


def vertex_normals(verts, faces):
    normals = np.zeros_like(verts)
    fn = np.cross(verts[faces[:, 1]] - verts[faces[:, 0]], verts[faces[:, 2]] - verts[faces[:, 0]])
    for i in range(3):
        np.add.at(normals, faces[:, i], fn)
    return normals / np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)


def rock_mesh(rng):
    """Unit-ish rock: lumpy sphere, cut by a few planes for facets, flat-ish base.

    Returns verts, faces, uvs, and its extents (radius in xy, top z, bottom z).
    """
    verts, faces = icosphere(3)
    lumps = np.zeros(len(verts))
    for _ in range(6):
        d = rng.standard_normal(3)
        d /= np.linalg.norm(d)
        lumps += rng.uniform(0.05, 0.14) * np.cos(rng.uniform(1.5, 4.0) * verts @ d + rng.uniform(0, 6.3))
    verts = verts * (1 + lumps)[:, None]
    verts *= np.array([rng.uniform(0.85, 1.15), rng.uniform(0.7, 1.0), rng.uniform(0.5, 0.75)])
    for _ in range(rng.integers(3, 7)):                     # facets
        n = rng.standard_normal(3)
        n /= np.linalg.norm(n)
        offset = rng.uniform(0.55, 0.85)
        depth = verts @ n - offset
        verts -= np.maximum(depth, 0)[:, None] * n
    bottom = -0.25 * (verts[:, 2].max() - verts[:, 2].min())
    verts[:, 2] = np.maximum(verts[:, 2], bottom)           # flat base, buried on placement
    verts -= np.array([0, 0, bottom])                         # base at z = 0
    radius = float(np.hypot(verts[:, 0], verts[:, 1]).max())
    scale = 1.0 / radius                                      # horizontal radius 1
    verts *= scale
    d = verts - verts.mean(axis=0)
    u = 0.5 + np.arctan2(d[:, 1], d[:, 0]) / (2 * np.pi)
    v = 0.5 + np.arcsin(np.clip(d[:, 2] / np.maximum(np.linalg.norm(d, axis=1), 1e-9), -1, 1)) / np.pi
    return verts, faces, np.column_stack([u * 3, v * 2]), {'radius': 1.0, 'top': float(verts[:, 2].max())}


def shrub_mesh(rng):
    """Dry shrub, about 1 m wide and 1 m tall before scaling: woody stems and foliage clumps."""
    verts, faces, uvs = [], [], []

    def add(v, f, uv):
        base = sum(len(x) for x in verts)
        verts.append(v)
        faces.append(f + base)
        uvs.append(uv)

    def stem(a, b, r0, r1):
        axis = b - a
        axis_n = axis / np.linalg.norm(axis)
        side = np.cross(axis_n, [0.3, 0.5, 0.8])
        side /= np.linalg.norm(side)
        up = np.cross(axis_n, side)
        ring = [np.cos(k * 2 * np.pi / 5) * side + np.sin(k * 2 * np.pi / 5) * up for k in range(5)]
        v = np.array([a + r0 * d for d in ring] + [b + r1 * d for d in ring])
        f = []
        for k in range(5):
            k1 = (k + 1) % 5
            f += [(k, k1, 5 + k1), (k, 5 + k1, 5 + k)]
        uv = np.column_stack([np.tile(np.linspace(0.05, 0.45, 5), 2), np.repeat([0.0, 1.0], 5)])
        add(v, np.array(f), uv)

    ball_v, ball_f = icosphere(1)

    def clump(c, r):
        v = ball_v * r * (1 + 0.25 * rng.standard_normal((len(ball_v), 1)))
        v[:, 2] *= 0.7
        add(v + c, ball_f.copy(), np.column_stack([0.55 + 0.4 * rng.random(len(v)), rng.random(len(v))]))

    def branch(a, direction, length, radius, depth):
        b = a + direction * length
        stem(a, b, radius, radius * 0.6)
        if depth == 0:
            clump(b, rng.uniform(0.09, 0.16))
            return
        for _ in range(rng.integers(2, 4)):
            d = direction + rng.normal(0, 0.55, 3)
            d[2] = abs(d[2]) + 0.25
            branch(b, d / np.linalg.norm(d), length * rng.uniform(0.55, 0.8), radius * 0.6, depth - 1)

    for _ in range(rng.integers(4, 7)):
        d = np.array([rng.normal(0, 0.5), rng.normal(0, 0.5), 1.0])
        branch(np.array([rng.normal(0, 0.05), rng.normal(0, 0.05), 0.0]), d / np.linalg.norm(d),
               rng.uniform(0.3, 0.42), 0.025, 2)
    v = np.concatenate(verts)
    v[:, 2] = np.maximum(v[:, 2], 0.0)
    span = max(np.hypot(v[:, 0], v[:, 1]).max(), 1e-6)
    v[:, :2] *= 0.5 / span                                     # horizontal radius 0.5
    v[:, 2] *= 1.0 / v[:, 2].max()                             # height 1
    return v, np.concatenate(faces), np.concatenate(uvs), {'radius': 0.5, 'top': 1.0}


def write_collada(path, verts, faces, uvs, texture, name):
    """COLLADA mesh with one texture. Each input gets its own index offset: Gazebo's
    loader misreads triangles whose inputs share one offset (meshes came out torn)."""
    normals = vertex_normals(verts, faces)

    def floats(a):
        return ' '.join(f'{x:.5f}' for x in np.asarray(a).ravel())

    n = len(verts)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_images><image id="tex"><init_from>{texture}</init_from></image></library_images>
  <library_effects><effect id="fx"><profile_COMMON>
    <newparam sid="surf"><surface type="2D"><init_from>tex</init_from></surface></newparam>
    <newparam sid="samp"><sampler2D><source>surf</source></sampler2D></newparam>
    <technique sid="common"><lambert>
      <ambient><color>0.7 0.7 0.7 1</color></ambient>
      <diffuse><texture texture="samp" texcoord="uv"/></diffuse>
    </lambert></technique>
  </profile_COMMON></effect></library_effects>
  <library_materials><material id="mat"><instance_effect url="#fx"/></material></library_materials>
  <library_geometries><geometry id="{name}"><mesh>
    <source id="pos"><float_array id="pos-a" count="{3 * n}">{floats(verts)}</float_array>
      <technique_common><accessor source="#pos-a" count="{n}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
    <source id="nrm"><float_array id="nrm-a" count="{3 * n}">{floats(normals)}</float_array>
      <technique_common><accessor source="#nrm-a" count="{n}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
    <source id="uv"><float_array id="uv-a" count="{2 * n}">{floats(uvs)}</float_array>
      <technique_common><accessor source="#uv-a" count="{n}" stride="2">
        <param name="S" type="float"/><param name="T" type="float"/>
      </accessor></technique_common></source>
    <vertices id="vtx"><input semantic="POSITION" source="#pos"/></vertices>
    <triangles material="m" count="{len(faces)}">
      <input semantic="VERTEX" source="#vtx" offset="0"/>
      <input semantic="NORMAL" source="#nrm" offset="1"/>
      <input semantic="TEXCOORD" source="#uv" offset="2" set="0"/>
      <p>{' '.join(f'{i} {i} {i}' for i in np.asarray(faces).ravel())}</p>
    </triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node id="{name}-node">
    <instance_geometry url="#{name}"><bind_material><technique_common>
      <instance_material symbol="m" target="#mat"><bind_vertex_input semantic="uv" input_semantic="TEXCOORD" input_set="0"/></instance_material>
    </technique_common></bind_material></instance_geometry>
  </node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
''')


def terrain_mesh(h):
    """Triangle mesh of the height grid (two triangles per cell) with ground-texture UVs."""
    coords = (np.arange(TERRAIN_PX) - (TERRAIN_PX - 1) / 2) * SPACING
    x, y = np.meshgrid(coords, coords)
    verts = np.column_stack([x.ravel(), y.ravel(), h.ravel()])
    uvs = np.column_stack([x.ravel() / TEXTURE_TILE, y.ravel() / TEXTURE_TILE])
    idx = np.arange(TERRAIN_PX * TERRAIN_PX).reshape(TERRAIN_PX, TERRAIN_PX)
    a, b = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel()      # row r: (c, c+1)
    c, d = idx[1:, :-1].ravel(), idx[1:, 1:].ravel()        # row r+1
    faces = np.concatenate([np.column_stack([a, b, d]), np.column_stack([a, d, c])])  # counter-clockwise from above
    return verts, faces, uvs


def model_folder(folder, name, description):
    folder.mkdir(parents=True, exist_ok=True)
    (folder / 'model.config').write_text(f'''<?xml version="1.0"?>
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author><name>rover_sim (scripts/generate_mars_world.py)</name></author>
  <description>{description}</description>
</model>
''')


# --------------------------------------------------------------------------- placement

def place_objects(rng, h, slope):
    """Rocks and shrubs: (kind, mesh index, x, y, yaw, scale)."""
    placed = []

    def free(x, y, r):
        if math.hypot(x, y) < SPAWN_CLEAR + r or abs(x) > 29 or abs(y) > 29:
            return False
        # The crater and its gully stay clear: the only way in must stay drivable.
        cx, cy = CRATER['x'], CRATER['y']
        if math.hypot(x - cx, y - cy) < CRATER['radius'] + 0.5 + r:
            return False
        if x > cx and abs(y - cy) < CRATER['gully_half_width'] + CRATER['gully_edge'] + r and x < 2.0:
            return False
        return all(math.hypot(x - o['x'], y - o['y']) > r + o['radius'] + 0.4 for o in placed)

    def local_slope(x, y):
        c = int(round(x / SPACING + (TERRAIN_PX - 1) / 2))
        r = int(round(y / SPACING + (TERRAIN_PX - 1) / 2))
        return float(slope[r, c])

    # (kind, count, horizontal radius range m, near-spawn share, max ground slope)
    groups = [('pebble', 60, (0.06, 0.13), 0.7, 30),
              ('rock', 45, (0.18, 0.4), 0.7, 25),
              ('boulder', 8, (0.6, 1.1), 0.5, 20),
              ('shrub', 22, (0.25, 0.45), 0.6, 18)]
    for kind, count, (rmin, rmax), near, max_slope in groups:
        made, tries = 0, 0
        while made < count and tries < count * 400:
            tries += 1
            # Near the spawn (within 16 m) or anywhere on the map.
            if rng.random() < near:
                a, d = rng.uniform(0, 2 * np.pi), rng.uniform(SPAWN_CLEAR + 0.5, 16.0)
                x, y = d * math.cos(a), d * math.sin(a)
            else:
                x, y = rng.uniform(-28, 28), rng.uniform(-28, 28)
            # Size: power law, small ones common.
            r = rmin * (rmax / rmin) ** (rng.random() ** 1.6)
            if not free(x, y, r) or local_slope(x, y) > max_slope:
                continue
            placed.append({'kind': kind, 'x': round(x, 3), 'y': round(y, 3), 'radius': round(r, 3),
                           'yaw': round(rng.uniform(-math.pi, math.pi), 3),
                           'mesh': int(rng.integers(0, 8 if kind != 'shrub' else 4))})
            made += 1
    return placed


# --------------------------------------------------------------------------- world

def world_sdf(h, objects, rock_info, shrub_info, z_min, z_range):
    parts = []
    for i, o in enumerate(objects):
        ground = float(sample(h, o['x'], o['y']))
        if o['kind'] == 'shrub':
            info = shrub_info[o['mesh']]
            s = o['radius'] / info['radius']
            height = o['height']
            name = f"shrub_{i:03d}"
            uri = f"model://mars_shrubs/meshes/shrub_{o['mesh']}.dae"
            z = ground - 0.03
            scale = f"{s:.4f} {s:.4f} {height:.4f}"
            # Collision: a cylinder around the stems and lower foliage.
            collision = (f'<collision name="collision"><pose>0 0 {height * 0.45:.3f} 0 0 0</pose><geometry><cylinder>'
                         f'<radius>{o["radius"] * 0.7:.3f}</radius><length>{height * 0.9:.3f}</length>'
                         f'</cylinder></geometry></collision>')
        else:
            info = rock_info[o['mesh']]
            s = o['radius'] / info['radius']
            z = ground - o['burial']
            name = f"{o['kind']}_{i:03d}"
            uri = f"model://mars_rocks/meshes/rock_{o['mesh']}.dae"
            scale = f"{s:.4f} {s:.4f} {s * o['flatten']:.4f}"
            collision = (f'<collision name="collision"><geometry><mesh><uri>{uri}</uri><scale>{scale}</scale>'
                         f'</mesh></geometry><surface><friction><ode><mu>{FRICTION}</mu><mu2>{FRICTION}</mu2>'
                         f'</ode></friction></surface></collision>')
        parts.append(f'''
    <model name="{name}">
      <static>true</static>
      <pose>{o['x']:.3f} {o['y']:.3f} {z:.3f} 0 0 {o['yaw']:.3f}</pose>
      <link name="link">
        {collision}
        <visual name="visual"><geometry><mesh><uri>{uri}</uri><scale>{scale}</scale></mesh></geometry></visual>
      </link>
    </model>''')
    return f'''<?xml version="1.0"?>
<!-- Generated by scripts/generate_mars_world.py; edit that script, not this file.
     Mars-like test terrain (Earth gravity, as chosen for the planner tests). -->
<sdf version="1.6">
  <world name="mars">
    <gravity>0 0 -9.81</gravity>
    <physics name="default_physics" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode><solver><type>quick</type><iters>150</iters><sor>1.0</sor></solver></ode>
    </physics>
    <scene>
      <ambient>0.62 0.52 0.44 1</ambient>
      <background>0.83 0.66 0.50 1</background>
      <shadows>true</shadows>
      <grid>false</grid>
    </scene>
    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <diffuse>1 0.93 0.84 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.45 0.35 -0.8</direction>
    </light>
    <gui fullscreen="0">
      <camera name="overview">
        <pose>-12 -16 12 0 0.5 0.9</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>
    <model name="mars_terrain">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><mesh><uri>model://mars_terrain/meshes/terrain.dae</uri></mesh></geometry>
          <surface><friction><ode><mu>{FRICTION}</mu><mu2>{FRICTION}</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>model://mars_terrain/meshes/terrain.dae</uri></mesh></geometry>
        </visual>
      </link>
    </model>{''.join(parts)}
  </world>
</sdf>
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--hill-slope', type=float, default=15.0, help='steepest hill flank (deg)')
    parser.add_argument('--mesa-slope', type=float, default=35.0, help='steepest mesa flank (deg)')
    parser.add_argument('--crater-wall', type=float, default=32.0, help='steepest crater wall (deg)')
    parser.add_argument('--crater-breach', type=float, default=12.0, help='steepest slope of the breach (deg)')
    args = parser.parse_args()
    rng = np.random.default_rng(SEED)

    x, y, h, info = terrain(args.hill_slope, args.mesa_slope, args.crater_wall, args.crater_breach, rng)
    slope = slope_deg(h)
    z_min, z_max = float(h.min()), float(h.max())
    z_range = z_max - z_min
    # The terrain is a triangle mesh, not a Gazebo <heightmap>: ODE tests every
    # LiDAR ray against every heightmap cell under it (720 rays x 30 m stalled the
    # simulation), while meshes are ray-tested through a bounding-box tree.
    # (Gazebo 11 heightmap PNGs are also 8-bit only: 12 mm terraces here.)
    terrain_dir = ROOT / 'models' / 'mars_terrain'
    verts, faces, uvs = terrain_mesh(h)
    write_collada(terrain_dir / 'meshes' / 'terrain.dae', verts, faces, uvs,
                  'regolith.png', 'terrain')
    np.save(terrain_dir / 'heightmap.npy', h.astype(np.float32))
    write_png(terrain_dir / 'meshes' / 'regolith.png', regolith_texture(rng))
    model_folder(terrain_dir, 'mars_terrain', 'Heightmap and regolith texture for worlds/mars.world.')
    (terrain_dir / 'model.sdf').write_text('<?xml version="1.0"?>\n<!-- Resource folder; the terrain is '
                                           'defined in worlds/mars.world. -->\n<sdf version="1.6"><model name='
                                           '"mars_terrain"><static>true</static><link name="link"/></model></sdf>\n')

    rock_dir, shrub_dir = ROOT / 'models' / 'mars_rocks', ROOT / 'models' / 'mars_shrubs'
    write_png(rock_dir / 'meshes' / 'rock.png', rock_texture(rng))
    write_png(shrub_dir / 'meshes' / 'shrub.png', shrub_texture(rng))
    rock_info, shrub_info = [], []
    for k in range(8):
        v, f, uv, meta = rock_mesh(rng)
        write_collada(rock_dir / 'meshes' / f'rock_{k}.dae', v, f, uv, 'rock.png', f'rock_{k}')
        rock_info.append(meta)
    for k in range(4):
        v, f, uv, meta = shrub_mesh(rng)
        write_collada(shrub_dir / 'meshes' / f'shrub_{k}.dae', v, f, uv, 'shrub.png', f'shrub_{k}')
        shrub_info.append(meta)
    for folder, name, text in ((rock_dir, 'mars_rocks', 'Rock meshes for worlds/mars.world.'),
                               (shrub_dir, 'mars_shrubs', 'Dry shrub meshes for worlds/mars.world.')):
        model_folder(folder, name, text)
        (folder / 'model.sdf').write_text(f'<?xml version="1.0"?>\n<!-- Resource folder of meshes. -->\n'
                                          f'<sdf version="1.6"><model name="{name}"><static>true</static>'
                                          f'<link name="link"/></model></sdf>\n')

    objects = place_objects(rng, h, slope)
    for o in objects:
        if o['kind'] == 'shrub':
            o['height'] = round(o['radius'] * rng.uniform(1.0, 1.5), 3)
            o['top_above_ground'] = round(o['height'] - 0.03, 3)
        else:
            meta = rock_info[o['mesh']]
            o['flatten'] = round(rng.uniform(0.7, 1.1), 3)
            full = meta['top'] * o['radius'] * o['flatten']       # mesh height after scaling
            o['burial'] = round(0.2 * full, 3)
            o['top_above_ground'] = round(full - o['burial'], 3)
    world = world_sdf(h, objects, rock_info, shrub_info, z_min, z_range)
    (ROOT / 'worlds' / 'mars.world').write_text(world)

    layout = {
        'generator': 'scripts/generate_mars_world.py', 'seed': SEED,
        'terrain': {'size_m': TERRAIN_SIZE, 'samples': TERRAIN_PX, 'spacing_m': SPACING,
                    'heights': 'models/mars_terrain/heightmap.npy ([row = y south->north, col = x west->east])',
                    'mesh': 'models/mars_terrain/meshes/terrain.dae (two triangles per cell)',
                    'z_min': round(z_min, 4), 'z_max': round(z_max, 4), 'friction': FRICTION},
        'features': {'hill': {**HILL, 'steepest_deg': args.hill_slope, 'height_m': info['hill_height_m']},
                     'mesa': {**MESA, 'steepest_deg': args.mesa_slope, 'flank_width_m': info['mesa_flank_width_m']},
                     'crater': {**CRATER, 'wall_deg': args.crater_wall, 'breach_deg': args.crater_breach,
                                'breach_side': 'east', 'floor_radius_m': info['crater_floor_radius_m'],
                                'gully_length_m': info['gully_length_m']},
                     'dunes': DUNES},
        'objects': objects,
    }
    (ROOT / 'worlds' / 'mars_layout.json').write_text(json.dumps(layout, indent=1))
    counts = {k: sum(o['kind'] == k for o in objects) for k in ('pebble', 'rock', 'boulder', 'shrub')}
    print(f'terrain {z_min:.2f}..{z_max:.2f} m, steepest {slope.max():.1f} deg; objects {counts}')


if __name__ == '__main__':
    main()
