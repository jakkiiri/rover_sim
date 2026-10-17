"""Rock placement: power-law sizes, clustering, slope rejection, chunking.

Size–frequency follows the Mars rock-abundance power law used by Golombek &
Rapp (1997, JGR 102, 4117) and Golombek et al. (2003, JGR 108, 8086) for
Viking/Pathfinder/MER sites: N(>D) = N0 * D^-b with b in 2–2.6 in the
0.05–1.2 m range. Rocks cluster around crater rims and hill bases, are
rejected on steep slopes, and are merged into one static model per tile so
Gazebo manages a few dozen models instead of a thousand.
"""
import math

import numpy as np

from rover_sim.terrain.heightmap import sample_height, sample_slope, smoothstep

BOULDER_HEIGHTS = (0.10, 0.15, 0.20, 0.25, 0.30, 0.40)
COLLISION_NONE_BELOW = 0.10     # metres: pebbles are visual only
COLLISION_HULL_FROM = 0.30      # metres: hull mesh collision above this


def sample_diameters(rng, count, d_min, d_max, exponent):
    """Inverse-CDF sampling of a truncated Pareto law N(>D) ~ D^-b."""
    u = rng.random(count)
    ratio = (d_min / d_max) ** exponent
    return d_min * (1.0 - u * (1.0 - ratio)) ** (-1.0 / exponent)


def density_weights(terrain):
    """Relative placement density per pixel: rims and hill bases are denser."""
    height, slope, params = terrain['height'], terrain['slope_deg'], terrain['params']
    n = height.shape[0]
    half = params.size / 2.0
    xs = np.linspace(-half, half, n)
    ys = np.linspace(half, -half, n)
    X, Y = np.meshgrid(xs, ys)
    weight = np.ones_like(height)
    for crater in terrain['craters']:
        r = np.hypot(X - crater['x'], Y - crater['y'])
        weight += 3.0 * np.exp(-((r - crater['radius']) / (0.3 * crater['radius'])) ** 2)
    # Hill base: below the local mean height, next to moderate smoothed slopes.
    k = max(3, int(12.0 / terrain['pixel']) | 1)
    padded = np.pad(height, k // 2, mode='edge')
    local = np.zeros_like(height)
    for dy in range(0, k, 4):
        for dx in range(0, k, 4):
            local += padded[dy:dy + n, dx:dx + n]
    local /= len(range(0, k, 4)) ** 2
    padded_slope = np.pad(slope, k // 2, mode='edge')
    smooth_slope = np.zeros_like(height)
    for dy in range(0, k, 4):
        for dx in range(0, k, 4):
            smooth_slope += padded_slope[dy:dy + n, dx:dx + n]
    smooth_slope /= len(range(0, k, 4)) ** 2
    hill_base = np.clip((local - height) / 2.5, 0, 1) * np.clip(smooth_slope / 12.0, 0, 1)
    weight += 2.0 * hill_base
    weight *= 1.0 - terrain['flat_weight']            # keep pad and garden clear
    weight[slope > 28.0] = 0.0                        # no rocks on steep faces
    border = int(2.0 / terrain['pixel'])
    weight[:border, :] = weight[-border:, :] = weight[:, :border] = weight[:, -border:] = 0.0
    return weight / weight.sum()


def place_rocks(terrain, library, count, seed, d_min=0.08, d_max=1.2, exponent=2.1):
    """Return a list of rock instance dicts placed on the heightmap surface."""
    rng = np.random.default_rng(seed + 3000)
    params = terrain['params']
    height = terrain['height']
    weights = density_weights(terrain)
    flat_index = rng.choice(weights.size, size=count, p=weights.ravel())
    rows, cols = np.divmod(flat_index, weights.shape[1])
    pixel = terrain['pixel']
    half = params.size / 2.0
    # Jitter inside the chosen pixel so positions are continuous.
    x = -half + (cols + rng.random(count)) * pixel
    y = half - (rows + rng.random(count)) * pixel
    diameters = sample_diameters(rng, count, d_min, d_max, exponent)
    order = np.argsort(-diameters)              # big rocks first for overlap checks
    instances = []
    placed_xy, placed_r = [], []
    for i in order:
        d = float(diameters[i])
        px, py = float(x[i]), float(y[i])
        slope_here = float(sample_slope(terrain['slope_deg'], params.size, px, py))
        if slope_here > 28.0 or math.hypot(px, py) < params.pad_radius + 1.5:
            continue
        if placed_xy and d >= 0.3:
            near = [j for j, (qx, qy) in enumerate(placed_xy) if abs(qx - px) < 2.0 and abs(qy - py) < 2.0]
            if any(math.hypot(px - placed_xy[j][0], py - placed_xy[j][1]) < 0.6 * (d + placed_r[j]) for j in near):
                continue
        rock = library[int(rng.integers(len(library)))]
        aspect = rng.uniform(0.85, 1.15, size=2)
        scale = [d * aspect[0], d * aspect[1], d * rng.uniform(0.8, 1.1)]
        burial = float(rng.uniform(0.10, 0.35))
        z_ground = float(sample_height(height, params.size, px, py))
        z_extent = scale[2] * (rock['z_max'] - rock['z_min'])
        z = z_ground - burial * z_extent - scale[2] * rock['z_min']
        tilt = math.radians(10.0)
        instances.append(dict(mesh=rock['index'], x=px, y=py, z=z, diameter=d, burial=burial,
                              scale=[float(s) for s in scale],
                              roll=float(rng.uniform(-tilt, tilt)), pitch=float(rng.uniform(-tilt, tilt)),
                              yaw=float(rng.uniform(-math.pi, math.pi)),
                              shade=float(rng.uniform(0.75, 1.15)), rust=float(rng.uniform(0.0, 0.5))))
        placed_xy.append((px, py))
        placed_r.append(d)
    return instances


def labeled_boulders(terrain, library, heights=BOULDER_HEIGHTS):
    """Test boulders in the flat garden strip: exposed height is the label."""
    params = terrain['params']
    x_line = 0.5 * (params.garden_x[0] + params.garden_x[1])
    lanes = np.linspace(-(len(heights) - 1) * 1.5, (len(heights) - 1) * 1.5, len(heights))
    rock = next(r for r in library if r['style'] == 'blocky')
    burial = 0.15
    boulders = []
    for h, lane in zip(heights, lanes):
        z_extent_unit = rock['z_max'] - rock['z_min']
        sz = h / ((1.0 - burial) * z_extent_unit)
        scale = [2.2 * h / rock['extent'][0], 1.6 / rock['extent'][1], sz]
        z_ground = float(sample_height(terrain['height'], params.size, x_line, float(lane)))
        z = z_ground - burial * sz * z_extent_unit - sz * rock['z_min']
        boulders.append(dict(name=f'boulder_{int(round(h * 100)):02d}cm', height=h, mesh=rock['index'],
                             x=float(x_line), y=float(lane), z=z, z_ground=z_ground, scale=scale,
                             burial=burial, yaw=0.0, lane=float(lane)))
    return boulders


def collision_mode(diameter):
    if diameter < COLLISION_NONE_BELOW:
        return 'none'
    if diameter < COLLISION_HULL_FROM:
        return 'sphere'
    return 'hull'


def rock_material(shade, rust):
    base = np.array([0.30, 0.24, 0.20]) * (1 - rust) + np.array([0.55, 0.33, 0.20]) * rust
    diffuse = np.clip(base * shade, 0, 1)
    ambient = diffuse * 0.6
    return ('<material><ambient>%.3f %.3f %.3f 1</ambient><diffuse>%.3f %.3f %.3f 1</diffuse>'
            '<specular>0.05 0.05 0.05 1</specular></material>' % (*ambient, *diffuse))


def _pose(x, y, z, roll, pitch, yaw):
    return f'<pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>'


def chunk_models(instances, library, model_uri, size, tile=20.0, prefix='rocks'):
    """Group instances into one static model per tile; returns SDF text."""
    tiles = {}
    for inst in instances:
        key = (int((inst['x'] + size / 2) // tile), int((inst['y'] + size / 2) // tile))
        tiles.setdefault(key, []).append(inst)
    chunks = []
    for (ix, iy), items in sorted(tiles.items()):
        cx = -size / 2 + (ix + 0.5) * tile
        cy = -size / 2 + (iy + 0.5) * tile
        parts = [f'<model name="{prefix}_{ix}_{iy}"><static>true</static><pose>{cx:.3f} {cy:.3f} 0 0 0 0</pose>'
                 '<link name="rocks">']
        for k, inst in enumerate(items):
            rock = library[inst['mesh']]
            sx, sy, sz = inst['scale']
            pose = _pose(inst['x'] - cx, inst['y'] - cy, inst['z'], inst['roll'], inst['pitch'], inst['yaw'])
            parts.append(f'<visual name="v{k}">{pose}<geometry><mesh><uri>{model_uri}/meshes/{rock["visual"]}</uri>'
                         f'<scale>{sx:.4f} {sy:.4f} {sz:.4f}</scale></mesh></geometry>'
                         f'{rock_material(inst["shade"], inst["rust"])}</visual>')
            mode = collision_mode(inst['diameter'])
            if mode == 'sphere':
                radius = 0.5 * inst['diameter'] * 0.85
                parts.append(f'<collision name="c{k}">{pose}<geometry><sphere><radius>{radius:.4f}</radius></sphere>'
                             '</geometry></collision>')
            elif mode == 'hull':
                parts.append(f'<collision name="c{k}">{pose}<geometry><mesh><uri>{model_uri}/meshes/{rock["collision"]}'
                             f'</uri><scale>{sx:.4f} {sy:.4f} {sz:.4f}</scale></mesh></geometry></collision>')
        parts.append('</link></model>')
        chunks.append(''.join(parts))
    return '\n'.join(chunks)


def boulder_models(boulders, library, model_uri):
    """Labeled boulders as individual static models with a visual height post."""
    colours = {0.10: '0 0.8 0', 0.15: '0.2 0.6 1', 0.20: '1 1 0', 0.25: '1 0.6 0',
               0.30: '1 0 0', 0.40: '0.7 0 0.9'}
    models = []
    for b in boulders:
        rock = library[b['mesh']]
        sx, sy, sz = b['scale']
        colour = colours.get(b['height'], '1 1 1')
        post_z = b['z_ground'] + b['height'] / 2
        models.append(
            f'<model name="{b["name"]}"><static>true</static><pose>{b["x"]:.4f} {b["y"]:.4f} 0 0 0 0</pose>'
            f'<link name="rock"><visual name="visual">{_pose(0, 0, b["z"], 0, 0, b["yaw"])}'
            f'<geometry><mesh><uri>{model_uri}/meshes/{rock["visual"]}</uri><scale>{sx:.4f} {sy:.4f} {sz:.4f}</scale>'
            f'</mesh></geometry>{rock_material(0.9, 0.2)}</visual>'
            f'<collision name="collision">{_pose(0, 0, b["z"], 0, 0, b["yaw"])}<geometry><mesh>'
            f'<uri>{model_uri}/meshes/{rock["collision"]}</uri><scale>{sx:.4f} {sy:.4f} {sz:.4f}</scale></mesh>'
            '</geometry><surface><friction><ode><mu>0.65</mu><mu2>0.65</mu2></ode></friction></surface></collision>'
            f'<visual name="label_post"><pose>0 1.1 {post_z:.4f} 0 0 0</pose><geometry><cylinder><radius>0.02</radius>'
            f'<length>{b["height"]:.4f}</length></cylinder></geometry><material><ambient>{colour} 1</ambient>'
            f'<diffuse>{colour} 1</diffuse><emissive>{colour} 1</emissive></material></visual>'
            '</link></model>')
    return '\n'.join(models)
