"""Named spawn sites and synthetic rock fields for the USGS survey world.

config/usgs_sites.yaml lists the sites; rover_desert.launch.py resolves
site:=<name> to x/y, and scripts/import_usgs_terrain.py --world-only places the
rock fields on the survey surface. Rocks reuse the Mars rock meshes
(models/mars_rocks: horizontal radius 1 m, flat base at z = 0).
"""
import math
from pathlib import Path
import re

import numpy as np
import yaml

ROCK_MESHES = 8
FRICTION = 0.8


def load_sites(path):
    data = yaml.safe_load(Path(path).read_text())
    if data['default_site'] not in data['sites']:
        raise ValueError('default_site is not one of the sites')
    return data


def is_survey_world(world):
    return world == 'usgs' or Path(str(world)).name == 'usgs.world'


def resolve_pose(world, site, x, y, yaw, sites):
    """Spawn (x, y, yaw): explicit values win; otherwise the site's on the survey world, else 0."""
    explicit = [str(v).strip() for v in (x, y, yaw)]
    base = (0.0, 0.0, 0.0)
    if is_survey_world(world):
        name = site or sites['default_site']
        if name not in sites['sites']:
            raise ValueError(f"Unknown site '{name}'; choose one of {sorted(sites['sites'])}")
        entry = sites['sites'][name]
        base = (float(entry['x']), float(entry['y']), float(entry.get('yaw', 0.0)))
    values = tuple(float(v) if v else b for v, b in zip(explicit, base))
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Spawn x/y/yaw must be finite')
    return values


def camera_pose(x, y, z, yaw, back=7.0, up=4.0):
    """GUI camera behind and above the rover, looking at it: (position, (w, x, y, z))."""
    px, py, pz = x - back * math.cos(yaw), y - back * math.sin(yaw), z + up
    pitch = math.atan2(up, back)
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return (px, py, pz), (cy * cp, -sy * sp, cy * sp, sy * cp)


def camera_request(x, y, z, yaw):
    (px, py, pz), (qw, qx, qy, qz) = camera_pose(x, y, z, yaw)
    return (f'pose: {{position: {{x: {px:.3f}, y: {py:.3f}, z: {pz:.3f}}}, '
            f'orientation: {{w: {qw:.6f}, x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}}}}}')


def mesh_top(dae_path):
    """Highest vertex z of a rock COLLADA mesh (its height at scale 1)."""
    text = Path(dae_path).read_text()
    values = np.array(re.search(r'<float_array[^>]*>([^<]*)</float_array>', text).group(1).split(), float)
    return float(values[2::3].max())


def ground_slope(surface, x, y, step=1.0):
    dx = (surface.height(x + step, y) - surface.height(x - step, y)) / (2 * step)
    dy = (surface.height(x, y + step) - surface.height(x, y - step)) / (2 * step)
    return math.hypot(dx, dy)


def place_rocks(field, centre, surface, tops):
    """Rocks around ``centre``: list of dicts (kind, x, y, z, radius, yaw, mesh, flatten)."""
    rng = np.random.default_rng(field['seed'])
    cx, cy = centre
    max_grade = math.tan(math.radians(field['max_slope_deg']))
    placed = []
    for kind, (count, rmin, rmax) in field['groups'].items():
        made = tries = 0
        while made < count and tries < count * 400:
            tries += 1
            angle = rng.uniform(0, 2 * math.pi)
            distance = rng.uniform(field['clear'], field['radius'])
            x, y = cx + distance * math.cos(angle), cy + distance * math.sin(angle)
            r = rmin * (rmax / rmin) ** (rng.random() ** 1.6)
            if distance < field['clear'] + r:
                continue
            if any(math.hypot(x - o['x'], y - o['y']) < r + o['radius'] + 0.4 for o in placed):
                continue
            try:
                grade = ground_slope(surface, x, y, max(r, 0.5))
                ground = surface.height(x, y)
            except ValueError:
                continue
            if grade > max_grade:
                continue
            mesh = int(rng.integers(0, ROCK_MESHES))
            flatten = float(rng.uniform(0.7, 1.1))
            height = tops[mesh] * r * flatten
            # Flat base: sink it by a fifth of its height plus the ground's
            # drop across its radius, so the downhill edge does not float.
            burial = 0.2 * height + r * grade
            if burial > 0.5 * height:   # mostly buried: skip rather than hide it
                continue
            placed.append({'kind': kind, 'x': round(x, 3), 'y': round(y, 3), 'z': round(ground - burial, 3),
                           'radius': round(r, 3), 'yaw': round(float(rng.uniform(-math.pi, math.pi)), 3),
                           'mesh': mesh, 'flatten': round(flatten, 3),
                           'top_above_ground': round(height - burial, 3)})
            made += 1
    return placed


def rocks_sdf(rocks, site):
    parts = []
    for i, o in enumerate(rocks):
        uri = f"model://mars_rocks/meshes/rock_{o['mesh']}.dae"
        s = o['radius']
        scale = f"{s:.4f} {s:.4f} {s * o['flatten']:.4f}"
        geometry = f'<geometry><mesh><uri>{uri}</uri><scale>{scale}</scale></mesh></geometry>'
        parts.append(
            f'    <model name="{site}_{o["kind"]}_{i:03d}"><static>true</static>'
            f'<pose>{o["x"]:.3f} {o["y"]:.3f} {o["z"]:.3f} 0 0 {o["yaw"]:.3f}</pose><link name="link">'
            f'<collision name="collision">{geometry}<surface><friction><ode><mu>{FRICTION}</mu>'
            f'<mu2>{FRICTION}</mu2></ode></friction></surface></collision>'
            f'<visual name="visual">{geometry}</visual></link></model>\n')
    return ''.join(parts)
