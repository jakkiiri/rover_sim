"""Procedural rock meshes: displaced icospheres with simplified convex hulls.

Every rock is normalised so that its largest horizontal extent is 1.0 m; the
scatter stage scales instances to the sampled diameter. Visual meshes are
icospheres subdivided twice (320 faces); collision hulls are convex hulls of
at most 30 extremal vertices (<= 56 faces).
"""
import math

import numpy as np
from scipy.spatial import ConvexHull

from rover_sim.terrain.noise import plane_wave_bumps

STYLES = ('rounded', 'flattened', 'angular', 'vesicular', 'elongated', 'blocky')


def icosphere(subdivisions=2):
    t = (1.0 + math.sqrt(5.0)) / 2.0
    vertices = [(-1, t, 0), (1, t, 0), (-1, -t, 0), (1, -t, 0),
                (0, -1, t), (0, 1, t), (0, -1, -t), (0, 1, -t),
                (t, 0, -1), (t, 0, 1), (-t, 0, -1), (-t, 0, 1)]
    faces = [(0, 11, 5), (0, 5, 1), (0, 1, 7), (0, 7, 10), (0, 10, 11),
             (1, 5, 9), (5, 11, 4), (11, 10, 2), (10, 7, 6), (7, 1, 8),
             (3, 9, 4), (3, 4, 2), (3, 2, 6), (3, 6, 8), (3, 8, 9),
             (4, 9, 5), (2, 4, 11), (6, 2, 10), (8, 6, 7), (9, 8, 1)]
    vertices = [np.array(v, dtype=np.float64) / np.linalg.norm(v) for v in vertices]
    for _ in range(subdivisions):
        cache = {}
        new_faces = []

        def midpoint(a, b):
            key = (min(a, b), max(a, b))
            if key not in cache:
                m = vertices[a] + vertices[b]
                vertices.append(m / np.linalg.norm(m))
                cache[key] = len(vertices) - 1
            return cache[key]

        for a, b, c in faces:
            ab, bc, ca = midpoint(a, b), midpoint(b, c), midpoint(c, a)
            new_faces += [(a, ab, ca), (b, bc, ab), (c, ca, bc), (ab, bc, ca)]
        faces = new_faces
    return np.array(vertices), np.array(faces, dtype=np.int64)


def vertex_normals(vertices, faces):
    normals = np.zeros_like(vertices)
    a, b, c = vertices[faces[:, 0]], vertices[faces[:, 1]], vertices[faces[:, 2]]
    face_normals = np.cross(b - a, c - a)
    for i in range(3):
        np.add.at(normals, faces[:, i], face_normals)
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    return normals / np.maximum(lengths, 1e-12)


def make_rock(rng, style):
    """Return (vertices, faces) of a unit-normalised rock of the given style."""
    directions, faces = icosphere(2)
    radius = np.ones(len(directions))
    radius += plane_wave_bumps(rng, directions, count=10, amplitude=0.16, frequency_range=(1.2, 3.0))
    radius += plane_wave_bumps(rng, directions, count=10, amplitude=0.05, frequency_range=(4.0, 8.0))
    if style == 'vesicular':
        radius += plane_wave_bumps(rng, directions, count=16, amplitude=0.03, frequency_range=(10.0, 16.0))
        for _ in range(18):
            centre = rng.normal(size=3)
            centre /= np.linalg.norm(centre)
            dot = np.clip(directions @ centre, -1, 1)
            radius -= 0.06 * np.exp(-((np.arccos(dot)) / 0.18) ** 2)
    vertices = directions * np.clip(radius, 0.55, None)[:, None]
    scale = np.array([1.0, rng.uniform(0.75, 1.0), rng.uniform(0.7, 0.95)])
    if style == 'flattened':
        scale[2] *= rng.uniform(0.45, 0.6)
    elif style == 'elongated':
        scale[0] *= rng.uniform(1.4, 1.8)
        scale[2] *= rng.uniform(0.7, 0.9)
    vertices = vertices * scale
    if style in ('angular', 'blocky'):
        planes = 6 if style == 'angular' else 9
        for _ in range(planes):
            normal = rng.normal(size=3)
            normal /= np.linalg.norm(normal)
            offset = rng.uniform(0.70, 0.92) * np.max(np.abs(vertices @ normal))
            distance = vertices @ normal - offset
            outside = distance > 0
            vertices[outside] -= distance[outside, None] * normal
    # Centre and normalise so the largest horizontal extent is exactly 1 m.
    vertices -= (vertices.min(axis=0) + vertices.max(axis=0)) / 2.0
    horizontal = max(np.ptp(vertices[:, 0]), np.ptp(vertices[:, 1]))
    vertices /= horizontal
    return vertices, faces


def farthest_points(vertices, count):
    chosen = [int(np.argmax(np.linalg.norm(vertices, axis=1)))]
    distances = np.linalg.norm(vertices - vertices[chosen[0]], axis=1)
    while len(chosen) < count:
        index = int(np.argmax(distances))
        chosen.append(index)
        distances = np.minimum(distances, np.linalg.norm(vertices - vertices[index], axis=1))
    return vertices[chosen]


def convex_hull(vertices, count=30):
    points = farthest_points(vertices, count)
    hull = ConvexHull(points)
    centroid = points[hull.vertices].mean(axis=0)
    faces = []
    for simplex in hull.simplices:
        a, b, c = points[simplex]
        if np.dot(np.cross(b - a, c - a), a - centroid) < 0:
            simplex = simplex[[0, 2, 1]]
        faces.append(simplex)
    used = sorted(set(int(i) for f in faces for i in f))
    remap = {old: new for new, old in enumerate(used)}
    return points[used], np.array([[remap[int(i)] for i in f] for f in faces], dtype=np.int64)


def write_obj(path, vertices, faces, normals=None, name='rock'):
    lines = [f'# rover_sim procedural rock\no {name}\n']
    lines += [f'v {v[0]:.5f} {v[1]:.5f} {v[2]:.5f}\n' for v in vertices]
    if normals is not None:
        lines += [f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n' for n in normals]
        lines += [f'f {a+1}//{a+1} {b+1}//{b+1} {c+1}//{c+1}\n' for a, b, c in faces]
    else:
        lines += [f'f {a+1} {b+1} {c+1}\n' for a, b, c in faces]
    path.write_text(''.join(lines))


def build_rock_library(directory, seed, count=14):
    """Write ``count`` rock meshes (+ hulls) and return their metadata list."""
    directory.mkdir(parents=True, exist_ok=True)
    rng = np.random.default_rng(seed + 2000)
    library = []
    for index in range(count):
        style = STYLES[index % len(STYLES)]
        vertices, faces = make_rock(rng, style)
        normals = vertex_normals(vertices, faces)
        hull_vertices, hull_faces = convex_hull(vertices)
        visual = directory / f'rock_{index:02d}.obj'
        collision = directory / f'rock_{index:02d}_hull.obj'
        write_obj(visual, vertices, faces, normals, name=f'rock_{index:02d}')
        # Collision hulls carry normals too: DART (Gazebo Fortress) rejects
        # collision meshes whose normal count differs from the vertex count.
        write_obj(collision, hull_vertices, hull_faces, vertex_normals(hull_vertices, hull_faces),
                  name=f'rock_{index:02d}_hull')
        library.append(dict(index=index, style=style, visual=visual.name, collision=collision.name,
                            faces=int(len(faces)), hull_faces=int(len(hull_faces)),
                            extent=[float(v) for v in np.ptp(vertices, axis=0)],
                            z_min=float(vertices[:, 2].min()), z_max=float(vertices[:, 2].max()),
                            mean_radius=float(np.linalg.norm(vertices, axis=1).mean())))
    return library
