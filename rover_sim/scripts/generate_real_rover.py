#!/usr/bin/env python3
"""Build the real rover's simulation meshes and geometry from its Onshape export.

Input: the Onshape URDF export in cad/assembly_1 (urdf/assembly_1.urdf and 442
STL meshes, 4.2 M triangles). Every joint in that export is fixed, its masses
are wrong (0.04 kg in total), it has no collision shapes and some parts float
in the wrong place (broken mates). So the export is used for looks and
dimensions only:

* each part is placed with the export's zero pose, then assigned to a sim link
  by where it is: wheel (spinning), steering module, rocker, body or arm;
* parts from broken mates are left out (they float above or beside the rover),
  and the arm parts are rearranged into a stowed pose folded down in front;
* both rockers are rotated about their pivots until their two wheels are level,
  so the joint zero is the level pose;
* each link's parts are merged per colour, simplified by vertex clustering and
  written as binary STL in that link's frame (meshes/real_rover/);
* dimensions, collision boxes and inertias go to config/real_rover.yaml, which
  urdf/real_rover.urdf.xacro reads.

Frames: CAD +x is the rover's left, CAD -y its front, CAD +z up. Sim base_link
is x forward, y left, z up, with its origin on the rocker pivot axis (in plan)
at the CAD z = 0 height. Run from the package root:

    python3 scripts/generate_real_rover.py [--cad cad/assembly_1] [--preview DIR]
"""
import argparse
import collections
import math
import os
import re
import struct
import xml.etree.ElementTree as ET

import numpy as np

TOTAL_MASS = 42.0          # kg, measured real rover (user, 2026-11-14)
LIDAR_MASS = 0.8           # kg, urdf/upstream/mars_lidar3d.urdf.xacro
POLE_MASS = 0.4            # kg, 1 m aluminium tube
WHEEL_MASS = 1.0           # kg each, tyre + rim
MODULE_MASS = 1.0          # kg each, steering module (wheel mount + motors)
ROCKER_MASS = 2.0          # kg each
ARM_MASS = 4.0             # kg, stowed arm
POLE_LENGTH = 1.0          # m above the top plate
ROCKER_Y = 0.241          # m, rocker pivot frames (Onshape suspension_left/right) from the centre line
POLE_RADIUS = 0.02
# Pole foot in CAD (x, y): plate centre, moved 0.08 m to CAD +x (rover left)
# so it clears the science box (CAD x -0.20..0.03).
POLE_CAD_XY = (0.08, 0.0)
LIDAR_ABOVE_POLE = 0.035   # housing (0.07 m) sits on the pole; optical centre at its middle
MIN_PART_SIZE = 0.012      # m, bounding-box diagonal; smaller parts (screws, nuts) are dropped
UPPER_ARM_LINK = 'new_shoulder_linkage_assem_1__1_'
CELL = 0.004               # m, vertex-clustering grid for simplification


# ---------------------------------------------------------------- CAD reading
def origin_matrix(element):
    xyz = [float(v) for v in (element.get('xyz') or '0 0 0').split()] if element is not None else [0, 0, 0]
    rpy = [float(v) for v in (element.get('rpy') or '0 0 0').split()] if element is not None else [0, 0, 0]
    a, b, c = rpy
    rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    ry = np.array([[math.cos(b), 0, math.sin(b)], [0, 1, 0], [-math.sin(b), 0, math.cos(b)]])
    rz = np.array([[math.cos(c), -math.sin(c), 0], [math.sin(c), math.cos(c), 0], [0, 0, 1]])
    m = np.eye(4)
    m[:3, :3] = rz @ ry @ rx
    m[:3, 3] = xyz
    return m


def read_stl(path):
    data = open(path, 'rb').read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        return np.array(re.findall(rb'vertex\s+(\S+)\s+(\S+)\s+(\S+)', data), float).reshape(-1, 3, 3)
    count = struct.unpack('<I', data[80:84])[0]
    rows = np.frombuffer(data[84:84 + 50 * count],
                         dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    return rows['v'].reshape(-1, 3, 3).astype(float)


Part = collections.namedtuple('Part', 'name link tris rgb')


def load_parts(cad):
    """Every mesh visual of the export as world (CAD) triangles at its zero pose."""
    root = ET.parse(os.path.join(cad, 'urdf', 'assembly_1.urdf')).getroot()
    children = collections.defaultdict(list)
    for joint in root.findall('joint'):
        children[joint.find('parent').get('link')].append(joint)
    world, stack = {'root': np.eye(4)}, ['root']
    while stack:
        parent = stack.pop()
        for joint in children[parent]:
            child = joint.find('child').get('link')
            world[child] = world[parent] @ origin_matrix(joint.find('origin'))
            stack.append(child)
    parts = []
    for link in root.findall('link'):
        for visual in link.findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None:        # marker spheres at the wheel centres
                continue
            m = world[link.get('name')] @ origin_matrix(visual.find('origin'))
            tris = read_stl(os.path.join(cad, mesh.get('filename').replace('package://assembly_1/', '')))
            tris = tris @ m[:3, :3].T + m[:3, 3]
            color = visual.find('material/color')
            rgb = tuple(float(v) for v in color.get('rgba').split()[:3]) if color is not None else (0.6, 0.6, 0.6)
            parts.append(Part(os.path.basename(mesh.get('filename')), link.get('name'), tris, rgb))
    return parts


def bounds(tris):
    points = tris.reshape(-1, 3)
    return points.min(0), points.max(0)


def touching_groups(parts, skip=(), tolerance=0.004):
    """Parts whose bounding boxes touch, grouped transitively (union-find)."""
    boxes = np.array([bounds(p.tris) for p in parts])
    boxes[list(skip), 0], boxes[list(skip), 1] = 1e9, 1e9 + 1    # out of reach of everything
    parent = list(range(len(parts)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i
    for i in range(len(parts)):
        overlap = np.all((boxes[:, 0] - tolerance <= boxes[i, 1]) & (boxes[:, 1] + tolerance >= boxes[i, 0]), axis=1)
        for j in np.nonzero(overlap)[0]:
            parent[find(i)] = find(int(j))
    groups = collections.defaultdict(list)
    for i in range(len(parts)):
        if i not in skip:
            groups[find(i)].append(i)
    return sorted(groups.values(), key=len, reverse=True)


def align(a, b):
    """Rotation matrix turning unit vector a onto unit vector b (Rodrigues)."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    k = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + k + k @ k / (1 + c)


def rotate_x(points, angle, pivot):
    """Rotate CAD points about the CAD x axis through pivot (y, z)."""
    c, s = math.cos(angle), math.sin(angle)
    out = points.copy()
    y, z = points[..., 1] - pivot[0], points[..., 2] - pivot[1]
    out[..., 1] = pivot[0] + c * y - s * z
    out[..., 2] = pivot[1] + s * y + c * z
    return out


# ------------------------------------------------------------ classification
def classify(parts):
    """Map each part index to a sim link name, or None to leave it out.

    Returns (assignment, wheels, pivot) with wheels = {name: centre (CAD)} and
    pivot = rocker pivot (CAD y, z). Also rearranges arm and levels rockers by
    editing the parts' triangles in place.
    """
    assignment = {}
    # The upper arm is one Onshape link, misplaced by a broken mate across the
    # rear of the body; it is taken out before grouping and stowed below.
    upper = [i for i, p in enumerate(parts) if p.link == UPPER_ARM_LINK]
    if not upper:
        raise RuntimeError(f'upper arm link {UPPER_ARM_LINK} not found in the export')
    groups = touching_groups(parts, skip=set(upper))
    main = groups[0]
    group_box = [np.array([bounds(np.concatenate([parts[i].tris for i in g]))]).reshape(2, 3) for g in groups]

    # Wheels: the four TPU tyres.
    tyres = [i for i in main if 'Outer_TPU' in parts[i].name]
    if len(tyres) != 4:
        raise RuntimeError(f'expected 4 tyres, found {len(tyres)}')
    wheels = {}
    for i in tyres:
        lo, hi = bounds(parts[i].tris)
        c = (lo + hi) / 2
        name = ('front' if c[1] < 0 else 'rear') + '_' + ('left' if c[0] > 0 else 'right')
        wheels[name] = c
    radius = float(np.mean([(bounds(parts[i].tris)[1] - bounds(parts[i].tris)[0])[1:].mean() / 2 for i in tyres]))
    width = float(np.mean([(bounds(parts[i].tris)[1] - bounds(parts[i].tris)[0])[0] for i in tyres]))

    # Rocker pivots: the split tapered bushings at CAD x +-0.26.
    bushings = [i for i in main if 'tapered_bushing' in parts[i].name.lower() or 'Tapered_Bushing' in parts[i].name]
    centres = [sum(bounds(parts[i].tris)) / 2 for i in bushings]
    side_bushings = [c for c in centres if abs(c[0]) > 0.2]
    pivot = (float(np.mean([c[1] for c in side_bushings])), float(np.mean([c[2] for c in side_bushings]))) \
        if side_bushings else (-0.036, -0.039)

    for i in main:
        lo, hi = bounds(parts[i].tris)
        c = (lo + hi) / 2
        link = None
        for name, w in wheels.items():
            radial = math.hypot(c[1] - w[1], c[2] - w[2])
            if np.sign(c[0]) == np.sign(w[0]) and radial < radius + 0.01:
                if abs(c[0] - w[0]) <= width / 2 + 0.015 and radial < radius and \
                        abs(c[0]) > abs(w[0]) - width / 2 - 0.015:
                    link = name + '_wheel'
                elif radial < 0.09 and abs(c[0]) > 0.29:
                    link = name + '_axle'
            if link:
                break
        if link is None:
            if abs(c[0]) > 0.245 and c[2] < 0.05:
                link = ('left' if c[0] > 0 else 'right') + '_rocker'
            else:
                link = 'base_link'
        assignment[i] = link

    # Everything else: science payload (body), arm pieces, and parts from
    # broken mates floating away from the rover.
    turntable_shift = None
    arm_groups = {'forearm': [], 'turntable': [], 'actuator': [], 'bracket': [], 'upper': upper}
    for g, (lo, hi) in zip(groups[1:], group_box[1:]):
        size = hi - lo
        if (hi[2] > 0.47 or lo[0] < -0.6 or hi[0] > 0.6 or abs(lo[0] + hi[0]) / 2 > 0.4
                or np.linalg.norm(size) < MIN_PART_SIZE):
            for i in g:
                assignment[i] = None                    # broken mate / degenerate
        elif lo[1] < -0.9 and hi[2] > 0.3:
            arm_groups['forearm'] += g
        elif hi[1] < -0.55 and hi[2] < 0.1:
            arm_groups['turntable'] += g
        elif hi[2] < -0.2 and lo[1] > -0.6:
            arm_groups['actuator'] += g
        elif hi[1] < -0.2 and lo[1] > -0.5 and hi[2] > 0.2:
            arm_groups['bracket'] += g
        else:
            for i in g:
                assignment[i] = 'base_link'
    for key, members in arm_groups.items():
        if not members:
            raise RuntimeError(f'arm group {key} not found in the export')
        for i in members:
            assignment[i] = 'arm_link'

    def move(members, transform):
        for i in members:
            parts[i] = parts[i]._replace(tris=transform(parts[i].tris))

    # Stowed arm. Turntable and its gearbox go under the shoulder bracket on the
    # front plate; the shoulder actuator beside the bracket on the shoulder axis;
    # the forearm/end effector hangs straight down in front of the bracket.
    blo, bhi = bounds(np.concatenate([parts[i].tris for i in arm_groups['bracket']]))
    plate = [i for i in arm_groups['turntable'] if 'base_plate' in parts[i].name]
    tlo, thi = bounds(np.concatenate([parts[i].tris for i in (plate or arm_groups['turntable'])]))
    turntable_shift = np.array([(blo[0] + bhi[0]) / 2 - (tlo[0] + thi[0]) / 2,
                                (blo[1] + bhi[1]) / 2 - (tlo[1] + thi[1]) / 2, blo[2] - thi[2]])
    move(arm_groups['turntable'], lambda t: t + turntable_shift)
    alo, ahi = bounds(np.concatenate([parts[i].tris for i in arm_groups['actuator']]))
    shoulder_z = 0.10           # flanged shoulder bearing centre in the bracket
    move(arm_groups['actuator'], lambda t: t + np.array([blo[0] - 0.005 - ahi[0],
                                                         (blo[1] + bhi[1]) / 2 - (alo[1] + ahi[1]) / 2,
                                                         shoulder_z - (alo[2] + ahi[2]) / 2]))
    # Upper arm: shoulder end in the bracket on the shoulder axis, pointing
    # straight up. Its shoulder end is the one nearer its carbon-fibre bracket.
    pts = np.concatenate([parts[i].tris for i in upper]).reshape(-1, 3)
    mean = pts.mean(0)
    axis = np.linalg.svd(pts - mean, full_matrices=False)[2][0]
    bracket = [i for i in upper if 'Shoulder_CF_Bracket' in parts[i].name]
    shoulder_end = np.concatenate([parts[i].tris for i in bracket]).reshape(-1, 3).mean(0) if bracket else mean
    if np.dot(shoulder_end - mean, axis) > 0:
        axis = -axis                       # axis now points shoulder -> elbow
    rotation = align(axis, np.array([0.0, 0.0, 1.0]))
    move(upper, lambda t: (t - mean) @ rotation.T)
    ulo, uhi = bounds(np.concatenate([parts[i].tris for i in upper]))
    shoulder = np.array([(blo[0] + bhi[0]) / 2, (blo[1] + bhi[1]) / 2, shoulder_z])
    move(upper, lambda t: t + np.array([shoulder[0] - (ulo[0] + uhi[0]) / 2,
                                        shoulder[1] - (ulo[1] + uhi[1]) / 2, shoulder[2] - 0.03 - ulo[2]]))
    ulo, uhi = bounds(np.concatenate([parts[i].tris for i in upper]))
    # Forearm and end effector: folded down from the elbow, in front of the upper arm.
    pts = np.concatenate([parts[i].tris for i in arm_groups['forearm']]).reshape(-1, 3)
    yz = pts[:, 1:] - pts[:, 1:].mean(0)
    axis = np.linalg.svd(yz, full_matrices=False)[2][0]          # long axis in (y, z)
    angle = -math.pi / 2 - math.atan2(axis[1], axis[0])          # turn it to point down
    centre = tuple(pts[:, 1:].mean(0))
    move(arm_groups['forearm'], lambda t: rotate_x(t, angle, centre))
    flo, fhi = bounds(np.concatenate([parts[i].tris for i in arm_groups['forearm']]))
    if fhi[2] - flo[2] < fhi[1] - flo[1]:                         # long axis came out horizontal
        move(arm_groups['forearm'], lambda t: rotate_x(t, math.pi / 2, centre))
        flo, fhi = bounds(np.concatenate([parts[i].tris for i in arm_groups['forearm']]))
    move(arm_groups['forearm'], lambda t: t + np.array([shoulder[0] - (flo[0] + fhi[0]) / 2,
                                                         min(blo[1], ulo[1]) - 0.005 - fhi[1],
                                                         uhi[2] - fhi[2]]))

    # Level each rocker: rotate it (with its modules and wheels) about the pivot
    # until its front and rear wheel centres are at the same height.
    for side in ('left', 'right'):
        front, rear = wheels['front_' + side], wheels['rear_' + side]
        angle = -math.atan2(rear[2] - front[2], rear[1] - front[1])
        members = [i for i, link in assignment.items()
                   if link and (link == side + '_rocker' or link.endswith(side + '_wheel')
                                or link.endswith(side + '_axle'))]
        move(members, lambda t: rotate_x(t, angle, pivot))
        for name in ('front_' + side, 'rear_' + side):
            wheels[name] = rotate_x(wheels[name][None, :], angle, pivot)[0]
    return assignment, wheels, pivot, radius, width


# ------------------------------------------------------------------- frames
def to_sim(points, pivot_y):
    """CAD points -> sim base_link points."""
    out = np.empty_like(points)
    out[..., 0] = -(points[..., 1] - pivot_y)
    out[..., 1] = points[..., 0]
    out[..., 2] = points[..., 2]
    return out


def simplify(tris, cell):
    """Vertex clustering: snap to a grid, average each cell, drop collapsed faces."""
    vertices = tris.reshape(-1, 3)
    keys = np.floor(vertices / cell).astype(np.int64)
    _, index, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    inverse = inverse.reshape(-1)
    sums = np.zeros((len(index), 3))
    np.add.at(sums, inverse, vertices)
    merged = sums / np.bincount(inverse)[:, None]
    faces = inverse.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    first = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)[1]   # drop duplicate faces
    return merged[faces[np.sort(first)]]


def write_stl(path, tris):
    tris = tris.astype(np.float32)
    normals = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-12)
    rows = np.zeros(len(tris), dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    rows['n'] = normals
    rows['v'] = tris.reshape(-1, 9)
    with open(path, 'wb') as f:
        f.write(b'rover_sim real rover'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(rows.tobytes())


PALETTE = {  # name: rgb; each part goes to the nearest
    'grey': (0.63, 0.63, 0.63), 'light': (0.88, 0.90, 0.93), 'dark': (0.15, 0.15, 0.17),
    'black': (0.05, 0.05, 0.05), 'orange': (0.95, 0.50, 0.05), 'blue': (0.76, 0.80, 0.95),
    'white': (0.98, 0.98, 0.98)}


def colour_name(rgb):
    return min(PALETTE, key=lambda k: sum((a - b) ** 2 for a, b in zip(PALETTE[k], rgb)))


def box_inertia(mass, size):
    x, y, z = (float(v) for v in size)
    return [mass * (y * y + z * z) / 12, mass * (x * x + z * z) / 12, mass * (x * x + y * y) / 12]


def r6(v):
    return [round(float(x), 4) for x in v]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    package = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument('--cad', default=os.path.join(package, 'cad', 'assembly_1'))
    parser.add_argument('--out', default=package)
    parser.add_argument('--preview', help='also write CAD-classification preview images here')
    args = parser.parse_args()

    parts = load_parts(args.cad)
    assignment, wheels, (pivot_y, pivot_z), radius, width = classify(parts)
    sim = {k: to_sim(v[None, :], pivot_y)[0] for k, v in wheels.items()}

    # Link frames in base_link (sim) at zero joint angles.
    frames = {'base_link': np.zeros(3), 'arm_link': np.zeros(3)}
    for side, sign in (('left', 1), ('right', -1)):
        frames[side + '_rocker'] = np.array([0.0, sign * ROCKER_Y, pivot_z])
    for name, c in sim.items():
        frames[name + '_axle'] = c
        frames[name + '_wheel'] = c

    mesh_dir = os.path.join(args.out, 'meshes', 'real_rover')
    os.makedirs(mesh_dir, exist_ok=True)
    for old in os.listdir(mesh_dir):
        if old.endswith('.stl'):
            os.remove(os.path.join(mesh_dir, old))
    by_link = collections.defaultdict(lambda: collections.defaultdict(list))
    kept = dropped = small = 0
    link_points = collections.defaultdict(list)
    for i, part in enumerate(parts):
        link = assignment.get(i)
        if link is None:
            dropped += 1
            continue
        lo, hi = bounds(part.tris)
        if np.linalg.norm(hi - lo) < MIN_PART_SIZE:
            small += 1
            continue
        kept += 1
        tris = to_sim(part.tris, pivot_y) - frames[link]
        if link.endswith('_wheel'):
            # Wheel joint rpy (pi/2, 0, 0): wheel frame = axle frame rotated +90 deg about x.
            tris = tris @ np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]]).T
        by_link[link][colour_name(part.rgb)].append(tris)
        link_points[link].append(to_sim(part.tris, pivot_y).reshape(-1, 3))

    visuals = collections.defaultdict(list)
    total = 0
    for link, colours in sorted(by_link.items()):
        for colour, chunks in sorted(colours.items()):
            tris = simplify(np.concatenate(chunks), CELL)
            if len(tris) == 0:
                continue
            filename = f'{link}_{colour}.stl'
            write_stl(os.path.join(mesh_dir, filename), tris)
            visuals[link].append({'mesh': filename, 'rgb': r6(PALETTE[colour])})
            total += len(tris)

    # Dimensions (sim frame).
    body_pts = np.concatenate(link_points['base_link'])
    arm_pts = np.concatenate(link_points['arm_link'])
    left = np.concatenate(link_points['left_rocker'])
    front_x = float(np.mean([sim['front_left'][0], sim['front_right'][0]]))
    rear_x = float(np.mean([sim['rear_left'][0], sim['rear_right'][0]]))
    wheel_z = float(np.mean([c[2] for c in sim.values()]))
    track_y = float(np.mean([abs(c[1]) for c in sim.values()]))
    ground_z = wheel_z - radius
    # Chassis box: the body below the top plate, without the payload on top.
    chassis = body_pts[(body_pts[:, 2] < 0.24) & (np.abs(body_pts[:, 0]) < 0.3) & (np.abs(body_pts[:, 1]) < 0.27)]
    clo, chi = np.percentile(chassis, 0.2, axis=0), np.percentile(chassis, 99.8, axis=0)
    payload = body_pts[(body_pts[:, 2] >= 0.24)]
    plo, phi = payload.min(0), payload.max(0)
    top_z = float(chi[2])
    alo, ahi = arm_pts.min(0), arm_pts.max(0)

    pole_xy = to_sim(np.array([[POLE_CAD_XY[0], POLE_CAD_XY[1], 0.0]]), pivot_y)[0][:2]
    lidar_z = top_z + POLE_LENGTH + LIDAR_ABOVE_POLE

    # Masses: the body carries what is left of TOTAL_MASS.
    body_mass = TOTAL_MASS - LIDAR_MASS - POLE_MASS - ARM_MASS - 2 * ROCKER_MASS - 4 * (WHEEL_MASS + MODULE_MASS)
    rocker_lo, rocker_hi = left.min(0) - frames['left_rocker'], left.max(0) - frames['left_rocker']

    def box(lo, hi):
        lo, hi = np.asarray(lo), np.asarray(hi)
        return {'xyz': r6((lo + hi) / 2), 'size': r6(hi - lo)}

    # Rocker collision: one box from the pivot to each wheel's module, under the tubes.
    def segment(x_end, z_end):
        length = math.hypot(x_end, z_end - 0.0) - 0.09
        pitch = math.atan2(-(z_end), x_end)
        mid = (x_end / 2 * (length / math.hypot(x_end, z_end)), z_end / 2 * (length / math.hypot(x_end, z_end)))
        return {'xyz': r6([mid[0], 0.0, mid[1]]), 'rpy': r6([0.0, pitch, 0.0]), 'size': r6([length, 0.05, 0.05])}
    fz = sim['front_left'][2] - pivot_z
    rocker_boxes = [segment(front_x, fz), segment(rear_x, fz)]

    geometry = {
        'source': 'scripts/generate_real_rover.py from cad/assembly_1 (Onshape export)',
        'total_mass': TOTAL_MASS,
        'wheel_radius': round(radius, 4), 'wheel_width': round(width, 4),
        'wheel_x_front': round(front_x, 4), 'wheel_x_rear': round(rear_x, 4),
        'wheel_y': round(track_y, 4), 'wheel_z': round(wheel_z, 4),
        'rocker_y': ROCKER_Y, 'rocker_z': round(pivot_z, 4),
        'ground_z': round(ground_z, 4),
        'top_plate_z': round(top_z, 4),
        'pole': {'xy': r6(pole_xy), 'base_z': round(top_z, 4), 'length': POLE_LENGTH,
                 'radius': POLE_RADIUS, 'mass': POLE_MASS},
        'lidar_xyz': r6([pole_xy[0], pole_xy[1], lidar_z]),
        'masses': {'body': round(body_mass, 4), 'arm': ARM_MASS, 'rocker': ROCKER_MASS,
                   'module': MODULE_MASS, 'wheel': WHEEL_MASS, 'lidar': LIDAR_MASS},
        'body': {'chassis': box(clo, chi), 'payload': box(plo, phi),
                 'inertia': box_inertia(body_mass, chi - clo), 'com': r6((clo + chi) / 2)},
        'arm': {'box': box(alo, ahi), 'inertia': box_inertia(ARM_MASS, ahi - alo), 'com': r6((alo + ahi) / 2)},
        'rocker': {'boxes': rocker_boxes, 'com': r6((rocker_lo + rocker_hi) / 2),
                   'inertia': box_inertia(ROCKER_MASS, rocker_hi - rocker_lo)},
        'extent': {'front': round(float(max(ahi[0], front_x + radius)), 4),
                   'rear': round(float(rear_x - radius), 4),
                   'half_width': round(track_y + width / 2, 4)},
        'visuals': {k: v for k, v in sorted(visuals.items())},
    }
    with open(os.path.join(args.out, 'config', 'real_rover.yaml'), 'w') as f:
        f.write('# GENERATED by scripts/generate_real_rover.py - do not edit by hand.\n')
        f.write('# Real rover dimensions in base_link (x forward, y left, z up), metres / kg.\n')
        write_yaml(f, geometry, 0)
    print(f'parts: {kept} kept, {small} too small, {dropped} left out (broken mates)')
    print(f'{total} triangles in {sum(len(v) for v in visuals.values())} meshes')
    print(f'wheels: radius {radius:.3f} width {width:.3f} x {front_x:.3f}/{rear_x:.3f} '
          f'y +-{track_y:.3f} z {wheel_z:.3f}; ground z {ground_z:.3f}; lidar z {lidar_z:.3f}')
    if args.preview:
        preview(parts, assignment, args.preview)


def write_yaml(f, value, indent):
    pad = '  ' * indent
    for key, item in value.items():
        if isinstance(item, dict):
            f.write(f'{pad}{key}:\n')
            write_yaml(f, item, indent + 1)
        elif isinstance(item, list) and item and isinstance(item[0], dict):
            f.write(f'{pad}{key}:\n')
            for entry in item:
                inner = ', '.join(f'{k}: {fmt(v)}' for k, v in entry.items())
                f.write(f'{pad}  - {{{inner}}}\n')
        else:
            f.write(f'{pad}{key}: {fmt(item)}\n')


def fmt(value):
    if isinstance(value, str):
        return f"'{value}'"
    if isinstance(value, list):
        return '[' + ', '.join(fmt(v) for v in value) + ']'
    return repr(value)


def preview(parts, assignment, folder):
    """Side and top views of the CAD coloured by sim link (needs OpenCV)."""
    import cv2
    colours = {}
    rng = np.random.default_rng(3)
    os.makedirs(folder, exist_ok=True)
    for view, (h_axis, v_axis, depth_axis, depth_sign) in {'side': (1, 2, 0, 1), 'top': (0, 1, 2, 1)}.items():
        scale, lo, hi = 500, (-1.0, -1.0), (1.0, 1.0)
        image = np.full((int((hi[1] - lo[1]) * scale), int((hi[0] - lo[0]) * scale), 3), 255, np.uint8)
        order = []
        for i, part in enumerate(parts):
            link = assignment.get(i)
            if link is None:
                continue
            colours.setdefault(link, tuple(int(c) for c in rng.integers(40, 220, 3)))
            for t in part.tris[::3]:
                order.append((t[:, depth_axis].mean() * depth_sign, t, colours[link]))
        order.sort(key=lambda e: e[0])
        for _, t, colour in order:
            p = np.stack([(t[:, h_axis] - lo[0]) * scale, (hi[1] - t[:, v_axis]) * scale], 1).astype(np.int32)
            cv2.fillConvexPoly(image, p, colour)
        y = 20
        for link, colour in sorted(colours.items()):
            cv2.putText(image, link, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
            y += 16
        cv2.imwrite(os.path.join(folder, f'classes_{view}.png'), image)


if __name__ == '__main__':
    main()
