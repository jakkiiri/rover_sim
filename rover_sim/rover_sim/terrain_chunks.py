"""Minecraft-style terrain chunks for the USGS world.

build_chunks() cuts the 294 survey tiles (128 m) into CHUNK_SIZE (16 m)
chunks. The triangles are unchanged (the 4 m and 1 m grids line up with 16 m
boundaries) and keep their tile's normals and UVs, so chunk edges shade
without seams. The ~17,500 chunk meshes (~140 MB of COLLADA) are not shipped:
the launch builds them once into ~/.cache/rover_sim (a few seconds) and
rebuilds when the tiles change.

Only chunks within `radius` chunks of the rover (a square, Chebyshev distance,
like Minecraft's render distance) are in the simulation:
  - at launch, chunk_world() writes a world holding the chunks around the spawn;
  - ChunkStreamer then spawns chunks that come into range through /sim/spawn_entity
    and removes those farther than radius + margin through /sim/delete_entity.
The margin keeps a chunk from being removed and re-added as the rover wanders
along a chunk edge.
"""
import hashlib
import os
import shutil
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

CHUNK_SIZE = 16.0
TEMPLATE_MARKER = '<!-- TERRAIN_CHUNKS -->'
_NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


# ---- assets -------------------------------------------------------------

def read_collada(path):
    """(vertices, normals, uvs, faces) of a single-mesh COLLADA file written by
    generate_mars_world.write_collada (one index per vertex in every input)."""
    root = ET.parse(path).getroot()

    def floats(source_id, width):
        text = root.find(f".//c:source[@id='{source_id}']/c:float_array", _NS).text
        return np.array(text.split(), dtype=float).reshape(-1, width)
    p = np.array(root.find('.//c:triangles/c:p', _NS).text.split(), dtype=np.int64)
    inputs = len(root.findall('.//c:triangles/c:input', _NS))
    faces = p[::inputs].reshape(-1, 3)
    return floats('pos', 3), floats('nrm', 3), floats('uv', 2), faces


def chunk_index(x, y, origin, size=CHUNK_SIZE):
    """Integer chunk (ix, iy) containing world point x, y."""
    return math.floor((x - origin[0]) / size), math.floor((y - origin[1]) / size)


def split_tile(vertices, normals, uvs, faces, tile_origin, grid_origin, size=CHUNK_SIZE):
    """Cut one tile's mesh into chunks by triangle centroid.

    Returns {(ix, iy): (vertices, normals, uvs, faces)} with vertices relative
    to the chunk's corner (grid_origin + (ix, iy) * size).
    """
    world = vertices[:, :2] + np.asarray(tile_origin, dtype=float)
    centroids = world[faces].mean(axis=1)
    ix = np.floor((centroids[:, 0] - grid_origin[0]) / size).astype(int)
    iy = np.floor((centroids[:, 1] - grid_origin[1]) / size).astype(int)
    chunks = {}
    for key in sorted(set(zip(ix.tolist(), iy.tolist()))):
        mine = faces[(ix == key[0]) & (iy == key[1])]
        used, local = np.unique(mine, return_inverse=True)
        corner = np.array([grid_origin[0] + key[0] * size, grid_origin[1] + key[1] * size])
        verts = vertices[used].copy()
        verts[:, :2] = world[used] - corner
        chunks[key] = (verts, normals[used], uvs[used], local.reshape(-1, 3))
    return chunks


def chunk_name(ix, iy):
    return f'chunk_{ix}_{iy}'


def write_collada(path, verts, normals, uvs, faces, texture, name):
    """Compact single-texture COLLADA in the layout of generate_mars_world.write_collada
    (one index offset per input: Gazebo misreads triangles whose inputs share one)."""
    def floats(a):
        return ' '.join(f'{x:.5f}' for x in np.asarray(a).ravel())
    n = len(verts)
    index = ' '.join(f'{i} {i} {i}' for i in np.asarray(faces).ravel())
    Path(path).write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n'
        '<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">'
        '<asset><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>'
        f'<library_images><image id="tex"><init_from>{texture}</init_from></image></library_images>'
        '<library_effects><effect id="fx"><profile_COMMON>'
        '<newparam sid="surf"><surface type="2D"><init_from>tex</init_from></surface></newparam>'
        '<newparam sid="samp"><sampler2D><source>surf</source></sampler2D></newparam>'
        '<technique sid="common"><lambert><ambient><color>0.7 0.7 0.7 1</color></ambient>'
        '<diffuse><texture texture="samp" texcoord="uv"/></diffuse></lambert></technique>'
        '</profile_COMMON></effect></library_effects>'
        '<library_materials><material id="mat"><instance_effect url="#fx"/></material></library_materials>'
        f'<library_geometries><geometry id="{name}"><mesh>'
        f'<source id="pos"><float_array id="pos-a" count="{3 * n}">{floats(verts)}</float_array>'
        f'<technique_common><accessor source="#pos-a" count="{n}" stride="3"><param name="X" type="float"/>'
        '<param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>'
        f'<source id="nrm"><float_array id="nrm-a" count="{3 * n}">{floats(normals)}</float_array>'
        f'<technique_common><accessor source="#nrm-a" count="{n}" stride="3"><param name="X" type="float"/>'
        '<param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>'
        f'<source id="uv"><float_array id="uv-a" count="{2 * n}">{floats(uvs)}</float_array>'
        f'<technique_common><accessor source="#uv-a" count="{n}" stride="2"><param name="S" type="float"/>'
        '<param name="T" type="float"/></accessor></technique_common></source>'
        '<vertices id="vtx"><input semantic="POSITION" source="#pos"/></vertices>'
        f'<triangles material="m" count="{len(faces)}">'
        '<input semantic="VERTEX" source="#vtx" offset="0"/><input semantic="NORMAL" source="#nrm" offset="1"/>'
        f'<input semantic="TEXCOORD" source="#uv" offset="2" set="0"/><p>{index}</p></triangles>'
        f'</mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene">'
        f'<node id="{name}-node"><instance_geometry url="#{name}"><bind_material><technique_common>'
        '<instance_material symbol="m" target="#mat"><bind_vertex_input semantic="uv" '
        'input_semantic="TEXCOORD" input_set="0"/></instance_material></technique_common></bind_material>'
        '</instance_geometry></node></visual_scene></library_visual_scenes>'
        '<scene><instance_visual_scene url="#scene"/></scene></COLLADA>\n')


BUILD_VERSION = 1   # bump when the chunk format changes


def default_cache():
    base = os.environ.get('XDG_CACHE_HOME') or os.path.join(os.path.expanduser('~'), '.cache')
    return Path(base) / 'rover_sim' / 'usgs_chunks'


def build_chunks(terrain_manifest, model_dir, cache=None, size=CHUNK_SIZE):
    """Chunk meshes and manifest for the tiles of terrain_manifest; returns the
    chunk manifest path. Reuses the cache when tiles, size and format match."""
    terrain_manifest = Path(terrain_manifest)
    stamp = hashlib.sha256(terrain_manifest.read_bytes() + f'{size}:{BUILD_VERSION}'.encode()).hexdigest()[:16]
    out = Path(cache or default_cache()) / stamp
    manifest = out / 'chunks.json'
    if manifest.is_file():
        return manifest
    tiles = json.loads(terrain_manifest.read_text())
    xmin, ymin = tiles['local_bounds_m'][:2]
    tmp = out.with_name(out.name + f'.tmp{os.getpid()}')
    shutil.rmtree(tmp, ignore_errors=True)
    tmp.mkdir(parents=True)
    shutil.copy(Path(model_dir) / 'meshes' / 'ground.png', tmp / 'ground.png')
    pieces = {}
    for tile in tiles['tiles']:
        mesh = read_collada(Path(model_dir) / 'meshes' / tile['mesh'])
        for key, part in split_tile(*mesh, tile['origin'], (xmin, ymin), size).items():
            pieces.setdefault(key, []).append(part)
    chunks, triangles = [], 0
    for key in sorted(pieces):
        parts, base, faces = pieces[key], 0, []
        for v, _, _, f in parts:
            faces.append(f + base)
            base += len(v)
        faces = np.concatenate(faces)
        name = chunk_name(*key)
        write_collada(tmp / f'{name}.dae', *(np.concatenate([p[i] for p in parts]) for i in range(3)),
                      faces, 'ground.png', name)
        chunks.append({'ix': key[0], 'iy': key[1], 'mesh': f'{name}.dae', 'triangles': len(faces)})
        triangles += len(faces)
    source = sum(t['triangles'] for t in tiles['tiles'])
    if triangles != source:
        raise RuntimeError(f'Chunking changed the triangle count ({source} -> {triangles})')
    (tmp / 'chunks.json').write_text(json.dumps(
        {'source': str(terrain_manifest), 'chunk_size': size, 'origin': [xmin, ymin],
         'triangles': triangles, 'chunks': chunks}) + '\n')
    shutil.rmtree(out, ignore_errors=True)
    tmp.rename(out)
    return manifest


# ---- which chunks -------------------------------------------------------

class ChunkIndex:
    """The chunk manifest: grid origin, size and the chunks that hold ground."""

    def __init__(self, manifest):
        data = json.loads(Path(manifest).read_text())
        self.size = float(data['chunk_size'])
        self.origin = tuple(data['origin'])
        self.chunks = {(c['ix'], c['iy']): c for c in data['chunks']}
        self.mesh_dir = Path(manifest).resolve().parent

    def index(self, x, y):
        return chunk_index(x, y, self.origin, self.size)

    def within(self, centre, radius):
        """Existing chunks whose Chebyshev distance from centre is <= radius."""
        cx, cy = centre
        return {(ix, iy) for ix in range(cx - radius, cx + radius + 1)
                for iy in range(cy - radius, cy + radius + 1) if (ix, iy) in self.chunks}

    def corner(self, key):
        return (self.origin[0] + key[0] * self.size, self.origin[1] + key[1] * self.size)

    def model_sdf(self, key, pose=True):
        """Static model SDF of one chunk; pose=False leaves the pose to the spawn request."""
        c = self.chunks[key]
        uri = (self.mesh_dir / c['mesh']).as_uri()
        geometry = f'<geometry><mesh><uri>{uri}</uri></mesh></geometry>'
        x, y = self.corner(key)
        pose_xml = f'<pose>{x:.5f} {y:.5f} 0 0 0 0</pose>' if pose else ''
        return (f'<model name="{chunk_name(*key)}"><static>true</static>{pose_xml}'
                f'<link name="ground"><collision name="collision">{geometry}<surface><friction><ode>'
                f'<mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>'
                f'<visual name="visual">{geometry}</visual></link></model>')


def chunk_world(template, index, x, y, radius):
    """World text from the chunk-free template with the chunks around x, y."""
    text = Path(template).read_text()
    if TEMPLATE_MARKER not in text:
        raise ValueError(f'{template} has no {TEMPLATE_MARKER} marker')
    keys = sorted(index.within(index.index(x, y), radius))
    models = '\n'.join(index.model_sdf(k) for k in keys)
    return text.replace(TEMPLATE_MARKER, models), keys


# ---- streaming node -----------------------------------------------------

def plan(loaded, centre, index, radius, margin):
    """(to_add, to_remove) for the rover in chunk `centre`."""
    wanted = index.within(centre, radius)
    cx, cy = centre
    far = {k for k in loaded if max(abs(k[0] - cx), abs(k[1] - cy)) > radius + margin}
    return sorted(wanted - loaded, key=lambda k: max(abs(k[0] - cx), abs(k[1] - cy))), sorted(far)


def main(args=None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from ros_gz_interfaces.srv import DeleteEntity, SpawnEntity

    from rover_sim import sim_services

    class ChunkStreamer(Node):
        def __init__(self):
            super().__init__('terrain_chunks')
            self.declare_parameter('manifest', '')
            self.declare_parameter('radius', 8)
            self.declare_parameter('margin', 2)
            self.declare_parameter('spawn_x', 0.0)
            self.declare_parameter('spawn_y', 0.0)
            self.declare_parameter('max_in_flight', 8)
            get = lambda n: self.get_parameter(n).value  # noqa: E731
            self.index = ChunkIndex(get('manifest'))
            self.radius, self.margin = int(get('radius')), int(get('margin'))
            self.max_in_flight = int(get('max_in_flight'))
            # The launch put these in the world file.
            self.loaded = self.index.within(self.index.index(get('spawn_x'), get('spawn_y')), self.radius)
            self.pending = {}
            self.centre = None
            self.spawn = self.create_client(SpawnEntity, sim_services.SPAWN)
            self.delete = self.create_client(DeleteEntity, sim_services.DELETE)
            self.create_subscription(Odometry, '/ground_truth/odom', self.on_odom, 10)
            self.create_timer(0.5, self.step)
            self.added = self.removed = 0
            self.create_timer(5.0, self.report)
            self.get_logger().info(
                f'{len(self.loaded)} of {len(self.index.chunks)} {self.index.size:g} m chunks loaded; '
                f'render distance {self.radius} chunks, unload beyond {self.radius + self.margin}')

        def on_odom(self, msg):
            p = msg.pose.pose.position
            self.centre = self.index.index(p.x, p.y)

        def step(self):
            if self.centre is None or not self.spawn.service_is_ready() or not self.delete.service_is_ready():
                return
            add, remove = plan(self.loaded, self.centre, self.index, self.radius, self.margin)
            for key in remove:
                if len(self.pending) >= self.max_in_flight:
                    return
                if key in self.pending:
                    continue
                self.pending[key] = 'remove'
                future = self.delete.call_async(sim_services.delete_request(chunk_name(*key)))
                future.add_done_callback(lambda f, k=key: self.done(f, k, False))
            for key in add:
                if len(self.pending) >= self.max_in_flight:
                    return
                if key in self.pending:
                    continue
                self.pending[key] = 'add'
                # Gazebo's create needs the <sdf> root, and replies success
                # before parsing: a bad string only shows in Gazebo's log.
                sdf = f'<sdf version="1.8">{self.index.model_sdf(key, pose=False)}</sdf>'
                request = sim_services.spawn_request(chunk_name(*key), sdf,
                                                     (*self.index.corner(key), 0.0))
                future = self.spawn.call_async(request)
                future.add_done_callback(lambda f, k=key: self.done(f, k, True))

        def done(self, future, key, added):
            self.pending.pop(key, None)
            ok = future.exception() is None and sim_services.succeeded(future.result())
            if not ok:
                self.get_logger().warn(f"{'Spawning' if added else 'Removing'} {chunk_name(*key)} failed")
                return
            (self.loaded.add if added else self.loaded.discard)(key)
            if added:
                self.added += 1
            else:
                self.removed += 1

        def report(self):
            if self.added or self.removed:
                self.get_logger().info(f'Chunks +{self.added} -{self.removed}: {len(self.loaded)} loaded '
                                       f'around chunk {self.centre}')
                self.added = self.removed = 0

    rclpy.init(args=args)
    node = ChunkStreamer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
