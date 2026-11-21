import json
from pathlib import Path

import numpy as np
import pytest

from rover_sim import terrain_chunks as tc

PACKAGE = Path(__file__).resolve().parents[1]


def grid_tile(n=9, spacing=4.0):
    """A (n-1)x(n-1)-cell grid mesh like the importer's, 32 m wide at 4 m."""
    xs = np.arange(n) * spacing
    x, y = np.meshgrid(xs, xs)
    verts = np.column_stack([x.ravel(), y.ravel(), (x + 2 * y).ravel() * 0.01])
    idx = np.arange(n * n).reshape(n, n)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel()
    faces = np.concatenate([np.column_stack([a, b, d]), np.column_stack([a, d, c])])
    normals = np.tile([0.0, 0.0, 1.0], (len(verts), 1))
    return verts, normals, verts[:, :2] / 10.0, faces


def test_split_keeps_every_triangle_exactly_once_and_in_its_chunk():
    verts, normals, uvs, faces = grid_tile()
    chunks = tc.split_tile(verts, normals, uvs, faces, tile_origin=(-32.0, 100.0), grid_origin=(-32.0, 100.0))
    assert set(chunks) == {(0, 0), (1, 0), (0, 1), (1, 1)}
    assert sum(len(c[3]) for c in chunks.values()) == len(faces)
    original = {tuple(sorted(map(tuple, (verts[f] + [-32.0, 100.0, 0.0]).round(6)))) for f in faces}
    rebuilt = set()
    for (ix, iy), (v, n, uv, f) in chunks.items():
        assert v[:, :2].min() >= -1e-9 and v[:, :2].max() <= tc.CHUNK_SIZE + 1e-9   # chunk-local
        corner = np.array([-32.0 + ix * 16, 100.0 + iy * 16, 0.0])
        rebuilt |= {tuple(sorted(map(tuple, (v[t] + corner).round(6)))) for t in f}
        assert np.allclose(n, [0, 0, 1])
    assert rebuilt == original


def test_collada_round_trip(tmp_path):
    verts, normals, uvs, faces = grid_tile(3)
    tc.write_collada(tmp_path / 'c.dae', verts, normals, uvs, faces, 'ground.png', 'c')
    v, n, uv, f = tc.read_collada(tmp_path / 'c.dae')
    assert np.allclose(v, verts) and np.allclose(n, normals) and np.allclose(uv, uvs)
    assert (f == faces).all()


def make_index(tmp_path, keys):
    manifest = tmp_path / 'chunks.json'
    manifest.write_text(json.dumps({'chunk_size': 16.0, 'origin': [0.0, 0.0], 'chunks': [
        {'ix': x, 'iy': y, 'mesh': f'chunk_{x}_{y}.dae', 'triangles': 2} for x, y in keys]}))
    return tc.ChunkIndex(manifest)


def test_render_distance_is_a_square_of_existing_chunks(tmp_path):
    index = make_index(tmp_path, [(x, y) for x in range(-20, 21) for y in range(-20, 21) if (x, y) != (1, 1)])
    near = index.within(index.index(5.0, 5.0), 8)
    assert len(near) == 17 * 17 - 1 and (8, -8) in near and (9, 0) not in near
    assert index.index(-0.1, 16.0) == (-1, 1)


def test_plan_adds_nearest_first_and_unloads_only_past_the_margin(tmp_path):
    index = make_index(tmp_path, [(x, y) for x in range(-30, 31) for y in range(-30, 31)])
    loaded = index.within((0, 0), 8)
    add, remove = tc.plan(loaded, (1, 0), index, 8, 2)
    assert len(add) == 17 and all(k[0] == 9 for k in add) and remove == []
    add, remove = tc.plan(loaded, (3, 0), index, 8, 2)
    assert {k[0] for k in remove} == {-8} and len(remove) == 17
    assert max(abs(k[0] - 3) for k in add[:1]) <= max(abs(k[0] - 3) for k in add[-1:])


def test_chunk_world_fills_the_template(tmp_path):
    index = make_index(tmp_path, [(0, 0), (1, 0), (5, 5)])
    template = tmp_path / 'w.world'
    template.write_text(f'<sdf><world name="usgs">{tc.TEMPLATE_MARKER}</world></sdf>')
    text, keys = tc.chunk_world(template, index, 1.0, 1.0, 1)
    assert keys == [(0, 0), (1, 0)] and text.count('<model') == 2 and tc.TEMPLATE_MARKER not in text
    assert (tmp_path / 'chunk_1_0.dae').as_uri() in text and '<pose>16.00000 0.00000 0' in text
    template.write_text('<sdf/>')
    with pytest.raises(ValueError):
        tc.chunk_world(template, index, 1.0, 1.0, 1)


def test_shipped_template_has_rocks_but_no_terrain_tiles():
    text = (PACKAGE / 'worlds/usgs_chunked.world').read_text()
    assert tc.TEMPLATE_MARKER in text and 'terrain_00_00' not in text and '<world name="usgs">' in text


def test_real_tiles_chunk_without_losing_triangles(tmp_path):
    manifest = tc.build_chunks(PACKAGE / 'worlds/usgs.terrain.json', PACKAGE / 'models/usgs_terrain', cache=tmp_path)
    data = json.loads(manifest.read_text())
    tiles = json.loads((PACKAGE / 'worlds/usgs.terrain.json').read_text())['tiles']
    assert data['triangles'] == sum(t['triangles'] for t in tiles)
    assert tc.build_chunks(PACKAGE / 'worlds/usgs.terrain.json', PACKAGE / 'models/usgs_terrain',
                           cache=tmp_path) == manifest   # cached


def test_streamed_chunk_sdf_parses_as_a_model():
    """Gazebo's create rejects a bare <model>: the streamer wraps it in <sdf>."""
    import inspect
    assert '<sdf version="1.8">' in inspect.getsource(tc.main)
