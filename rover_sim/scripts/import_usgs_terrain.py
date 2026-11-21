#!/usr/bin/env python3
"""Convert classified LAZ tiles into a portable Gazebo Classic ground world.

Use requirements-terrain.txt in an isolated environment. Input files are read
only; outputs stay in this package. Metres are preserved, without flattening.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
import sys

import laspy
import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from rover_sim.survey_terrain import GroundGrid, grid_height, grid_mesh
from generate_mars_world import write_collada


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest() if hasattr(hashlib, 'file_digest') else _digest(stream)


def _digest(stream):
    result = hashlib.sha256()
    for block in iter(lambda: stream.read(1024*1024), b''):
        result.update(block)
    return result.hexdigest()


def read_tiles(paths, grid, records=None):
    for index, path in enumerate(paths):
        classes = np.zeros(256, np.int64)
        used = withheld_count = 0
        with laspy.open(path) as stream:
            for points in stream.chunk_iterator(500000):
                classification = np.asarray(points.classification)
                classes += np.bincount(classification, minlength=256)
                withheld = np.asarray(points.withheld, dtype=bool)
                ground = (classification == 2) & ~withheld
                used += int(ground.sum())
                withheld_count += int(withheld.sum())
                x, y, z = (np.asarray(getattr(points, key)) for key in ('x', 'y', 'z'))
                grid.add(x[~withheld], y[~withheld], z[~withheld], ground[~withheld])
        if records is not None:
            records[index].update(ground_points=used, withheld_points=withheld_count,
                                  classifications={str(i): int(n) for i, n in enumerate(classes) if n})
        print(f'Read {path.name}: {int(classes.sum()):,} points, {used:,} ground', flush=True)


def select_spawn(heights, valid, spacing, tile_cells):
    nearest = distance_transform_edt(~valid, return_distances=False, return_indices=True)
    filled = heights[tuple(nearest)]
    gy, gx = np.gradient(filled, spacing)
    slope = np.degrees(np.arctan(np.hypot(gx, gy)))
    edge = distance_transform_edt(np.pad(valid, 1))[1:-1, 1:-1]*spacing
    rough = maximum_filter(slope, size=5, mode='nearest')
    choices = []
    for row in range(tile_cells//2, heights.shape[0]-tile_cells//2, tile_cells):
        for col in range(tile_cells//2, heights.shape[1]-tile_cells//2, tile_cells):
            if valid[row, col] and edge[row, col] >= 24 and rough[row, col] <= 8:
                center = math.hypot((col/heights.shape[1]-.5), (row/heights.shape[0]-.5))
                choices.append((rough[row, col]+center, row, col))
    if not choices:
        raise ValueError('No surveyed low-slope spawn patch found; choose a smaller grid spacing')
    _, row, col = min(choices)
    return row, col, float(slope[row, col]), float(rough[row, col])


def tile_link(name, mesh, x, y):
    uri = f'model://usgs_terrain/meshes/{mesh}'
    geometry = f'<geometry><mesh><uri>{uri}</uri></mesh></geometry>'
    return f'''<link name="{name}"><pose>{x:.5f} {y:.5f} 0 0 0 0</pose>
      <collision name="ground">{geometry}<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>
      <visual name="ground">{geometry}</visual></link>'''


def world_xml(tiles):
    models = '\n'.join(f'<model name="{t["name"]}"><static>true</static>' +
                        tile_link('ground', t['mesh'], *t['origin'])+'</model>' for t in tiles)
    return f'''<?xml version="1.0"?>
<!-- GENERATED from the six user-provided USGS LAZ tiles. See usgs.terrain.json. -->
<sdf version="1.6"><world name="usgs">
  <gravity>0 0 -9.81</gravity>
  <physics name="default_physics" type="ode"><max_step_size>0.001</max_step_size>
    <real_time_update_rate>1000</real_time_update_rate>
    <ode><solver><type>quick</type><iters>150</iters><sor>1.0</sor></solver></ode>
  </physics>
  <scene><ambient>0.65 0.62 0.57 1</ambient><background>0.65 0.78 0.89 1</background>
    <shadows>false</shadows><grid>false</grid></scene>
  <light name="sun" type="directional"><cast_shadows>false</cast_shadows>
    <pose>0 0 50 0 0 0</pose><diffuse>0.95 0.92 0.85 1</diffuse>
    <specular>0.05 0.05 0.05 1</specular><direction>-0.4 0.2 -0.85</direction></light>
  <gui fullscreen="0"><camera name="overview"><pose>-15 -18 14 0 0.45 0.85</pose>
    <view_controller>orbit</view_controller></camera></gui>
  {models}
</world></sdf>
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('inputs', type=Path, nargs='+')
    parser.add_argument('--spacing', type=float, default=4., help='Background grid spacing, metres')
    parser.add_argument('--tile-size', type=float, default=128., help='Mesh tile width, metres')
    parser.add_argument('--local-spacing', type=float, default=1., help='Spawn tile grid spacing, metres')
    args = parser.parse_args()
    for key in ('spacing', 'tile_size', 'local_spacing'):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) <= 0:
            parser.error(key+' must be positive and finite')
    tile_cells = round(args.tile_size/args.spacing)
    if tile_cells < 2 or not math.isclose(tile_cells*args.spacing, args.tile_size):
        parser.error('tile-size must be a multiple of spacing and at least two cells')
    if args.local_spacing > args.spacing or not math.isclose(round(args.spacing/args.local_spacing)*args.local_spacing, args.spacing):
        parser.error('local-spacing must divide spacing')
    paths = sorted(p.resolve() for p in args.inputs)
    if len(set(paths)) != len(paths):
        parser.error('Duplicate input paths')
    records, reference_crs = [], None
    for path in paths:
        with laspy.open(path) as stream:
            h = stream.header
            crs = h.parse_crs()
            if crs is None or not crs.is_projected or len(crs.axis_info) < 3:
                raise ValueError('A projected CRS with vertical datum is required: '+path.name)
            if any(not math.isclose(a.unit_conversion_factor, 1.) for a in crs.axis_info[:3]):
                raise ValueError('All three axes must be metres: '+path.name)
            if reference_crs is not None and not crs.equals(reference_crs):
                raise ValueError('Input CRS mismatch: '+path.name)
            reference_crs = crs
            records.append(dict(filename=path.name, sha256=digest(path), point_count=int(h.point_count),
                                min=h.mins.tolist(), max=h.maxs.tolist()))
    xmin, ymin = (math.floor(min(r['min'][k] for r in records)/args.spacing)*args.spacing for k in (0, 1))
    xmax, ymax = (math.ceil(max(r['max'][k] for r in records)/args.spacing)*args.spacing for k in (0, 1))
    shape = (round((ymax-ymin)/args.spacing)+1, round((xmax-xmin)/args.spacing)+1)
    coarse = GroundGrid(xmin, ymin, args.spacing, shape)
    read_tiles(paths, coarse, records)
    heights, valid, coarse_stats = coarse.finish()
    row, col, slope, patch_slope = select_spawn(heights, valid, args.spacing, tile_cells)
    spawn_x, spawn_y = xmin+col*args.spacing, ymin+row*args.spacing
    ty, tx = row//tile_cells, col//tile_cells
    patch_x, patch_y = xmin+tx*args.tile_size, ymin+ty*args.tile_size
    nlocal = round(args.tile_size/args.local_spacing)+1
    fine = GroundGrid(patch_x, patch_y, args.local_spacing, (nlocal, nlocal))
    needed = [p for p, r in zip(paths, records) if r['max'][0] >= patch_x and r['min'][0] <= patch_x+args.tile_size
              and r['max'][1] >= patch_y and r['min'][1] <= patch_y+args.tile_size]
    read_tiles(needed, fine)
    detailed, detailed_valid, fine_stats = fine.finish()
    # Stitch the fine tile onto the coarse boundary over one coarse cell.
    for j in range(nlocal):
        for i in range(nlocal):
            distance = min(i, j, nlocal-1-i, nlocal-1-j)*args.local_spacing
            if distance >= args.spacing:
                continue
            try:
                border = grid_height(heights, (xmin, ymin), args.spacing,
                                     patch_x+i*args.local_spacing, patch_y+j*args.local_spacing)
                if detailed_valid[j, i]:
                    blend = distance/args.spacing
                    detailed[j, i] = blend*detailed[j, i]+(1-blend)*border
            except ValueError:
                detailed[j, i] = np.nan
    zorigin = grid_height(detailed, (patch_x, patch_y), args.local_spacing, spawn_x, spawn_y)
    heights -= zorigin
    detailed -= zorigin
    asset = PACKAGE/'models/usgs_terrain'
    mesh_dir = asset/'meshes'
    mesh_dir.mkdir(parents=True, exist_ok=True)
    for path in mesh_dir.glob('terrain_*.dae'):
        path.unlink()
    shutil.copyfile(PACKAGE/'models/mars_terrain/meshes/regolith.png', mesh_dir/'ground.png')
    np.savez_compressed(asset/'coarse.npz', heights=heights.astype(np.float32))
    np.savez_compressed(asset/'spawn.npz', heights=detailed.astype(np.float32))
    tiles = []

    def export(name, h, spacing, x, y):
        vertices, faces, uvs = grid_mesh(h, spacing)
        if not len(faces):
            return
        mesh = name+'.dae'
        write_collada(mesh_dir/mesh, vertices, faces, uvs, 'ground.png', name)
        tiles.append(dict(name=name, mesh=mesh, origin=[x-spawn_x, y-spawn_y],
                          vertices=len(vertices), triangles=len(faces), sha256=digest(mesh_dir/mesh)))

    for j in range(0, shape[0]-1, tile_cells):
        for i in range(0, shape[1]-1, tile_cells):
            if (j//tile_cells, i//tile_cells) == (ty, tx):
                continue
            h = heights[j:min(j+tile_cells+1, shape[0]), i:min(i+tile_cells+1, shape[1])]
            export(f'terrain_{j//tile_cells:02d}_{i//tile_cells:02d}', h, args.spacing,
                   xmin+i*args.spacing, ymin+j*args.spacing)
    export('terrain_spawn', detailed, args.local_spacing, patch_x, patch_y)
    (asset/'model.config').write_text('<?xml version="1.0"?><model><name>USGS survey terrain</name>'
                                    '<version>1.0</version><sdf version="1.6">model.sdf</sdf>'
                                    '<description>Ground-only LAZ-derived meshes, metres, local origin.</description></model>\n')
    links = '\n'.join(tile_link(t['name'], t['mesh'], *t['origin']) for t in tiles)
    (asset/'model.sdf').write_text('<sdf version="1.6"><model name="usgs_terrain"><static>true</static>'+links+'</model></sdf>\n')
    world = PACKAGE/'worlds/usgs.world'
    world.write_text(world_xml(tiles))
    metadata = dict(schema=1, source='Six user-provided USGS LPC Southern Utah 2018 tiles (LAS 2019)',
                    crs_wkt=reference_crs.to_wkt(), horizontal_epsg=6341, vertical_datum='NAVD88 / Geoid12B, metres',
                    origin_projected_m=[spawn_x, spawn_y, zorigin],
                    axes={'x': 'easting', 'y': 'northing', 'z': 'up'}, inputs=records,
                    source_bounds_m=[xmin, ymin, xmax, ymax],
                    local_bounds_m=[xmin-spawn_x, ymin-spawn_y, xmax-spawn_x, ymax-spawn_y],
                    source_ground_elevation_m=[float(np.nanmin(heights)+zorigin), float(np.nanmax(heights)+zorigin)],
                    spawn={'x': 0., 'y': 0., 'ground_z': 0., 'coarse_slope_deg': slope,
                           'maximum_nearby_coarse_slope_deg': patch_slope},
                    grids=[dict(path='../models/usgs_terrain/coarse.npz', spacing=args.spacing,
                                origin=[xmin-spawn_x, ymin-spawn_y], shape=list(shape), **coarse_stats),
                           dict(path='../models/usgs_terrain/spawn.npz', spacing=args.local_spacing,
                                origin=[patch_x-spawn_x, patch_y-spawn_y], shape=list(detailed.shape), **fine_stats)],
                    tiles=tiles, triangles=sum(t['triangles'] for t in tiles),
                    processing={'ground_class': 2, 'withheld_excluded': True, 'maximum_hole_fill_m': 8.,
                                'method': 'Local least-squares planes at grid nodes; nearest small-gap fill; tiled triangles',
                                'local_boundary_blend_m': args.spacing, 'vertical_exaggeration': 1.,
                                'texture': 'Synthetic regolith reused from this package, not aerial imagery',
                                'non_ground_objects': 'Not reconstructed; this is bare-earth terrain'})
    world.with_suffix('.terrain.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: metadata[k] for k in ('origin_projected_m', 'source_ground_elevation_m', 'triangles', 'spawn')}, indent=2))
    print(f'Wrote {world} with {len(tiles)} terrain tiles', flush=True)


if __name__ == '__main__':
    main()
