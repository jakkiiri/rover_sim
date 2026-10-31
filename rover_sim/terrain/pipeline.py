"""One-command generation of every Mars terrain asset from a seed."""
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from rover_sim.terrain import heightmap as hm
from rover_sim.terrain import mesh_visual as mv
from rover_sim.terrain import rocks as rk
from rover_sim.terrain import scatter as sc
from rover_sim.terrain import textures as tx
from rover_sim.terrain import world as wd

GENERATOR_VERSION = 1
DEFAULT_SEED = 7
DEFAULT_ROCK_COUNTS = dict(low=300, medium=700, high=1500)
MODEL_NAME = 'mars_terrain'


def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for block in iter(lambda: handle.read(1 << 20), b''):
            digest.update(block)
    return digest.hexdigest()


def generate_all(output_dir, seed=DEFAULT_SEED, resolution=513, size=120.0, relief=15.0,
                 rock_counts=None, texture_size=1024, sampling=2, rock_library_size=14,
                 log=print, extra_params=None, terrain_visual='mesh', mesh_step=2, albedo_size=2048):
    """Generate heightmap, textures, rocks, worlds and manifest under ``output_dir``."""
    started = time.time()
    output_dir = Path(output_dir)
    model_dir = output_dir / 'models' / MODEL_NAME
    worlds_dir = output_dir / 'worlds'
    for directory in (model_dir, model_dir / 'textures', model_dir / 'meshes', worlds_dir):
        directory.mkdir(parents=True, exist_ok=True)
    rock_counts = dict(DEFAULT_ROCK_COUNTS if rock_counts is None else rock_counts)
    params = hm.TerrainParams(seed=seed, resolution=resolution, size=size, relief=relief,
                              **(extra_params or {}))

    log(f'[terrain] heightmap {resolution}x{resolution}, {size:.0f} m, seed {seed}')
    terrain = hm.generate_heightmap(params)
    height = terrain['height']
    hm.write_geotiff(model_dir / 'heightmap.tif', height, size)
    hm.write_preview_png(model_dir / 'heightmap_preview.png', height)
    hm.write_preview_png(model_dir / 'slope_map.png', terrain['slope_deg'], 0.0, 45.0)
    hm.write_preview_png(model_dir / 'roughness.png', terrain['roughness'], 0.0, 0.15)
    stats = hm.slope_statistics(terrain['slope_deg'], terrain['flat_weight'])
    stats['relief_m'] = float(np.ptp(height))
    stats['roughness_rms_m'] = float(np.sqrt(np.mean(terrain['roughness'] ** 2)))
    (model_dir / 'model.config').write_text(
        f'<?xml version="1.0"?>\n<model><name>{MODEL_NAME}</name><version>1.0</version>'
        '<sdf version="1.6">model.sdf</sdf><author><name>rover_sim generator</name></author>'
        '<description>Generated Mars terrain assets</description></model>\n')
    (model_dir / 'model.sdf').write_text('<?xml version="1.0"?>\n<sdf version="1.6"><model name="mars_terrain_assets">'
                                         '<static>true</static><link name="empty"/></model></sdf>\n')

    log(f'[terrain] textures {texture_size}px')
    tx.write_textures(model_dir / 'textures', seed, texture_size)

    log(f'[terrain] rock library ({rock_library_size} meshes)')
    library = rk.build_rock_library(model_dir / 'meshes', seed, rock_library_size)

    mesh_info = None
    if terrain_visual == 'mesh':
        log(f'[terrain] mesh visual (grid step {mesh_step}, {albedo_size}px slope-blended albedo)')
        vertices, faces = mv.write_terrain_mesh(model_dir / 'meshes' / 'terrain_visual.obj', height, size, mesh_step)
        weights = mv.bake_albedo(model_dir / 'meshes' / 'terrain_albedo.png', height, terrain['slope_deg'], seed, albedo_size)
        mesh_info = dict(vertices=vertices, faces=faces, albedo_px=albedo_size, material_fractions=weights)

    boulders = sc.labeled_boulders(terrain, library)
    boulders_sdf = sc.boulder_models(boulders, library, wd.MODEL_URI)
    low_blend = float(np.percentile(height, 35))
    high_blend = float(np.percentile(height, 80))
    rocks_summary = {}
    for density, requested in rock_counts.items():
        instances = sc.place_rocks(terrain, library, requested, seed)
        rocks_sdf = sc.chunk_models(instances, library, wd.MODEL_URI, size)
        text = wd.mars_world(f'mars_{density}', terrain['pad_level'], (low_blend, high_blend),
                             rocks_sdf, boulders_sdf, sampling=sampling, visual=terrain_visual)
        (worlds_dir / f'mars_{density}.world').write_text(text)
        diameters = np.array([i['diameter'] for i in instances])
        rocks_summary[density] = dict(
            requested=requested, placed=int(len(instances)),
            chunks=int(text.count('<model name="rocks_')),
            collision_none=int(np.sum(diameters < sc.COLLISION_NONE_BELOW)),
            collision_sphere=int(np.sum((diameters >= sc.COLLISION_NONE_BELOW) & (diameters < sc.COLLISION_HULL_FROM))),
            collision_hull=int(np.sum(diameters >= sc.COLLISION_HULL_FROM)),
            diameter_min=float(diameters.min()) if len(diameters) else None,
            diameter_max=float(diameters.max()) if len(diameters) else None,
            diameter_median=float(np.median(diameters)) if len(diameters) else None,
            rock_area_fraction=float(np.sum(np.pi * (diameters / 2) ** 2) / size ** 2))
        log(f'[terrain] {density}: {len(instances)} rocks in {rocks_summary[density]["chunks"]} chunks')
    # Flat boulder garden uses the same specs on z = 0 ground.
    flat_terrain = dict(terrain, height=np.zeros_like(height))
    flat_boulders = sc.labeled_boulders(flat_terrain, library)
    (worlds_dir / 'boulder_garden.world').write_text(
        wd.boulder_garden_world(sc.boulder_models(flat_boulders, library, wd.MODEL_URI)))
    (worlds_dir / 'step_test.world').write_text(wd.step_test_world())
    (worlds_dir / 'slope_test.world').write_text(wd.slope_test_world())

    files = sorted(p for p in output_dir.rglob('*') if p.is_file() and p.name != 'terrain_manifest.json')
    manifest = dict(
        generator_version=GENERATOR_VERSION, seed=seed, resolution=resolution, size=size, relief=relief,
        sampling=sampling, params=hm.params_to_json(params), pad=dict(x=0.0, y=0.0, z=terrain['pad_level'],
                                                                   radius=params.pad_radius),
        garden=dict(x=list(params.garden_x), half_width=params.garden_half_width),
        spawn=dict(x=0.0, y=0.0, z=terrain['pad_level'], yaw=0.0),
        slope_stats=stats, craters=terrain['craters'],
        rock_size_law=dict(form='N(>D) = N0 * D^-b', exponent=2.1, d_min=0.08, d_max=1.2,
                           reference='Golombek & Rapp 1997 (JGR 102, 4117); Golombek et al. 2003 (JGR 108, 8086)'),
        rocks=rocks_summary, boulders=boulders, texture_blend=[low_blend, high_blend],
        rock_library=library, model_uri=wd.MODEL_URI, terrain_visual=terrain_visual, mesh_visual=mesh_info,
        files={str(p.relative_to(output_dir)): sha256(p) for p in files},
        generation_seconds=time.time() - started)
    (output_dir / 'terrain_manifest.json').write_text(json.dumps(manifest, indent=1) + '\n')
    log(f'[terrain] done in {manifest["generation_seconds"]:.1f} s -> {output_dir}')
    return manifest


def load_manifest(output_dir):
    with open(Path(output_dir) / 'terrain_manifest.json') as handle:
        return json.load(handle)


def read_height_field(output_dir):
    """Load the generated DEM as a float array (for launch-time spawn height)."""
    from PIL import Image
    image = Image.open(Path(output_dir) / 'models' / MODEL_NAME / 'heightmap.tif')
    return np.array(image, dtype=np.float64)
