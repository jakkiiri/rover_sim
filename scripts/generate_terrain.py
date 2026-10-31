#!/usr/bin/env python3
"""Generate every Mars terrain asset deterministically from a seed.

    generate_terrain.py --output-dir DIR [--seed 7] [--resolution 513]
                        [--size 120] [--relief 15] [--rock-counts 300,700,1500]

Outputs ``models/mars_terrain`` (DEM, textures, rock meshes), ``worlds/``
(``mars_low|medium|high.world``, ``step_test``, ``slope_test``,
``boulder_garden``) and ``terrain_manifest.json`` with every parameter,
statistic and file hash.
"""
import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rover_sim.terrain.pipeline import DEFAULT_ROCK_COUNTS, DEFAULT_SEED, generate_all  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=DEFAULT_SEED)
    parser.add_argument('--resolution', type=int, default=513, help='2^n + 1 pixels (513 or 1025)')
    parser.add_argument('--size', type=float, default=120.0, help='terrain side in metres')
    parser.add_argument('--relief', type=float, default=15.0, help='vertical relief in metres')
    parser.add_argument('--sampling', type=int, default=2, help='Gazebo heightmap collision subsampling')
    parser.add_argument('--rock-counts', default=','.join(str(v) for v in DEFAULT_ROCK_COUNTS.values()),
                        help='requested rocks for low,medium,high')
    parser.add_argument('--texture-size', type=int, default=1024)
    parser.add_argument('--terrain-visual', choices=['mesh', 'heightmap'], default='mesh',
                        help='mesh: textured OBJ (default, renders on every driver); heightmap: Gazebo heightmap material')
    parser.add_argument('--mesh-step', type=int, default=2, help='DEM decimation for the visual mesh')
    parser.add_argument('--albedo-size', type=int, default=2048)
    parser.add_argument('--quiet', action='store_true')
    args = parser.parse_args()
    counts = dict(zip(('low', 'medium', 'high'), (int(v) for v in args.rock_counts.split(','))))
    generate_all(args.output_dir, seed=args.seed, resolution=args.resolution, size=args.size,
                 relief=args.relief, rock_counts=counts, texture_size=args.texture_size,
                 sampling=args.sampling, log=(lambda *a: None) if args.quiet else print,
                 terrain_visual=args.terrain_visual, mesh_step=args.mesh_step, albedo_size=args.albedo_size)


if __name__ == '__main__':
    main()
