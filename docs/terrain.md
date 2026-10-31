# Mars terrain

Everything under `generated/` (heightmap DEM, textures, rock meshes, worlds,
`terrain_manifest.json`) is produced deterministically from an integer seed by
`scripts/generate_terrain.py`, which `colcon build` runs for the default seed
(7). No binary asset is committed.

```bash
# regenerate by hand (about 7 s)
ros2 run rover_sim generate_terrain.py --output-dir /tmp/mars --seed 42 --resolution 513
# use a different seed from the launch (generated once into ~/.cache/rover_sim)
ros2 launch rover_sim rover_mars.launch.py world:=mars seed:=42 rock_density:=high
```

## Empirical Gazebo Classic 11.10.2 findings (2026-10-17)

Measured with resting probes on a 129 px analytic heightmap
(`docs/validation/mars_terrain.md` reproduces the probe method):

| Format | Result |
| --- | --- |
| 16-bit grayscale PNG | Misread: the terrain collapsed to ~0.1 m of relief. Not usable. |
| 8-bit grayscale PNG with `<size>` | Correct but quantised to `size_z / 255` (7.8 cm steps for 20 m relief). |
| float32 GeoTIFF DEM with `<size>` | Heights rescaled by an unexpected factor (~1.6). |
| **float32 GeoTIFF DEM without `<size>`** | **Heights within ±2 cm of the float data; extent from the georeference.** |

Other conventions verified: image row 0 is +Y (north) and column 0 is -X;
Gazebo's DEM extent uses a 6371 km spherical Earth, so the GeoTIFF pixel
scale is written as `size / (2π·6371000/360) / (n-1)` degrees. Collision
subsampling (`<sampling>`) defaults to 2, i.e. a 513 px image becomes a
1025-vertex ODE heightfield. Spheres resting exactly on the x = 0 column of a
heightfield sank 0.22 m while boxes did not; wheel collision shapes are
therefore tested across that seam in Phase B.

The DEM is written with PIL plus raw GeoTIFF tags (ModelPixelScale,
ModelTiepoint, GeoKeyDirectory with EPSG:4326). No GDAL Python binding is
needed; Gazebo's own GDAL build reads it.

## Heightmap layers (`rover_sim/terrain/heightmap.py`)

| # | Layer | Default parameters | Purpose |
| --- | --- | --- | --- |
| 1 | Ridged multifractal hills | wavelength 120 m, 4 octaves, gain 0.40, exponent 1.8, domain warp 0.25 | ridges and hills with broad valleys |
| 2 | fBm undulation | 10 % of relief, wavelength 30 m, 5 octaves | rolling mid-scale ground |
| 3 | Micro relief | 1.2 cm at 1.2 m, 2 octaves | rocky floor, never perfectly smooth |
| 4 | Impact craters | 7, radii 3–14 m, N(>R) ∝ R⁻², depth 0.12 R, raised rim + ejecta falloff | craters |
| 5 | Domain warping | 0.25 lattice units | breaks grid alignment of layer 1 |
| 6 | Wind ripples | 6 cm, 1.6 m wavelength, low areas only | dunes/ripples |
| 7 | Spawn pad + garden | pad r = 8 m, blend 12 m; garden x ∈ [6, 17] m, |y| ≤ 9.5 m, blend 8 m | flat, rock-free start area and boulder test strip |

fBm is `h(x) = Σ aᵏ n(fᵏ x)` with `a = 0.5`, `f = 2`. The ridged layer is
Musgrave's `(offset − |n|)²` with octave weighting. The sum is rescaled so the
pre-pad relief equals `relief` (15 m) and offset so the minimum is 0.

`slope_map.png`, `roughness.png` and `heightmap_preview.png` are written next
to the DEM; the slope statistics are stored in the manifest. For seed 7 at
513 px:

| Statistic | Value |
| --- | --- |
| relief | 15.0 m |
| mean / median slope | 15.9° / 13.7° |
| terrain below 20° | 70 % |
| terrain above 15° / 25° / 35° | 46 % / 19 % / 6.3 % |
| maximum slope | 61° |
| spawn pad maximum slope | 0.47° (flat; blend ring excluded) |

(Values are re-read from `terrain_manifest.json` by `validate_terrain.py`;
the generator is deterministic so these numbers are reproducible.)

## Mars look

**Visual mesh (default since 2026-10-31).** Gazebo Classic's heightmap
material shader does not compile on this machine's Mesa driver (the Ogre
log fills with `clamp(error, float, float)` type-mismatch errors) and the
terrain renders black regardless of shadows, normal maps or blends (measured
with a headless camera: mean RGB 13/8/6). The generator therefore also writes
`meshes/terrain_visual.obj` (the DEM decimated 2:1 → 257² vertices, 131 072
triangles, smooth normals, UVs) with a 2048² albedo baked over the whole
terrain by **slope and height** (sand in low flat ground, gravel on flanks,
basalt on steep and high ground, plus detail noise and slope shading), and
the world uses that mesh as the visual while the collision stays on the exact
heightfield. Rendered mean RGB with the mesh: 135/76/40. The heightmap visual
remains available with `generate_terrain.py --terrain-visual heightmap`
(tileable sand/gravel/basalt albedo + normal maps blended by height only, the
only blending Gazebo's heightmap material offers).

Lighting: one directional sun at 37° elevation, butterscotch ambient
(0.55/0.46/0.40) and background, linear dust haze from 40 m to 260 m, no
sky/clouds. Shadows are off by default (`shadows:=true` to enable).

Gravity is **3.71 m/s²** in every generated world (`gravity:=9.81` to run
Earth tests). This changes rover weight and therefore normal forces, traction
limits (`F = μN`), suspension static deflection and the torque needed on
slopes; see `docs/mechanics.md`.

## Rocks and boulders (`rover_sim/terrain/rocks.py`, `scatter.py`)

Fourteen rock meshes (rounded, flattened, angular, vesicular, elongated,
blocky) are displaced icospheres (320 faces) normalised to 1 m horizontal
extent; each has a separate convex-hull collision mesh of ≤ 56 faces built
from 30 extremal vertices. Instances scale the meshes to the sampled diameter.

Sizes follow the Mars rock-abundance power law `N(>D) = N₀ D⁻ᵇ` with
`b = 2.1`, `D ∈ [0.08, 1.2] m`, after Golombek & Rapp (1997, *JGR* 102,
4117–4129) and Golombek et al. (2003, *JGR* 108, 8086). The count budget is
set by measurement (below); the resulting rock area coverage is far below
real landing-site values (5–20 %), which is the price of keeping the physics
cheap.

Placement: a density map (1 + 3·crater-rim band + 2·hill-base term, zero on
the pad/garden and on slopes > 28°) is sampled, rocks are dropped onto the
sampled DEM height, buried 10–35 %, yawed randomly and tilted ±10°. Rocks are
merged into one static `<model>` per 20 × 20 m tile (≤ 36 models). Collision
policy by diameter: `< 0.10 m` visual only, `0.10–0.30 m` analytic sphere,
`≥ 0.30 m` convex hull mesh.

Labeled test boulders (`boulder_10cm` … `boulder_40cm`, exposed heights
0.10/0.15/0.20/0.25/0.30/0.40 m, 1.6 m long across the lane) sit in the
flat garden strip at x = 11.5 m, lanes y = −7.5 … +7.5 m (3 m spacing), with
a colour-coded post of the same height beside each.

## Validation worlds

| World | Content |
| --- | --- |
| `step_test` | rigid steps 5/10/15/20/25/30/40 cm, 4 m long, 3 m wide, lanes 4 m apart (y = −12 … +12), step starts at x = 3 m |
| `slope_test` | ramps 5–40° in 5° steps, 3 m flat run-up, 8 m horizontal run, 4 m plateau, lanes 4 m apart (y = −14 … +14) |
| `boulder_garden` | the labeled boulders on flat ground |
| `flat` | empty flat ground |

All worlds use μ = 0.65 ground, `max_step_size` 1 ms, ODE quick solver (60
iterations, SOR 1.3) and include `libgazebo_ros_state.so` for validation.

## Performance

Headless (`gui:=false`, LiDAR off), proxy rover resting on the pad, two 10 s windows each, 2026-10-17. gzserver CPU is in cores (fraction of one core x 1); RSS memory of gzserver.

| World / variant | median RTF | gzserver CPU | gzserver RSS |
| --- | --- | --- | --- |
| mars, low (295 rocks), 513 px, sampling 2 | 0.999 | 0.53 | 462 MB |
| mars, medium (691), 513 px, sampling 2 | 0.999 | 0.56 | 594 MB |
| mars, high (1480), 513 px, sampling 2 | 0.999 | 0.56 | 830 MB |
| mars, medium, 513 px, sampling 1 | 0.999 | 0.56 | 591 MB |
| mars, medium, 1025 px, sampling 2 | 0.999 | 0.55 | 614 MB |
| mars, medium, 1025 px, sampling 1 | 0.999 | 0.56 | 603 MB |
| flat | 0.999 | 0.53 | 285 MB |
| step_test | 0.999 | 0.53 | 299 MB |
| slope_test | 0.999 | 0.54 | 316 MB |
| boulder_garden | 0.999 | 0.53 | 298 MB |

At rest the heightmap resolution and subsampling make no measurable difference; the rock
count mainly costs memory (mesh visuals are also loaded by the server). The choice of
513 px with sampling 2 (1025-vertex collision field, 0.23 m pixels) is kept as the default;
1025 px is available via `resolution:=1025`. Driving and LiDAR costs are measured in the
later phases (`docs/mechanics.md`, `docs/lidar.md`, `docs/validation/`).

## Validation

See [validation/mars_terrain.md](validation/mars_terrain.md) (2026-10-17): determinism,
manifest envelope, world load, pad stability, height probes and RTF all passed.

## Reproducibility and limitations

`terrain_manifest.json` records the seed, parameters, slope statistics,
crater list, rock counts per density, boulder poses, rock-law parameters and
the SHA-256 of every file. `validate_terrain.py` regenerates twice and
compares the hashes.

Limitations: ODE terrain is rigid; sinkage, bulldozing and slip-sinkage of
Martian regolith are not modelled and are only approximated by contact
friction/stiffness. The baked albedo has ~6 cm texels (no close-up detail texture). Rock density is far
below real Mars for performance. Rocks smaller than 10 cm have no collision.
The heightmap resolution (0.23 m per pixel at 513 px) limits micro-relief to
wavelengths above about 0.5 m.
