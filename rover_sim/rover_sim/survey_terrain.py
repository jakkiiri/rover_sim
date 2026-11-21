"""Survey gridding and exact height lookup on the exported triangle surfaces.

Runtime lookup needs only numpy. SciPy is imported only by conversion helpers.
Coordinates and spacing are metres; array row direction is increasing north.
"""
from functools import lru_cache
import json
import math
from pathlib import Path

import numpy as np


class GroundGrid:
    """Streaming least-squares ground plane in each nearest-node footprint.

    Fitting z=c+a*dx+b*dy evaluates the surface at the actual node, avoiding
    the half-cell horizontal shift produced by treating cell means as corners.
    """

    def __init__(self, xmin, ymin, spacing, shape):
        if not all(math.isfinite(v) for v in (xmin, ymin, spacing)) or spacing <= 0:
            raise ValueError('Grid origin must be finite and spacing positive')
        if len(shape) != 2 or min(shape) < 2:
            raise ValueError('Grid needs at least two rows and columns')
        self.xmin, self.ymin, self.spacing = xmin, ymin, spacing
        self.shape = tuple(shape)
        self.coverage = np.zeros(shape, np.uint32)
        self.moments = np.zeros((9, *shape), np.float64)

    def add(self, x, y, z, ground):
        x, y, z, ground = map(np.asarray, (x, y, z, ground))
        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        x, y, z, ground = x[finite], y[finite], z[finite], ground[finite]
        ix = np.floor((x-self.xmin)/self.spacing+.5).astype(np.int64)
        iy = np.floor((y-self.ymin)/self.spacing+.5).astype(np.int64)
        inside = (ix >= 0) & (iy >= 0) & (ix < self.shape[1]) & (iy < self.shape[0])
        x, y, z, ground, ix, iy = (a[inside] for a in (x, y, z, ground, ix, iy))
        flat = iy*self.shape[1]+ix
        size = self.coverage.size
        self.coverage += np.bincount(flat, minlength=size).reshape(self.shape).astype(np.uint32)
        flat, ix, iy = flat[ground], ix[ground], iy[ground]
        dx = x[ground]-(self.xmin+ix*self.spacing)
        dy = y[ground]-(self.ymin+iy*self.spacing)
        z = z[ground]
        for i, weight in enumerate((np.ones(len(z)), dx, dy, z, dx*dx, dx*dy, dy*dy, dx*z, dy*z)):
            self.moments[i] += np.bincount(flat, weights=weight, minlength=size).reshape(self.shape)

    def finish(self, maximum_hole_metres=8.):
        from scipy.ndimage import binary_fill_holes, distance_transform_edt
        n, sx, sy, sz, sxx, sxy, syy, sxz, syz = self.moments
        divisor = np.maximum(n, 1)
        xx, xy, yy = sxx-sx*sx/divisor, sxy-sx*sy/divisor, syy-sy*sy/divisor
        xz, yz = sxz-sx*sz/divisor, syz-sy*sz/divisor
        determinant = xx*yy-xy*xy
        # Sparse or nearly collinear returns cannot constrain a plane. Large
        # extrapolations from those fits create spikes absent from the survey.
        fit = ((n >= 6) & (determinant > 1e-10) & (determinant > .01*xx*yy)
               & (xx > n*(self.spacing*.025)**2) & (yy > n*(self.spacing*.025)**2))
        a, b = np.zeros(self.shape), np.zeros(self.shape)
        a[fit] = (xz[fit]*yy[fit]-yz[fit]*xy[fit])/determinant[fit]
        b[fit] = (yz[fit]*xx[fit]-xz[fit]*xy[fit])/determinant[fit]
        mean = sz/divisor
        heights = (sz-a*sx-b*sy)/divisor
        unstable = (np.hypot(a, b) > math.tan(math.radians(75))) | (np.abs(heights-mean) > self.spacing)
        heights[unstable] = mean[unstable]
        observed = n > 0
        if not observed.any():
            raise ValueError('No usable class-2 ground returns in this grid')
        distance, indices = distance_transform_edt(~observed, return_indices=True)
        footprint = binary_fill_holes(self.coverage > 0)
        valid = footprint & (distance*self.spacing <= maximum_hole_metres)
        filled = ~observed & valid
        heights[filled] = heights[tuple(indices[:, filled])]
        heights[~valid] = np.nan
        return heights, valid, {'ground_nodes': int(observed.sum()), 'filled_nodes': int(filled.sum()),
                                'unsupported_nodes': int((~valid).sum())}


def grid_height(heights, origin, spacing, x, y):
    """Height on the same two CCW triangles exported for every grid cell."""
    if not all(math.isfinite(v) for v in (x, y)):
        raise ValueError('Terrain coordinates must be finite')
    fx, fy = (x-origin[0])/spacing, (y-origin[1])/spacing
    rows, cols = heights.shape
    if not -1e-8 <= fx <= cols-1+1e-8 or not -1e-8 <= fy <= rows-1+1e-8:
        raise ValueError('Position is outside the surveyed terrain')
    ix, iy = min(max(int(math.floor(fx)), 0), cols-2), min(max(int(math.floor(fy)), 0), rows-2)
    u, v = min(max(fx-ix, 0.), 1.), min(max(fy-iy, 0.), 1.)
    if u+v <= 1:
        values = np.array([heights[iy, ix], heights[iy, ix+1], heights[iy+1, ix]])
        weights = np.array([1-u-v, u, v])
    else:
        values = np.array([heights[iy+1, ix+1], heights[iy+1, ix], heights[iy, ix+1]])
        weights = np.array([u+v-1, 1-u, 1-v])
    # A zero-weight corner is not required (important on survey boundaries).
    if not np.isfinite(values[weights > 1e-10]).all():
        raise ValueError('Position has no supported survey surface')
    return float(np.dot(np.nan_to_num(values), weights))


def grid_mesh(heights, spacing):
    """Finite triangles only; NaN footprints remain open rather than invented."""
    rows, cols = heights.shape
    yy, xx = np.mgrid[:rows, :cols]
    vertices = np.column_stack((xx.ravel()*spacing, yy.ravel()*spacing,
                                np.nan_to_num(heights).ravel()))
    a = (yy[:-1, :-1]*cols+xx[:-1, :-1]).ravel()
    faces = np.concatenate((np.column_stack((a, a+1, a+cols)),
                            np.column_stack((a+1, a+cols+1, a+cols))))
    faces = faces[np.isfinite(heights.ravel()[faces]).all(axis=1)]
    if not len(faces):
        return np.empty((0, 3)), np.empty((0, 3), dtype=int), np.empty((0, 2))
    used, remap = np.unique(faces, return_inverse=True)
    vertices = vertices[used]
    return vertices, remap.reshape(-1, 3), vertices[:, :2]/8.


class SurveySurface:
    def __init__(self, manifest):
        path = Path(manifest)
        self.metadata = json.loads(path.read_text())
        self.grids = []
        for entry in self.metadata['grids']:
            with np.load(path.parent/entry['path'], allow_pickle=False) as data:
                self.grids.append((float(entry['spacing']), tuple(entry['origin']), data['heights'].copy()))
        self.grids.sort(key=lambda item: item[0])

    def height(self, x, y):
        for spacing, origin, heights in self.grids:
            xmax = origin[0]+spacing*(heights.shape[1]-1)
            ymax = origin[1]+spacing*(heights.shape[0]-1)
            if origin[0]-1e-8 <= x <= xmax+1e-8 and origin[1]-1e-8 <= y <= ymax+1e-8:
                # Fine patch replaces the coarse tile, including its holes.
                return grid_height(heights, origin, spacing, x, y)
        raise ValueError('Position is outside the surveyed terrain')


@lru_cache(maxsize=8)
def load_surface(manifest):
    return SurveySurface(manifest)


def spawn_height(world, share, x, y, z='auto'):
    """Keep explicit z; otherwise place the existing rover 0.36 m above ground."""
    x, y = float(x), float(y)
    if not all(math.isfinite(v) for v in (x, y)):
        raise ValueError('Spawn x/y must be finite')
    if str(z).lower() != 'auto':
        value = float(z)
        if not math.isfinite(value):
            raise ValueError('Spawn z must be finite or auto')
        return value
    candidate = Path(world).expanduser()
    if '/' not in world and not world.endswith('.world'):
        candidate = Path(share)/'worlds'/(world+'.world')
    manifest = candidate.with_suffix('.terrain.json')
    if manifest.is_file():
        return load_surface(str(manifest.resolve())).height(x, y)+.36
    if world == 'usgs':
        raise FileNotFoundError('Imported terrain metadata is missing; build rover_sim first')
    return .36
