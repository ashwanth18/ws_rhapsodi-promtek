"""Task-container geometry in ``scooping_container_frame``.

Everything here is derived from the same mesh the planning scene uses, so a
layout change (RS6 → RS3, new pose) needs no extra configuration.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from scoop_vision.mesh import rasterize_top, sample_surface


@dataclass(frozen=True)
class Grid2D:
    """Regular XY grid; cell ``(i, j)`` spans ``x0 + i*cell`` … ``+cell``."""

    x0: float
    y0: float
    cell: float
    nx: int
    ny: int

    @property
    def shape(self) -> tuple[int, int]:
        return (self.nx, self.ny)

    def centers(self) -> tuple[np.ndarray, np.ndarray]:
        xs = self.x0 + (np.arange(self.nx) + 0.5) * self.cell
        ys = self.y0 + (np.arange(self.ny) + 0.5) * self.cell
        return np.meshgrid(xs, ys, indexing="ij")

    def index(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Cell indices and an in-bounds mask for points ``(x, y)``."""
        i = np.floor((np.asarray(x) - self.x0) / self.cell).astype(np.int64)
        j = np.floor((np.asarray(y) - self.y0) / self.cell).astype(np.int64)
        ok = (i >= 0) & (i < self.nx) & (j >= 0) & (j < self.ny)
        return i, j, ok


class ContainerModel:
    """Floor map, powder interior, rim cells and a clearance field.

    * ``floor_map``: highest container surface per cell (NaN off the mesh).
    * ``interior``: cells where powder can lie (below the lowest rim).
    * ``rim``: wall-top cells, used by the alignment check.
    * ``clearance(points)``: distance from each point to the container
      shell or the table plane (``z < 0``), from a voxel distance field.
    """

    def __init__(
        self,
        tris: np.ndarray,
        *,
        cell: float = 0.005,
        pad: float = 0.15,
        voxel: float = 0.004,
        rim_margin: float = 0.005,
        sdf_pad_xy: float = 0.08,
        sdf_pad_z: float = 0.20,
        wall_margin_xy: float = 0.0,
    ) -> None:
        self.tris = np.asarray(tris, dtype=np.float64)
        pts = self.tris.reshape(-1, 3)
        self.bounds_min = pts.min(axis=0)
        self.bounds_max = pts.max(axis=0)

        lo = self.bounds_min[:2] - pad
        hi = self.bounds_max[:2] + pad
        nx = int(np.ceil((hi[0] - lo[0]) / cell))
        ny = int(np.ceil((hi[1] - lo[1]) / cell))
        self.grid = Grid2D(float(lo[0]), float(lo[1]), cell, nx, ny)

        self.floor_map = rasterize_top(self.tris, lo[0], lo[1], cell, nx, ny)
        footprint = np.isfinite(self.floor_map)
        # Rim = lowest wall top on the outer edge of the footprint (the open
        # front lip on RS-style bins). Powder sits below it.
        outer = footprint & ~ndimage.binary_erosion(footprint, border_value=0)
        self.rim_z = float(np.nanmin(self.floor_map[outer]))
        interior = footprint & (self.floor_map < self.rim_z - rim_margin)
        labels, n = ndimage.label(interior)
        if n > 1:
            sizes = ndimage.sum(interior, labels, index=np.arange(1, n + 1))
            interior = labels == (1 + int(np.argmax(sizes)))
        if not interior.any():
            raise ValueError("Container mesh has no open interior below its rim")
        self.interior = interior
        self.rim = footprint & ~interior
        self.floor_z = float(np.nanmin(self.floor_map[interior]))
        xs, ys = self.grid.centers()
        self.interior_min = np.array([xs[interior].min(), ys[interior].min()]) - cell / 2
        self.interior_max = np.array([xs[interior].max(), ys[interior].max()]) + cell / 2

        self.wall_margin_xy = float(wall_margin_xy)
        self._build_sdf(voxel, sdf_pad_xy, sdf_pad_z)

    def _build_sdf(self, voxel: float, pad_xy: float, pad_z: float) -> None:
        """Two distance fields: walls/rims, and floor + table.

        Walls are grown sideways by ``wall_margin_xy`` before the distance
        transform: bin-pose and hand-eye errors are horizontal (the rim fit
        pins Z to ~1 mm), so a scoop beside the back wall gets the extra
        margin while one passing over the low front lip does not.
        """
        lo = self.bounds_min.copy()
        hi = self.bounds_max.copy()
        lo[:2] -= pad_xy
        hi[:2] += pad_xy
        lo[2] = min(lo[2], 0.0) - 0.02
        hi[2] += pad_z
        shape = np.ceil((hi - lo) / voxel).astype(int) + 1

        floor_tri = np.all(self.tris[:, :, 2] <= self.floor_z + 0.002, axis=1)

        def field(tris: np.ndarray, table: bool, grow_xy: float = 0.0) -> np.ndarray:
            occ = np.zeros(shape, dtype=bool)
            if len(tris):
                surf = sample_surface(tris, voxel * 0.5)
                idx = np.clip(np.floor((surf - lo) / voxel).astype(int), 0, shape - 1)
                occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
            r = int(round(grow_xy / voxel))
            if r > 0:
                yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
                disk = (xx * xx + yy * yy <= r * r)[:, :, None]
                occ = ndimage.binary_dilation(occ, structure=disk)
            if table:  # everything below the container base plane
                occ[:, :, : max(int(np.floor((0.0 - lo[2]) / voxel)), 0)] = True
            return ndimage.distance_transform_edt(~occ) * voxel

        self._sdf_wall = field(self.tris[~floor_tri], table=False, grow_xy=self.wall_margin_xy)
        self._sdf_floor = field(self.tris[floor_tri], table=True)
        self._sdf_lo = lo
        self._sdf_voxel = voxel
        self._sdf_shape = shape

    def clearances(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """``(wall, floor)`` distance (m) per point.

        Outside the fields points are far from the bin (1 m), except below
        the table plane, where the floor distance is 0.
        """
        pts = np.asarray(points, dtype=np.float64)
        idx = np.floor((pts - self._sdf_lo) / self._sdf_voxel).astype(np.int64)
        inside = np.all((idx >= 0) & (idx < self._sdf_shape), axis=1)
        wall = np.ones(len(pts))
        floor = np.ones(len(pts))
        ii = idx[inside]
        wall[inside] = self._sdf_wall[ii[:, 0], ii[:, 1], ii[:, 2]]
        floor[inside] = self._sdf_floor[ii[:, 0], ii[:, 1], ii[:, 2]]
        floor[~inside & (pts[:, 2] < 0.0)] = 0.0
        return wall, floor

    def clearance(self, points: np.ndarray) -> np.ndarray:
        """Distance (m) from each point to the container or table."""
        wall, floor = self.clearances(points)
        return np.minimum(wall, floor)

    def interior_corners(self) -> np.ndarray:
        """Eight corners of the interior box (floor → rim), container frame."""
        (x0, y0), (x1, y1) = self.interior_min, self.interior_max
        return np.array([
            [x, y, z]
            for x in (x0, x1)
            for y in (y0, y1)
            for z in (self.floor_z, self.rim_z)
        ])
