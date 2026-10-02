"""Powder surface height map from depth points (container frame)."""

from __future__ import annotations

import warnings
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from scoop_vision.container import ContainerModel, Grid2D


def unproject_depth(
    depth_m: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    *,
    stride: int = 2,
    z_min: float = 0.15,
    z_max: float = 2.0,
) -> np.ndarray:
    """Depth image (metres) → ``(n, 3)`` points in the optical frame."""
    d = depth_m[::stride, ::stride]
    v, u = np.mgrid[0:depth_m.shape[0]:stride, 0:depth_m.shape[1]:stride]
    ok = np.isfinite(d) & (d > z_min) & (d < z_max)
    z = d[ok]
    x = (u[ok] - cx) * z / fx
    y = (v[ok] - cy) * z / fy
    return np.column_stack([x, y, z])


def cell_median(
    points: np.ndarray,
    grid: Grid2D,
    keep: np.ndarray | None = None,
    quantile: float = 0.5,
) -> tuple[np.ndarray, np.ndarray]:
    """Per-cell z quantile (median by default; NaN where empty) and counts.

    ``keep`` optionally restricts to cells where it is True.
    """
    h = np.full(grid.shape, np.nan)
    counts = np.zeros(grid.shape, dtype=np.int64)
    if len(points) == 0:
        return h, counts
    i, j, ok = grid.index(points[:, 0], points[:, 1])
    if keep is not None:
        ok &= keep[np.clip(i, 0, grid.nx - 1), np.clip(j, 0, grid.ny - 1)]
    i, j, z = i[ok], j[ok], points[ok, 2]
    if len(z) == 0:
        return h, counts
    flat = i * grid.ny + j
    order = np.lexsort((z, flat))
    flat, z = flat[order], z[order]
    uniq, start, cnt = np.unique(flat, return_index=True, return_counts=True)
    med = z[start + np.floor(quantile * (cnt - 1) + 0.5).astype(np.int64)]
    h.flat[uniq] = med
    counts.flat[uniq] = cnt
    return h, counts


@dataclass
class SurfaceMap:
    """Fused powder height map over the container interior.

    ``height`` is defined on every interior cell (gaps filled from the
    nearest measured cell) and NaN elsewhere. ``measured`` marks cells with
    real depth returns.
    """

    height: np.ndarray
    measured: np.ndarray
    measured_fraction: float
    frames: int

    def stats(self, container: ContainerModel) -> dict:
        h = self.height[container.interior]
        depth = h - container.floor_map[container.interior]
        cell_area = container.grid.cell ** 2
        return {
            "surface_mean_m": float(np.mean(h)),
            "surface_max_m": float(np.max(h)),
            "surface_min_m": float(np.min(h)),
            "powder_volume_m3": float(np.sum(np.clip(depth, 0.0, None)) * cell_area),
            "measured_fraction": self.measured_fraction,
            "frames": self.frames,
        }


def build_surface(
    frames: list[np.ndarray],
    container: ContainerModel,
    *,
    min_points_per_cell: int = 1,
    above_top_m: float = 0.08,
    below_floor_tol_m: float = 0.01,
    min_frame_fraction: float = 0.3,
) -> SurfaceMap:
    """Fuse several point sets (container frame) into one :class:`SurfaceMap`.

    Each frame becomes a per-cell median; frames are combined with a
    per-cell median too, which removes D455 speckle on white flour.
    """
    grid = container.grid
    interior = container.interior
    floor = container.floor_map
    per_frame = []
    for pts in frames:
        if len(pts) == 0:
            continue
        i, j, ok = grid.index(pts[:, 0], pts[:, 1])
        ok &= interior[np.clip(i, 0, grid.nx - 1), np.clip(j, 0, grid.ny - 1)]
        f = floor[np.clip(i, 0, grid.nx - 1), np.clip(j, 0, grid.ny - 1)]
        ok &= pts[:, 2] >= f - below_floor_tol_m
        # Flour heaps above the low front lip; clip against the tallest wall.
        ok &= pts[:, 2] <= container.bounds_max[2] + above_top_m
        h, counts = cell_median(pts[ok], grid)
        h[counts < min_points_per_cell] = np.nan
        per_frame.append(h)
    if not per_frame:
        raise ValueError("No depth points fell inside the container")
    stack = np.stack(per_frame)
    valid = np.isfinite(stack).sum(axis=0)
    need = max(1, int(np.ceil(min_frame_fraction * len(per_frame))))
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN cells
        fused = np.nanmedian(stack, axis=0)
    measured = interior & (valid >= need) & np.isfinite(fused)
    n_interior = int(interior.sum())
    frac = float(measured.sum()) / n_interior if n_interior else 0.0
    if not measured.any():
        raise ValueError("Depth returned no valid cells inside the container")

    height = np.where(measured, fused, np.nan)
    # Fill holes (specular spots, wall shadows) from the nearest measured cell.
    _, (ii, jj) = ndimage.distance_transform_edt(~measured, return_indices=True)
    filled = height[ii, jj]
    filled = np.where(interior, np.maximum(filled, floor), np.nan)
    return SurfaceMap(height=filled, measured=measured, measured_fraction=frac, frames=len(per_frame))
