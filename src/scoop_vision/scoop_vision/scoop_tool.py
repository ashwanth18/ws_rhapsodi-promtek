"""Scoop geometry (from the robot's tool STL) and bowl capacity."""

from __future__ import annotations

import heapq

import numpy as np

from scoop_vision.mesh import rasterize_top, sample_surface, voxel_dedupe
from scoop_vision.transforms import quat_to_matrix


def trapped_volume(top: np.ndarray, cell: float) -> float:
    """Water a 2.5D surface holds before spilling ("trapping rain water II").

    ``top`` is the highest solid surface per cell; NaN cells are open air
    and drain freely, as does the grid border.
    """
    nx, ny = top.shape
    solid = np.isfinite(top)
    level = np.where(solid, top, -np.inf)
    seen = np.zeros_like(solid)
    heap: list[tuple[float, int, int]] = []
    for i in range(nx):
        for j in range(ny):
            border = i in (0, nx - 1) or j in (0, ny - 1)
            if not solid[i, j]:
                seen[i, j] = True
                heap.append((-np.inf, i, j))
            elif border:
                seen[i, j] = True
                heap.append((float(level[i, j]), i, j))
    heapq.heapify(heap)
    water = 0.0
    while heap:
        h, i, j = heapq.heappop(heap)
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < nx and 0 <= b < ny and not seen[a, b]:
                seen[a, b] = True
                t = float(level[a, b])
                if t < h:
                    water += h - t
                heapq.heappush(heap, (max(h, t), a, b))
    return water * cell * cell


class ScoopTool:
    """Scoop mesh expressed in the ``tcp_link`` frame.

    The Niryo URDF puts the scoop STL on ``tool_link`` with an identity
    visual origin and ``tcp_link`` at a pure translation from it
    (``tcp_visual_offset_xyz`` in ``scooping_controller/config/robots.yaml``).
    """

    def __init__(
        self,
        tris_tool: np.ndarray,
        tcp_offset_xyz,
        *,
        sample_spacing: float = 0.004,
        collision_voxel: float = 0.006,
    ) -> None:
        offset = np.asarray(tcp_offset_xyz, dtype=np.float64)
        self.tris_tcp = np.asarray(tris_tool, dtype=np.float64) - offset
        self.points = sample_surface(self.tris_tcp, sample_spacing)
        self.collision_points = voxel_dedupe(self.points, collision_voxel)

    def capacity_m3(self, orientation_xyzw, *, cell: float = 0.001) -> float:
        """Level-fill (water) volume of the bowl held at ``orientation``.

        ``cell`` must be finer than the thinnest rim wall: the decimated Niryo
        scoop leaks at 2 mm (42 ml) but matches the hi-res mesh at 1 mm (~74 ml).

        ``orientation`` is the TCP orientation in a z-up frame (the lift /
        retain pose). Cohesive powders heap above this; that is folded into
        the planner's ``fill_efficiency``.
        """
        rot = quat_to_matrix(orientation_xyzw)
        tris = self.tris_tcp @ rot.T
        pts = tris.reshape(-1, 3)
        lo = pts.min(axis=0)[:2] - 2 * cell
        hi = pts.max(axis=0)[:2] + 2 * cell
        nx = int(np.ceil((hi[0] - lo[0]) / cell))
        ny = int(np.ceil((hi[1] - lo[1]) / cell))
        top = rasterize_top(tris, lo[0], lo[1], cell, nx, ny)
        return trapped_volume(top, cell)
