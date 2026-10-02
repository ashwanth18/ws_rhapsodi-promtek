"""Kinematic powder: RS6 heightfield, scoop payload and pour into RS3.

Geometry only, no particles. The bed is a 2.5D surface on the
``ContainerModel`` grid (same mesh/grid scoop_vision plans on). The scoop's
lower envelope carves it; carved powder goes into the bowl, and powder above
the bowl's capacity at the current tilt slides off its lowest point into
RS3, back into RS6, or onto the table. Good enough to exercise the
capture → plan → scoop → pour flow, not to calibrate fill.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from isaac_twin.cell import matrix_to_quat
from scoop_vision.container import ContainerModel
from scoop_vision.mesh import voxel_dedupe
from scoop_vision.scoop_tool import ScoopTool


def _transform(points: np.ndarray, t: np.ndarray) -> np.ndarray:
    return points @ t[:3, :3].T + t[:3, 3]


class PowderBed:
    """Powder surface (container frame z) over ``container.interior``."""

    def __init__(self, container: ContainerModel, fill_depth_m: float) -> None:
        self.container = container
        self.grid = container.grid
        self.mask = container.interior
        self.floor = np.where(self.mask, container.floor_map, np.nan)
        self.cell_area = self.grid.cell ** 2
        self.reset(fill_depth_m)

    def reset(self, fill_depth_m: float) -> None:
        level = min(self.container.floor_z + fill_depth_m, self.container.rim_z)
        self.surface = np.where(self.mask, np.maximum(self.floor, level), np.nan)
        self.version = 0

    def volume_m3(self) -> float:
        return float(np.nansum(self.surface - self.floor) * self.cell_area)

    def carve(self, points: np.ndarray) -> float:
        """Lower the surface to the lowest point per cell; return removed m³."""
        if len(points) == 0:
            return 0.0
        i, j, ok = self.grid.index(points[:, 0], points[:, 1])
        i, j, z = i[ok], j[ok], points[ok, 2]
        if len(z) == 0:
            return 0.0
        inside = self.mask[i, j]
        i, j, z = i[inside], j[inside], z[inside]
        if len(z) == 0:
            return 0.0
        lowest = np.full(self.mask.shape, np.inf)
        np.minimum.at(lowest, (i, j), z)
        cut = self.mask & (lowest < self.surface)
        if not cut.any():
            return 0.0
        new = np.maximum(self.floor[cut], lowest[cut])
        removed = float(np.sum(self.surface[cut] - new) * self.cell_area)
        self.surface[cut] = new
        self.version += 1
        return removed

    def touching(self, points: np.ndarray, tol: float = 0.001) -> bool:
        """Any point at or below the surface (the scoop is still in the bed)."""
        i, j, ok = self.grid.index(points[:, 0], points[:, 1])
        if not ok.any():
            return False
        i, j, z = i[ok], j[ok], points[ok, 2]
        inside = self.mask[i, j]
        return bool(np.any(z[inside] < self.surface[i[inside], j[inside]] + tol))

    def deposit(self, volume_m3: float) -> None:
        """Spread powder evenly over the bed (falls back in from the scoop)."""
        if volume_m3 <= 0:
            return
        self.surface[self.mask] += volume_m3 / (self.mask.sum() * self.cell_area)
        self.version += 1

    def contains_xy(self, x: float, y: float) -> bool:
        i, j, ok = self.grid.index(np.array([x]), np.array([y]))
        return bool(ok[0] and self.mask[i[0], j[0]])

    def mesh(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """``(vertex_cells, face_counts, face_indices)``: fixed topology, z from ``surface``.

        ``vertex_cells`` is ``(n, 2)`` grid indices; ``mesh_points`` fills z.
        """
        idx = -np.ones(self.mask.shape, dtype=np.int64)
        cells = np.argwhere(self.mask)
        idx[cells[:, 0], cells[:, 1]] = np.arange(len(cells))
        a = idx[:-1, :-1]
        b = idx[1:, :-1]
        c = idx[1:, 1:]
        d = idx[:-1, 1:]
        quad = (a >= 0) & (b >= 0) & (c >= 0) & (d >= 0)
        faces = np.stack([a[quad], b[quad], c[quad], d[quad]], axis=1)
        return cells, np.full(len(faces), 4, dtype=np.int32), faces.reshape(-1).astype(np.int32)

    def mesh_points(self, cells: np.ndarray) -> np.ndarray:
        xs, ys = self.grid.centers()
        i, j = cells[:, 0], cells[:, 1]
        return np.stack([xs[i, j], ys[i, j], self.surface[i, j]], axis=1)


@dataclass
class ScoopParams:
    density_g_per_ml: float = 0.55
    capture_ratio: float = 1.0
    heap_factor: float = 1.3
    vibration_flow_g_per_s: float = 4.0
    min_pour_tilt_deg: float = 10.0
    spill_tau_s: float = 0.3

    @classmethod
    def from_twin_config(cls, cfg: dict) -> "ScoopParams":
        """From ``config/twin.yaml`` (``powder`` + ``scoop`` sections)."""
        pcfg, scfg = cfg["powder"], cfg["scoop"]
        return cls(
            density_g_per_ml=float(pcfg["density_g_per_ml"]),
            capture_ratio=float(scfg["capture_ratio"]),
            heap_factor=float(scfg["heap_factor"]),
            vibration_flow_g_per_s=float(scfg["vibration_flow_g_per_s"]),
            min_pour_tilt_deg=float(scfg["min_pour_tilt_deg"]),
            spill_tau_s=float(scfg["spill_tau_s"]),
        )


class PowderCell:
    """RS6 bed + scoop payload + RS3 contents, all in grams for reporting."""

    def __init__(
        self,
        rs6: ContainerModel,
        base_to_rs6: np.ndarray,
        rs3: ContainerModel | None,
        base_to_rs3: np.ndarray | None,
        tool: ScoopTool,
        fill_depth_m: float,
        params: ScoopParams | None = None,
        on_capacity_miss=None,
    ) -> None:
        """``on_capacity_miss(base_to_tcp)``: compute capacity elsewhere (Isaac
        worker thread) instead of inline; overflow waits until it is cached."""
        self.bed = PowderBed(rs6, fill_depth_m)
        self.base_to_rs6 = base_to_rs6
        self.rs6_from_base = np.linalg.inv(base_to_rs6)
        self.rs3 = rs3
        self.rs3_from_base = None if base_to_rs3 is None else np.linalg.inv(base_to_rs3)
        self.tool = tool
        self.points = voxel_dedupe(tool.points, 0.003)
        self.params = params or ScoopParams()
        self.fill_depth_m = fill_depth_m
        self.on_capacity_miss = on_capacity_miss
        self._capacity_cache: dict[tuple, float] = {}
        self.reset()

    def reset(self, fill_depth_m: float | None = None) -> None:
        if fill_depth_m is not None:
            self.fill_depth_m = fill_depth_m
        self.bed.reset(self.fill_depth_m)
        self.payload_m3 = 0.0
        self.rs3_m3 = 0.0
        self.table_m3 = 0.0
        self.carved_m3 = 0.0
        self.capacity_m3 = 0.0
        self.submerged = False

    def grams(self, volume_m3: float) -> float:
        return volume_m3 * 1e6 * self.params.density_g_per_ml

    @property
    def bed_g(self) -> float:
        return self.grams(self.bed.volume_m3())

    @property
    def payload_g(self) -> float:
        return self.grams(self.payload_m3)

    @property
    def rs3_g(self) -> float:
        return self.grams(self.rs3_m3)

    @staticmethod
    def _capacity_key(base_to_tcp: np.ndarray) -> tuple:
        """Gravity in the tcp frame: yaw about vertical does not change capacity."""
        down = base_to_tcp[:3, :3].T @ np.array([0.0, 0.0, -1.0])
        return tuple(np.round(down / 0.03).astype(int))

    def capacity(self, base_to_tcp: np.ndarray) -> float:
        """Bowl capacity (m³) at this tilt, cached."""
        key = self._capacity_key(base_to_tcp)
        if key not in self._capacity_cache:
            self._capacity_cache[key] = self.tool.capacity_m3(matrix_to_quat(base_to_tcp[:3, :3]), cell=0.002)
        return self._capacity_cache[key]

    @staticmethod
    def tilt_deg(base_to_tcp: np.ndarray) -> float:
        return math.degrees(math.acos(float(np.clip(base_to_tcp[2, 2], -1.0, 1.0))))

    def update_capacity(self, base_to_tcp: np.ndarray) -> None:
        self.capacity_m3 = self.capacity(base_to_tcp)

    def _current_capacity(self, base_to_tcp: np.ndarray) -> float | None:
        cached = self._capacity_cache.get(self._capacity_key(base_to_tcp))
        if cached is not None:
            return cached
        if self.on_capacity_miss is None:
            return self.capacity(base_to_tcp)
        self.on_capacity_miss(base_to_tcp)
        return None

    def step(self, dt: float, base_to_tcp: np.ndarray, vibration: float = 0.0) -> None:
        pts_base = _transform(self.points, base_to_tcp)
        pts_rs6 = _transform(pts_base, self.rs6_from_base)
        carved = self.bed.carve(pts_rs6)
        self.carved_m3 += carved
        self.payload_m3 += carved * self.params.capture_ratio
        self.bed.deposit(carved * (1.0 - self.params.capture_ratio))
        self.submerged = self.bed.touching(pts_rs6)
        if self.submerged or self.payload_m3 <= 0.0:
            return

        p = self.params
        spill = 0.0
        capacity = self._current_capacity(base_to_tcp)
        if capacity is not None:
            self.capacity_m3 = capacity
            # Shaking knocks the heap off: only the level fill stays.
            heap = 1.0 if vibration > 0 else p.heap_factor
            excess = self.payload_m3 - capacity * heap
            if excess > 0:
                spill += excess * min(1.0, dt / max(p.spill_tau_s, 1e-3))
        if vibration > 0 and capacity is not None and self.tilt_deg(base_to_tcp) >= p.min_pour_tilt_deg:
            # Vibration feeds powder over the lip only when tipped toward it
            # (holds less than level); tipped back (lift, transport) it holds.
            level = self._current_capacity(np.eye(4))
            if level is not None and capacity < level:
                spill += vibration * p.vibration_flow_g_per_s * dt / (1e6 * p.density_g_per_ml)
        spill = min(spill, self.payload_m3)
        if spill <= 0:
            return
        self.payload_m3 -= spill
        lip = pts_base[int(np.argmin(pts_base[:, 2]))]
        self._land(spill, lip)

    def _land(self, volume: float, lip_base: np.ndarray) -> None:
        if self.rs3 is not None:
            p = _transform(lip_base[None, :], self.rs3_from_base)[0]
            i, j, ok = self.rs3.grid.index(np.array([p[0]]), np.array([p[1]]))
            if ok[0] and self.rs3.interior[i[0], j[0]]:
                self.rs3_m3 += volume
                return
        p = _transform(lip_base[None, :], self.rs6_from_base)[0]
        if self.bed.contains_xy(p[0], p[1]):
            self.bed.deposit(volume)
            return
        self.table_m3 += volume
