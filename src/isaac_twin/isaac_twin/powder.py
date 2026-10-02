"""Kinematic powder: RS6 heightfield, scoop payload and pour into RS3.

Geometry only, no particles. The bed is a 2.5D surface on the
``ContainerModel`` grid (same mesh/grid scoop_vision plans on). The scoop's
lower envelope carves it; carved powder goes into the bowl. The bowl holds
what a liquid would if its surface could slope up to the powder's angle of
repose (less while vibrating); the rest slides off its lowest point into RS3,
back into RS6, or onto the table. Good enough to exercise the
capture → plan → scoop → pour flow, not to calibrate fill.
"""

from __future__ import annotations

import hashlib
import json
import math
import threading
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from isaac_twin.cell import matrix_to_quat
from scoop_vision.container import ContainerModel
from scoop_vision.mesh import voxel_dedupe
from scoop_vision.scoop_tool import ScoopTool

# Gravity in the tcp frame with the tcp z axis up.
UPRIGHT = np.array([0.0, 0.0, -1.0])


def _transform(points: np.ndarray, t: np.ndarray) -> np.ndarray:
    return points @ t[:3, :3].T + t[:3, 3]


def gravity_in_tcp(base_to_tcp: np.ndarray) -> np.ndarray:
    return base_to_tcp[:3, :3].T @ np.array([0.0, 0.0, -1.0])


def _rotation_between(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Smallest rotation taking unit vector ``a`` to ``b``."""
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c < -1.0 + 1e-9:
        perp = np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(a, [0.0, 1.0, 0.0])
        perp /= np.linalg.norm(perp)
        return 2.0 * np.outer(perp, perp) - np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1.0 + c)


def rotate_toward(a: np.ndarray, b: np.ndarray, max_rad: float) -> np.ndarray:
    """Unit vector ``a`` turned toward ``b`` by at most ``max_rad``."""
    angle = math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0)))
    if angle <= max_rad:
        return np.asarray(b, dtype=float)
    axis = np.cross(a, b)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.asarray(a, dtype=float)
    axis /= n
    s, c = math.sin(max_rad), math.cos(max_rad)
    return a * c + np.cross(axis, a) * s + axis * np.dot(axis, a) * (1 - c)


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


class BowlCapacity:
    """Level-fill volume of the scoop vs gravity direction in the tcp frame.

    Each value is a ``ScoopTool.capacity_m3`` voxelisation (~0.3 s), so they
    are cached per scoop mesh: in memory, and in ``cache_dir`` if given.
    """

    STEP = 0.03  # gravity unit-vector quantum (~1.7 deg)

    def __init__(self, tool: ScoopTool, cache_dir: str | Path | None = None, cell: float = 0.002) -> None:
        self.tool = tool
        self.cell = cell
        self._lock = threading.Lock()
        self._table: dict[tuple, float] = {}
        self._hold: np.ndarray | None = None
        self._path = None
        if cache_dir is not None:
            digest = hashlib.sha1(np.ascontiguousarray(tool.tris_tcp, dtype=np.float64).tobytes())
            digest.update(f"{cell}:{self.STEP}".encode())
            self._path = Path(cache_dir) / f"scoop_capacity_{digest.hexdigest()[:16]}.json"
            if self._path.is_file():
                doc = json.loads(self._path.read_text(encoding="utf-8"))
                self._table = {tuple(k): v for k, v in doc.get("table", [])}
                self._hold = np.asarray(doc["hold"]) if doc.get("hold") else None

    @classmethod
    def key(cls, gravity: np.ndarray) -> tuple:
        return tuple(int(v) for v in np.round(np.asarray(gravity) / cls.STEP))

    def cached(self, gravity: np.ndarray) -> float | None:
        return self._table.get(self.key(gravity))

    def compute(self, gravity: np.ndarray) -> float:
        key = self.key(gravity)
        value = self._table.get(key)
        if value is None:
            g = np.asarray(gravity, dtype=float)
            rot = _rotation_between(g / np.linalg.norm(g), np.array([0.0, 0.0, -1.0]))
            value = float(self.tool.capacity_m3(matrix_to_quat(rot), cell=self.cell))
            with self._lock:
                self._table[key] = value
                self._save()
        return value

    def hold_direction(self) -> np.ndarray:
        """Gravity (tcp frame) at which the bowl holds the most, within 60° of upright."""
        if self._hold is not None:
            return self._hold

        def around(center: np.ndarray, tilts, azimuths) -> list[np.ndarray]:
            x = np.cross(center, [1.0, 0.0, 0.0])
            if np.linalg.norm(x) < 1e-6:
                x = np.cross(center, [0.0, 1.0, 0.0])
            x /= np.linalg.norm(x)
            y = np.cross(center, x)
            out = []
            for t in tilts:
                for a in azimuths:
                    d = math.cos(t) * center + math.sin(t) * (math.cos(a) * x + math.sin(a) * y)
                    out.append(d / np.linalg.norm(d))
            return out

        coarse = [UPRIGHT] + around(UPRIGHT, np.radians([15, 30, 45, 60]), np.radians(np.arange(0, 360, 45)))
        best = max(coarse, key=self.compute)
        fine = [best] + around(best, np.radians([5, 10]), np.radians(np.arange(0, 360, 45)))
        self._hold = max(fine, key=self.compute)
        with self._lock:
            self._save()
        return self._hold

    def _save(self) -> None:
        if self._path is None:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        doc = {
            "hold": None if self._hold is None else [float(v) for v in self._hold],
            "table": [[list(k), v] for k, v in self._table.items()],
        }
        tmp = self._path.with_suffix(".tmp")
        tmp.write_text(json.dumps(doc), encoding="utf-8")
        tmp.replace(self._path)


@dataclass
class ScoopParams:
    density_g_per_ml: float = 0.55
    capture_ratio: float = 1.0
    heap_factor: float = 1.3
    # The powder surface in the bowl can slope this much before it slides.
    repose_deg: float = 35.0
    repose_vibrating_deg: float = 10.0
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
            repose_deg=float(scfg["repose_deg"]),
            repose_vibrating_deg=float(scfg["repose_vibrating_deg"]),
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
        capacity_cache_dir: str | Path | None = None,
    ) -> None:
        """``on_capacity_miss(gravity_tcp)``: compute ``bowl.compute`` elsewhere
        (Isaac worker thread) instead of inline; the payload is held until cached."""
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
        self.bowl = BowlCapacity(tool, capacity_cache_dir)
        self.hold = self.bowl.hold_direction()
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

    @property
    def table_g(self) -> float:
        return self.grams(self.table_m3)

    @staticmethod
    def tilt_deg(base_to_tcp: np.ndarray) -> float:
        return math.degrees(math.acos(float(np.clip(base_to_tcp[2, 2], -1.0, 1.0))))

    def held_gravity(self, gravity: np.ndarray, vibrating: bool) -> np.ndarray:
        """Gravity the held powder behaves as if under: tilted toward the bowl's
        best-holding direction by up to the angle of repose."""
        repose = self.params.repose_vibrating_deg if vibrating else self.params.repose_deg
        return rotate_toward(gravity, self.hold, math.radians(repose))

    def capacity_gravities(self, base_to_tcp: np.ndarray) -> list[np.ndarray]:
        """Every capacity ``step`` can look up at this pose (for prewarming)."""
        g = gravity_in_tcp(base_to_tcp)
        return [g, self.held_gravity(g, False), self.held_gravity(g, True), UPRIGHT]

    def capacity(self, base_to_tcp: np.ndarray, vibrating: bool = False) -> float:
        """What the bowl holds (m³) at this pose, computed now if needed."""
        return self.bowl.compute(self.held_gravity(gravity_in_tcp(base_to_tcp), vibrating))

    def update_capacity(self, base_to_tcp: np.ndarray) -> None:
        self.capacity_m3 = self.capacity(base_to_tcp)

    def _lookup(self, gravity: np.ndarray) -> float | None:
        cached = self.bowl.cached(gravity)
        if cached is not None:
            return cached
        if self.on_capacity_miss is None:
            return self.bowl.compute(gravity)
        self.on_capacity_miss(gravity)
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
        g = gravity_in_tcp(base_to_tcp)
        vibrating = vibration > 0
        held = self._lookup(self.held_gravity(g, vibrating))
        if held is None:
            return
        self.capacity_m3 = held
        spill = 0.0
        if not vibrating:
            # At rest: anything past the repose-angle surface (plus heap) avalanches.
            excess = self.payload_m3 - held * p.heap_factor
            if excess > 0:
                spill = excess * min(1.0, dt / max(p.spill_tau_s, 1e-3))
        else:
            # Vibrating: the powder flows at the feed rate, down to what the
            # bowl holds at the vibrating repose angle, or over the lip without
            # limit while tipped toward it (holds less than level).
            flow = vibration * p.vibration_flow_g_per_s * dt / (1e6 * p.density_g_per_ml)
            toward_lip = False
            if self.tilt_deg(base_to_tcp) >= p.min_pour_tilt_deg:
                water, level = self._lookup(g), self._lookup(UPRIGHT)
                toward_lip = water is not None and level is not None and water < level
            spill = flow if toward_lip else min(flow, max(self.payload_m3 - held, 0.0))
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
