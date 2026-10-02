"""Particle powder bookkeeping that does not need Isaac: seeding and accounting.

The PhysX particle system (``isaac_twin.scene.particles``) moves the grains;
this module places them in the RS6 bed and says where each one is (bed,
scoop, RS3 or table), so the twin reports the same grams as the heightfield
model. Each particle stands for ``spacing³`` of powder at the bulk density.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from scoop_vision.container import ContainerModel

BED, SCOOP, RS3, TABLE = 0, 1, 2, 3


def _transform(points: np.ndarray, t: np.ndarray) -> np.ndarray:
    return points @ t[:3, :3].T + t[:3, 3]


def particle_grams(spacing_m: float, density_g_per_ml: float, settle_ratio: float = 1.0) -> float:
    """Mass of one grain: the ``spacing³`` cell it is seeded in, shrunk by
    ``settle_ratio`` (settled / seeded bed height) as the bed packs down."""
    return spacing_m ** 3 * settle_ratio * 1e6 * density_g_per_ml


def bed_depth_m(container: ContainerModel, base_to_container: np.ndarray, pos_base: np.ndarray, cell_m: float = 0.01) -> float:
    """Median height of the top grain centre above the floor, per ``cell_m`` column."""
    p = _transform(np.asarray(pos_base, dtype=float), np.linalg.inv(base_to_container))
    p = p[_inside_xy(container, p[:, 0], p[:, 1]) & (p[:, 2] > container.floor_z - 0.01) & (p[:, 2] < container.rim_z)]
    if len(p) == 0:
        return 0.0
    _, col = np.unique(np.floor(p[:, :2] / cell_m).astype(np.int64), axis=0, return_inverse=True)
    top = np.full(col.max() + 1, -np.inf)
    np.maximum.at(top, col.ravel(), p[:, 2])
    return float(np.median(top) - container.floor_z)


def _inside_xy(container: ContainerModel, x: np.ndarray, y: np.ndarray, with_rim: bool = False) -> np.ndarray:
    i, j, ok = container.grid.index(x, y)
    out = np.zeros(len(x), dtype=bool)
    mask = container.interior | container.rim if with_rim else container.interior
    out[ok] = mask[i[ok], j[ok]]
    return out


def seed_bed(
    container: ContainerModel,
    fill_depth_m: float,
    spacing_m: float,
    jitter: float = 0.1,
    seed: int = 0,
) -> np.ndarray:
    """Particle centres (container frame) filling the interior to ``fill_depth_m``.

    A cubic lattice at ``spacing_m``, kept half a spacing off the walls and
    floor, with ``jitter`` × spacing of noise so the pile does not stack as
    a perfect crystal.
    """
    s = float(spacing_m)
    lo, hi = container.interior_min + s / 2, container.interior_max - s / 2
    xs = np.arange(lo[0], hi[0] + 1e-9, s)
    ys = np.arange(lo[1], hi[1] + 1e-9, s)
    gx, gy = (a.ravel() for a in np.meshgrid(xs, ys, indexing="ij"))
    keep = np.ones(len(gx), dtype=bool)
    for dx, dy in ((0, 0), (s / 2, 0), (-s / 2, 0), (0, s / 2), (0, -s / 2)):
        keep &= _inside_xy(container, gx + dx, gy + dy)
    gx, gy = gx[keep], gy[keep]
    i, j, _ = container.grid.index(gx, gy)
    floor = container.floor_map[i, j]
    level = min(container.floor_z + fill_depth_m, container.rim_z)
    cols = []
    for x, y, f in zip(gx, gy, floor):
        zs = np.arange(f + s / 2, level - s / 2 + 1e-9, s)
        if len(zs):
            cols.append(np.column_stack([np.full(len(zs), x), np.full(len(zs), y), zs]))
    if not cols:
        return np.zeros((0, 3))
    pts = np.concatenate(cols)
    rng = np.random.default_rng(seed)
    return pts + rng.uniform(-jitter * s, jitter * s, size=pts.shape)


@dataclass
class ParticleTotals:
    bed_g: float = 0.0
    payload_g: float = 0.0
    rs3_g: float = 0.0
    table_g: float = 0.0


class ParticleAccounting:
    """Labels particles (base frame) as bed / scoop / RS3 / table.

    * scoop: inside the scoop mesh's tcp-frame bounding box (plus margin);
    * RS3 / RS6: over the container footprint, between its floor and a little
      above its rim (a heap still counts);
    * anything else is spilled (table, floor, robot).
    """

    def __init__(
        self,
        rs6: ContainerModel,
        base_to_rs6: np.ndarray,
        rs3: ContainerModel | None,
        base_to_rs3: np.ndarray | None,
        scoop_tris_tcp: np.ndarray,
        particle_g: float,
        rim_margin_m: float = 0.03,
        scoop_margin_m: float = 0.004,
    ) -> None:
        self.rs6 = rs6
        self.rs6_from_base = np.linalg.inv(base_to_rs6)
        self.rs3 = rs3
        self.rs3_from_base = None if base_to_rs3 is None else np.linalg.inv(base_to_rs3)
        pts = np.asarray(scoop_tris_tcp, dtype=float).reshape(-1, 3)
        self.scoop_lo = pts.min(axis=0) - scoop_margin_m
        self.scoop_hi = pts.max(axis=0) + scoop_margin_m
        self.particle_g = float(particle_g)
        self.rim_margin_m = float(rim_margin_m)

    def _in_container(self, container: ContainerModel, from_base: np.ndarray, pos_base: np.ndarray) -> np.ndarray:
        p = _transform(pos_base, from_base)
        # Grains against the walls sit in the grid's rim cells.
        ok = _inside_xy(container, p[:, 0], p[:, 1], with_rim=True)
        return ok & (p[:, 2] > container.floor_z - 0.01) & (p[:, 2] < container.rim_z + self.rim_margin_m)

    def in_scoop(self, pos_base: np.ndarray, base_to_tcp: np.ndarray) -> np.ndarray:
        p = _transform(pos_base, np.linalg.inv(base_to_tcp))
        return np.all((p >= self.scoop_lo) & (p <= self.scoop_hi), axis=1)

    def classify(self, pos_base: np.ndarray, base_to_tcp: np.ndarray) -> np.ndarray:
        labels = np.full(len(pos_base), TABLE, dtype=np.int8)
        if len(pos_base) == 0:
            return labels
        labels[self._in_container(self.rs6, self.rs6_from_base, pos_base)] = BED
        if self.rs3 is not None:
            labels[self._in_container(self.rs3, self.rs3_from_base, pos_base)] = RS3
        labels[self.in_scoop(pos_base, base_to_tcp)] = SCOOP
        return labels

    def totals(self, labels: np.ndarray) -> ParticleTotals:
        counts = np.bincount(labels, minlength=4) * self.particle_g
        return ParticleTotals(
            bed_g=float(counts[BED]),
            payload_g=float(counts[SCOOP]),
            rs3_g=float(counts[RS3]),
            table_g=float(counts[TABLE]),
        )
