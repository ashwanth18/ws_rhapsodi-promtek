"""Choose the next scoop as a pure shift of the authored scoop poses.

The authored approach → contact → scoop → lift → transport_ready poses are
kept exactly. The planner only searches ``(dx, dy, dz)``, the same
``pattern_offset_x/y/z`` that ``scooping_mtc_node`` already applies.

For each XY shift (whole grid cells, so the swept footprint shifts exactly):

1. The swept scoop footprint gives ``B(c)``, the lowest scoop surface over
   cell ``c`` during approach → lift. Engaged volume at height shift ``dz`` is
   ``V = Σ max(0, H(c) − B(c) − dz) · cell²`` over the powder surface ``H``.
2. ``dz`` is the highest value whose predicted fill
   (``fill_efficiency · V / capacity``) reaches ``target_fill_ratio``, i.e.
   the shallowest scoop that should still fill the bowl. It is then pushed
   up by hard limits: maximum penetration below the surface, the approach
   pose staying above the powder, and the ``dz`` range.
3. The candidates are ranked by fill, then by surface height under the
   sweep (dig the peaks to keep the bed level), then by shift distance.
   The best ones are checked against the container/table clearance field
   along the whole interpolated path. MoveIt does not do this check: the
   MTC scene allows the scoop and wrist links to touch the task vessel.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field

import numpy as np

from scoop_vision.container import ContainerModel
from scoop_vision.heightmap import SurfaceMap
from scoop_vision.mesh import voxel_dedupe
from scoop_vision.scoop_tool import ScoopTool
from scoop_vision.transforms import Pose, interpolate_path, quat_to_matrix

# Segments that cut powder: approach→contact, contact→scoop, scoop→lift.
_CUT_SEGMENTS = (0, 1, 2)


@dataclass
class PlannerParams:
    xy_step_m: float = 0.01
    # XY search window. NaN = derive from the container interior so the
    # swept footprint can reach every part of the bed.
    shift_x_min_m: float = float("nan")
    shift_x_max_m: float = float("nan")
    shift_y_min_m: float = float("nan")
    shift_y_max_m: float = float("nan")
    dz_min_m: float = -0.06
    dz_max_m: float = 0.12
    # Fraction of the engaged powder volume that ends up in the bowl.
    # Fit this from logged plans vs MeasureScoopedMass.
    fill_efficiency: float = 0.5
    # Aim above 1.0: flour heaps, and the pour handles precision.
    target_fill_ratio: float = 1.2
    max_penetration_m: float = 0.065
    # Dig this much deeper than the fill model asks for (still bounded by
    # max_penetration_m, the approach rule, and floor clearance).
    extra_depth_m: float = 0.0
    # TEMPORARY calibration compensation: the camera reads the powder this
    # much too high, so the measured surface is lowered by it before planning
    # (fill, depth cap and approach rule all use the corrected surface; the
    # floor check uses the bin model and is unaffected). Set back to 0 after
    # the hand-eye recalibration.
    surface_correction_m: float = 0.0
    min_penetration_m: float = 0.008
    # Clearance to walls/rims and to floor/table. The extra horizontal margin
    # for bin-pose / hand-eye error lives in ContainerModel(wall_margin_xy).
    wall_clearance_m: float = 0.008
    floor_clearance_m: float = 0.008
    approach_clearance_m: float = 0.01
    empty_fill_ratio: float = 0.15
    level_weight: float = 0.3
    shift_weight_per_m: float = 0.2
    max_collision_checks: int = 600
    feasible_pool: int = 40
    raise_step_m: float = 0.005
    raise_max_steps: int = 12


@dataclass
class ScoopPlan:
    success: bool
    message: str
    offset_x: float = 0.0
    offset_y: float = 0.0
    offset_z: float = 0.0
    predicted_fill_ratio: float = 0.0
    predicted_volume_m3: float = 0.0
    engaged_volume_m3: float = 0.0
    capacity_m3: float = 0.0
    max_penetration_m: float = 0.0
    surface_height_m: float = 0.0
    min_clearance_m: float = 0.0
    min_wall_clearance_m: float = 0.0
    min_floor_clearance_m: float = 0.0
    container_empty: bool = False
    score: float = 0.0
    candidates: int = 0
    collision_checks: int = 0
    ik_checks: int = 0
    swept_points: np.ndarray | None = field(default=None, repr=False)

    def to_dict(self) -> dict:
        d = asdict(self)
        d.pop("swept_points", None)
        return d


def solve_dz_for_volume(depths: np.ndarray, target_sum: float) -> float:
    """``dz`` with ``Σ max(0, depths − dz) == target_sum`` (piecewise linear)."""
    d = np.sort(np.asarray(depths, dtype=np.float64))[::-1]
    if target_sum <= 0.0:
        return float(d[0])
    m = np.arange(1, len(d) + 1)
    dzs = (np.cumsum(d) - target_sum) / m
    nxt = np.append(d[1:], -np.inf)
    ok = (dzs <= d) & (dzs >= nxt)
    return float(dzs[int(np.argmax(ok))])


def _cell_min(grid, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    i, j, ok = grid.index(pts[:, 0], pts[:, 1])
    if not ok.all():
        raise ValueError("Authored scoop path leaves the planning grid; widen the grid pad")
    flat = i * grid.ny + j
    best = np.full(grid.nx * grid.ny, np.inf)
    np.minimum.at(best, flat, pts[:, 2])
    cells = np.nonzero(np.isfinite(best))[0]
    return cells // grid.ny, cells % grid.ny, best[cells]


class ScoopPlanner:
    def __init__(
        self,
        container: ContainerModel,
        tool: ScoopTool,
        poses: list[Pose],
        params: PlannerParams | None = None,
    ) -> None:
        if len(poses) != 5:
            raise ValueError("Expected 5 scoop poses (approach, contact, scoop, lift, transport_ready)")
        self.container = container
        self.tool = tool
        self.poses = poses
        self.params = params or PlannerParams()
        grid = container.grid

        path = interpolate_path(poses)
        cut = [
            tool.points @ rot.T + t
            for rot, t, seg in path
            if seg in _CUT_SEGMENTS or seg == -1
        ]
        self._fi, self._fj, self._fb = _cell_min(grid, np.vstack(cut))

        rot0 = quat_to_matrix(poses[0].orientation)
        approach = tool.points @ rot0.T + np.asarray(poses[0].position)
        self._ai, self._aj, self._ab = _cell_min(grid, approach)

        sweep = np.vstack([tool.collision_points @ rot.T + t for rot, t, _ in path])
        self._collision = voxel_dedupe(sweep, 0.004)

        self.capacity_m3 = tool.capacity_m3(poses[3].orientation)
        if self.capacity_m3 <= 0.0:
            raise ValueError("Scoop holds no volume at the lift orientation")
        wall, floor = container.clearances(self._collision)
        self.authored_wall_clearance_m = float(wall.min())
        self.authored_floor_clearance_m = float(floor.min())
        self.authored_clearance_m = min(self.authored_wall_clearance_m, self.authored_floor_clearance_m)

    def clearance_ok(self, dx: float, dy: float, dz: float) -> tuple[bool, float, float]:
        wall, floor = self.container.clearances(self.swept_points(dx, dy, dz))
        w, f = float(wall.min()), float(floor.min())
        return w >= self.params.wall_clearance_m and f >= self.params.floor_clearance_m, w, f

    def shift_window(self) -> tuple[float, float, float, float]:
        """XY shift limits (m): explicit params, else footprint vs interior."""
        p = self.params
        c = self.container
        cell = c.grid.cell
        xs = c.grid.x0 + (self._fi + 0.5) * cell
        ys = c.grid.y0 + (self._fj + 0.5) * cell
        auto = (
            c.interior_min[0] - xs.min(),
            c.interior_max[0] - xs.max(),
            c.interior_min[1] - ys.min(),
            c.interior_max[1] - ys.max(),
        )
        given = (p.shift_x_min_m, p.shift_x_max_m, p.shift_y_min_m, p.shift_y_max_m)
        lo_x, hi_x, lo_y, hi_y = (a if math.isnan(g) else g for g, a in zip(given, auto))
        # A footprint longer than the bin (approach over the lip) still gets
        # a zero shift; the clearance check decides.
        return min(lo_x, 0.0), max(hi_x, 0.0), min(lo_y, 0.0), max(hi_y, 0.0)

    def swept_points(self, dx: float, dy: float, dz: float) -> np.ndarray:
        return self._collision + np.array([dx, dy, dz])

    def shifted_poses(self, dx: float, dy: float, dz: float) -> list[Pose]:
        d = np.array([dx, dy, dz])
        return [Pose(tuple(float(v) for v in np.asarray(p.position) + d), p.orientation) for p in self.poses]

    def plan(self, surface: SurfaceMap, reachable=None) -> ScoopPlan:
        """Best shift for ``surface``.

        ``reachable(poses) -> (ok, reason)`` optionally checks the robot can
        reach all 5 shifted poses (IK); candidates are tried best-first and the
        first reachable one wins. Without it, reachability is not checked.
        """
        p = self.params
        c = self.container
        grid = c.grid
        cell = grid.cell
        area = cell * cell
        corrected = np.maximum(surface.height - p.surface_correction_m, c.floor_map)
        h = np.where(c.interior, corrected, -np.inf)
        span = max(c.rim_z - c.floor_z, 1e-3)
        target_sum = p.target_fill_ratio * self.capacity_m3 / p.fill_efficiency / area

        step = max(1, int(round(p.xy_step_m / cell)))
        lo_x, hi_x, lo_y, hi_y = self.shift_window()
        si_range = range(int(math.ceil(lo_x / cell)), int(math.floor(hi_x / cell)) + 1)
        sj_range = range(int(math.ceil(lo_y / cell)), int(math.floor(hi_y / cell)) + 1)

        def evaluate(si: int, sj: int, dz_floor: float | None = None):
            ci, cj = self._fi + si, self._fj + sj
            inb = (ci >= 0) & (ci < grid.nx) & (cj >= 0) & (cj < grid.ny)
            if not inb.all():
                return None
            hs = h[ci, cj]
            powder = np.isfinite(hs)
            if not powder.any():
                return None
            depth = hs[powder] - self._fb[powder]
            dz = solve_dz_for_volume(depth, target_sum) - p.extra_depth_m
            lb = max(p.dz_min_m, float(depth.max()) - p.max_penetration_m)
            ai, aj = self._ai + si, self._aj + sj
            ainb = (ai >= 0) & (ai < grid.nx) & (aj >= 0) & (aj < grid.ny)
            ha = np.full(len(ai), -np.inf)
            ha[ainb] = h[ai[ainb], aj[ainb]]
            above = np.isfinite(ha)
            if above.any():
                lb = max(lb, float((ha[above] - self._ab[above]).max()) + p.approach_clearance_m)
            dz = max(dz, lb)
            if dz_floor is not None:
                dz = max(dz, dz_floor)
            if dz > p.dz_max_m:
                return None
            engaged = float(np.clip(depth - dz, 0.0, None).sum()) * area
            pen = float(depth.max()) - dz
            fill = p.fill_efficiency * engaged / self.capacity_m3
            if pen < p.min_penetration_m:
                fill = 0.0
            surf = float(hs[powder].mean())
            dx, dy = si * cell, sj * cell
            score = (
                min(fill, p.target_fill_ratio) / p.target_fill_ratio
                + p.level_weight * (surf - c.floor_z) / span
                - p.shift_weight_per_m * math.hypot(dx, dy)
            )
            return {
                "si": si, "sj": sj, "dx": dx, "dy": dy, "dz": dz,
                "fill": fill, "engaged": engaged, "pen": pen,
                "surface": surf, "score": score,
            }

        cands = []
        for si in si_range:
            if si % step:
                continue
            for sj in sj_range:
                if sj % step:
                    continue
                r = evaluate(si, sj)
                if r is not None:
                    cands.append(r)
        if not cands:
            return ScoopPlan(False, "No scoop shift reaches powder inside the container")
        cands.sort(key=lambda r: r["score"], reverse=True)

        feasible = []
        checks = 0
        for r in cands:
            if checks >= p.max_collision_checks or len(feasible) >= p.feasible_pool:
                break
            cur = r
            last_wall = -1.0
            for _ in range(p.raise_max_steps + 1):
                checks += 1
                ok, wall, floor = self.clearance_ok(cur["dx"], cur["dy"], cur["dz"])
                if ok:
                    cur = dict(cur, clearance=min(wall, floor), wall=wall, floor=floor)
                    feasible.append(cur)
                    break
                # Raising helps floor and front-lip contacts. A side/back wall
                # does not get further away by raising: stop once it stalls.
                if wall < p.wall_clearance_m and wall <= last_wall + 0.001:
                    break
                last_wall = wall
                nxt = evaluate(cur["si"], cur["sj"], cur["dz"] + p.raise_step_m)
                if nxt is None:
                    break
                cur = nxt

        if not feasible:
            return ScoopPlan(
                False,
                f"All {checks} checked scoop shifts come too close to the container "
                f"(need walls ≥ {p.wall_clearance_m * 1000:.0f} mm, floor ≥ "
                f"{p.floor_clearance_m * 1000:.0f} mm; authored path has walls "
                f"{self.authored_wall_clearance_m * 1000:.0f} mm, floor "
                f"{self.authored_floor_clearance_m * 1000:.0f} mm)",
                candidates=len(cands),
                collision_checks=checks,
                capacity_m3=self.capacity_m3,
            )
        feasible.sort(key=lambda r: r["score"], reverse=True)
        best = feasible[0]
        ik_checks = 0
        if reachable is not None:
            reasons: dict[str, int] = {}
            best = None
            for r in feasible:
                ik_checks += 1
                ok, reason = reachable(self.shifted_poses(r["dx"], r["dy"], r["dz"]))
                if ok:
                    best = r
                    break
                reasons[reason] = reasons.get(reason, 0) + 1
            if best is None:
                why = "; ".join(f"{k} ×{v}" for k, v in sorted(reasons.items(), key=lambda kv: -kv[1]))
                return ScoopPlan(
                    False,
                    f"None of the {len(feasible)} safe scoop shifts is reachable by the arm ({why})",
                    candidates=len(cands),
                    collision_checks=checks,
                    ik_checks=ik_checks,
                    capacity_m3=self.capacity_m3,
                )
        empty = best["fill"] < p.empty_fill_ratio
        plan = ScoopPlan(
            success=not empty,
            message=(
                f"Container looks empty: best predicted fill {best['fill']:.0%}"
                if empty
                else f"Scoop shift ({best['dx']:+.3f}, {best['dy']:+.3f}, {best['dz']:+.3f}) m, "
                f"predicted fill {best['fill']:.0%}, clearance walls {best['wall'] * 1000:.0f} mm / "
                f"floor {best['floor'] * 1000:.0f} mm"
            ),
            offset_x=best["dx"],
            offset_y=best["dy"],
            offset_z=best["dz"],
            predicted_fill_ratio=best["fill"],
            predicted_volume_m3=min(best["fill"], 1.0) * self.capacity_m3,
            engaged_volume_m3=best["engaged"],
            capacity_m3=self.capacity_m3,
            max_penetration_m=best["pen"],
            surface_height_m=best["surface"],
            min_clearance_m=best["clearance"],
            min_wall_clearance_m=best["wall"],
            min_floor_clearance_m=best["floor"],
            container_empty=empty,
            score=best["score"],
            candidates=len(cands),
            collision_checks=checks,
            ik_checks=ik_checks,
        )
        plan.swept_points = self.swept_points(best["dx"], best["dy"], best["dz"])
        return plan
