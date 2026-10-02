"""Register the task-container mesh to what the camera sees.

The scoop poses, the clearance field and the height map all assume the
container sits exactly where the cell layout says. Measured wall-top heights
(per-cell 90th percentile, in ``scooping_container_frame``) are compared with
the mesh rim over an X/Y/yaw search:

* :func:`check_alignment` (X/Y only, fast) gates every capture;
* :func:`fit_container_offset` (X/Y/yaw, sub-cell refined) feeds the
  layout-pose proposal, i.e. camera-based container calibration.

Accuracy is bounded by the hand-eye calibration: this finds where the bin is
*in the camera's estimate of base_link*.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

from scoop_vision.container import ContainerModel
from scoop_vision.heightmap import cell_median


@dataclass
class AlignmentResult:
    ok: bool
    message: str
    shift_x_m: float = 0.0
    shift_y_m: float = 0.0
    z_offset_m: float = 0.0
    yaw_offset_deg: float = 0.0
    rim_mad_at_layout_m: float = 0.0
    rim_mad_at_best_m: float = 0.0
    rim_cells_seen: int = 0
    rim_cells_total: int = 0
    at_search_limit: bool = False

    def to_dict(self) -> dict:
        return asdict(self)


def _rim_residuals(measured: np.ndarray, container: ContainerModel, si: int, sj: int) -> np.ndarray:
    ri, rj = np.nonzero(container.rim)
    mi, mj = ri + si, rj + sj
    g = container.grid
    ok = (mi >= 0) & (mi < g.nx) & (mj >= 0) & (mj < g.ny)
    res = np.full(len(ri), np.nan)
    res[ok] = measured[mi[ok], mj[ok]] - container.floor_map[ri[ok], rj[ok]]
    return res


def _flat_rim(container: ContainerModel, tol: float = 0.003) -> np.ndarray:
    """Per rim cell (in ``np.nonzero(container.rim)`` order): all 4 neighbours
    are wall top at the same height."""
    f = np.where(container.rim, container.floor_map, np.nan)
    flat = container.rim.copy()
    for axis in (0, 1):
        for step in (1, -1):
            nb = np.roll(f, step, axis=axis)
            with np.errstate(invalid="ignore"):
                flat &= np.abs(nb - f) <= tol
    return flat[np.nonzero(container.rim)]


def _score(res: np.ndarray, cap: float) -> tuple[float, int]:
    """Mean |residual − median|, capped per cell; unseen cells cost ``cap``.

    Removing the median keeps the XY fit independent of a constant Z error;
    the cap stops a few cells of spilled powder on the rim from dominating.
    """
    seen = np.isfinite(res)
    if not seen.any():
        return cap, 0
    r = res[seen]
    dev = np.minimum(np.abs(r - float(np.median(r))), cap)
    return (float(dev.sum()) + cap * float((~seen).sum())) / len(res), int(seen.sum())


def _xy_scores(
    measured: np.ndarray,
    container: ContainerModel,
    n: int,
    cap: float,
    centre: tuple[int, int] = (0, 0),
    window: int | None = None,
):
    """Scores on the full (2n+1)² shift grid; cells outside ``window`` around
    ``centre`` are left at +inf (coarse-to-fine yaw search)."""
    scores = np.full((2 * n + 1, 2 * n + 1), np.inf)
    seen = np.zeros_like(scores, dtype=int)
    for a, si in enumerate(range(-n, n + 1)):
        if window is not None and abs(si - centre[0]) > window:
            continue
        for b, sj in enumerate(range(-n, n + 1)):
            if window is not None and abs(sj - centre[1]) > window:
                continue
            scores[a, b], seen[a, b] = _score(_rim_residuals(measured, container, si, sj), cap)
    return scores, seen


def _parabolic(sm: float, s0: float, sp: float) -> float:
    """Vertex offset (in steps, within ±0.5) of a parabola through 3 scores."""
    denom = sm - 2.0 * s0 + sp
    if not np.isfinite(denom) or denom <= 1e-12:
        return 0.0
    return float(np.clip(0.5 * (sm - sp) / denom, -0.5, 0.5))


def _rotate_z(points: np.ndarray, yaw_rad: float) -> np.ndarray:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    out = points.copy()
    out[:, 0] = c * points[:, 0] - s * points[:, 1]
    out[:, 1] = s * points[:, 0] + c * points[:, 1]
    return out


def fit_container_offset(
    points: np.ndarray,
    container: ContainerModel,
    *,
    search_m: float = 0.08,
    yaw_range_deg: float = 4.0,
    yaw_step_deg: float = 0.5,
    tolerance_xy_m: float = 0.01,
    tolerance_z_m: float = 0.01,
    tolerance_yaw_deg: float = 1.0,
    min_rim_fraction: float = 0.4,
    cap_m: float = 0.03,
    max_points: int = 250_000,
) -> AlignmentResult:
    """Pose of the real bin relative to the layout, in ``scooping_container_frame``.

    The real bin frame is ``layout_frame · Trans(shift_x, shift_y, z_offset) ·
    Rz(yaw_offset)``. ``points`` must be in the layout's container frame and
    should include the rim and the table around it (arm out of view).
    """
    g = container.grid
    total = int(container.rim.sum())
    n = int(round(search_m / g.cell))
    yaws = (
        np.arange(-yaw_range_deg, yaw_range_deg + 1e-9, yaw_step_deg)
        if yaw_range_deg > 0
        else np.array([0.0])
    )

    if len(points) > max_points:
        points = points[np.random.default_rng(0).choice(len(points), max_points, replace=False)]

    # Coarse: full XY search at zero yaw. Fine: every yaw, XY near that.
    # A few degrees of yaw moves the rim by at most a few cells.
    measured0, _ = cell_median(points, g, quantile=0.9)
    scores0, _ = _xy_scores(measured0, container, n, cap_m)
    a0, b0 = np.unravel_index(int(np.argmin(scores0)), scores0.shape)
    centre = (int(a0) - n, int(b0) - n)
    window = max(2, int(math.ceil(
        math.radians(yaw_range_deg) * float(np.abs(container.bounds_max[:2]).max()) / g.cell
    )))

    per_yaw = []
    for yaw in yaws:
        if yaw == 0.0:
            measured = measured0
            scores, seen = _xy_scores(measured, container, n, cap_m, centre, window)
        else:
            # Undo a candidate bin yaw so the mesh rim grid can be searched in XY.
            measured, _ = cell_median(_rotate_z(points, -math.radians(yaw)), g, quantile=0.9)
            scores, seen = _xy_scores(measured, container, n, cap_m, centre, window)
        a, b = np.unravel_index(int(np.argmin(scores)), scores.shape)
        per_yaw.append((float(scores[a, b]), a, b, scores, seen, measured))

    k = int(np.argmin([p[0] for p in per_yaw]))
    best, a, b, scores, seen, measured = per_yaw[k]
    if seen[a, b] < min_rim_fraction * total:
        return AlignmentResult(
            False,
            "Too few container rim cells visible; move the arm out of view and retry",
            rim_cells_total=total,
        )

    yaw = float(yaws[k])
    if 0 < k < len(yaws) - 1:
        yaw += yaw_step_deg * _parabolic(per_yaw[k - 1][0], best, per_yaw[k + 1][0])
    fa = _parabolic(*(scores[a - 1:a + 2, b] if 0 < a < 2 * n else (0, 0, 0)))
    fb = _parabolic(*(scores[a, b - 1:b + 2] if 0 < b < 2 * n else (0, 0, 0)))
    si, sj = a - n, b - n
    # Shift found in the de-rotated frame; express it in the layout frame.
    sx, sy = (si + fa) * g.cell, (sj + fb) * g.cell
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    dx, dy = c * sx - s * sy, s * sx + c * sy

    # Z from rim cells away from wall edges (edge cells mix in lower
    # surfaces). Powder on a rim only reads high: low percentile, not median.
    res = _rim_residuals(measured, container, si, sj)
    flat = _flat_rim(container) & np.isfinite(res)
    zres = res[flat] if flat.any() else res[np.isfinite(res)]
    dz = float(np.percentile(zres, 20))

    mad0, _ = _score(_rim_residuals(measured0, container, 0, 0), cap_m)
    at_limit = max(abs(si), abs(sj)) >= n or (len(yaws) > 1 and k in (0, len(yaws) - 1))
    ok = bool(
        not at_limit
        and math.hypot(dx, dy) <= tolerance_xy_m
        and abs(dz) <= tolerance_z_m
        and abs(yaw) <= tolerance_yaw_deg
    )
    yaw_text = f", {yaw:+.1f}° yaw" if len(yaws) > 1 else ""
    msg = (
        f"{'Container matches layout' if ok else 'Container differs from layout'}: "
        f"best rim fit at ({dx * 1000:+.0f}, {dy * 1000:+.0f}) mm XY, {dz * 1000:+.0f} mm Z{yaw_text} "
        f"(rim error {best * 1000:.1f} mm vs {mad0 * 1000:.1f} mm at layout, "
        f"{seen[a, b]}/{total} rim cells)"
        + (" — at the search limit, offset may be larger" if at_limit else "")
    )
    return AlignmentResult(
        ok=ok,
        message=msg,
        shift_x_m=float(dx),
        shift_y_m=float(dy),
        z_offset_m=float(dz),
        yaw_offset_deg=float(yaw),
        rim_mad_at_layout_m=float(mad0),
        rim_mad_at_best_m=float(best),
        rim_cells_seen=int(seen[a, b]),
        rim_cells_total=total,
        at_search_limit=bool(at_limit),
    )


def check_alignment(
    points: np.ndarray,
    container: ContainerModel,
    *,
    search_m: float = 0.08,
    tolerance_xy_m: float = 0.01,
    tolerance_z_m: float = 0.01,
    min_rim_fraction: float = 0.4,
    cap_m: float = 0.03,
) -> AlignmentResult:
    """Fast X/Y-only check used to gate every capture."""
    return fit_container_offset(
        points,
        container,
        search_m=search_m,
        yaw_range_deg=0.0,
        tolerance_xy_m=tolerance_xy_m,
        tolerance_z_m=tolerance_z_m,
        min_rim_fraction=min_rim_fraction,
        cap_m=cap_m,
    )
