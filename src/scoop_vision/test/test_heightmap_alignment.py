import numpy as np

from scoop_vision.alignment import check_alignment
from scoop_vision.heightmap import build_surface, unproject_depth
from scoop_vision.mesh import sample_surface


def test_unproject_centre_pixel():
    depth = np.full((4, 4), 0.5)
    pts = unproject_depth(depth, 100.0, 100.0, 2.0, 2.0, stride=1)
    assert np.allclose(pts[(pts[:, 0] == 0) & (pts[:, 1] == 0)], [0.0, 0.0, 0.5])


def _bed_points(rs6, z, rng, holes=0.0):
    xs, ys = rs6.grid.centers()
    m = rs6.interior
    x = np.repeat(xs[m], 4) + rng.uniform(-0.002, 0.002, 4 * m.sum())
    y = np.repeat(ys[m], 4) + rng.uniform(-0.002, 0.002, 4 * m.sum())
    zz = np.full_like(x, z) + rng.normal(0.0, 0.003, len(x))
    keep = rng.random(len(x)) >= holes
    return np.column_stack([x, y, zz])[keep]


def test_build_surface_from_noisy_frames(rs6):
    rng = np.random.default_rng(1)
    frames = [_bed_points(rs6, 0.07, rng, holes=0.3) for _ in range(6)]
    # One frame with the arm's shadow spike should not survive the median.
    frames[0] = np.vstack([frames[0], [[0.0, 0.0, 0.18]] * 50])
    s = build_surface(frames, rs6)
    h = s.height[rs6.interior]
    assert np.all(np.isfinite(h))
    assert np.median(np.abs(h - 0.07)) < 0.002
    assert s.measured_fraction > 0.9


def _camera_view(rs6, shift, rng):
    """Bin surface (moved by ``shift``) plus the table plane around it."""
    pts = sample_surface(rs6.tris, 0.002)
    pts = pts[pts[:, 2] > 0.005] + np.array([shift[0], shift[1], 0.0])
    xs, ys = rs6.grid.centers()
    table = np.column_stack([xs.ravel(), ys.ravel(), np.zeros(xs.size)])
    pts = np.vstack([pts, table])
    pts[:, 2] += rng.normal(0.0, 0.002, len(pts))
    return pts


def test_alignment_recovers_shift(rs6):
    rng = np.random.default_rng(2)
    r = check_alignment(_camera_view(rs6, (0.010, -0.015), rng), rs6)
    assert np.isclose(r.shift_x_m, 0.010, atol=0.005), r.message
    assert np.isclose(r.shift_y_m, -0.015, atol=0.005), r.message
    assert not r.ok
    aligned = check_alignment(_camera_view(rs6, (0.0, 0.0), rng), rs6)
    assert aligned.ok, aligned.message


def test_alignment_finds_55mm_and_ignores_powder_on_rim(rs6):
    rng = np.random.default_rng(3)
    pts = _camera_view(rs6, (0.055, 0.005), rng)
    # Flour heaped over part of the back rim reads high there.
    back = pts[:, 0] < -0.09
    pts[back & (rng.random(len(pts)) < 0.5), 2] += 0.04
    r = check_alignment(pts, rs6)
    assert np.isclose(r.shift_x_m, 0.055, atol=0.005), r.message
    assert abs(r.z_offset_m) < 0.006, r.message
    assert not r.ok


def test_fit_recovers_shift_and_yaw(rs6):
    from scoop_vision.alignment import _rotate_z, fit_container_offset

    rng = np.random.default_rng(4)
    pts = _camera_view(rs6, (0.0, 0.0), rng)
    pts = _rotate_z(pts, np.radians(-3.0))
    pts[:, 0] += 0.061
    pts[:, 1] += 0.018
    r = fit_container_offset(pts, rs6)
    assert np.isclose(r.shift_x_m, 0.061, atol=0.005), r.message
    assert np.isclose(r.shift_y_m, 0.018, atol=0.005), r.message
    assert np.isclose(r.yaw_offset_deg, -3.0, atol=0.4), r.message
    assert not r.ok and not r.at_search_limit
