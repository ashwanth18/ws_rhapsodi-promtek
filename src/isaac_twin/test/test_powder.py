import numpy as np
import pytest

from isaac_twin.powder import PowderBed, PowderCell, ScoopParams
from scoop_vision.container import ContainerModel
from scoop_vision.scoop_tool import ScoopTool


def _box_tris(x0, x1, y0, y1, z0, z1):
    """Open-top box: floor + four walls, as triangles."""
    def quad(a, b, c, d):
        return [[a, b, c], [a, c, d]]

    p = np.array
    tris = []
    tris += quad(p([x0, y0, z0]), p([x1, y0, z0]), p([x1, y1, z0]), p([x0, y1, z0]))
    tris += quad(p([x0, y0, z0]), p([x1, y0, z0]), p([x1, y0, z1]), p([x0, y0, z1]))
    tris += quad(p([x0, y1, z0]), p([x1, y1, z0]), p([x1, y1, z1]), p([x0, y1, z1]))
    tris += quad(p([x0, y0, z0]), p([x0, y1, z0]), p([x0, y1, z1]), p([x0, y0, z1]))
    tris += quad(p([x1, y0, z0]), p([x1, y1, z0]), p([x1, y1, z1]), p([x1, y0, z1]))
    return np.asarray(tris, dtype=float)


def _slab(x0, x1, y0, y1, z0, z1):
    """Closed box (with top), so it shows up in a top-down raster."""
    p = np.array
    top = [[p([x0, y0, z1]), p([x1, y0, z1]), p([x1, y1, z1])], [p([x0, y0, z1]), p([x1, y1, z1]), p([x0, y1, z1])]]
    return np.concatenate([_box_tris(x0, x1, y0, y1, z0, z1), np.asarray(top, dtype=float)])


def _thick_box(half, depth, wall=0.01):
    h, w = half, wall
    return np.concatenate([
        _slab(-h, h, -h, h, 0.0, w),
        _slab(-h, -h + w, -h, h, 0.0, depth),
        _slab(h - w, h, -h, h, 0.0, depth),
        _slab(-h, h, -h, -h + w, 0.0, depth),
        _slab(-h, h, h - w, h, 0.0, depth),
    ])


@pytest.fixture(scope="module")
def bin_model():
    return ContainerModel(_thick_box(0.1, 0.08), cell=0.005, pad=0.02, voxel=0.006)


def _plate(x0, x1, y0, y1, z, step=0.002):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], axis=1)


def test_fill_and_volume(bin_model):
    bed = PowderBed(bin_model, fill_depth_m=0.03)
    area = bin_model.interior.sum() * 0.005 ** 2
    surf_above_floor = np.nanmean(bed.surface - bed.floor)
    assert bed.volume_m3() == pytest.approx(area * surf_above_floor)
    assert surf_above_floor == pytest.approx(bin_model.floor_z + 0.03 - np.nanmean(bed.floor), abs=1e-3)


def test_carve_removes_powder_above_plate(bin_model):
    bed = PowderBed(bin_model, fill_depth_m=0.03)
    before = bed.volume_m3()
    level = bin_model.floor_z + 0.03
    removed = bed.carve(_plate(-0.03, 0.03, -0.03, 0.03, level - 0.01))
    # ~6 x 6 cm footprint, 1 cm deep (cell quantisation at the plate edge).
    assert removed == pytest.approx(0.06 * 0.06 * 0.01, rel=0.25)
    assert bed.volume_m3() == pytest.approx(before - removed)
    assert bed.carve(_plate(-0.03, 0.03, -0.03, 0.03, level - 0.01)) == 0.0


def test_carve_never_goes_below_floor(bin_model):
    bed = PowderBed(bin_model, fill_depth_m=0.03)
    bed.carve(_plate(-0.05, 0.05, -0.05, 0.05, -1.0))
    assert np.all(bed.surface[bed.mask] >= bed.floor[bed.mask] - 1e-12)


def test_deposit_conserves_volume(bin_model):
    bed = PowderBed(bin_model, fill_depth_m=0.03)
    before = bed.volume_m3()
    bed.deposit(1e-5)
    assert bed.volume_m3() == pytest.approx(before + 1e-5)


def test_mesh_topology(bin_model):
    bed = PowderBed(bin_model, fill_depth_m=0.03)
    cells, counts, idx = bed.mesh()
    assert len(cells) == bin_model.interior.sum()
    assert idx.max() < len(cells)
    assert len(idx) == 4 * len(counts)
    pts = bed.mesh_points(cells)
    assert np.all(np.isfinite(pts))


def _flat_scoop():
    # 4 x 4 cm tray with 1 cm walls, opening up (+z), in tcp frame.
    return ScoopTool(_thick_box(0.02, 0.01, wall=0.002), [0.0, 0.0, 0.0], sample_spacing=0.002)


def test_async_capacity_holds_payload_until_known(bin_model):
    misses = []
    cell = PowderCell(
        bin_model, np.eye(4), None, None, _flat_scoop(), fill_depth_m=0.03,
        params=ScoopParams(spill_tau_s=0.01, heap_factor=1.0), on_capacity_miss=misses.append,
    )
    level = bin_model.floor_z + 0.03
    tcp = np.eye(4)
    tcp[:3, 3] = [0.0, 0.0, level - 0.009]
    cell.step(0.01, tcp)
    tcp[:3, 3] = [0.0, 0.0, 0.2]
    loaded = cell.payload_g
    for _ in range(20):
        cell.step(0.01, tcp)
    assert misses and cell.payload_g == pytest.approx(loaded)
    cell.capacity(misses[-1])  # what the worker thread does
    for _ in range(50):
        cell.step(0.01, tcp)
    assert cell.payload_g <= cell.grams(cell.capacity_m3) + 1e-9


def _tilted(deg: float) -> np.ndarray:
    a = np.radians(deg)
    t = np.eye(4)
    t[:3, :3] = [[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]]
    t[:3, 3] = [0.0, 0.0, 0.3]
    return t


@pytest.mark.parametrize("tilted_capacity, drains", [(2e-5, False), (0.5e-5, True)])
def test_vibration_feeds_only_toward_the_lip(bin_model, tilted_capacity, drains):
    cell = PowderCell(
        bin_model, np.eye(4), None, None, _flat_scoop(), fill_depth_m=0.03,
        params=ScoopParams(heap_factor=1.5, min_pour_tilt_deg=10.0, vibration_flow_g_per_s=4.0, spill_tau_s=0.01),
    )
    tcp = _tilted(15.0)
    cell._capacity_cache[cell._capacity_key(np.eye(4))] = 1e-5
    cell._capacity_cache[cell._capacity_key(tcp)] = tilted_capacity
    cell.payload_m3 = 0.4e-5
    for _ in range(50):
        cell.step(0.01, tcp, vibration=1.0)
    if drains:
        assert cell.payload_m3 == pytest.approx(0.4e-5 - 2.0 / (1e6 * 0.55), rel=1e-6)
    else:
        assert cell.payload_m3 == pytest.approx(0.4e-5)


def test_vibration_knocks_off_the_heap(bin_model):
    cell = PowderCell(
        bin_model, np.eye(4), None, None, _flat_scoop(), fill_depth_m=0.03,
        params=ScoopParams(heap_factor=1.5, spill_tau_s=0.01),
    )
    tcp = _tilted(15.0)
    cell._capacity_cache[cell._capacity_key(np.eye(4))] = 1e-5
    cell._capacity_cache[cell._capacity_key(tcp)] = 2e-5
    cell.payload_m3 = 2.8e-5
    cell.step(0.01, tcp)
    assert cell.payload_m3 == pytest.approx(2.8e-5)
    for _ in range(50):
        cell.step(0.01, tcp, vibration=1.0)
    assert cell.payload_m3 == pytest.approx(2e-5, rel=1e-6)


def test_scoop_fills_then_pours_into_rs3(bin_model):
    rs3 = ContainerModel(_thick_box(0.06, 0.05), cell=0.005, pad=0.02, voxel=0.006)
    base_to_rs6 = np.eye(4)
    base_to_rs3 = np.eye(4)
    base_to_rs3[:3, 3] = [0.0, -0.4, 0.0]
    cell = PowderCell(
        bin_model, base_to_rs6, rs3, base_to_rs3, _flat_scoop(), fill_depth_m=0.03,
        params=ScoopParams(spill_tau_s=0.05, heap_factor=1.0),
    )
    start_total = cell.bed_g
    level = bin_model.floor_z + 0.03

    tcp = np.eye(4)
    tcp[:3, 3] = [0.0, 0.0, level - 0.008]
    cell.step(0.01, tcp)
    assert cell.submerged
    assert cell.payload_g > 0

    tcp[:3, 3] = [0.0, 0.0, 0.2]
    cell.update_capacity(tcp)
    for _ in range(50):
        cell.step(0.01, tcp)
    held = cell.payload_g
    assert held > 0
    assert held <= cell.grams(cell.capacity_m3) * 1.01 + 1e-9

    # Over RS3, tipped 90 degrees: capacity ~0, everything slides into RS3.
    pour = np.eye(4)
    pour[:3, :3] = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
    pour[:3, 3] = [0.0, -0.4, 0.15]
    cell.update_capacity(pour)
    for _ in range(200):
        cell.step(0.01, pour, vibration=1.0)
    assert cell.payload_g == pytest.approx(0.0, abs=1e-6)
    assert cell.rs3_g > 0
    total = cell.bed_g + cell.payload_g + cell.rs3_g + cell.grams(cell.table_m3)
    assert total == pytest.approx(start_total, rel=1e-9)
