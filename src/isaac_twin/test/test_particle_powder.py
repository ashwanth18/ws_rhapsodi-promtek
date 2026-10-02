import numpy as np
import pytest

from isaac_twin.particle_powder import (
    BED,
    RS3,
    SCOOP,
    TABLE,
    ParticleAccounting,
    bed_depth_m,
    particle_grams,
    seed_bed,
)
from isaac_twin.scoop_path import sample_knots, timed_scoop
from scoop_vision.container import ContainerModel
from test_powder import _box_tris, _thick_box

S = 0.006


@pytest.fixture(scope="module")
def bin_model():
    return ContainerModel(_thick_box(0.1, 0.08), cell=0.005, pad=0.02, voxel=0.006)


def _offset(x, y=0.0, z=0.0):
    t = np.eye(4)
    t[:3, 3] = (x, y, z)
    return t


def test_particle_grams():
    assert particle_grams(0.004, 0.55) == pytest.approx(0.0352)
    assert particle_grams(0.004, 0.55, settle_ratio=0.5) == pytest.approx(0.0176)


def test_bed_depth(bin_model):
    pts = seed_bed(bin_model, 0.03, S, jitter=0.0)
    top = pts[:, 2].max() - bin_model.floor_z
    assert bed_depth_m(bin_model, _offset(0.2), pts + [0.2, 0.0, 0.0]) == pytest.approx(top)
    assert bed_depth_m(bin_model, np.eye(4), np.zeros((0, 3))) == 0.0


def test_seed_bed_fills_interior(bin_model):
    pts = seed_bed(bin_model, 0.03, S, jitter=0.05, seed=1)
    assert len(pts) > 0
    lo, hi = bin_model.interior_min, bin_model.interior_max
    assert np.all(pts[:, :2] > lo[:2]) and np.all(pts[:, :2] < hi[:2])
    assert pts[:, 2].min() > bin_model.floor_z
    assert pts[:, 2].max() < bin_model.floor_z + 0.03
    area = bin_model.interior.sum() * 0.005 ** 2
    assert len(pts) * S ** 3 == pytest.approx(area * 0.03, rel=0.3)
    np.testing.assert_array_equal(pts, seed_bed(bin_model, 0.03, S, jitter=0.05, seed=1))


def test_seed_bed_stops_at_rim(bin_model):
    deep = seed_bed(bin_model, 1.0, S, jitter=0.0)
    assert deep[:, 2].max() < bin_model.rim_z


def test_classify(bin_model):
    base_to_rs6, base_to_rs3 = _offset(0.0), _offset(0.5)
    scoop = _box_tris(-0.02, 0.02, -0.02, 0.02, -0.01, 0.01)
    acc = ParticleAccounting(bin_model, base_to_rs6, bin_model, base_to_rs3, scoop, particle_g=0.5)
    z = bin_model.floor_z + 0.01
    pos = np.array([
        [0.0, 0.0, z],          # RS6 bed
        [0.5, 0.0, z],          # RS3
        [0.25, 0.0, z],         # between the containers
        [0.0, 0.0, 0.5],        # far above RS6
        [0.03, 0.03, z],        # in the bed, but inside the scoop box
    ])
    base_to_tcp = _offset(0.03, 0.03, z)
    labels = acc.classify(pos, base_to_tcp)
    assert labels.tolist() == [BED, RS3, TABLE, TABLE, SCOOP]
    totals = acc.totals(labels)
    assert (totals.bed_g, totals.rs3_g, totals.table_g, totals.payload_g) == (0.5, 0.5, 1.0, 0.5)


def test_classify_without_rs3(bin_model):
    acc = ParticleAccounting(bin_model, np.eye(4), None, None, _box_tris(0, 0.01, 0, 0.01, 0, 0.01), 1.0)
    labels = acc.classify(np.array([[0.5, 0.0, 0.02]]), _offset(1.0))
    assert labels.tolist() == [TABLE]


class _LineChain:
    """Two joints that are the TCP's x and y."""

    joint_names = ["a", "b"]

    def tip_pose(self, q):
        return _offset(q["a"], q["b"])


def test_timed_scoop_inserts_shake_before_segment_3():
    joints = [np.array([0.1 * k, 0.0]) for k in range(5)]
    knots = timed_scoop(
        _LineChain(), joints, np.zeros(2), tcp_speed_m_s=0.1, shake_s=2.0, shake_intensity=0.5, settle_s=1.0,
        lead_s=1.0, hold_s=3.0, max_step_rad=0.05,
    )
    times = [t for t, _, _ in knots]
    assert times == sorted(times)
    shakes = [k for k in knots if k[2] > 0]
    assert len(shakes) == 1 and shakes[0][2] == 0.5
    np.testing.assert_allclose(shakes[0][1], joints[3])
    # lead + 3 segments of 0.1 m at 0.1 m/s + shake + settle + last segment + hold
    assert times[-1] == pytest.approx(1.0 + 3.0 + 2.0 + 1.0 + 1.0 + 3.0)
    np.testing.assert_allclose(knots[-1][1], joints[-1])


def test_sample_knots():
    knots = [(0.0, np.zeros(2), 0.0), (1.0, np.array([1.0, 2.0]), 0.0), (3.0, np.array([1.0, 2.0]), 0.75)]
    q, vib = sample_knots(knots, 0.5)
    np.testing.assert_allclose(q, [0.5, 1.0])
    assert vib == 0.0
    q, vib = sample_knots(knots, 2.0)
    np.testing.assert_allclose(q, [1.0, 2.0])
    assert vib == 0.75
    q, vib = sample_knots(knots, -1.0)
    np.testing.assert_allclose(q, [0.0, 0.0])
    q, vib = sample_knots(knots, 10.0)
    assert vib == 0.0
