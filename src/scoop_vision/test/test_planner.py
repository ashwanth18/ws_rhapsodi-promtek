import numpy as np
import pytest
from conftest import make_surface

from scoop_vision.planner import PlannerParams, ScoopPlanner, solve_dz_for_volume
from scoop_vision.transforms import apply_mtc_shape


@pytest.fixture(scope="module")
def planner(rs6, scoop, authored_poses):
    return ScoopPlanner(rs6, scoop, apply_mtc_shape(authored_poses), PlannerParams())


def test_solve_dz_for_volume():
    d = np.array([0.03, 0.02, 0.01])
    dz = solve_dz_for_volume(d, 0.025)
    assert np.isclose(np.clip(d - dz, 0, None).sum(), 0.025)
    assert np.isclose(solve_dz_for_volume(d, 0.0), 0.03)


def test_authored_path_is_clear(planner):
    assert planner.authored_floor_clearance_m > planner.params.floor_clearance_m
    assert planner.authored_wall_clearance_m > 0.01


def test_horizontal_wall_margin(rs6, scoop, authored_poses):
    """With a 12 mm sideways wall margin, nothing comes within 20 mm of a
    wall horizontally, and the authored lane still fills on a normal bed."""
    from conftest import RS6

    from scoop_vision.container import ContainerModel
    from scoop_vision.mesh import load_stl

    safe = ContainerModel(load_stl(str(RS6)) * 0.001, wall_margin_xy=0.012)
    p = ScoopPlanner(safe, scoop, apply_mtc_shape(authored_poses), PlannerParams())
    xs, ys = safe.grid.centers()
    mound = 0.04 + 0.04 * np.exp(-((xs + 0.02) ** 2 + (ys - 0.10) ** 2) / (2 * 0.06 ** 2))
    for bed in (np.full(safe.grid.shape, 0.06), mound, np.where(ys > 0.15, 0.09, 0.02)):
        plan = p.plan(make_surface(safe, bed))
        assert plan.success, plan.message
        pts = plan.swept_points
        # Points beside a wall (below the lowest rim, so not passing over the
        # front lip) are >= 20 mm away horizontally.
        wall, _ = rs6.clearances(pts)
        beside = pts[:, 2] < rs6.rim_z - 0.005
        assert wall[beside].min() >= 0.02 - 0.004  # 4 mm voxel quantisation
    flat = p.plan(make_surface(safe, np.full(safe.grid.shape, 0.06)))
    assert flat.predicted_fill_ratio >= 1.0


@pytest.mark.parametrize("level", [0.09, 0.06, 0.045])
def test_flat_bed_tracks_surface(planner, rs6, level):
    plan = planner.plan(make_surface(rs6, np.full(rs6.grid.shape, level)))
    assert plan.success, plan.message
    assert plan.predicted_fill_ratio >= 1.0
    assert 0.0 < plan.max_penetration_m <= planner.params.max_penetration_m + 1e-9
    assert plan.min_clearance_m >= planner.params.wall_clearance_m
    # Mean under the sweep; the sloped front wall lifts it slightly.
    assert level <= plan.surface_height_m < level + 0.01


def test_lower_bed_scoops_lower(planner, rs6):
    hi = planner.plan(make_surface(rs6, np.full(rs6.grid.shape, 0.09)))
    lo = planner.plan(make_surface(rs6, np.full(rs6.grid.shape, 0.05)))
    assert lo.offset_z < hi.offset_z - 0.02


def test_penetration_cap_on_deep_bed(rs6, scoop, authored_poses):
    params = PlannerParams(max_penetration_m=0.01, target_fill_ratio=5.0)
    p = ScoopPlanner(rs6, scoop, apply_mtc_shape(authored_poses), params)
    plan = p.plan(make_surface(rs6, np.full(rs6.grid.shape, 0.09)))
    assert plan.max_penetration_m <= 0.01 + 1e-9


def test_empty_container(planner, rs6):
    plan = planner.plan(make_surface(rs6, np.full(rs6.grid.shape, 0.012)))
    assert not plan.success and plan.container_empty


def test_goes_to_the_heap(planner, rs6):
    xs, ys = rs6.grid.centers()
    heap = 0.03 + 0.05 * np.exp(-((xs ** 2 + (ys - 0.12) ** 2) / (2 * 0.05 ** 2)))
    plan = planner.plan(make_surface(rs6, heap))
    assert plan.success, plan.message
    # Authored lane is y = -0.119; the heap is at y = +0.12.
    assert abs(-0.119 + plan.offset_y - 0.12) < 0.04


def test_never_crosses_a_wall(planner, rs6):
    # Bed piled against the +y wall: the lane must stop short of it.
    xs, ys = rs6.grid.centers()
    pile = np.where(ys > 0.15, 0.09, 0.02)
    plan = planner.plan(make_surface(rs6, pile))
    assert plan.success, plan.message
    wall, floor = rs6.clearances(plan.swept_points)
    assert wall.min() >= planner.params.wall_clearance_m
    assert floor.min() >= planner.params.floor_clearance_m


def test_reachability_callback_skips_unreachable(planner, rs6):
    """A candidate whose approach is out of reach is skipped for the next one."""
    bed = make_surface(rs6, np.full(rs6.grid.shape, 0.09))
    best = planner.plan(bed)
    calls = []

    def reachable(poses):
        calls.append(poses)
        # Pretend the best candidate's approach is out of reach.
        far = abs(poses[0].position[2] - (planner.poses[0].position[2] + best.offset_z)) < 1e-9 and \
            abs(poses[0].position[1] - (planner.poses[0].position[1] + best.offset_y)) < 1e-9
        return (not far, "approach no_ik")

    plan = planner.plan(bed, reachable=reachable)
    assert plan.success, plan.message
    assert (plan.offset_y, plan.offset_z) != (best.offset_y, best.offset_z)
    assert plan.ik_checks == len(calls) >= 2

    none = planner.plan(bed, reachable=lambda poses: (False, "approach no_ik"))
    assert not none.success and "reachable" in none.message


def test_extra_depth_digs_deeper(rs6, scoop, authored_poses):
    bed = make_surface(rs6, np.full(rs6.grid.shape, 0.09))
    shallow = ScoopPlanner(rs6, scoop, apply_mtc_shape(authored_poses), PlannerParams(extra_depth_m=0.0)).plan(bed)
    deep = ScoopPlanner(rs6, scoop, apply_mtc_shape(authored_poses), PlannerParams(extra_depth_m=0.015)).plan(bed)
    assert shallow.success and deep.success
    assert deep.max_penetration_m == pytest.approx(shallow.max_penetration_m + 0.015, abs=0.003)
    assert deep.max_penetration_m <= PlannerParams().max_penetration_m + 1e-9


def test_surface_correction_lowers_the_scoop(rs6, scoop, authored_poses):
    """A camera that reads the powder 30 mm high gets the scoop 30 mm lower."""
    bed = make_surface(rs6, np.full(rs6.grid.shape, 0.10))
    true_bed = make_surface(rs6, np.full(rs6.grid.shape, 0.07))
    shaped = apply_mtc_shape(authored_poses)
    corrected = ScoopPlanner(rs6, scoop, shaped, PlannerParams(surface_correction_m=0.03)).plan(bed)
    truth = ScoopPlanner(rs6, scoop, shaped, PlannerParams()).plan(true_bed)
    assert corrected.success and truth.success
    assert corrected.offset_z == pytest.approx(truth.offset_z, abs=1e-6)
    assert corrected.min_floor_clearance_m >= PlannerParams().floor_clearance_m
