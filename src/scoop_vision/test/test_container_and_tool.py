import numpy as np


def test_rs6_interior_from_mesh(rs6):
    assert np.isclose(rs6.floor_z, 0.010, atol=1e-3)
    assert np.isclose(rs6.rim_z, 0.102, atol=1e-3)  # open front lip
    size = rs6.interior_max - rs6.interior_min
    assert 0.33 < size[0] < 0.37 and 0.37 < size[1] < 0.39


def test_clearance_field(rs6):
    centre = np.array([[0.0, 0.0, 0.06]])
    assert np.isclose(rs6.clearance(centre)[0], 0.05, atol=0.006)
    assert rs6.clearance(np.array([[0.0, 0.0, -0.05]]))[0] == 0.0
    assert rs6.clearance(np.array([[2.0, 2.0, 0.5]]))[0] == 1.0
    near_wall = np.array([[0.0, 0.185, 0.06]])
    assert rs6.clearance(near_wall)[0] < 0.008


def test_scoop_capacity_is_plausible(scoop, authored_poses):
    lift = authored_poses[3].orientation
    ml = scoop.capacity_m3(lift) * 1e6
    assert 60.0 < ml < 90.0
    # Tipped well forward (scoop pose) it holds much less.
    assert scoop.capacity_m3(authored_poses[2].orientation) * 1e6 < ml / 2


def test_decimated_scoop_holds_as_much_as_hires(scoop, authored_poses):
    """Thin rim walls in the light mesh must not leak at the capacity grid."""
    from conftest import NIRYO_TCP, SCOOP

    from scoop_vision.mesh import load_stl
    from scoop_vision.scoop_tool import ScoopTool

    hires = ScoopTool(load_stl(str(SCOOP.parent / "hires" / SCOOP.name)) * 0.001, NIRYO_TCP)
    lift = authored_poses[3].orientation
    assert abs(scoop.capacity_m3(lift) / hires.capacity_m3(lift) - 1.0) < 0.1
