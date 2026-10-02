import numpy as np
import pytest

from isaac_twin import cell


def test_optical_axes_match_realsense():
    r = cell.quat_to_matrix(cell.OPTICAL_FROM_BODY_XYZW)
    np.testing.assert_allclose(r @ [0, 0, 1], [1, 0, 0], atol=1e-12)  # forward
    np.testing.assert_allclose(r @ [1, 0, 0], [0, -1, 0], atol=1e-12)  # image right
    np.testing.assert_allclose(r @ [0, 1, 0], [0, 0, -1], atol=1e-12)  # image down


def test_quat_roundtrip():
    q = cell.rpy_deg_to_quat([10, -20, 179.99])
    out = cell.matrix_to_quat(cell.quat_to_matrix(q))
    np.testing.assert_allclose(out * np.sign(np.dot(out, q)), q, atol=1e-9)


def test_rpy_matches_cpp_yaw():
    q = cell.rpy_deg_to_quat([0, 0, 90])
    np.testing.assert_allclose(q, [0, 0, np.sqrt(0.5), np.sqrt(0.5)], atol=1e-12)


def test_camera_link_composes_back_to_calib():
    calib = cell.pose_matrix([0.34, -0.017, 0.77], cell.rpy_deg_to_quat([178, 3, -92]))
    link = cell.camera_link_in_base(calib)
    np.testing.assert_allclose(link @ cell.d455_link_to_optical()["color"], calib, atol=1e-12)


def test_layout_objects_from_repo():
    ws = cell.workspace_root()
    doc = cell.load_layout(ws / "config" / "layouts" / "dual-container.yaml")
    objs = {o.id: o for o in cell.layout_objects(doc)}
    assert {"rs6", "rs3", "table"} <= set(objs)
    assert objs["rs6"].geometry_type == "mesh"
    np.testing.assert_allclose(objs["rs6"].scale, [0.001] * 3)
    np.testing.assert_allclose(objs["table"].dimensions, [1.02, 0.61, 0.03])


def test_resolve_package_uri():
    path = cell.resolve_uri("package://scooping_controller/models/scooping_container/meshes/rs6_container.STL")
    assert path.endswith("rs6_container.STL")


def test_disabled_objects_are_skipped():
    doc = {
        "objects": [
            {"id": "a", "enabled": False, "geometry_type": "box", "position_xyz": [0, 0, 0],
             "orientation": {"quat_xyzw": [0, 0, 0, 1]}},
        ]
    }
    assert cell.layout_objects(doc) == []


def test_missing_orientation_raises():
    with pytest.raises(ValueError):
        cell.layout_objects({"objects": [{"id": "a", "geometry_type": "box", "position_xyz": [0, 0, 0]}]})
