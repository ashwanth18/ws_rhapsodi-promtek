import struct

import numpy as np

from scoop_vision.mesh import load_stl, rasterize_top
from scoop_vision.scoop_tool import trapped_volume
from scoop_vision.transforms import (
    apply_mtc_shape,
    interpolate_path,
    quat_from_rpy,
    quat_multiply,
)


def test_binary_stl_roundtrip(tmp_path):
    tri = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], dtype=np.float32)
    path = tmp_path / "t.stl"
    with open(path, "wb") as fh:
        fh.write(b"\0" * 80 + struct.pack("<I", 1))
        fh.write(np.zeros(3, np.float32).tobytes() + tri.tobytes() + b"\0\0")
    assert np.allclose(load_stl(str(path))[0], tri)


def test_rasterize_top_keeps_highest_face():
    low = [[0, 0, 0.1], [1, 0, 0.1], [0, 1, 0.1]]
    high = [[0, 0, 0.5], [1, 0, 0.5], [0, 1, 0.5]]
    top = rasterize_top(np.array([low, high], float), 0.0, 0.0, 0.1, 10, 10)
    assert np.isclose(top[1, 1], 0.5)
    assert np.isnan(top[9, 9])  # outside the triangle


def test_trapped_volume_of_open_box():
    top = np.zeros((7, 7))
    top[0, :] = top[-1, :] = top[:, 0] = top[:, -1] = 1.0
    # 5x5 inner cells, 1 m deep, 1 m cells; a gap in the wall drains it.
    assert np.isclose(trapped_volume(top, 1.0), 25.0)
    top[3, 0] = 0.4
    assert np.isclose(trapped_volume(top, 1.0), 25 * 0.4)


def test_mtc_shape_defaults_are_identity(authored_poses):
    for got, want in zip(apply_mtc_shape(authored_poses), authored_poses):
        assert np.allclose(got.position, want.position)
        assert np.allclose(got.orientation, want.orientation, atol=1e-9)


def test_mtc_shape_matches_cpp_order(authored_poses):
    shaped = apply_mtc_shape(
        authored_poses, sweep_scale=0.5, pitch_offset_rad=0.1, lift_offset_z=0.02
    )
    contact = np.array(authored_poses[1].position)
    scoop = np.array(authored_poses[2].position)
    assert np.allclose(shaped[2].position, contact + 0.5 * (scoop - contact))
    lift = contact + 0.5 * (np.array(authored_poses[3].position) - contact)
    assert np.isclose(shaped[3].position[2], lift[2] + 0.02)
    q = quat_multiply(authored_poses[0].orientation, quat_from_rpy(0, 0.1, 0))
    assert np.allclose(np.abs(shaped[0].orientation), np.abs(q / np.linalg.norm(q)))


def test_interpolated_path_hits_every_waypoint(authored_poses):
    path = interpolate_path(authored_poses)
    for pose in authored_poses:
        assert min(np.linalg.norm(t - pose.position) for _, t, _ in path) < 1e-9
