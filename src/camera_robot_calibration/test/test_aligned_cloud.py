"""Unit tests for aligned-depth unprojection."""

from __future__ import annotations

import numpy as np

from camera_robot_calibration.aligned_cloud import unproject_aligned


def test_center_pixel_is_along_optical_z():
    depth = np.zeros((5, 5), dtype=np.float32)
    depth[2, 2] = 1.0
    xyz, valid = unproject_aligned(depth, fx=200.0, fy=200.0, cx=2.0, cy=2.0, stride=1)
    assert valid[2, 2]
    assert xyz.shape == (1, 3)
    np.testing.assert_allclose(xyz[0], [0.0, 0.0, 1.0], atol=1e-6)


def test_off_center_uses_pinhole():
    depth = np.zeros((3, 3), dtype=np.float32)
    depth[0, 2] = 2.0
    xyz, _ = unproject_aligned(
        depth, fx=100.0, fy=100.0, cx=1.0, cy=1.0, stride=1, z_max=3.0
    )
    # u=2, v=0, z=2 → x=(2-1)*2/100=0.02, y=(0-1)*2/100=-0.02
    np.testing.assert_allclose(xyz[0], [0.02, -0.02, 2.0], atol=1e-6)
