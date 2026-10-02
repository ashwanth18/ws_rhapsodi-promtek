"""Unit tests for hand-eye / RealSense TF composition."""

from __future__ import annotations

import numpy as np
import pytest
from geometry_msgs.msg import Transform

from camera_robot_calibration.tf_compose import (
    base_to_camera_link,
    mat_to_transform,
    quat_xyzw_to_rot,
    rot_to_quat_xyzw,
    transform_to_mat,
)


def _identity() -> Transform:
    t = Transform()
    t.rotation.w = 1.0
    return t


def test_roundtrip_identity_quat():
    r = np.eye(3)
    q = rot_to_quat_xyzw(r)
    np.testing.assert_allclose(quat_xyzw_to_rot(q), r, atol=1e-9)


def test_identity_extrinsics_keeps_calib_xyz():
    calib = Transform()
    calib.translation.x = 0.344
    calib.translation.z = 0.611
    calib.rotation.w = 1.0
    out = base_to_camera_link(calib, _identity())
    assert out.translation.x == pytest.approx(0.344)
    assert out.translation.y == pytest.approx(0.0)
    assert out.translation.z == pytest.approx(0.611)
    assert out.rotation.w == pytest.approx(1.0)


def test_optical_offset_subtracted_from_camera_link():
    calib = Transform()
    calib.translation.z = 0.50
    calib.rotation.w = 1.0
    optical_in_link = Transform()
    optical_in_link.translation.z = 0.02
    optical_in_link.rotation.w = 1.0
    out = base_to_camera_link(calib, optical_in_link)
    assert out.translation.z == pytest.approx(0.48)
    assert out.translation.x == pytest.approx(0.0)


def test_transform_mat_roundtrip():
    t = Transform()
    t.translation.x = 0.1
    t.translation.y = -0.2
    t.translation.z = 0.3
    t.rotation.w = 1.0
    again = mat_to_transform(transform_to_mat(t))
    np.testing.assert_allclose(
        [again.translation.x, again.translation.y, again.translation.z],
        [0.1, -0.2, 0.3],
        atol=1e-9,
    )
