"""4x4 pose helpers for composing hand-eye TF with RealSense extrinsics."""

from __future__ import annotations

import numpy as np
from geometry_msgs.msg import Transform


def quat_xyzw_to_rot(q: np.ndarray) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=float,
    )


def rot_to_quat_xyzw(r: np.ndarray) -> np.ndarray:
    m00, m01, m02 = float(r[0, 0]), float(r[0, 1]), float(r[0, 2])
    m10, m11, m12 = float(r[1, 0]), float(r[1, 1]), float(r[1, 2])
    m20, m21, m22 = float(r[2, 0]), float(r[2, 1]), float(r[2, 2])
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0])


def transform_to_mat(transform: Transform) -> np.ndarray:
    t = transform.translation
    r = transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = quat_xyzw_to_rot(np.array([r.x, r.y, r.z, r.w], dtype=float))
    mat[:3, 3] = np.array([t.x, t.y, t.z], dtype=float)
    return mat


def mat_to_transform(mat: np.ndarray) -> Transform:
    out = Transform()
    out.translation.x, out.translation.y, out.translation.z = (float(v) for v in mat[:3, 3])
    q = rot_to_quat_xyzw(mat[:3, :3])
    out.rotation.x, out.rotation.y, out.rotation.z, out.rotation.w = (
        float(q[0]),
        float(q[1]),
        float(q[2]),
        float(q[3]),
    )
    return out


def base_to_camera_link(
    t_base_optical: Transform, t_camera_link_optical: Transform
) -> Transform:
    """Compose calib ``parent → optical`` with RealSense ``camera_link → optical``.

    Publishing ``parent → optical`` while RealSense already parents ``optical``
    under ``camera_link`` splits TF into two trees, so the depth cloud never
    reaches ``base_link``. Publish ``parent → camera_link`` instead.
    """
    t_bo = transform_to_mat(t_base_optical)
    t_lo = transform_to_mat(t_camera_link_optical)
    t_bl = t_bo @ np.linalg.inv(t_lo)
    return mat_to_transform(t_bl)
