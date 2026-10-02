"""Unproject aligned depth with color intrinsics (same frame as hand-eye)."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def unproject_aligned(
    depth_m: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    *,
    stride: int = 1,
    z_min: float = 0.12,
    z_max: float = 1.6,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (xyz Nx3, pixel index mask into the strided image)."""
    if stride < 1:
        raise ValueError("stride must be >= 1")
    z = depth_m[::stride, ::stride].astype(np.float32, copy=False)
    u = np.arange(0, depth_m.shape[1], stride, dtype=np.float32)
    v = np.arange(0, depth_m.shape[0], stride, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    valid = np.isfinite(z) & (z > z_min) & (z < z_max)
    x = (uu - float(cx)) * z / float(fx)
    y = (vv - float(cy)) * z / float(fy)
    xyz = np.stack((x[valid], y[valid], z[valid]), axis=-1)
    return xyz, valid


def xyzrgb_to_cloud(header: Header, xyz: np.ndarray, rgb_u8: np.ndarray) -> PointCloud2:
    n = int(xyz.shape[0])
    packed = (
        (rgb_u8[:, 0].astype(np.uint32) << 16)
        | (rgb_u8[:, 1].astype(np.uint32) << 8)
        | rgb_u8[:, 2].astype(np.uint32)
    )
    blob = np.zeros(n, dtype=np.dtype([
        ("x", np.float32),
        ("y", np.float32),
        ("z", np.float32),
        ("rgb", np.float32),
    ]))
    blob["x"] = xyz[:, 0]
    blob["y"] = xyz[:, 1]
    blob["z"] = xyz[:, 2]
    blob["rgb"] = np.ascontiguousarray(packed).view(np.float32)
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = blob.tobytes()
    return msg
