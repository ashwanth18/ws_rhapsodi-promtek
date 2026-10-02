"""Rotation-rich pose set generation for eye-on-base hand-eye sampling."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class SamplePose:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


def _quat_multiply(
    aw: float, ax: float, ay: float, az: float,
    bw: float, bx: float, by: float, bz: float,
) -> tuple[float, float, float, float]:
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _axis_angle_quat(axis: str, angle_rad: float) -> tuple[float, float, float, float]:
    half = 0.5 * angle_rad
    s = math.sin(half)
    c = math.cos(half)
    if axis == "x":
        return (c, s, 0.0, 0.0)
    if axis == "y":
        return (c, 0.0, s, 0.0)
    if axis == "z":
        return (c, 0.0, 0.0, s)
    raise ValueError(f"Unknown axis: {axis}")


def _normalize_quat(w: float, x: float, y: float, z: float) -> tuple[float, float, float, float]:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return (w / n, x / n, y / n, z / n)


def generate_sample_poses(
    seed: SamplePose,
    *,
    num_samples: int = 12,
    rotation_deg: float = 25.0,
    translation_m: float = 0.02,
    include_seed: bool = True,
) -> list[SamplePose]:
    """Build a rotation-rich pose set around ``seed``.

    Tsai tips: maximize rotation, minimize translation. We sample +/- rotations
    about each axis and a few small translations, then truncate to ``num_samples``.
    """
    if num_samples < 1:
        raise ValueError("num_samples must be >= 1")

    angle = math.radians(float(rotation_deg))
    candidates: list[SamplePose] = []
    if include_seed:
        candidates.append(seed)

    def rotated(base: SamplePose, dq: tuple[float, float, float, float]) -> SamplePose:
        w, x, y, z = _normalize_quat(
            *_quat_multiply(base.qw, base.qx, base.qy, base.qz, *dq)
        )
        return SamplePose(base.x, base.y, base.z, x, y, z, w)

    for axis in ("x", "y", "z"):
        for sign in (1.0, -1.0):
            candidates.append(rotated(seed, _axis_angle_quat(axis, sign * angle)))

    for a1, a2 in (("x", "y"), ("y", "z"), ("z", "x")):
        for s1, s2 in ((1.0, 1.0), (1.0, -1.0), (-1.0, 1.0)):
            q1 = _axis_angle_quat(a1, s1 * angle * 0.7)
            q2 = _axis_angle_quat(a2, s2 * angle * 0.7)
            dq = _quat_multiply(*q1, *q2)
            candidates.append(rotated(seed, dq))

    if translation_m > 0.0:
        for dx, dy, dz in (
            (translation_m, 0.0, 0.0),
            (-translation_m, 0.0, 0.0),
            (0.0, translation_m, 0.0),
            (0.0, -translation_m, 0.0),
            (0.0, 0.0, translation_m * 0.5),
            (0.0, 0.0, -translation_m * 0.5),
        ):
            moved = SamplePose(
                seed.x + dx,
                seed.y + dy,
                seed.z + dz,
                seed.qx,
                seed.qy,
                seed.qz,
                seed.qw,
            )
            candidates.append(rotated(moved, _axis_angle_quat("z", angle * 0.4)))

    unique: list[SamplePose] = []
    seen: set[tuple[float, ...]] = set()
    for pose in candidates:
        key = (
            round(pose.x, 4),
            round(pose.y, 4),
            round(pose.z, 4),
            round(pose.qx, 4),
            round(pose.qy, 4),
            round(pose.qz, 4),
            round(pose.qw, 4),
        )
        if key in seen:
            continue
        seen.add(key)
        unique.append(pose)

    if len(unique) >= num_samples:
        return unique[:num_samples]

    out = list(unique)
    i = 0
    while len(out) < num_samples:
        base = unique[i % len(unique)]
        scale = 0.5 + 0.1 * (len(out) % 5)
        sign = 1.0 if len(out) % 2 == 0 else -1.0
        out.append(rotated(base, _axis_angle_quat("y", angle * scale * sign)))
        i += 1
    return out


def poses_to_stamped(poses: Iterable[SamplePose], *, frame_id: str):
    """Convert SamplePose list to geometry_msgs PoseStamped (ROS runtime only)."""
    from geometry_msgs.msg import PoseStamped

    stamped = []
    for pose in poses:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.pose.position.x = pose.x
        msg.pose.position.y = pose.y
        msg.pose.position.z = pose.z
        msg.pose.orientation.x = pose.qx
        msg.pose.orientation.y = pose.qy
        msg.pose.orientation.z = pose.qz
        msg.pose.orientation.w = pose.qw
        stamped.append(msg)
    return stamped
