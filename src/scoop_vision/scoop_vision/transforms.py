"""Quaternion / pose helpers. Quaternions are ``(x, y, z, w)`` like ROS."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Pose:
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]  # x, y, z, w


def quat_normalize(q) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    return np.array([0.0, 0.0, 0.0, 1.0]) if n <= 0.0 else q / n


def quat_multiply(a, b) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return np.array([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = quat_normalize(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def quat_slerp(a, b, t: float) -> np.ndarray:
    a = quat_normalize(a)
    b = quat_normalize(b)
    dot = float(np.dot(a, b))
    if dot < 0.0:
        b = -b
        dot = -dot
    if dot > 0.9995:
        return quat_normalize(a + t * (b - a))
    theta = math.acos(min(dot, 1.0))
    s = math.sin(theta)
    return (math.sin((1 - t) * theta) * a + math.sin(t * theta) * b) / s


def quat_angle(a, b) -> float:
    dot = abs(float(np.dot(quat_normalize(a), quat_normalize(b))))
    return 2.0 * math.acos(min(dot, 1.0))


def apply_mtc_shape(
    poses: list[Pose],
    *,
    sweep_scale: float = 1.0,
    pitch_offset_rad: float = 0.0,
    lift_offset_z: float = 0.0,
) -> list[Pose]:
    """Mirror ``ScoopingMtcNode::apply_pattern_offset`` with zero XYZ offset.

    After this, any ``pattern_offset_x/y/z`` the MTC node applies is a pure
    translation of every pose, which is what the planner searches over.
    """
    if len(poses) < 5:
        return list(poses)
    pos = [np.array(p.position, dtype=np.float64) for p in poses]
    contact = pos[1].copy()
    for i in range(2, len(pos)):
        pos[i] = contact + sweep_scale * (pos[i] - contact)
    pos[3][2] += lift_offset_z
    pos[4][2] += lift_offset_z
    quats = [quat_normalize(p.orientation) for p in poses]
    if pitch_offset_rad != 0.0:
        q_off = quat_from_rpy(0.0, pitch_offset_rad, 0.0)
        quats = [quat_normalize(quat_multiply(q, q_off)) for q in quats]
    return [
        Pose(tuple(float(v) for v in p), tuple(float(v) for v in q))
        for p, q in zip(pos, quats)
    ]


def interpolate_path(
    poses: list[Pose],
    *,
    max_step_m: float = 0.005,
    max_step_rad: float = 0.05,
) -> list[tuple[np.ndarray, np.ndarray, int]]:
    """Straight-line / slerp samples between consecutive poses.

    Returns ``(rotation, translation, segment_index)`` per sample, where
    segment ``k`` runs from ``poses[k]`` to ``poses[k + 1]`` (the first pose
    itself is reported as segment ``-1``). The real controller follows a
    joint-space plan between these close waypoints, so this is an
    approximation; keep a few mm of clearance margin on top.
    """
    out: list[tuple[np.ndarray, np.ndarray, int]] = []
    if not poses:
        return out
    first = poses[0]
    out.append((quat_to_matrix(first.orientation), np.array(first.position), -1))
    for k in range(len(poses) - 1):
        a, b = poses[k], poses[k + 1]
        pa, pb = np.array(a.position), np.array(b.position)
        dist = float(np.linalg.norm(pb - pa))
        ang = quat_angle(a.orientation, b.orientation)
        steps = max(1, int(math.ceil(max(dist / max_step_m, ang / max_step_rad))))
        for s in range(1, steps + 1):
            t = s / steps
            q = quat_slerp(a.orientation, b.orientation, t)
            out.append((quat_to_matrix(q), pa + t * (pb - pa), k))
    return out
