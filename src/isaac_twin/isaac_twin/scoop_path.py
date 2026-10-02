"""Authored scoop poses and the TCP path the arm follows through them."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

from isaac_twin.kinematics import Chain
from scoop_vision.transforms import Pose, interpolate_path, quat_to_matrix

MARKERS = ["approach_marker", "contact_marker", "scoop_marker", "lift_marker", "transport_ready_marker"]
IK_SEEDS = [np.zeros(6), np.array([0.0, -0.5, 0.5, 0.0, -1.0, 0.0])]


def load_scoop_poses(path: str | Path) -> list[Pose]:
    """The 5 scoop markers of a ``poses.yaml`` (container frame)."""
    doc = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    by_name = {m["name"]: m["pose"] for m in doc["markers"]}
    out = []
    for name in MARKERS:
        p, o = by_name[name]["position"], by_name[name]["orientation"]
        out.append(Pose((p["x"], p["y"], p["z"]), (o["x"], o["y"], o["z"], o["w"])))
    return out


def pose_matrix(pose: Pose) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(pose.orientation)
    t[:3, 3] = pose.position
    return t


def chained_ik(chain: Chain, targets: list[np.ndarray]) -> list[np.ndarray] | None:
    """IK of each base_link target, seeded from the previous solution."""
    out, q = [], None
    for target in targets:
        sol = None
        for seed in ([q] if q is not None else []) + IK_SEEDS:
            sol = chain.ik(target, seed)
            if sol is not None:
                break
        if sol is None:
            return None
        out.append(sol)
        q = sol
    return out


def cartesian_path(poses: list[Pose]) -> list[tuple[np.ndarray, int]]:
    """``(frame -> tcp, segment)``: straight lines + slerp, as scoop_vision models it."""
    out = []
    for rot, t, seg in interpolate_path(poses, max_step_m=0.002, max_step_rad=0.02):
        tcp = np.eye(4)
        tcp[:3, :3], tcp[:3, 3] = rot, t
        out.append((tcp, seg))
    return out


def timed_scoop(
    chain: Chain,
    joints: list[np.ndarray],
    q_start: np.ndarray,
    tcp_speed_m_s: float = 0.2,
    shake_s: float = 5.0,
    shake_intensity: float = 0.75,
    settle_s: float = 1.5,
    lead_s: float = 3.0,
    hold_s: float = 5.0,
    max_step_rad: float = 0.01,
) -> list[tuple[float, np.ndarray, float]]:
    """``(t, joints, vibration)`` knots for one authored scoop, as the BT runs it:
    move to approach, joint-space through the markers at ``tcp_speed_m_s``, the
    MTC shake-off at the lift pose, then hold at transport-ready."""
    names = chain.joint_names
    knots = [(0.0, np.asarray(q_start, dtype=float), 0.0), (lead_s, np.asarray(joints[0], dtype=float), 0.0)]
    t = lead_s
    prev_tcp = chain.tip_pose(dict(zip(names, joints[0])))
    for seg, (a, b) in enumerate(zip(joints[:-1], joints[1:])):
        if seg == 3:
            knots.append((t + shake_s, knots[-1][1], shake_intensity))
            knots.append((t + shake_s + settle_s, knots[-1][1], 0.0))
            t += shake_s + settle_s
        n = max(1, int(np.ceil(np.abs(b - a).max() / max_step_rad)))
        for s in range(1, n + 1):
            q = a + (b - a) * (s / n)
            tcp = chain.tip_pose(dict(zip(names, q)))
            t += max(float(np.linalg.norm(tcp[:3, 3] - prev_tcp[:3, 3])) / tcp_speed_m_s, 1e-3)
            prev_tcp = tcp
            knots.append((t, q, 0.0))
    knots.append((t + hold_s, knots[-1][1], 0.0))
    return knots


def sample_knots(knots: list[tuple[float, np.ndarray, float]], t: float) -> tuple[np.ndarray, float]:
    """Joints (linear between knots) and vibration (of the segment being run) at ``t``."""
    if t <= knots[0][0]:
        return knots[0][1], 0.0
    for (t0, q0, _), (t1, q1, vib) in zip(knots[:-1], knots[1:]):
        if t <= t1:
            u = (t - t0) / max(t1 - t0, 1e-9)
            return q0 + (q1 - q0) * u, vib
    return knots[-1][1], 0.0


def joint_path(chain: Chain, joints: list[np.ndarray], max_step_rad: float = 0.01) -> list[tuple[np.ndarray, int]]:
    """``(base_link -> tcp, segment)``: joint-space interpolation, like MTC's
    pipeline-planned segments between the scoop poses."""
    names = chain.joint_names
    out = [(chain.tip_pose(dict(zip(names, joints[0]))), -1)]
    for seg, (a, b) in enumerate(zip(joints[:-1], joints[1:])):
        n = max(1, int(np.ceil(np.abs(b - a).max() / max_step_rad)))
        for s in range(1, n + 1):
            out.append((chain.tip_pose(dict(zip(names, a + (b - a) * (s / n)))), seg))
    return out
