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
