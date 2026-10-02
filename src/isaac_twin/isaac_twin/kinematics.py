"""Forward/inverse kinematics from a URDF string (revolute/fixed chains, numpy only)."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass

import numpy as np


def _rpy_matrix(rpy) -> np.ndarray:
    """URDF fixed-axis rpy: ``Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    r, p, y = (float(v) for v in rpy)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )


def _rotvec(r: np.ndarray) -> np.ndarray:
    """Axis * angle of rotation matrix ``r``."""
    angle = math.acos(float(np.clip((np.trace(r) - 1.0) * 0.5, -1.0, 1.0)))
    skew = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    if angle < 1e-6:
        return 0.5 * skew
    if math.pi - angle < 1e-4:
        # Near 180°: axis from the symmetric part.
        axis = np.sqrt(np.clip((np.diag(r) + 1.0) * 0.5, 0.0, None))
        k = int(np.argmax(axis))
        axis = (r[:, k] + np.eye(3)[:, k]) / (2.0 * axis[k])
        return axis / np.linalg.norm(axis) * angle
    return skew * (angle / (2.0 * math.sin(angle)))


@dataclass
class _Joint:
    name: str
    type: str
    parent: str
    child: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float = -math.inf
    upper: float = math.inf


class Chain:
    """Serial chain ``base -> tip``; ``fk`` returns ``base -> link`` per link."""

    def __init__(self, urdf_xml: str, base: str, tip: str) -> None:
        root = ET.fromstring(urdf_xml)
        by_child: dict[str, _Joint] = {}
        for j in root.findall("joint"):
            origin = np.eye(4)
            o = j.find("origin")
            if o is not None:
                origin[:3, :3] = _rpy_matrix((o.get("rpy") or "0 0 0").split())
                origin[:3, 3] = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
            a = j.find("axis")
            axis = np.array([float(v) for v in (a.get("xyz") if a is not None else "1 0 0").split()])
            joint = _Joint(
                j.get("name"),
                j.get("type"),
                j.find("parent").get("link"),
                j.find("child").get("link"),
                origin,
                axis / np.linalg.norm(axis),
            )
            lim = j.find("limit")
            if joint.type == "revolute" and lim is not None:
                joint.lower = float(lim.get("lower", -math.inf))
                joint.upper = float(lim.get("upper", math.inf))
            by_child[joint.child] = joint
        joints = []
        link = tip
        while link != base:
            if link not in by_child:
                raise ValueError(f"{tip} is not below {base} in the URDF")
            joints.append(by_child[link])
            link = by_child[link].parent
        self._joints = list(reversed(joints))
        self.base = base
        self.tip = tip
        self.joint_names = [j.name for j in self._joints if j.type in ("revolute", "continuous")]
        for j in self._joints:
            if j.type not in ("revolute", "continuous", "fixed"):
                raise ValueError(f"Joint {j.name}: unsupported type {j.type}")
        moving = [j for j in self._joints if j.type != "fixed"]
        self.lower = np.array([j.lower for j in moving])
        self.upper = np.array([j.upper for j in moving])

    def fk(self, positions: dict[str, float]) -> dict[str, np.ndarray]:
        t = np.eye(4)
        out = {self.base: t.copy()}
        for j in self._joints:
            t = t @ j.origin
            if j.type != "fixed":
                motion = np.eye(4)
                motion[:3, :3] = _axis_angle(j.axis, float(positions.get(j.name, 0.0)))
                t = t @ motion
            out[j.child] = t.copy()
        return out

    def tip_pose(self, positions: dict[str, float]) -> np.ndarray:
        return self.fk(positions)[self.tip]

    def _tip_and_jacobian(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """``base -> tip`` and the 6xN geometric Jacobian (linear; angular) in ``base``."""
        t = np.eye(4)
        axes, origins = [], []
        k = 0
        for j in self._joints:
            t = t @ j.origin
            if j.type != "fixed":
                axes.append(t[:3, :3] @ j.axis)
                origins.append(t[:3, 3].copy())
                motion = np.eye(4)
                motion[:3, :3] = _axis_angle(j.axis, float(q[k]))
                t = t @ motion
                k += 1
        jac = np.zeros((6, len(axes)))
        for c, (z, o) in enumerate(zip(axes, origins)):
            jac[:3, c] = np.cross(z, t[:3, 3] - o)
            jac[3:, c] = z
        return t, jac

    def ik(
        self,
        target: np.ndarray,
        seed: np.ndarray | None = None,
        *,
        tol_m: float = 1e-4,
        tol_rad: float = 1e-3,
        max_iter: int = 150,
        damping: float = 0.02,
    ) -> np.ndarray | None:
        """Joint positions (``joint_names`` order) reaching ``base -> tip`` = ``target``, or None.

        Damped least squares within the URDF limits; one seed, so a miss is not
        proof the pose is unreachable (MoveIt's ``/compute_ik`` restarts randomly).
        """
        q = np.clip(np.zeros(len(self.joint_names)) if seed is None else np.array(seed, float), self.lower, self.upper)
        lam2 = damping * damping
        for _ in range(max_iter):
            t, jac = self._tip_and_jacobian(q)
            err = np.concatenate([target[:3, 3] - t[:3, 3], _rotvec(target[:3, :3] @ t[:3, :3].T)])
            if np.linalg.norm(err[:3]) < tol_m and np.linalg.norm(err[3:]) < tol_rad:
                return q
            dq = jac.T @ np.linalg.solve(jac @ jac.T + lam2 * np.eye(6), err)
            q = np.clip(q + dq, self.lower, self.upper)
        return None
