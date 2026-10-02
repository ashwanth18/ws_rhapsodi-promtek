"""Cell geometry shared by Isaac and the ROS nodes: layout, D455 pose, URIs.

numpy + yaml only: this module runs under Isaac's ``python.sh`` (no ROS).
"""

from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

# realsense-ros frame layout: depth at camera_link, colour beside it, and
# camera_*_frame -> camera_*_optical_frame is a fixed axis swap.
D455_COLOR_OFFSET_M = (0.0, -0.059, 0.0)
OPTICAL_FROM_BODY_XYZW = (-0.5, 0.5, -0.5, 0.5)


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
        ]
    )


def matrix_to_quat(r: np.ndarray) -> np.ndarray:
    """``xyzw`` of a rotation matrix (Shepperd)."""
    m = np.asarray(r, dtype=float)
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    q = np.asarray(q)
    return q / np.linalg.norm(q)


def rpy_deg_to_quat(rpy_deg) -> np.ndarray:
    """Same convention as scooping_controller ``rpy_deg_to_quaternion`` (ZYX)."""
    r, p, y = (math.radians(float(v)) / 2.0 for v in rpy_deg)
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array(
        [
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        ]
    )


def pose_matrix(xyz, quat_xyzw) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = quat_to_matrix(quat_xyzw)
    t[:3, 3] = np.asarray(xyz, dtype=float)
    return t


def workspace_root() -> Path:
    """Repo root: ``ISAAC_TWIN_WS`` or the checkout this file lives in."""
    env = os.environ.get("ISAAC_TWIN_WS", "").strip()
    if env:
        return Path(env).resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "config" / "layouts").is_dir() and (parent / "src").is_dir():
            return parent
    raise RuntimeError("Workspace root not found; set ISAAC_TWIN_WS")


_PACKAGE_CACHE: dict[str, Path] = {}
_NAME_RE = re.compile(r"<name>\s*([^<\s]+)\s*</name>")


def package_dir(pkg: str, ws: Path | None = None) -> Path:
    """Share dir of ``pkg``: ament index, else ``install/``, else the source tree."""
    if pkg in _PACKAGE_CACHE:
        return _PACKAGE_CACHE[pkg]
    try:
        from ament_index_python.packages import get_package_share_directory

        found = Path(get_package_share_directory(pkg))
        _PACKAGE_CACHE[pkg] = found
        return found
    except Exception:
        pass
    ws = ws or workspace_root()
    installed = ws / "install" / pkg / "share" / pkg
    if installed.is_dir():
        _PACKAGE_CACHE[pkg] = installed
        return installed
    for manifest in (ws / "src").rglob("package.xml"):
        match = _NAME_RE.search(manifest.read_text(encoding="utf-8", errors="ignore"))
        if match and match.group(1) == pkg:
            _PACKAGE_CACHE[pkg] = manifest.parent
            return manifest.parent
    raise FileNotFoundError(f"Package {pkg} not found under {ws}")


def resolve_uri(uri: str, ws: Path | None = None) -> str:
    if uri.startswith("package://"):
        pkg, _, rel = uri[len("package://"):].partition("/")
        return str(package_dir(pkg, ws) / rel)
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri


@dataclass
class CellObject:
    id: str
    geometry_type: str  # "mesh" | "box"
    pose: np.ndarray  # base_link -> object
    scale: np.ndarray  # metres per mesh unit (mesh) or 1 (box)
    dimensions: np.ndarray  # box size (m)
    color: tuple[float, float, float]
    mesh_resource: str = ""


def load_layout(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as fh:
        return yaml.safe_load(fh) or {}


def layout_objects(doc: dict) -> list[CellObject]:
    """Enabled objects with the same pose/scale rules as the C++ scene loader."""
    out = []
    for obj in doc.get("objects") or []:
        if not obj.get("enabled", True):
            continue
        ori = obj.get("orientation") or {}
        if "quat_xyzw" in ori:
            quat = np.asarray(ori["quat_xyzw"], dtype=float)
        elif "rpy_deg" in ori:
            quat = rpy_deg_to_quat(ori["rpy_deg"])
        else:
            raise ValueError(f"Object {obj.get('id')} has no orientation")
        scale = np.asarray(obj.get("scale_xyz", [1, 1, 1]), dtype=float)
        if obj.get("mesh_units") == "mm":
            scale = scale * 0.001
        out.append(
            CellObject(
                id=str(obj["id"]),
                geometry_type=str(obj["geometry_type"]),
                pose=pose_matrix(obj["position_xyz"], quat),
                scale=scale,
                dimensions=np.asarray(obj.get("dimensions_xyz", [0, 0, 0]), dtype=float),
                color=tuple(float(c) for c in obj.get("color_rgb", [0.7, 0.7, 0.7])),
                mesh_resource=str(obj.get("mesh_resource") or ""),
            )
        )
    return out


def _deep_merge(base: dict, over: dict) -> dict:
    out = dict(base)
    for key, value in over.items():
        out[key] = _deep_merge(out[key], value) if isinstance(value, dict) and isinstance(out.get(key), dict) else value
    return out


def twin_scoop_vision_params(nodes_yaml: str | Path | None = None, ws: Path | None = None) -> dict:
    """The cell's scoop_vision.yaml with the twin's ``scoop_vision`` overrides on top."""
    cell_yaml = package_dir("scoop_vision", ws) / "config" / "scoop_vision.yaml"
    params = yaml.safe_load(cell_yaml.read_text(encoding="utf-8"))
    nodes_yaml = Path(nodes_yaml) if nodes_yaml else package_dir("isaac_twin", ws) / "config" / "twin_nodes.yaml"
    overrides = {"scoop_vision": (yaml.safe_load(nodes_yaml.read_text(encoding="utf-8")) or {}).get("scoop_vision") or {}}
    return _deep_merge(params, overrides)


def robot_tool_config(robot_key: str = "niryo", ws: Path | None = None) -> dict:
    """``tool`` block (scoop mesh + tcp offset) of a robot profile, as scoop_vision reads it."""
    ws = ws or workspace_root()
    for path in (ws / "config" / "robots" / f"{robot_key}.yaml", ws / "src" / "scooping_controller" / "config" / "robots.yaml"):
        if path.is_file():
            doc = yaml.safe_load(path.read_text(encoding="utf-8"))
            profile = doc["robots"][robot_key] if "robots" in doc else doc
            return profile["tool"]
    raise FileNotFoundError(f"{robot_key} robot profile not found")


def find_calibration(name: str, ws: Path | None = None) -> Path:
    """The file ``handeye_camera_link_publisher`` reads, else the repo copy."""
    home = Path.home() / ".ros2" / "easy_handeye2" / "calibrations" / f"{name}.calib"
    if home.is_file():
        return home
    repo = (ws or workspace_root()) / "src" / "camera_robot_calibration" / "calibrations" / f"{name}.calib"
    if repo.is_file():
        return repo
    raise FileNotFoundError(f"Calibration {name}.calib not found in {home.parent} or {repo.parent}")


def load_calibration(path: str | Path) -> np.ndarray:
    """``base_link -> camera_color_optical_frame`` from an easy_handeye2 file."""
    with open(path, encoding="utf-8") as fh:
        doc = yaml.safe_load(fh)
    params = doc["parameters"]
    if params.get("calibration_type") != "eye_on_base":
        raise ValueError(f"{path}: only eye_on_base is supported")
    t = doc["transform"]["translation"]
    r = doc["transform"]["rotation"]
    return pose_matrix([t["x"], t["y"], t["z"]], [r["x"], r["y"], r["z"], r["w"]])


def d455_link_to_optical() -> dict[str, np.ndarray]:
    """``camera_link -> camera_{depth,color}_optical_frame``."""
    optical = pose_matrix([0, 0, 0], OPTICAL_FROM_BODY_XYZW)
    color_body = pose_matrix(D455_COLOR_OFFSET_M, [0, 0, 0, 1])
    return {"depth": optical, "color": color_body @ optical}


def camera_link_in_base(base_to_color_optical: np.ndarray) -> np.ndarray:
    """Same composition as ``camera_robot_calibration.tf_compose.base_to_camera_link``."""
    return base_to_color_optical @ np.linalg.inv(d455_link_to_optical()["color"])
