"""Validated, normalized cell-layout loading shared by backend tooling."""
from __future__ import annotations

import hashlib
import json
import math
import os
from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[4]


def _quaternion_from_rpy_deg(rpy: list[float]) -> list[float]:
    roll, pitch, yaw = (math.radians(value) for value in rpy)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def _validate(layout: dict[str, Any]) -> None:
    try:
        import jsonschema
    except ImportError as exc:
        raise RuntimeError(
            "jsonschema is required to validate cell layouts") from exc
    schema_path = _repo_root() / "config/layouts/schema.json"
    schema = json.loads(schema_path.read_text())
    jsonschema.validate(layout, schema)
    object_ids = [obj["id"] for obj in layout["objects"]]
    if len(object_ids) != len(set(object_ids)):
        raise ValueError("layout object ids must be unique")
    if layout["task_container_id"] not in object_ids:
        raise ValueError("task_container_id is not an object in this layout")


def _resolve_rel(source: Path, value: str) -> str:
    candidate = Path(value)
    return str(
        candidate if candidate.is_absolute()
        else (source.parent / candidate).resolve()
    )


def load_layout(path: str | Path) -> dict[str, Any]:
    """Load and validate a layout, expanding asset paths and orientations."""
    source = Path(path).resolve()
    with source.open() as stream:
        layout = yaml.safe_load(stream)
    if not isinstance(layout, dict):
        raise ValueError(f"layout must be a mapping: {source}")
    _validate(layout)
    normalized = deepcopy(layout)
    normalized["_source_path"] = str(source)
    normalized["poses_yaml"] = _resolve_rel(source, normalized["poses_yaml"])
    resolved_targets: dict[str, str] = {}
    for robot_key, rel in (normalized.get("targets_by_robot") or {}).items():
        resolved_targets[str(robot_key)] = _resolve_rel(source, str(rel))
    normalized["targets_by_robot"] = resolved_targets
    for obj in normalized["objects"]:
        orientation = obj["orientation"]
        obj["quat_xyzw"] = (
            orientation.get("quat_xyzw")
            or _quaternion_from_rpy_deg(orientation["rpy_deg"])
        )
        unit_scale = 0.001 if obj.get("mesh_units", "m") == "mm" else 1.0
        obj["resolved_scale_xyz"] = [
            unit_scale * value for value in obj["scale_xyz"]
        ]
    return normalized


def targets_for_robot(layout: dict[str, Any], robot_key: str) -> str:
    """Return absolute targets YAML path for robot_key or raise KeyError."""
    mapping = layout.get("targets_by_robot") or {}
    if robot_key not in mapping:
        commissioned = ", ".join(sorted(mapping)) or "(none)"
        raise KeyError(
            f"layout '{layout.get('layout_id')}' is not commissioned for "
            f"robot '{robot_key}' (commissioned: {commissioned})"
        )
    return str(mapping[robot_key])


def layout_hash(layout: dict[str, Any]) -> str:
    """Stable provenance hash excluding loader-only fields."""
    canonical = {
        key: value for key, value in layout.items() if not key.startswith("_")
    }
    payload = json.dumps(
        canonical, sort_keys=True, separators=(",", ":")
    ).encode()
    return hashlib.sha256(payload).hexdigest()


def mode_to_layout_id(mode: str, profiles_yaml: str | Path) -> str:
    profiles = yaml.safe_load(Path(profiles_yaml).read_text()) or {}
    mappings = profiles.get("mode_layouts", {})
    if mode not in mappings:
        raise KeyError(f"no layout configured for mode '{mode}'")
    return mappings[mode]


def configured_layout_id(mode: str) -> str:
    """Resolve a runtime mode's layout from env overrides or profiles.yaml."""
    for name in ("MODE_LAYOUTS", "CELL_LAYOUTS"):
        raw = os.environ.get(name, "").strip()
        if not raw:
            continue
        try:
            mappings = json.loads(raw)
        except json.JSONDecodeError:
            mappings = dict(
                item.split("=", 1) for item in raw.split(",") if "=" in item
            )
        if isinstance(mappings, dict) and mappings.get(mode):
            return str(mappings[mode])

    profiles = Path(
        os.environ.get("PROFILES_YAML", "/ws/config/profiles.yaml")
    )
    if not profiles.is_file():
        profiles = _repo_root() / "config/profiles.yaml"
    return mode_to_layout_id(mode, profiles)


def layouts_dir() -> Path:
    """Resolve the layouts directory (env override, then repo checkout)."""
    configured = Path(os.environ.get("CELL_LAYOUTS_DIR", "/ws/config/layouts"))
    if configured.is_dir():
        return configured.resolve()
    return (_repo_root() / "config/layouts").resolve()


def layout_path(layout_id: str) -> Path:
    """Return the configured layout file, rejecting traversal outside the dir."""
    root = layouts_dir()
    candidate = (root / f"{layout_id}.yaml").resolve()
    if candidate.parent != root:
        raise ValueError("layout_id must name a top-level layout")
    return candidate


def list_layouts() -> list[dict[str, Any]]:
    """Enumerate top-level layout YAML files with hashes."""
    root = layouts_dir()
    items: list[dict[str, Any]] = []
    if not root.is_dir():
        return items
    for path in sorted(root.glob("*.yaml")):
        try:
            layout = load_layout(path)
        except Exception as exc:  # noqa: BLE001 — skip invalid; list the rest
            items.append(
                {
                    "layout_id": path.stem,
                    "layout_hash": None,
                    "error": str(exc),
                }
            )
            continue
        items.append(
            {
                "layout_id": str(layout["layout_id"]),
                "layout_hash": layout_hash(layout),
                "tool_id": str(layout.get("tool_id") or ""),
                "task_container_id": str(layout.get("task_container_id") or ""),
                "commissioned_robots": sorted(
                    (layout.get("targets_by_robot") or {}).keys()
                ),
            }
        )
    return items


def layout_provenance(
    layout: dict[str, Any], *, robot_key: str = ""
) -> dict[str, Any]:
    """Return the layout metadata stored with a run."""
    poses = yaml.safe_load(Path(layout["poses_yaml"]).read_text()) or {}
    authored_in = str(poses.get("authored_in") or "")
    poses_robot_key = str(poses.get("robot_key") or "")
    poses_provenance_ok = (
        poses.get("layout_id") == layout["layout_id"]
        and poses.get("tool_id") == layout.get("tool_id")
        and bool(authored_in)
    )
    if robot_key and poses_robot_key and poses_robot_key != robot_key:
        poses_provenance_ok = False
    return {
        "layout_id": str(layout["layout_id"]),
        "layout_hash": layout_hash(layout),
        "poses_hash": hashlib.sha256(
            Path(layout["poses_yaml"]).read_bytes()
        ).hexdigest(),
        "tool_id": str(layout.get("tool_id") or ""),
        "authored_in": authored_in,
        "robot_key": robot_key or poses_robot_key,
        "poses_provenance_ok": poses_provenance_ok,
        "commissioned_robots": sorted(
            (layout.get("targets_by_robot") or {}).keys()
        ),
    }


def provenance_is_safe(
    provenance: dict[str, Any], *, environment: str, production_mode: bool
) -> tuple[bool, str]:
    """Validate the minimal provenance required before starting a run."""
    required = ("layout_id", "layout_hash", "poses_hash", "tool_id")
    missing = [key for key in required if not provenance.get(key)]
    if missing:
        return False, f"layout provenance missing: {', '.join(missing)}"
    if not provenance.get("poses_provenance_ok", True):
        return False, "layout poses provenance check failed"
    if production_mode and environment == "real" and provenance.get("authored_in") != "real":
        return False, "real production run requires poses authored_in=real"
    return True, ""
