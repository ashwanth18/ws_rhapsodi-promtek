#!/usr/bin/env python3
"""Validate every versioned cell layout against schema + model catalog."""
from __future__ import annotations

import importlib.util
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "cell_layout", ROOT / "src/backend/app/modes/cell_layout.py")
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
load_layout = MODULE.load_layout

PACKAGE_MESH_RE = re.compile(r"^package://([^/]+)/(.+)$")


def load_catalog() -> dict[str, dict]:
    catalog_path = ROOT / "config/models/catalog.yaml"
    if not catalog_path.is_file():
        raise FileNotFoundError(f"missing model catalog: {catalog_path}")
    data = yaml.safe_load(catalog_path.read_text()) or {}
    models = {}
    for entry in data.get("models") or []:
        models[str(entry["model_id"])] = entry
    return models


def resolve_mesh_resource(mesh_resource: str) -> Path | None:
    match = PACKAGE_MESH_RE.match(mesh_resource)
    if not match:
        return None
    package, rel = match.group(1), match.group(2)
    # Prefer source tree for scooping_controller meshes.
    if package == "scooping_controller":
        candidate = ROOT / "src/scooping_controller" / rel
        if candidate.is_file():
            return candidate
    return None


def catalog_mesh_resources(catalog: dict[str, dict]) -> set[str]:
    return {
        str(entry.get("mesh_resource") or "")
        for entry in catalog.values()
        if entry.get("geometry_type") == "mesh" and entry.get("mesh_resource")
    }


def validate_extra(path: Path, layout: dict, catalog: dict[str, dict]) -> list[str]:
    errors: list[str] = []
    object_ids = [obj["id"] for obj in layout["objects"]]
    if len(object_ids) != len(set(object_ids)):
        errors.append("object ids must be unique")
    if layout["task_container_id"] not in object_ids:
        errors.append("task_container_id is not an object in this layout")

    known_meshes = catalog_mesh_resources(catalog)
    for obj in layout["objects"]:
        if obj.get("geometry_type") != "mesh":
            continue
        mesh = str(obj.get("mesh_resource") or "")
        if not mesh:
            errors.append(f"{obj['id']}: mesh object missing mesh_resource")
            continue
        if mesh not in known_meshes:
            errors.append(
                f"{obj['id']}: mesh_resource not listed in config/models/catalog.yaml: {mesh}"
            )
        resolved = resolve_mesh_resource(mesh)
        if resolved is None or not resolved.is_file():
            errors.append(f"{obj['id']}: mesh_resource does not resolve to a file: {mesh}")

    # Production layouts should carry calibration provenance (layout-level or
    # per-object). Soft warning becomes an error when CALIBRATION_REQUIRED=1.
    has_cal = bool(layout.get("calibration"))
    has_obj_cal = any(bool(obj.get("calibration")) for obj in layout["objects"])
    if not has_cal and not has_obj_cal:
        import os

        if os.environ.get("CALIBRATION_REQUIRED", "").strip() in {"1", "true", "yes"}:
            errors.append("missing calibration provenance (layout or object level)")
    return errors


def main() -> int:
    try:
        catalog = load_catalog()
    except Exception as exc:  # noqa: BLE001
        print(f"catalog error: {exc}", file=sys.stderr)
        return 1

    layouts = sorted((ROOT / "config/layouts").glob("*.yaml"))
    failures: list[str] = []
    for path in layouts:
        try:
            layout = load_layout(path)
            extra = validate_extra(path, layout, catalog)
            if extra:
                raise ValueError("; ".join(extra))
            print(f"valid: {path.relative_to(ROOT)}")
        except Exception as exc:  # report all invalid layouts in CI
            failures.append(f"{path}: {exc}")
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
