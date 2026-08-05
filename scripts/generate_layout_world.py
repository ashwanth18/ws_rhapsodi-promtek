#!/usr/bin/env python3
"""Emit a Gazebo SDF include fragment (or full world) from a cell layout."""
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
import sys

import yaml


def rpy_from_quat(q: list[float]) -> tuple[float, float, float]:
    x, y, z, w = q
    return (
        math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
        math.asin(max(-1, min(1, 2 * (w * y - z * x)))),
        math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)),
    )


def object_pose(obj: dict) -> str:
    x, y, z = obj["position_xyz"]
    orient = obj["orientation"]
    rpy = orient.get("rpy_deg")
    if rpy:
        rpy = [math.radians(value) for value in rpy]
    else:
        rpy = rpy_from_quat(orient["quat_xyzw"])
    return f"{x} {y} {z} {rpy[0]} {rpy[1]} {rpy[2]}"


def layout_models_sdf(layout: dict) -> str:
    blocks = [
        f"<!-- GENERATED from layout_id={layout.get('layout_id')}; "
        "do not hand-edit world placements. -->"
    ]
    for obj in layout["objects"]:
        if not obj["enabled"]:
            continue
        pose = object_pose(obj)
        if obj["geometry_type"] == "box":
            size = " ".join(str(value) for value in obj["dimensions_xyz"])
            blocks.append(
                f'<model name="{obj["id"]}"><static>true</static>'
                f"<pose>{pose}</pose><link name=\"link\">"
                f'<visual name="visual"><geometry><box><size>{size}</size>'
                f"</box></geometry></visual>"
                f'<collision name="collision"><geometry><box><size>{size}</size>'
                f"</box></geometry></collision></link></model>"
            )
        else:
            blocks.append(
                f"<include><uri>model://{obj['id']}</uri>"
                f"<name>{obj['id']}</name><pose>{pose}</pose></include>"
            )
    return "\n".join(blocks) + "\n"


def compose_world(template_text: str, layout: dict) -> str:
    """Insert generated layout models before </world> in a template SDF."""
    models = layout_models_sdf(layout)
    marker = "<!-- LAYOUT_OBJECTS -->"
    if marker in template_text:
        return template_text.replace(marker, models.rstrip("\n"))
    if "</world>" not in template_text:
        raise ValueError("template SDF missing </world>")
    return template_text.replace("</world>", models + "</world>", 1)


def enabled_object_ids(layout: dict) -> list[str]:
    return [obj["id"] for obj in layout["objects"] if obj.get("enabled")]


def assert_world_contains_layout(world_text: str, layout: dict) -> None:
    """Raise AssertionError if any enabled layout object is missing from world."""
    for obj in layout["objects"]:
        if not obj.get("enabled"):
            continue
        obj_id = obj["id"]
        if obj["geometry_type"] == "box":
            pattern = rf'<model name="{re.escape(obj_id)}"'
        else:
            pattern = (
                rf'<include>\s*<uri>model://{re.escape(obj_id)}</uri>'
                rf'.*?<name>{re.escape(obj_id)}</name>'
            )
        if not re.search(pattern, world_text, re.S):
            raise AssertionError(f"{obj_id} missing from generated world")
        # Pose xyz must match layout (first three floats of pose).
        pose_match = re.search(
            rf'(?:name="{re.escape(obj_id)}"[^>]*>|<name>{re.escape(obj_id)}</name>)\s*'
            rf'.*?<pose>([^<]+)</pose>',
            world_text,
            re.S,
        )
        if not pose_match:
            raise AssertionError(f"{obj_id} pose missing from generated world")
        values = [float(value) for value in pose_match.group(1).split()[:3]]
        expected = [float(value) for value in obj["position_xyz"]]
        if values != expected:
            raise AssertionError(
                f"{obj_id} pose differs: {values} != {expected}"
            )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("layout", type=Path, help="Path to layout YAML")
    parser.add_argument("--output", type=Path, help="Write fragment or world here")
    parser.add_argument(
        "--template",
        type=Path,
        help="Optional SDF template; when set, emit a full world with layout objects",
    )
    args = parser.parse_args()
    layout = yaml.safe_load(args.layout.read_text())
    if args.template:
        output = compose_world(args.template.read_text(), layout)
    else:
        output = layout_models_sdf(layout)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output)
    else:
        print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
