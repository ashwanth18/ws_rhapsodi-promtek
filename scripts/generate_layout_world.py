#!/usr/bin/env python3
"""Emit a Gazebo SDF include fragment from a versioned cell layout."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import yaml


def rpy_from_quat(q: list[float]) -> tuple[float, float, float]:
    x, y, z, w = q
    return (
        math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)),
        math.asin(max(-1, min(1, 2 * (w*y - z*x)))),
        math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z)),
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("layout", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    layout = yaml.safe_load(args.layout.read_text())
    blocks = ["<!-- GENERATED from %s; do not hand-edit world placements. -->" % args.layout.name]
    for obj in layout["objects"]:
        if not obj["enabled"]:
            continue
        x, y, z = obj["position_xyz"]
        orient = obj["orientation"]
        rpy = orient.get("rpy_deg")
        if rpy:
            rpy = [math.radians(value) for value in rpy]
        else:
            rpy = rpy_from_quat(orient["quat_xyzw"])
        pose = f"{x} {y} {z} {rpy[0]} {rpy[1]} {rpy[2]}"
        if obj["geometry_type"] == "box":
            size = " ".join(str(value) for value in obj["dimensions_xyz"])
            blocks.append(f"<model name=\"{obj['id']}\"><static>true</static><pose>{pose}</pose><link name=\"link\"><visual name=\"visual\"><geometry><box><size>{size}</size></box></geometry></visual><collision name=\"collision\"><geometry><box><size>{size}</size></box></geometry></collision></link></model>")
        else:
            blocks.append(f"<include><uri>model://{obj['id']}</uri><name>{obj['id']}</name><pose>{pose}</pose></include>")
    output = "\n".join(blocks) + "\n"
    if args.output:
        args.output.write_text(output)
    else:
        print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
