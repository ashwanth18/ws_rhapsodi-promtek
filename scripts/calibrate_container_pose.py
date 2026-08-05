#!/usr/bin/env python3
"""Fit a container pose from three touched TCP points without jogging a robot.

Example:
  scripts/calibrate_container_pose.py config/layouts/dual-container.yaml \
    rs6 '[0.40,-0.10,0.12]' '[0.50,-0.10,0.12]' '[0.40,0.00,0.12]' \
    --operator ash

The script only edits YAML. Collect touch points through an approved, supervised
authoring workflow; it never creates ROS nodes or commands motion.
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import math
from pathlib import Path

import yaml


def point(value: str) -> list[float]:
    parsed = json.loads(value)
    if not isinstance(parsed, list) or len(parsed) != 3:
        raise argparse.ArgumentTypeError("point must be a JSON [x,y,z] array")
    return [float(v) for v in parsed]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("layout", type=Path)
    parser.add_argument("object_id")
    parser.add_argument("p1", type=point)
    parser.add_argument("p2", type=point)
    parser.add_argument("p3", type=point)
    parser.add_argument("--operator", required=True)
    args = parser.parse_args()
    points = [args.p1, args.p2, args.p3]
    centroid = [sum(p[i] for p in points) / 3 for i in range(3)]
    ux = [args.p2[i] - args.p1[i] for i in range(3)]
    uy = [args.p3[i] - args.p1[i] for i in range(3)]
    normal = [
        ux[1] * uy[2] - ux[2] * uy[1],
        ux[2] * uy[0] - ux[0] * uy[2],
        ux[0] * uy[1] - ux[1] * uy[0],
    ]
    magnitude = math.sqrt(sum(v * v for v in normal))
    if magnitude < 1e-9:
        raise SystemExit("touch points are collinear")
    normal = [v / magnitude for v in normal]
    yaw = math.degrees(math.atan2(ux[1], ux[0]))
    residual = max(abs(sum((p[i] - centroid[i]) * normal[i] for i in range(3))) for p in points)

    layout = yaml.safe_load(args.layout.read_text())
    obj = next((item for item in layout["objects"] if item["id"] == args.object_id), None)
    if obj is None:
        raise SystemExit(f"object not found: {args.object_id}")
    obj["position_xyz"] = centroid
    obj["orientation"] = {"rpy_deg": [0.0, 0.0, yaw]}
    obj["calibration"] = {
        "method": "touch_off_3point",
        "date": dt.date.today().isoformat(),
        "operator": args.operator,
        "residual_m": residual,
        "plane_normal": normal,
    }
    args.layout.write_text(yaml.safe_dump(layout, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
