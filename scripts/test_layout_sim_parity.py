#!/usr/bin/env python3
"""CI guard that keeps the deprecated handwritten world aligned to dual-container."""
from pathlib import Path
import re
import sys
import yaml

ROOT = Path(__file__).resolve().parents[1]
layout = yaml.safe_load((ROOT / "config/layouts/dual-container.yaml").read_text())
world = (ROOT / "src/scooping_controller/worlds/scooping_container_world.sdf").read_text()

for obj in layout["objects"]:
    if not obj["enabled"] or obj["geometry_type"] != "mesh":
        continue
    match = re.search(rf'<include>\s*<uri>model://{re.escape(obj["id"])}</uri>.*?<pose>([^<]+)</pose>', world, re.S)
    if not match:
        raise AssertionError(f"{obj['id']} missing from handwritten world")
    values = [float(value) for value in match.group(1).split()[:3]]
    assert values == obj["position_xyz"], f"{obj['id']} pose differs: {values}"
print("dual-container layout and handwritten world placements agree")
