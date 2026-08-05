#!/usr/bin/env python3
"""CI guard: every layout composes a Gazebo world containing all enabled objects."""
from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = (
    ROOT / "src/scooping_controller/worlds/scooping_container_world.sdf"
)
GEN_SPEC = importlib.util.spec_from_file_location(
    "generate_layout_world", ROOT / "scripts/generate_layout_world.py"
)
assert GEN_SPEC and GEN_SPEC.loader
GEN = importlib.util.module_from_spec(GEN_SPEC)
sys.modules[GEN_SPEC.name] = GEN
GEN_SPEC.loader.exec_module(GEN)


def main() -> int:
    if not TEMPLATE.is_file():
        print(f"missing world template: {TEMPLATE}", file=sys.stderr)
        return 1
    template = TEMPLATE.read_text()
    layouts = sorted((ROOT / "config/layouts").glob("*.yaml"))
    if not layouts:
        print("no layouts found", file=sys.stderr)
        return 1
    for path in layouts:
        layout = yaml.safe_load(path.read_text())
        world = GEN.compose_world(template, layout)
        GEN.assert_world_contains_layout(world, layout)
        print(f"parity ok: {path.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
