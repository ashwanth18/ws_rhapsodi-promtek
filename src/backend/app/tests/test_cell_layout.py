from pathlib import Path
import importlib.util

ROOT = Path(__file__).resolve().parents[4]
SPEC = importlib.util.spec_from_file_location(
    "cell_layout", ROOT / "src/backend/app/modes/cell_layout.py")
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
layout_hash = MODULE.layout_hash
load_layout = MODULE.load_layout
mode_to_layout_id = MODULE.mode_to_layout_id


def test_dual_container_normalizes_mm_mesh_and_rpy():
    layout = load_layout(ROOT / "config/layouts/dual-container.yaml")
    vessel = next(obj for obj in layout["objects"] if obj["id"] == "scooping_container")
    assert vessel["resolved_scale_xyz"] == [0.001, 0.001, 0.001]
    assert vessel["quat_xyzw"][2] == 1.0
    assert len(layout_hash(layout)) == 64


def test_mode_layout_mapping():
    assert mode_to_layout_id("lightsout", ROOT / "config/profiles.yaml") == "lightsout-single-vessel"
