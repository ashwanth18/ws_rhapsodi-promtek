#!/usr/bin/env bash
# CI-only structural reachability gate. Full MoveIt plan-only runs on a bench.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$ROOT/scripts/validate_layouts.py"
python3 - "$ROOT" <<'PY'
import sys
from pathlib import Path
import yaml

root = Path(sys.argv[1])
required_markers = {
    "approach_marker", "contact_marker", "scoop_marker", "lift_marker",
    "transport_ready_marker",
}
required_targets = {
    "MoveToScoopingContainer", "MoveToWeighingContainer", "ReturnHome",
}
for path in sorted((root / "config/layouts").glob("*.yaml")):
    layout = yaml.safe_load(path.read_text())
    poses = yaml.safe_load((path.parent / layout["poses_yaml"]).read_text()) or {}
    targets = yaml.safe_load((path.parent / layout["targets_yaml"]).read_text()) or {}
    marker_names = {marker["name"] for marker in poses.get("markers", [])}
    missing = required_markers - marker_names
    if missing:
        raise SystemExit(f"{path}: missing pose markers {sorted(missing)}")
    missing = required_targets - set(targets.get("targets", {}))
    if missing:
        raise SystemExit(f"{path}: missing targets {sorted(missing)}")
    envelope = layout["scoop_envelope"]
    for marker in poses["markers"]:
        position = marker["pose"]["position"]
        if not (envelope["x_min"] <= position["x"] <= envelope["x_max"]
                and envelope["y_min"] <= position["y"] <= envelope["y_max"]
                and envelope["z_min"] <= position["z"] <= envelope["z_max"]):
            raise SystemExit(f"{path}: marker {marker['name']} outside scoop_envelope")
print("layout structural reachability gate passed")
PY
