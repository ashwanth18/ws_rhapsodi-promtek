#!/usr/bin/env bash
# Generate Prometheus file_sd targets from Tailscale robots.
# Run on the monitoring host (Jetson) periodically or before compose up.
#
# Usage:
#   bash scripts/generate_prom_targets.sh
#   # writes monitoring/prometheus/targets-node.json and targets-cadvisor.json
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_NODE="${ROOT_DIR}/monitoring/prometheus/targets-node.json"
OUT_CAD="${ROOT_DIR}/monitoring/prometheus/targets-cadvisor.json"
ROBOT_TAG="${TAILSCALE_ROBOT_TAG:-tag:robot}"

python3 - <<PY
import json, os, subprocess
root = "${ROOT_DIR}"
robot_tag = "${ROBOT_TAG}"
data = json.loads(subprocess.check_output(["tailscale", "status", "--json"], text=True))
devices = []
if data.get("Self"):
    devices.append(data["Self"])
devices.extend((data.get("Peer") or {}).values())

node_targets = []
cad_targets = []
for d in devices:
    tags = d.get("Tags") or []
    name = (d.get("DNSName") or d.get("HostName") or "").rstrip(".").split(".")[0]
    addrs = d.get("TailscaleIPs") or []
    ip = next((a for a in addrs if ":" not in a), None)
    if not name or not ip:
        continue
    is_robot = robot_tag in tags or name.startswith("rhapsodi") or name.endswith("-pi5") or "robot" in name
    is_builder = name == "jetson" or any(t.endswith(":builder") for t in tags)
    if not (is_robot or is_builder):
        continue
    site = "home" if is_builder or name == "jetson" else "field"
    node_targets.append({
        "targets": [f"{ip}:9100"],
        "labels": {"instance": name, "site": site, "role": "node"},
    })
    cad_targets.append({
        "targets": [f"{ip}:9190"],
        "labels": {"instance": name, "site": site, "role": "cadvisor"},
    })

# Always include localhost for the monitoring host itself if empty
if not node_targets:
    node_targets = [{"targets": ["127.0.0.1:9100"], "labels": {"instance": "local", "site": "home", "role": "node"}}]
    cad_targets = [{"targets": ["127.0.0.1:9190"], "labels": {"instance": "local", "site": "home", "role": "cadvisor"}}]

open("${OUT_NODE}", "w").write(json.dumps(node_targets, indent=2) + "\n")
open("${OUT_CAD}", "w").write(json.dumps(cad_targets, indent=2) + "\n")
print(f"Wrote {len(node_targets)} node targets and {len(cad_targets)} cadvisor targets")
PY
