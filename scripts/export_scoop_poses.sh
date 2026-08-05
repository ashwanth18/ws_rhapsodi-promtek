#!/usr/bin/env bash
# Export authored scoop poses into the versioned layout tree
# (config/layouts/<layout_id>/poses.yaml) with provenance stamps.
# Requires a running scooping_marker_server (bench or arm-session).
# Never commands arm motion.
set -euo pipefail

echo "Calling /export_scoop_poses (writes config/layouts/<layout_id>/poses.yaml)..."
ros2 service call /export_scoop_poses std_srvs/srv/Trigger {}
