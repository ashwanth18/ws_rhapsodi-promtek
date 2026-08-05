#!/usr/bin/env bash
# Save a timestamped scoop pose set under
# config/layouts/<layout_id>/poses/sets/ (default poses.yaml is never overwritten).
# Requires a running scooping_marker_server (bench or arm-session).
# Never commands arm motion.
set -euo pipefail

NOTE="${1:-}"
echo "Calling /save_scoop_pose_set (timestamped set under poses/sets/; default intact)..."
ros2 service call /save_scoop_pose_set robot_common_msgs/srv/SaveScoopPoseSet "{note: '${NOTE}'}"
