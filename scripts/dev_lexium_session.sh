#!/usr/bin/env bash
# Native RViz session against a laptop Lexium/Jaka cell.
#
# Compose stays headless (use_rviz:=false). Containers use network_mode: host,
# so host RViz shares the laptop ROS graph with scooping_stack / lexium_driver.
# Never sends a motion command — Bring Up / STOP live in the Lexium Safety panel.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT="${1:-${ROBOT_TYPE:-jaka}}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
[[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
set -u

if ! ros2 pkg prefix lexium_rviz_plugins >/dev/null 2>&1; then
  echo "lexium_rviz_plugins not in the ROS overlay." >&2
  echo "Build once:" >&2
  echo "  cd $ROOT && colcon build --packages-select lexium_msgs lexium_rviz_plugins" >&2
  exit 1
fi

echo "Launching scooping RViz for robot=${ROBOT} (Lexium Safety panel in scooping_jaka.rviz)."
echo "Ensure scooping_stack is up on ROS_DOMAIN_ID=${ROS_DOMAIN_ID}."
exec ros2 launch scooping_controller scooping_rviz_only.launch.py "robot:=${ROBOT}"
