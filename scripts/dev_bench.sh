#!/usr/bin/env bash
# Laptop-only scooping authoring loop (mock hardware, no Gazebo, no Pi).
# Isolates the ROS graph on domain 42 so it cannot collide with a live cell.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAYOUT_ID="${1:-dual-container}"
ROBOT="${2:-${ROBOT:-niryo}}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export CELL_LAYOUTS_DIR="${CELL_LAYOUTS_DIR:-$ROOT/config/layouts}"
export ROBOT_TYPE="${ROBOT_TYPE:-$ROBOT}"

# Prefer workspace overlay when built locally.
# ROS setup.bash references optional vars (e.g. AMENT_TRACE_SETUP_FILES); with
# `set -u` that aborts unless we temporarily allow unbound variables.
source_ros() {
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  if [[ -f "$ROOT/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROOT/install/setup.bash"
  fi
  set -u
}
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  source_ros
else
  echo "ROS 2 Jazzy not found" >&2
  exit 1
fi

echo "dev_bench: ROS_DOMAIN_ID=$ROS_DOMAIN_ID robot=$ROBOT layout_id=$LAYOUT_ID"
echo "dev_bench: CELL_LAYOUTS_DIR=$CELL_LAYOUTS_DIR"
echo "dev_bench: isolated from Pi (domain 0). Ctrl-C to stop."

exec ros2 launch scooping_controller scooping_bench.launch.py \
  robot:="$ROBOT" \
  layout_id:="$LAYOUT_ID" \
  layouts_dir:="$CELL_LAYOUTS_DIR" \
  use_rviz:=true
