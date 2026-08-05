#!/usr/bin/env bash
# Laptop Gazebo sim loop for a robot + cell layout.
# Isolates the ROS graph on domain 43 so it cannot collide with bench (42) or a live cell (0).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT="${1:-niryo}"
LAYOUT_ID="${2:-dual-container}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-43}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
export CELL_LAYOUTS_DIR="${CELL_LAYOUTS_DIR:-$ROOT/config/layouts}"
export ROBOT_TYPE="${ROBOT_TYPE:-$ROBOT}"

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

echo "dev_sim: ROS_DOMAIN_ID=$ROS_DOMAIN_ID robot=$ROBOT layout_id=$LAYOUT_ID"
echo "dev_sim: CELL_LAYOUTS_DIR=$CELL_LAYOUTS_DIR"
echo "dev_sim: Gazebo world composed from layout. Ctrl-C to stop."

exec ros2 launch scooping_controller scooping_simulation.launch.py \
  robot:="$ROBOT" \
  layout_id:="$LAYOUT_ID" \
  layouts_dir:="$CELL_LAYOUTS_DIR" \
  use_gazebo_gui:=true \
  headless:=false
