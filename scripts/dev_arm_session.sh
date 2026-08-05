#!/usr/bin/env bash
# Supervised real-arm authoring session. This never sends a motion command.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAYOUT_ID="${1:-${LAYOUT:-dual-container}}"
ROBOT="${2:-${ROBOT_TYPE:-niryo}}"
PI_HOST="${ROBOT_HOST:-rhapsodi-pi5}"
PI_SSH="${ROBOT_SSH:-admin@${PI_HOST}}"
PI_HTTP="${ROBOT_HTTP_URL:-http://${PI_HOST}:8000}"
DEST="${ROBOT_WORKSPACE:-/opt/rhapsodi/ws_rhapsodi-promtek}"

if [[ ! -f "$ROOT/config/layouts/${LAYOUT_ID}.yaml" ]]; then
  echo "Unknown layout_id='$LAYOUT_ID' (expected $ROOT/config/layouts/${LAYOUT_ID}.yaml)" >&2
  exit 1
fi

# Match ansible/fleet-agent: env-file supplies COMPOSE_FILE=compose/devices/pi5.yml.
pi_compose() {
  # shellcheck disable=SC2029
  ssh "$PI_SSH" \
    "cd '$DEST' && docker compose --project-directory '$DEST' --env-file robot-prod.env $*"
}

active="$(curl --fail --silent --show-error "${PI_HTTP}/runtime/mode")"
if [[ "$active" != *'"active_run":null'* ]]; then
  echo "Refusing arm session: robot has an active run: $active" >&2
  exit 1
fi

pi_compose stop scooping_stack
if pi_compose ps --status running -q scooping_stack | grep -q .; then
  echo "Refusing arm session: scooping_stack is still running" >&2
  exit 1
fi

cleanup() {
  pi_compose up -d scooping_stack || true
}
trap cleanup EXIT INT TERM

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROBOT_IP="${ROBOT_IP:-169.254.200.200}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export ROS_STATIC_PEERS="${ROS_STATIC_PEERS:-169.254.200.201}"
export FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
# ROS setup.bash references optional vars (e.g. AMENT_TRACE_SETUP_FILES); with
# `set -u` that aborts unless we temporarily allow unbound variables.
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
[[ -f "$ROOT/install/setup.bash" ]] && source "$ROOT/install/setup.bash"
set -u

# ros2cli daemon caches graph state across domain peers; after stopping Pi
# scooping_stack it can still report /move_to until flushed.
ros2 daemon stop >/dev/null 2>&1 || true
moved_clear=0
for _ in $(seq 1 40); do
  if ! ros2 action list 2>/dev/null | grep -qx '/move_to'; then
    moved_clear=1
    break
  fi
  sleep 0.25
done
if [[ "$moved_clear" -ne 1 ]]; then
  echo "Refusing arm session: /move_to still visible after stopping scooping_stack." >&2
  echo "Kill a leftover laptop authoring stack (rviz2 / ros2 launch), then retry." >&2
  pgrep -af 'rviz2|scooping_real|scooping_marker_server|move_to_server_node' >&2 || true
  exit 1
fi

echo "Launching authoring stack only; no motion is commanded by this script."
echo "dev_arm_session: layout_id=$LAYOUT_ID robot=$ROBOT (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
export CELL_MODELS_CATALOG="${CELL_MODELS_CATALOG:-$ROOT/config/models/catalog.yaml}"
export ROBOT_TYPE="${ROBOT_TYPE:-$ROBOT}"
ros2 launch scooping_controller scooping_real.launch.py \
  use_rviz:=true \
  layout_edit:=true \
  robot:="$ROBOT" \
  layout_id:="$LAYOUT_ID" \
  layouts_dir:="${CELL_LAYOUTS_DIR:-$ROOT/config/layouts}"
