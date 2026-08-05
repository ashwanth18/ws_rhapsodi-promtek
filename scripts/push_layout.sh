#!/usr/bin/env bash
# Copy versioned layout inputs to a Pi and request an activation; never moves the arm.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI_HOST="${ROBOT_HOST:-niryo-rhapsodi}"
PI_SSH="${ROBOT_SSH:-admin@${PI_HOST}}"
DEST="${ROBOT_WORKSPACE:-/opt/rhapsodi/ws_rhapsodi-promtek}"
ADAPTER_URL="${ROBOT_LAYOUT_ADAPTER_URL:-http://${PI_HOST}:8010/apply_cell_layout}"

if [[ "${1:-}" == "--revert" ]]; then
  ssh "$PI_SSH" "set -e; cd '$DEST/config'; test -d layouts/.prev; rm -rf layouts; mv layouts/.prev layouts"
  echo "Restored $PI_HOST config/layouts from layouts/.prev; apply a layout explicitly."
  exit 0
fi

LAYOUT_ID="${1:?usage: $0 <layout_id> | --revert}"
test -f "$ROOT/config/layouts/${LAYOUT_ID}.yaml"
ssh "$PI_SSH" "set -e; cd '$DEST/config'; rm -rf layouts/.prev; test ! -d layouts || cp -a layouts layouts/.prev"
rsync -a --delete "$ROOT/config/layouts/" "$PI_SSH:$DEST/config/layouts/"
rsync -a "$ROOT/config/profiles.yaml" "$PI_SSH:$DEST/config/profiles.yaml"
curl --fail --silent --show-error \
  -H 'content-type: application/json' \
  -d "{\"layout_id\":\"${LAYOUT_ID}\"}" \
  "$ADAPTER_URL"
echo
