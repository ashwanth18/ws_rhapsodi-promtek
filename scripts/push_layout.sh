#!/usr/bin/env bash
# DEV FAST-PATH ONLY — bypasses release provenance.
#
# Prefer the professional path:
#   1. commit config/layouts/**
#   2. CI validates
#   3. release ships the bundle
#   4. Fleet Console → Apply layout (fleet-agent → :8010/apply_cell_layout)
#
# This script is analogous to LOCAL_SYNC=1 on the Jetson deploy helper: useful
# for laptop→Pi iteration before a release, not for production rollout.
# It never moves the arm.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI_HOST="${ROBOT_HOST:-niryo-rhapsodi}"
PI_SSH="${ROBOT_SSH:-admin@${PI_HOST}}"
DEST="${ROBOT_WORKSPACE:-/opt/rhapsodi/ws_rhapsodi-promtek}"
ADAPTER_URL="${ROBOT_LAYOUT_ADAPTER_URL:-http://${PI_HOST}:8010/apply_cell_layout}"

echo "WARNING: push_layout.sh bypasses release provenance (dev fast-path)." >&2
echo "         Prefer Fleet Console Apply layout after commit/release." >&2

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
