#!/usr/bin/env bash
# Start Fleet Console bound to all interfaces (Tailscale-reachable).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
CONSOLE_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$CONSOLE_DIR"

if [[ -f .venv/bin/activate ]]; then
  # shellcheck disable=SC1091
  source .venv/bin/activate
fi

export REPO_ROOT="${REPO_ROOT:-$ROOT}"
export ANSIBLE_DIR="${ANSIBLE_DIR:-$REPO_ROOT/ansible}"
export ANSIBLE_CONFIG="${ANSIBLE_CONFIG:-$ANSIBLE_DIR/ansible.cfg}"
export FLEET_DATA_DIR="${FLEET_DATA_DIR:-/tmp/fleet-console-data}"
export FLEET_LOG_DIR="${FLEET_LOG_DIR:-$FLEET_DATA_DIR/logs}"
export CI_REPORT_TOKEN="${CI_REPORT_TOKEN:-dev-ci-token}"
export FLEET_CONSOLE_URL="${FLEET_CONSOLE_URL:-http://$(hostname -I | awk '{print $1}'):8090}"
export PATH="${PATH}:/tmp/ansible-venv/bin"
mkdir -p "$FLEET_DATA_DIR" "$FLEET_LOG_DIR"

# Kill a stale listener on 8090 if present.
if command -v fuser >/dev/null 2>&1; then
  fuser -k 8090/tcp 2>/dev/null || true
fi

exec uvicorn app.main:app --host 0.0.0.0 --port 8090
