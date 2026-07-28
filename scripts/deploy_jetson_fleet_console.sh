#!/usr/bin/env bash
# Deploy Fleet Console (and optionally hub monitoring) to the Jetson.
#
# Prefer Tailscale IP so LAN DNS/ssh Host overrides do not time out.
#
# Usage (from laptop repo):
#   bash scripts/deploy_jetson_fleet_console.sh
#   JETSON_BRANCH=feature/fleet-console bash scripts/deploy_jetson_fleet_console.sh
#   WITH_MONITORING=1 bash scripts/deploy_jetson_fleet_console.sh
set -euo pipefail

JETSON_USER="${JETSON_USER:-ashwanth}"
JETSON_HOST="${JETSON_HOST:-}"
if [[ -z "${JETSON_HOST}" ]]; then
  JETSON_HOST="$(tailscale ip -4 jetson 2>/dev/null || true)"
fi
JETSON_HOST="${JETSON_HOST:-jetson}"

JETSON_REPO="${JETSON_REPO:-/home/ashwanth/ws_rhapsodi-promtek-dev}"
JETSON_BRANCH="${JETSON_BRANCH:-feature/fleet-console}"
WITH_MONITORING="${WITH_MONITORING:-0}"
SSH=(ssh -o ConnectTimeout=20 -o BatchMode=yes "${JETSON_USER}@${JETSON_HOST}")

echo "Deploying to ${JETSON_USER}@${JETSON_HOST}:${JETSON_REPO} (branch ${JETSON_BRANCH})"

"${SSH[@]}" bash -s -- "${JETSON_REPO}" "${JETSON_BRANCH}" "${WITH_MONITORING}" <<'REMOTE'
set -euo pipefail
REPO="$1"
BRANCH="$2"
WITH_MONITORING="$3"
cd "${REPO}"
git fetch origin
git checkout "${BRANCH}"
git pull --ff-only origin "${BRANCH}"
cd monitoring
if [[ ! -f fleet-console.env ]]; then
  echo "missing monitoring/fleet-console.env on Jetson" >&2
  exit 1
fi
docker compose -f docker-compose.fleet-console.yml --env-file fleet-console.env up -d --build
if [[ "${WITH_MONITORING}" == "1" ]]; then
  if [[ -f monitoring.env ]]; then
    docker compose -f docker-compose.monitoring.yml --env-file monitoring.env up -d
  else
    docker compose -f docker-compose.monitoring.yml up -d
  fi
fi
docker ps --filter name=fleet-console --format '{{.Names}} {{.Status}} {{.Image}}'
echo "Fleet Console: http://jetson:8090 (or http://$(tailscale ip -4 2>/dev/null || hostname -I | awk '{print $1}'):8090)"
REMOTE
