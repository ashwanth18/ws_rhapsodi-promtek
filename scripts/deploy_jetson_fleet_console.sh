#!/usr/bin/env bash
# Deploy Fleet Console (and optionally hub monitoring) to the Jetson.
#
# Prefer Tailscale IP so LAN DNS/ssh Host overrides do not time out.
# Syncs code via git pull on the Jetson (push your branch first). If the
# remote is unavailable, set LOCAL_SYNC=1 to rsync from this laptop instead.
#
# Usage (from laptop repo):
#   bash scripts/deploy_jetson_fleet_console.sh
#   JETSON_BRANCH=feature/fleet-console bash scripts/deploy_jetson_fleet_console.sh
#   WITH_MONITORING=1 bash scripts/deploy_jetson_fleet_console.sh
#   LOCAL_SYNC=1 bash scripts/deploy_jetson_fleet_console.sh
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JETSON_USER="${JETSON_USER:-ashwanth}"
JETSON_HOST="${JETSON_HOST:-}"
if [[ -z "${JETSON_HOST}" ]]; then
  JETSON_HOST="$(tailscale ip -4 jetson 2>/dev/null || true)"
fi
JETSON_HOST="${JETSON_HOST:-jetson}"

JETSON_REPO="${JETSON_REPO:-/home/ashwanth/ws_rhapsodi-promtek-dev}"
JETSON_BRANCH="${JETSON_BRANCH:-feature/fleet-console}"
WITH_MONITORING="${WITH_MONITORING:-0}"
LOCAL_SYNC="${LOCAL_SYNC:-0}"
SSH=(ssh -o ConnectTimeout=20 -o BatchMode=yes "${JETSON_USER}@${JETSON_HOST}")
RSYNC_SSH='ssh -o ConnectTimeout=20 -o BatchMode=yes'

echo "Deploying to ${JETSON_USER}@${JETSON_HOST}:${JETSON_REPO} (branch ${JETSON_BRANCH})"

if [[ "${LOCAL_SYNC}" == "1" ]]; then
  echo "LOCAL_SYNC=1 — rsyncing fleet_console + compose + profiles from ${ROOT}"
  rsync -az --delete \
    --exclude node_modules --exclude .venv --exclude '__pycache__' \
    -e "${RSYNC_SSH}" \
    "${ROOT}/src/fleet_console/" \
    "${JETSON_USER}@${JETSON_HOST}:${JETSON_REPO}/src/fleet_console/"
  scp -o ConnectTimeout=20 -o BatchMode=yes \
    "${ROOT}/monitoring/docker-compose.fleet-console.yml" \
    "${JETSON_USER}@${JETSON_HOST}:${JETSON_REPO}/monitoring/"
  scp -o ConnectTimeout=20 -o BatchMode=yes \
    "${ROOT}/scripts/deploy_jetson_fleet_console.sh" \
    "${JETSON_USER}@${JETSON_HOST}:${JETSON_REPO}/scripts/"
  # Profile env (DATA_OUTPUT_ROOT, SIM_ALLOWED, …) is read from /repo/config.
  scp -o ConnectTimeout=20 -o BatchMode=yes \
    "${ROOT}/config/profiles.yaml" \
    "${JETSON_USER}@${JETSON_HOST}:${JETSON_REPO}/config/profiles.yaml"
else
  "${SSH[@]}" bash -s -- "${JETSON_REPO}" "${JETSON_BRANCH}" <<'REMOTE'
set -euo pipefail
REPO="$1"
BRANCH="$2"
cd "${REPO}"
git fetch origin
git checkout "${BRANCH}"
# Discard dirty tree left by prior LOCAL_SYNC / hotfixes so pull can ff.
git reset --hard "origin/${BRANCH}"
git clean -fd -- scripts/deploy_jetson_fleet_console.sh src/fleet_console || true
git pull --ff-only origin "${BRANCH}"
REMOTE
fi

"${SSH[@]}" bash -s -- "${JETSON_REPO}" "${WITH_MONITORING}" <<'REMOTE'
set -euo pipefail
REPO="$1"
WITH_MONITORING="$2"
cd "${REPO}/monitoring"
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
echo "Fleet Console: http://jetson:8090"
REMOTE
