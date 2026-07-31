#!/usr/bin/env bash
set -euo pipefail

REGISTRY="${REGISTRY:-iserenity}"
REPOSITORY="${REPOSITORY:-rhapsodi-promtek}"
# Prefer the multi-node builder (laptop amd64 + Jetson arm64). Fall back to legacy name.
BUILDER_NAME="${BUILDER_NAME:-multiarch}"
PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS:-8}"
BUILD_CACHE_DIR="${BUILD_CACHE_DIR:-.buildx-cache}"
# registry = persist across GitHub Actions runs (recommended for CI).
# local = laptop-only .buildx-cache (lost on ephemeral runners).
# auto = registry when GITHUB_ACTIONS=true, else local.
BUILD_CACHE_BACKEND="${BUILD_CACHE_BACKEND:-auto}"
VITE_API_BASE="${VITE_API_BASE:-http://localhost:8000}"
VITE_ROSBRIDGE_URL="${VITE_ROSBRIDGE_URL:-ws://localhost:9090}"
VITE_MICROROS_HEARTBEAT_TOPIC="${VITE_MICROROS_HEARTBEAT_TOPIC:-/microros/heartbeat}"
PUSH_SHA_TAGS="${PUSH_SHA_TAGS:-1}"
TIMINGS_FILE="${TIMINGS_FILE:-build-timings.json}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

GIT_SHA="$(git rev-parse --short HEAD)"
DASHBOARD_APP_VERSION="$(node -p "require('./src/dashboard/package.json').version")"
echo "Building images for ${REGISTRY}/${REPOSITORY} @ ${GIT_SHA} (builder=${BUILDER_NAME})"
echo "Dashboard app version ${DASHBOARD_APP_VERSION}"

if [[ "${BUILD_CACHE_BACKEND}" == "auto" ]]; then
  if [[ "${GITHUB_ACTIONS:-}" == "true" ]]; then
    BUILD_CACHE_BACKEND=registry
  else
    BUILD_CACHE_BACKEND=local
  fi
fi
echo "Build cache backend: ${BUILD_CACHE_BACKEND}"

if docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
  docker buildx use "${BUILDER_NAME}"
elif docker buildx inspect multiarch-builder >/dev/null 2>&1; then
  echo "Builder '${BUILDER_NAME}' missing; falling back to multiarch-builder"
  BUILDER_NAME=multiarch-builder
  docker buildx use "${BUILDER_NAME}"
else
  echo "No buildx builder found. Run: bash scripts/setup_jetson_builder.sh"
  echo "Or create a local one: docker buildx create --name ${BUILDER_NAME} --use"
  exit 1
fi

if [[ "${BUILD_CACHE_BACKEND}" == "local" ]]; then
  mkdir -p "${BUILD_CACHE_DIR}"
fi

# role=seconds lines collected for JSON export
TIMING_LINES=()
BUILD_STARTED_AT="$(date +%s)"

# Tags: always push the role tag; optionally also push :<role>-<git-sha> for pin/rollback.
image_tags() {
  local role="$1"
  local tags=(-t "${REGISTRY}/${REPOSITORY}:${role}")
  if [[ "${PUSH_SHA_TAGS}" == "1" ]]; then
    tags+=(-t "${REGISTRY}/${REPOSITORY}:${role}-${GIT_SHA}")
  fi
  printf '%s\n' "${tags[@]}"
}

build_with_cache() {
  local role="$1"
  shift
  local -a tags
  local -a cache_args=()
  local start end elapsed
  local role_tag="${REGISTRY}/${REPOSITORY}:${role}"
  local cache_ref="${REGISTRY}/${REPOSITORY}:buildcache-${role}"
  mapfile -t tags < <(image_tags "${role}")
  start="$(date +%s)"
  echo "=== Building ${role} (platforms=${PLATFORMS}) ==="

  # Always try previous image layers (cheap if tag missing).
  cache_args+=(--cache-from="type=registry,ref=${role_tag}")

  if [[ "${BUILD_CACHE_BACKEND}" == "registry" ]]; then
    # Survives ephemeral GHA runners; per-role so ros-prod cache is not wiped by dashboard.
    cache_args+=(
      --cache-from="type=registry,ref=${cache_ref}"
      --cache-to="type=registry,ref=${cache_ref},mode=max"
    )
    echo "Using registry cache ${cache_ref}"
  else
    local role_cache="${BUILD_CACHE_DIR}/${role}"
    mkdir -p "${role_cache}"
    cache_args+=(
      --cache-from="type=local,src=${role_cache}"
      --cache-to="type=local,dest=${role_cache}-new,mode=max"
    )
    echo "Using local cache ${role_cache}"
  fi

  docker buildx build \
    --builder "${BUILDER_NAME}" \
    --platform "${PLATFORMS}" \
    "${cache_args[@]}" \
    "${tags[@]}" \
    --push \
    "$@"
  end="$(date +%s)"
  elapsed=$((end - start))
  TIMING_LINES+=("${role}=${elapsed}")
  echo "TIMING role=${role} seconds=${elapsed}"

  if [[ "${BUILD_CACHE_BACKEND}" == "local" ]]; then
    local role_cache="${BUILD_CACHE_DIR}/${role}"
    rm -rf "${role_cache}"
    mv "${role_cache}-new" "${role_cache}"
  fi
}

build_with_cache \
  ros-prod \
  --build-arg COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS}" \
  -f docker/ros/Dockerfile.prod .

build_with_cache \
  backend \
  ./src/backend

build_with_cache \
  processing \
  ./src/backend/processing

build_with_cache \
  webhook \
  ./src/backend/webhook_service

build_with_cache \
  dashboard \
  --build-arg VITE_API_BASE="${VITE_API_BASE}" \
  --build-arg VITE_ROSBRIDGE_URL="${VITE_ROSBRIDGE_URL}" \
  --build-arg VITE_MICROROS_HEARTBEAT_TOPIC="${VITE_MICROROS_HEARTBEAT_TOPIC}" \
  --build-arg VITE_APP_VERSION="${DASHBOARD_APP_VERSION}" \
  --build-arg VITE_GIT_SHA="${GIT_SHA}" \
  -f docker/dashboard.Dockerfile .

build_with_cache \
  condor-agent \
  -f docker/condor-agent.Dockerfile .

BUILD_FINISHED_AT="$(date +%s)"
TOTAL_SECONDS=$((BUILD_FINISHED_AT - BUILD_STARTED_AT))

# Write machine-readable timings for CI summary + Fleet Console report.
python3 - "${TIMINGS_FILE}" "${GIT_SHA}" "${BUILDER_NAME}" "${PLATFORMS}" "${TOTAL_SECONDS}" "${TIMING_LINES[@]}" <<'PY'
import json, sys
path, git_sha, builder, platforms, total = sys.argv[1:6]
roles = {}
for item in sys.argv[6:]:
    role, secs = item.split("=", 1)
    roles[role] = int(secs)
payload = {
    "git_sha": git_sha,
    "builder": builder,
    "platforms": platforms,
    "total_seconds": int(total),
    "roles": roles,
}
with open(path, "w", encoding="utf-8") as fh:
    json.dump(payload, fh, indent=2)
    fh.write("\n")
PY

echo
echo "TIMING total_seconds=${TOTAL_SECONDS}"
echo "Wrote timings to ${TIMINGS_FILE}"
echo "Pushed role tags and sha tags ending in -${GIT_SHA}"
echo "Pin a robot with IMAGE_TAG=${GIT_SHA} in robot-prod.env (see robot-prod.env.example)."
echo "Then publish the slim deploy bundle: bash scripts/publish_deploy_bundle.sh"
