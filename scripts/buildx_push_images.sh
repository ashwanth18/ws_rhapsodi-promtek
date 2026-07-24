#!/usr/bin/env bash
set -euo pipefail

REGISTRY="${REGISTRY:-iserenity}"
REPOSITORY="${REPOSITORY:-rhapsodi-promtek}"
# Prefer the multi-node builder (laptop amd64 + Jetson arm64). Fall back to legacy name.
BUILDER_NAME="${BUILDER_NAME:-multiarch}"
PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS:-8}"
BUILD_CACHE_DIR="${BUILD_CACHE_DIR:-.buildx-cache}"
VITE_API_BASE="${VITE_API_BASE:-http://localhost:8000}"
VITE_ROSBRIDGE_URL="${VITE_ROSBRIDGE_URL:-ws://localhost:9090}"
VITE_MICROROS_HEARTBEAT_TOPIC="${VITE_MICROROS_HEARTBEAT_TOPIC:-/microros/heartbeat}"
PUSH_SHA_TAGS="${PUSH_SHA_TAGS:-1}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

GIT_SHA="$(git rev-parse --short HEAD)"
echo "Building images for ${REGISTRY}/${REPOSITORY} @ ${GIT_SHA} (builder=${BUILDER_NAME})"

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

mkdir -p "${BUILD_CACHE_DIR}"
TMP_CACHE_DIR="${BUILD_CACHE_DIR}-new"
rm -rf "${TMP_CACHE_DIR}"

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
  mapfile -t tags < <(image_tags "${role}")
  docker buildx build \
    --builder "${BUILDER_NAME}" \
    --platform "${PLATFORMS}" \
    --cache-from=type=local,src="${BUILD_CACHE_DIR}" \
    --cache-to=type=local,dest="${TMP_CACHE_DIR}",mode=max \
    "${tags[@]}" \
    --push \
    "$@"
  rm -rf "${BUILD_CACHE_DIR}"
  mv "${TMP_CACHE_DIR}" "${BUILD_CACHE_DIR}"
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
  -f docker/dashboard.Dockerfile .

build_with_cache \
  condor-agent \
  -f docker/condor-agent.Dockerfile .

echo
echo "Pushed role tags and sha tags ending in -${GIT_SHA}"
echo "Pin a robot with IMAGE_TAG=${GIT_SHA} in robot-prod.env (see robot-prod.env.example)."
echo "Then publish the slim deploy bundle: bash scripts/publish_deploy_bundle.sh"
