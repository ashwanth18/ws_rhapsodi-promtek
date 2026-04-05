#!/usr/bin/env bash
set -euo pipefail

REGISTRY="${REGISTRY:-iserenity}"
REPOSITORY="${REPOSITORY:-rhapsodi-promtek}"
PLATFORMS="${PLATFORMS:-linux/amd64,linux/arm64}"
COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS:-8}"
DASHBOARD_PORT="${DASHBOARD_PORT:-8080}"
BUILD_CACHE_DIR="${BUILD_CACHE_DIR:-.buildx-cache}"
VITE_API_BASE="${VITE_API_BASE:-http://localhost:8000}"
VITE_ROSBRIDGE_URL="${VITE_ROSBRIDGE_URL:-ws://localhost:9090}"
VITE_MICROROS_HEARTBEAT_TOPIC="${VITE_MICROROS_HEARTBEAT_TOPIC:-/microros/heartbeat}"

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

docker buildx inspect multiarch-builder >/dev/null 2>&1 || docker buildx create --name multiarch-builder --use
docker buildx use multiarch-builder

mkdir -p "${BUILD_CACHE_DIR}"
TMP_CACHE_DIR="${BUILD_CACHE_DIR}-new"
rm -rf "${TMP_CACHE_DIR}"

build_with_cache() {
  local tag="$1"
  shift
  docker buildx build \
    --platform "${PLATFORMS}" \
    --cache-from=type=local,src="${BUILD_CACHE_DIR}" \
    --cache-to=type=local,dest="${TMP_CACHE_DIR}" \
    -t "${tag}" \
    --push \
    "$@"
  rm -rf "${BUILD_CACHE_DIR}"
  mv "${TMP_CACHE_DIR}" "${BUILD_CACHE_DIR}"
}

build_with_cache \
  "${REGISTRY}/${REPOSITORY}:ros-prod" \
  --build-arg COLCON_PARALLEL_WORKERS="${COLCON_PARALLEL_WORKERS}" \
  -f docker/ros/Dockerfile.prod .

build_with_cache \
  "${REGISTRY}/${REPOSITORY}:backend" \
  ./src/backend

build_with_cache \
  "${REGISTRY}/${REPOSITORY}:processing" \
  ./src/backend/processing

build_with_cache \
  "${REGISTRY}/${REPOSITORY}:webhook" \
  ./src/backend/webhook_service

build_with_cache \
  "${REGISTRY}/${REPOSITORY}:dashboard" \
  --build-arg VITE_API_BASE="${VITE_API_BASE}" \
  --build-arg VITE_ROSBRIDGE_URL="${VITE_ROSBRIDGE_URL}" \
  --build-arg VITE_MICROROS_HEARTBEAT_TOPIC="${VITE_MICROROS_HEARTBEAT_TOPIC}" \
  -f docker/dashboard.Dockerfile .
