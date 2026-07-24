#!/usr/bin/env bash
# Verify BuildKit cache mounts / ccache wiring and (optionally) time an
# incremental ROS image rebuild after touching one package.
#
# Default mode is a fast static check (no multi-hour build).
# Full timing mode:
#   RUN_BUILD=1 bash scripts/verify_incremental_ros_build.sh
#
# Optional:
#   PLATFORM=linux/amd64   # default; use linux/arm64 with Jetson builder
#   BUILDER_NAME=multiarch
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

PLATFORM="${PLATFORM:-linux/amd64}"
BUILDER_NAME="${BUILDER_NAME:-multiarch}"
RUN_BUILD="${RUN_BUILD:-0}"
TOUCH_FILE="${TOUCH_FILE:-src/pouring_controller/package.xml}"

echo "==> Static checks"
grep -q 'ccache' docker/ros/apt-packages.txt
grep -q 'mount=type=cache,target=/ws/build' docker/ros/Dockerfile.prod
grep -q 'DCMAKE_CXX_COMPILER_LAUNCHER=ccache' docker/ros/Dockerfile.prod
grep -q 'mount=type=cache,target=/ws/build' docker/ros/Dockerfile.dev || \
  grep -q 'mount=type=cache,target=/ws/build,id=colcon-dev-build' docker/ros/Dockerfile.dev
grep -q 'role}-\${GIT_SHA}' scripts/buildx_push_images.sh || grep -q 'GIT_SHA' scripts/buildx_push_images.sh
echo "Dockerfile / apt / buildx script look correct."

if [[ "${RUN_BUILD}" != "1" ]]; then
  echo
  echo "Skipped timed rebuild (set RUN_BUILD=1 to execute)."
  echo "Recommended after scripts/setup_jetson_builder.sh:"
  echo "  1) BUILDER_NAME=${BUILDER_NAME} PLATFORMS=${PLATFORM} bash scripts/buildx_push_images.sh"
  echo "  2) touch ${TOUCH_FILE}"
  echo "  3) rebuild and confirm only pouring_controller (+ deps) recompile"
  exit 0
fi

if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
  if docker buildx inspect multiarch-builder >/dev/null 2>&1; then
    BUILDER_NAME=multiarch-builder
  else
    echo "No buildx builder '${BUILDER_NAME}'. Run scripts/setup_jetson_builder.sh" >&2
    exit 1
  fi
fi

echo "==> Baseline build (may take a long time on first run)"
START=$(date +%s)
docker buildx build \
  --builder "${BUILDER_NAME}" \
  --platform "${PLATFORM}" \
  --progress=plain \
  -f docker/ros/Dockerfile.prod \
  --target builder \
  -t rhapsodi-promtek:ros-prod-incremental-test \
  --load \
  . 2>&1 | tee /tmp/rhapsodi-colcon-baseline.log
BASE_ELAPSED=$(( $(date +%s) - START ))
echo "Baseline elapsed: ${BASE_ELAPSED}s"

echo "==> Touch ${TOUCH_FILE} and rebuild"
touch "${TOUCH_FILE}"
START=$(date +%s)
docker buildx build \
  --builder "${BUILDER_NAME}" \
  --platform "${PLATFORM}" \
  --progress=plain \
  -f docker/ros/Dockerfile.prod \
  --target builder \
  -t rhapsodi-promtek:ros-prod-incremental-test \
  --load \
  . 2>&1 | tee /tmp/rhapsodi-colcon-incremental.log
INC_ELAPSED=$(( $(date +%s) - START ))
echo "Incremental elapsed: ${INC_ELAPSED}s"

echo "==> Packages that finished in incremental log:"
grep -E 'Finished <<<|Starting >>>' /tmp/rhapsodi-colcon-incremental.log | tail -40 || true

if [[ "${INC_ELAPSED}" -ge "${BASE_ELAPSED}" ]]; then
  echo "WARN: incremental build was not faster than baseline (${INC_ELAPSED}s vs ${BASE_ELAPSED}s)."
  echo "Cache mounts may not be sticky on this builder node yet."
  exit 2
fi

echo "OK: incremental build faster (${INC_ELAPSED}s < ${BASE_ELAPSED}s)."
