#!/usr/bin/env bash
# One-time (or rare) setup for the Jetson Orin Nano as a native ARM64 buildx node.
# Run from your laptop. Requires passwordless SSH to the Jetson and one sudo prompt
# on the Jetson for docker group membership.
#
# Usage:
#   JETSON_HOST=jetson bash scripts/setup_jetson_builder.sh
set -euo pipefail

JETSON_HOST="${JETSON_HOST:-jetson}"
JETSON_USER="${JETSON_USER:-ashwanth}"
BUILDER_NAME="${BUILDER_NAME:-multiarch}"
ARM_NODE_NAME="${ARM_NODE_NAME:-arm64node}"

echo "==> Checking SSH to ${JETSON_USER}@${JETSON_HOST}"
ssh -o BatchMode=yes -o ConnectTimeout=10 "${JETSON_USER}@${JETSON_HOST}" 'uname -m; df -h / | tail -1'

echo "==> Ensuring current user can talk to Docker on the Jetson"
set +e
ssh -tt "${JETSON_USER}@${JETSON_HOST}" "bash -lc '
set -euo pipefail
if ! groups | grep -qw docker; then
  echo \"Adding \$(whoami) to the docker group (needs sudo password once)...\"
  sudo usermod -aG docker \"\$(whoami)\"
  echo \"Log out/in (or run: newgrp docker) so group membership applies, then re-run this script.\"
  echo \"Quick test without re-login: sg docker -c \\\"docker info\\\"\"
  exit 2
fi
docker info >/dev/null
echo \"Docker OK: \$(docker version --format \\\"{{.Server.Version}}\\\") arch=\$(uname -m)\"
'"
status=$?
set -e
if [[ "${status}" -eq 2 ]]; then
  echo "Re-run after docker group membership is active."
  exit 2
elif [[ "${status}" -ne 0 ]]; then
  exit "${status}"
fi

echo "==> Creating / refreshing buildx builder '${BUILDER_NAME}'"
if docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
  echo "Builder ${BUILDER_NAME} already exists"
else
  docker buildx create --name "${BUILDER_NAME}" --driver docker-container --platform linux/amd64 --use
fi

if docker buildx inspect "${BUILDER_NAME}" | grep -q "${ARM_NODE_NAME}"; then
  echo "Node ${ARM_NODE_NAME} already attached"
else
  docker buildx create --name "${BUILDER_NAME}" --append --node "${ARM_NODE_NAME}" \
    --platform linux/arm64 \
    "ssh://${JETSON_USER}@${JETSON_HOST}"
fi

docker buildx use "${BUILDER_NAME}"
docker buildx inspect --bootstrap "${BUILDER_NAME}"

echo
echo "Done. Use: docker buildx build --builder ${BUILDER_NAME} --platform linux/amd64,linux/arm64 ..."
echo "Or: BUILDER_NAME=${BUILDER_NAME} bash scripts/buildx_push_images.sh"
