#!/usr/bin/env bash
# Publish a slim "deploy" orphan branch + deploy-<git-sha> tag containing only
# the files an edge device needs to run docker compose (no ROS source tree).
#
# Bundle contents:
#   docker-compose.robot-prod.yml (compat shim → compose/devices/pi5.yml)
#   compose/devices/** (hardware compose)
#   compose/README.md
#   robot-prod.env.example
#   config/device.yaml.example
#   config/device_classes.yaml
#   config/profiles.yaml
#   config/profiles/** (pose/scene layout overrides)
#   monitoring/exporters/docker-compose.exporters.yml
#   monitoring/exporters/promtail-config.yml
#
# Usage (after scripts/buildx_push_images.sh):
#   bash scripts/publish_deploy_bundle.sh
#   bash scripts/publish_deploy_bundle.sh --dry-run
#
# Edge devices then shallow-checkout:
#   git clone --depth 1 --branch deploy <repo> /opt/rhapsodi/...
#   # or pin exactly:
#   git fetch --depth 1 origin tag deploy-<sha> && git checkout deploy-<sha>
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

DRY_RUN=0
if [[ "${1:-}" == "--dry-run" ]]; then
  DRY_RUN=1
fi

GIT_SHA="$(git rev-parse --short HEAD)"
DEPLOY_TAG="deploy-${GIT_SHA}"
DEPLOY_BRANCH="deploy"
REMOTE="${REMOTE:-origin}"
# Resolve the real push URL up front. A --local clone would otherwise keep
# origin pointed at this working tree and never reach GitHub.
REMOTE_URL="$(git remote get-url "${REMOTE}")"

BUNDLE_FILES=(
  docker-compose.robot-prod.yml
  compose/README.md
  robot-prod.env.example
  config/device.yaml.example
  config/device_classes.yaml
  config/profiles.yaml
  monitoring/exporters/docker-compose.exporters.yml
  monitoring/exporters/promtail-config.yml
)

for f in "${BUNDLE_FILES[@]}"; do
  if [[ ! -f "${f}" ]]; then
    echo "Missing required bundle file: ${f}" >&2
    exit 1
  fi
done

if [[ ! -d config/profiles ]]; then
  echo "Missing required directory: config/profiles" >&2
  exit 1
fi

if [[ ! -d compose/devices ]]; then
  echo "Missing required directory: compose/devices" >&2
  exit 1
fi

TMP_DIR="$(mktemp -d)"
cleanup() { rm -rf "${TMP_DIR}"; }
trap cleanup EXIT

echo "Preparing slim deploy bundle @ ${GIT_SHA}"
echo "Remote: ${REMOTE_URL}"
git clone --local --no-hardlinks --shared "${ROOT_DIR}" "${TMP_DIR}/repo" >/dev/null 2>&1 \
  || git clone --local "${ROOT_DIR}" "${TMP_DIR}/repo" >/dev/null
cd "${TMP_DIR}/repo"
git remote set-url origin "${REMOTE_URL}"


# Orphan branch with only the bundle files.
git checkout --orphan "${DEPLOY_BRANCH}" >/dev/null 2>&1
git rm -rf --quiet . >/dev/null 2>&1 || true
# Clear any leftover untracked junk from the shared clone.
git clean -fdx >/dev/null 2>&1 || true

mkdir -p config monitoring/exporters compose/devices
for f in "${BUNDLE_FILES[@]}"; do
  mkdir -p "$(dirname "${f}")"
  cp "${ROOT_DIR}/${f}" "${f}"
done
# Per-profile pose/scene layout packs
cp -a "${ROOT_DIR}/config/profiles" config/
# Hardware compose files
cp -a "${ROOT_DIR}/compose/devices/." compose/devices/

# Minimal README so operators know what this branch is.
cat > README.md <<EOF
# Rhapsodi edge deploy bundle

Slim checkout used by Pi / Jetson edge devices.

- Source commit: \`${GIT_SHA}\`
- Tag: \`${DEPLOY_TAG}\`
- Do **not** develop here — edit files on \`main\` and re-run
  \`scripts/publish_deploy_bundle.sh\`.

Provision:
\`\`\`bash
sudo TAILSCALE_AUTHKEY=... IMAGE_TAG=${GIT_SHA} bash scripts/provision_device.sh
\`\`\`

(The provision script lives on \`main\`; devices only need this branch.)
EOF

git add -A
git -c user.email="deploy-bundle@rhapsodi.local" \
    -c user.name="deploy-bundle" \
    commit -m "Deploy bundle ${GIT_SHA}" >/dev/null

git tag -f "${DEPLOY_TAG}"

if [[ "${DRY_RUN}" == "1" ]]; then
  echo "Dry run — would push:"
  echo "  ${REMOTE} ${DEPLOY_BRANCH} (force)"
  echo "  ${REMOTE} tag ${DEPLOY_TAG} (force)"
  echo "Bundle files:"
  git ls-files
  exit 0
fi

# Force-push orphan branch + immutable-ish sha tag. Tags are overwritten only
# when re-publishing the same sha (idempotent rebuilds).
git push --force "${REMOTE}" "HEAD:${DEPLOY_BRANCH}"
git push --force "${REMOTE}" "refs/tags/${DEPLOY_TAG}"

echo
echo "Published deploy bundle:"
echo "  branch: ${DEPLOY_BRANCH}"
echo "  tag:    ${DEPLOY_TAG}"
echo "Pin devices with IMAGE_TAG=${GIT_SHA}"
