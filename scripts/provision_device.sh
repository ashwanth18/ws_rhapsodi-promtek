#!/usr/bin/env bash
# Bootstrap a new Rhapsodi edge device (Pi / Jetson / etc.).
# Idempotent where practical. Requires root (sudo).
#
# Devices get a *slim* checkout of the orphan `deploy` branch (compose +
# env.example + config.example + exporters only) — not the full monorepo.
#
# Required env:
#   TAILSCALE_AUTHKEY   — reusable/tagged Tailscale auth key (tag:robot recommended)
#
# Optional env:
#   DEVICE_HOSTNAME     — hostname to set (default: keep current)
#   WORKSPACE_DIR       — install path (default: /opt/rhapsodi/ws_rhapsodi-promtek)
#   GIT_REPO            — git URL (default: git@github.com:ashwanth18/ws_rhapsodi-promtek.git)
#   DEPLOY_BRANCH       — slim bundle branch (default: deploy)
#   IMAGE_TAG           — optional git-sha pin (checks out deploy-<sha> tag)
#   DOCKERHUB_USER      — for private image pulls (optional; Ansible also logs in)
#   DOCKERHUB_TOKEN     — Docker Hub access token / password
#   POSTGRES_PASSWORD   — overrides default in robot-prod.env
#   SKIP_COMPOSE_UP     — set to 1 to skip starting the stack
#
# Usage:
#   # Copy this script onto a fresh Pi (from main), then:
#   sudo TAILSCALE_AUTHKEY=tskey-... IMAGE_TAG=abc1234 bash provision_device.sh
#   # or from a checked-out monorepo on a bootstrap host:
#   sudo TAILSCALE_AUTHKEY=tskey-... bash scripts/provision_device.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "Run as root: sudo TAILSCALE_AUTHKEY=... bash $0" >&2
  exit 1
fi

TAILSCALE_AUTHKEY="${TAILSCALE_AUTHKEY:-}"
DEVICE_HOSTNAME="${DEVICE_HOSTNAME:-}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/opt/rhapsodi/ws_rhapsodi-promtek}"
GIT_REPO="${GIT_REPO:-git@github.com:ashwanth18/ws_rhapsodi-promtek.git}"
DEPLOY_BRANCH="${DEPLOY_BRANCH:-deploy}"
IMAGE_TAG="${IMAGE_TAG:-}"
DOCKERHUB_USER="${DOCKERHUB_USER:-}"
DOCKERHUB_TOKEN="${DOCKERHUB_TOKEN:-}"
POSTGRES_PASSWORD="${POSTGRES_PASSWORD:-}"
SKIP_COMPOSE_UP="${SKIP_COMPOSE_UP:-0}"
INSTALL_USER="${SUDO_USER:-${INSTALL_USER:-admin}}"

export DEBIAN_FRONTEND=noninteractive

log() { echo "==> $*"; }

if [[ -z "${TAILSCALE_AUTHKEY}" ]]; then
  if command -v tailscale >/dev/null 2>&1 && tailscale status --self >/dev/null 2>&1; then
    log "TAILSCALE_AUTHKEY unset but this host is already on the tailnet — continuing"
  else
    echo "TAILSCALE_AUTHKEY is required (create a tagged key with tag:robot in the Tailscale admin console)." >&2
    exit 1
  fi
fi

if [[ -n "${DEVICE_HOSTNAME}" ]]; then
  log "Setting hostname to ${DEVICE_HOSTNAME}"
  hostnamectl set-hostname "${DEVICE_HOSTNAME}"
fi

log "Installing base packages"
apt-get update -y
apt-get install -y --no-install-recommends \
  ca-certificates curl git jq gnupg lsb-release \
  openssh-server python3 python3-venv

log "Installing Docker Engine"
if ! command -v docker >/dev/null 2>&1; then
  install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    | gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  chmod a+r /etc/apt/keyrings/docker.gpg
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "${VERSION_CODENAME}") stable" \
    > /etc/apt/sources.list.d/docker.list
  apt-get update -y
  apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi

usermod -aG docker "${INSTALL_USER}" || true
systemctl enable --now docker

if [[ -n "${DOCKERHUB_USER}" && -n "${DOCKERHUB_TOKEN}" ]]; then
  log "Docker Hub login as ${DOCKERHUB_USER}"
  echo "${DOCKERHUB_TOKEN}" | docker login -u "${DOCKERHUB_USER}" --password-stdin
fi

log "Installing / joining Tailscale"
if ! command -v tailscale >/dev/null 2>&1; then
  curl -fsSL https://tailscale.com/install.sh | sh
fi
# --ssh enables Tailscale SSH for Cursor agent / operators
if [[ -n "${TAILSCALE_AUTHKEY}" ]]; then
  tailscale up \
    --auth-key="${TAILSCALE_AUTHKEY}" \
    --ssh \
    --accept-routes \
    --hostname="$(hostname)" \
    || tailscale up --ssh --accept-routes
else
  tailscale up --ssh --accept-routes || true
fi

log "Shallow-cloning slim deploy bundle at ${WORKSPACE_DIR}"
mkdir -p "$(dirname "${WORKSPACE_DIR}")"

checkout_bundle() {
  local ref="$1"
  if [[ -d "${WORKSPACE_DIR}/.git" ]]; then
    git -C "${WORKSPACE_DIR}" remote set-url origin "${GIT_REPO}" || true
    git -C "${WORKSPACE_DIR}" fetch --depth 1 origin "${ref}"
    git -C "${WORKSPACE_DIR}" checkout --force FETCH_HEAD
  else
    # Orphan deploy branch / tag — depth 1, no ROS source.
    git clone --depth 1 --branch "${ref}" "${GIT_REPO}" "${WORKSPACE_DIR}" \
      || {
        # Tag refs sometimes need an explicit fetch path.
        mkdir -p "${WORKSPACE_DIR}"
        git -C "${WORKSPACE_DIR}" init
        git -C "${WORKSPACE_DIR}" remote add origin "${GIT_REPO}"
        git -C "${WORKSPACE_DIR}" fetch --depth 1 origin "refs/tags/${ref}:refs/tags/${ref}" \
          || git -C "${WORKSPACE_DIR}" fetch --depth 1 origin "${ref}"
        git -C "${WORKSPACE_DIR}" checkout --force FETCH_HEAD
      }
  fi
}

if [[ -n "${IMAGE_TAG}" ]]; then
  log "Pinning to deploy-${IMAGE_TAG}"
  checkout_bundle "deploy-${IMAGE_TAG}"
else
  log "Checking out floating ${DEPLOY_BRANCH} branch"
  checkout_bundle "${DEPLOY_BRANCH}"
fi

chown -R "${INSTALL_USER}:${INSTALL_USER}" "${WORKSPACE_DIR}" || true

# Per-device state: copy examples once; never overwrite existing files.
# Owned by INSTALL_USER so later Ansible deploys can rewrite IMAGE_TAG without root.
if [[ ! -f "${WORKSPACE_DIR}/robot-prod.env" ]]; then
  cp "${WORKSPACE_DIR}/robot-prod.env.example" "${WORKSPACE_DIR}/robot-prod.env"
fi
if [[ ! -f "${WORKSPACE_DIR}/config/device.yaml" ]]; then
  mkdir -p "${WORKSPACE_DIR}/config"
  cp "${WORKSPACE_DIR}/config/device.yaml.example" "${WORKSPACE_DIR}/config/device.yaml"
fi
chown "${INSTALL_USER}:${INSTALL_USER}" \
  "${WORKSPACE_DIR}/robot-prod.env" \
  "${WORKSPACE_DIR}/config/device.yaml" || true

pin_images() {
  local tag="$1"
  local env_file="${WORKSPACE_DIR}/robot-prod.env"
  local registry
  registry="$(grep -E '^IMAGE_REGISTRY=' "${env_file}" | cut -d= -f2- || true)"
  registry="${registry:-iserenity/rhapsodi-promtek}"
  sed -i "s|^IMAGE_TAG=.*|IMAGE_TAG=${tag}|" "${env_file}"
  sed -i "s|^ROS_PROD_IMAGE=.*|ROS_PROD_IMAGE=${registry}:ros-prod-${tag}|" "${env_file}"
  sed -i "s|^BACKEND_IMAGE=.*|BACKEND_IMAGE=${registry}:backend-${tag}|" "${env_file}"
  sed -i "s|^PROCESSING_IMAGE=.*|PROCESSING_IMAGE=${registry}:processing-${tag}|" "${env_file}"
  sed -i "s|^WEBHOOK_IMAGE=.*|WEBHOOK_IMAGE=${registry}:webhook-${tag}|" "${env_file}"
  sed -i "s|^DASHBOARD_IMAGE=.*|DASHBOARD_IMAGE=${registry}:dashboard-${tag}|" "${env_file}"
  if grep -q '^CONDOR_AGENT_IMAGE=' "${env_file}"; then
    sed -i "s|^CONDOR_AGENT_IMAGE=.*|CONDOR_AGENT_IMAGE=${registry}:condor-agent-${tag}|" "${env_file}"
  else
    echo "CONDOR_AGENT_IMAGE=${registry}:condor-agent-${tag}" >> "${env_file}"
  fi
}

if [[ -n "${IMAGE_TAG}" ]]; then
  log "Pinning IMAGE_TAG=${IMAGE_TAG}"
  pin_images "${IMAGE_TAG}"
fi

if [[ -n "${POSTGRES_PASSWORD}" ]]; then
  if grep -q '^POSTGRES_PASSWORD=' "${WORKSPACE_DIR}/robot-prod.env"; then
    sed -i "s|^POSTGRES_PASSWORD=.*|POSTGRES_PASSWORD=${POSTGRES_PASSWORD}|" "${WORKSPACE_DIR}/robot-prod.env"
  else
    echo "POSTGRES_PASSWORD=${POSTGRES_PASSWORD}" >> "${WORKSPACE_DIR}/robot-prod.env"
  fi
fi

log "Starting node_exporter + cadvisor (host/container metrics)"
EXPORTERS_COMPOSE="${WORKSPACE_DIR}/monitoring/exporters/docker-compose.exporters.yml"
if [[ ! -f "${EXPORTERS_COMPOSE}" ]]; then
  echo "Missing ${EXPORTERS_COMPOSE} — is the deploy bundle published?" >&2
  exit 1
fi
docker compose -f "${EXPORTERS_COMPOSE}" up -d

if [[ "${SKIP_COMPOSE_UP}" != "1" ]]; then
  log "Pulling and starting robot stack"
  cd "${WORKSPACE_DIR}"
  docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml pull || true
  docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml up -d
fi

log "Provision complete"
echo "  Tailscale: $(tailscale ip -4 2>/dev/null || echo unknown)"
echo "  Workspace: ${WORKSPACE_DIR} (slim deploy bundle)"
echo "  Metrics:   node_exporter :9100  cadvisor :9190"
echo "  Next:      device appears in Tailscale with tag:robot and Ansible inventory picks it up"
echo "  Update:    ansible-playbook ansible/deploy.yml -e image_tag=<sha>"
