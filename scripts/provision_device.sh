#!/usr/bin/env bash
# Bootstrap a new Rhapsodi edge device (Pi / Jetson / etc.).
# Idempotent where practical. Requires root (sudo).
#
# Required env:
#   TAILSCALE_AUTHKEY   — reusable/tagged Tailscale auth key (tag:robot recommended)
#
# Optional env:
#   DEVICE_HOSTNAME     — hostname to set (default: keep current)
#   WORKSPACE_DIR       — install path (default: /opt/rhapsodi/ws_rhapsodi-promtek)
#   GIT_REPO            — git URL to clone (default: git@github.com:ashwanth18/ws_rhapsodi-promtek.git)
#   GIT_BRANCH          — branch to check out (default: main)
#   IMAGE_TAG           — optional git-sha pin for images
#   SKIP_COMPOSE_UP     — set to 1 to skip starting the stack
#
# Usage:
#   curl -fsSL ... | sudo TAILSCALE_AUTHKEY=tskey-... bash
#   # or from a checked-out repo:
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
GIT_BRANCH="${GIT_BRANCH:-main}"
IMAGE_TAG="${IMAGE_TAG:-}"
SKIP_COMPOSE_UP="${SKIP_COMPOSE_UP:-0}"
INSTALL_USER="${SUDO_USER:-${INSTALL_USER:-admin}}"

export DEBIAN_FRONTEND=noninteractive

log() { echo "==> $*"; }

if [[ -z "${TAILSCALE_AUTHKEY}" ]]; then
  echo "TAILSCALE_AUTHKEY is required (create a tagged key with tag:robot in the Tailscale admin console)." >&2
  exit 1
fi

if [[ -n "${DEVICE_HOSTNAME}" ]]; then
  log "Setting hostname to ${DEVICE_HOSTNAME}"
  hostnamectl set-hostname "${DEVICE_HOSTNAME}"
fi

log "Installing base packages"
apt-get update -y
apt-get install -y --no-install-recommends \
  ca-certificates curl git jq ca-certificates gnupg lsb-release \
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

log "Installing / joining Tailscale"
if ! command -v tailscale >/dev/null 2>&1; then
  curl -fsSL https://tailscale.com/install.sh | sh
fi
# --ssh enables Tailscale SSH for Cursor agent / operators
tailscale up \
  --auth-key="${TAILSCALE_AUTHKEY}" \
  --ssh \
  --accept-routes \
  --hostname="$(hostname)" \
  || tailscale up --ssh --accept-routes

log "Cloning / updating workspace at ${WORKSPACE_DIR}"
mkdir -p "$(dirname "${WORKSPACE_DIR}")"
if [[ -d "${WORKSPACE_DIR}/.git" ]]; then
  git -C "${WORKSPACE_DIR}" fetch origin
  git -C "${WORKSPACE_DIR}" checkout "${GIT_BRANCH}"
  git -C "${WORKSPACE_DIR}" pull --ff-only origin "${GIT_BRANCH}" || true
else
  git clone --branch "${GIT_BRANCH}" "${GIT_REPO}" "${WORKSPACE_DIR}"
fi
chown -R "${INSTALL_USER}:${INSTALL_USER}" "${WORKSPACE_DIR}" || true

if [[ ! -f "${WORKSPACE_DIR}/robot-prod.env" ]]; then
  cp "${WORKSPACE_DIR}/robot-prod.env.example" "${WORKSPACE_DIR}/robot-prod.env"
fi

if [[ -n "${IMAGE_TAG}" ]]; then
  log "Pinning IMAGE_TAG=${IMAGE_TAG}"
  REGISTRY="$(grep -E '^IMAGE_REGISTRY=' "${WORKSPACE_DIR}/robot-prod.env" | cut -d= -f2- || true)"
  REGISTRY="${REGISTRY:-iserenity/rhapsodi-promtek}"
  sed -i "s|^IMAGE_TAG=.*|IMAGE_TAG=${IMAGE_TAG}|" "${WORKSPACE_DIR}/robot-prod.env"
  sed -i "s|^ROS_PROD_IMAGE=.*|ROS_PROD_IMAGE=${REGISTRY}:ros-prod-${IMAGE_TAG}|" "${WORKSPACE_DIR}/robot-prod.env"
  sed -i "s|^BACKEND_IMAGE=.*|BACKEND_IMAGE=${REGISTRY}:backend-${IMAGE_TAG}|" "${WORKSPACE_DIR}/robot-prod.env"
  sed -i "s|^PROCESSING_IMAGE=.*|PROCESSING_IMAGE=${REGISTRY}:processing-${IMAGE_TAG}|" "${WORKSPACE_DIR}/robot-prod.env"
  sed -i "s|^WEBHOOK_IMAGE=.*|WEBHOOK_IMAGE=${REGISTRY}:webhook-${IMAGE_TAG}|" "${WORKSPACE_DIR}/robot-prod.env"
  sed -i "s|^DASHBOARD_IMAGE=.*|DASHBOARD_IMAGE=${REGISTRY}:dashboard-${IMAGE_TAG}|" "${WORKSPACE_DIR}/robot-prod.env"
fi

log "Starting node_exporter + cadvisor (host/container metrics)"
mkdir -p "${WORKSPACE_DIR}/monitoring/exporters"
cat > "${WORKSPACE_DIR}/monitoring/exporters/docker-compose.exporters.yml" <<'YAML'
services:
  node_exporter:
    image: quay.io/prometheus/node-exporter:v1.8.2
    network_mode: host
    pid: host
    restart: unless-stopped
    command:
      - --path.rootfs=/host
    volumes:
      - /:/host:ro,rslave

  cadvisor:
    image: gcr.io/cadvisor/cadvisor:v0.49.1
    network_mode: host
    privileged: true
    restart: unless-stopped
    command:
      - --port=9190
    devices:
      - /dev/kmsg
    volumes:
      - /:/rootfs:ro
      - /var/run:/var/run:ro
      - /sys:/sys:ro
      - /var/lib/docker/:/var/lib/docker:ro
      - /dev/disk/:/dev/disk:ro
YAML

docker compose -f "${WORKSPACE_DIR}/monitoring/exporters/docker-compose.exporters.yml" up -d

if [[ "${SKIP_COMPOSE_UP}" != "1" ]]; then
  log "Pulling and starting robot stack"
  cd "${WORKSPACE_DIR}"
  docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml pull || true
  docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml up -d
fi

log "Provision complete"
echo "  Tailscale: $(tailscale ip -4 2>/dev/null || echo unknown)"
echo "  Workspace: ${WORKSPACE_DIR}"
echo "  Metrics:   node_exporter :9100  cadvisor :9190"
echo "  Next:      device appears in Tailscale with tag:robot and Ansible inventory picks it up"
