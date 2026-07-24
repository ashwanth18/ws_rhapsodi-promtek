# Edge build & fleet operations

This repo uses a two-layer edge strategy:

1. **Fast builds** — native ARM64 on the Jetson Orin Nano via `docker buildx`, plus BuildKit cache mounts + `ccache` so ROS package changes do not rebuild the whole workspace from scratch.
2. **Fleet platform** — Tailscale + Ansible + Prometheus/Grafana for provisioning, canary/rolling updates, health monitoring, and SSH-based fault checks (including Cursor agents).

## One-time: Jetson as ARM64 builder

```bash
# On the Jetson (once): allow your user to talk to Docker
sudo usermod -aG docker "$USER"
# re-login or: newgrp docker

# From your laptop
JETSON_HOST=jetson bash scripts/setup_jetson_builder.sh
```

Then push multi-arch images (role tags + `-<git-sha>` tags):

```bash
BUILDER_NAME=multiarch bash scripts/buildx_push_images.sh
```

Verify caching wiring (fast) or time an incremental rebuild:

```bash
bash scripts/verify_incremental_ros_build.sh
RUN_BUILD=1 PLATFORM=linux/amd64 bash scripts/verify_incremental_ros_build.sh
```

## Pin / rollback images on a robot

In `robot-prod.env`:

```bash
IMAGE_TAG=abc1234   # short sha printed by buildx_push_images.sh
ROS_PROD_IMAGE=iserenity/rhapsodi-promtek:ros-prod-abc1234
BACKEND_IMAGE=iserenity/rhapsodi-promtek:backend-abc1234
# ... same for processing/webhook/dashboard
```

Or let Ansible rewrite those lines during deploy.

## Provision a new device

Create a Tailscale auth key tagged `tag:robot`, then on the device:

```bash
sudo TAILSCALE_AUTHKEY=tskey-auth-... \
  DEVICE_HOSTNAME=rhapsodi-site2-pi5 \
  IMAGE_TAG=abc1234 \
  bash scripts/provision_device.sh
```

This joins Tailscale (with `--ssh`), installs Docker, starts `node_exporter` (:9100) + `cadvisor` (:9190), and boots `docker-compose.robot-prod.yml`.

## Deploy updates (Ansible)

```bash
cd ansible
ansible-inventory -i inventory/tailscale.py --list
ansible-playbook -i inventory/tailscale.py deploy.yml \
  -e image_tag=$(git -C .. rev-parse --short HEAD) \
  -e serial_batch=1          # canary one device
# then
ansible-playbook -i inventory/tailscale.py deploy.yml \
  -e image_tag=$(git -C .. rev-parse --short HEAD) \
  -e serial_batch=25%
```

## Monitoring (Jetson)

```bash
bash scripts/generate_prom_targets.sh
cd monitoring
docker compose -f docker-compose.monitoring.yml up -d
```

- Grafana: `http://<jetson>:3001` (default admin/admin — change it)
- Prometheus: `http://<jetson>:9091`
- Alertmanager: `http://<jetson>:9093`

## Fault checks / Cursor agent

```bash
bash scripts/fleet_health_check.sh
bash scripts/fleet_health_check.sh rhapsodi-pi5
```

See [cursor-fleet-fault-check.md](cursor-fleet-fault-check.md) for a scheduled Cursor Automation draft.
