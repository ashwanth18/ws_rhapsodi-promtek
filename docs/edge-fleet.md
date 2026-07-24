# Edge build & fleet operations

This repo uses a two-layer edge strategy:

1. **Fast builds** — native ARM64 on the Jetson Orin Nano via `docker buildx`, plus BuildKit cache mounts + `ccache` so ROS package changes do not rebuild the whole workspace from scratch.
2. **Fleet platform** — Tailscale + Ansible + Prometheus/Grafana for provisioning, canary/rolling updates, health monitoring, and SSH-based fault checks (including Cursor agents).

Pi / robot devices run a **slim deploy bundle** (compose + env example + device config example + exporters). They do **not** clone the full ROS monorepo.

## One-time: Jetson as ARM64 builder

```bash
# On the Jetson (once): allow your user to talk to Docker
sudo usermod -aG docker "$USER"
# re-login or: newgrp docker

# From your laptop
JETSON_HOST=jetson bash scripts/setup_jetson_builder.sh
```

Then push multi-arch images (role tags + `-<git-sha>` tags) and publish the slim deploy bundle:

```bash
BUILDER_NAME=multiarch bash scripts/buildx_push_images.sh
bash scripts/publish_deploy_bundle.sh
```

CI can do the same on every merge to `main` via [`.github/workflows/build-and-release.yml`](../.github/workflows/build-and-release.yml) (build + bundle only — **not** an automatic fleet rollout).

Verify caching wiring (fast) or time an incremental rebuild:

```bash
bash scripts/verify_incremental_ros_build.sh
RUN_BUILD=1 PLATFORM=linux/amd64 bash scripts/verify_incremental_ros_build.sh
```

## Slim deploy bundle

`scripts/publish_deploy_bundle.sh` force-pushes an orphan `deploy` branch and a `deploy-<git-sha>` tag containing only:

- `docker-compose.robot-prod.yml`
- `robot-prod.env.example`
- `config/device.yaml.example`
- `monitoring/exporters/docker-compose.exporters.yml`

Edge devices shallow-checkout that branch/tag. Per-device state (`robot-prod.env`, `config/device.yaml`) is created once from the examples and never overwritten by bundle updates.

## Secrets (Ansible Vault)

Fleet secrets live in `ansible/group_vars/all/vault.yml` (AES256 encrypted). See `vault.yml.example` for required keys:

- `vault_dockerhub_username` / `vault_dockerhub_token` — private image pulls
- `vault_postgres_password` — templated into each device's `robot-prod.env`

```bash
cd ansible
echo 'changeme' > .vault_pass   # placeholder vault password; change after editing secrets
chmod 600 .vault_pass
ansible-vault edit group_vars/all/vault.yml
# then rekey to a strong password:
ansible-vault rekey group_vars/all/vault.yml
```

`.vault_pass` is gitignored. Playbooks read it via `ansible.cfg` (`vault_password_file = .vault_pass`), or pass `--ask-vault-pass`.

## Pin / rollback images on a robot

In `robot-prod.env` (or let Ansible rewrite these during deploy):

```bash
IMAGE_TAG=abc1234   # short sha printed by buildx_push_images.sh
ROS_PROD_IMAGE=iserenity/rhapsodi-promtek:ros-prod-abc1234
BACKEND_IMAGE=iserenity/rhapsodi-promtek:backend-abc1234
# ... same for processing/webhook/dashboard/condor-agent
```

If a deploy's backend health check fails, `ansible/deploy.yml` automatically re-pins the previous `IMAGE_TAG`, redeploys, and still fails the play so you are alerted.

Manual rollback:

```bash
ansible-playbook -i ansible/inventory/tailscale.py ansible/deploy.yml \
  --limit rhapsodi-pi5 -e image_tag=<previous-sha> -e serial_batch=1
```

## Provision a new device

1. Publish a deploy bundle for the sha you want (`publish_deploy_bundle.sh`).
2. Create a Tailscale auth key tagged `tag:robot`.
3. Copy `scripts/provision_device.sh` onto the device (from `main`), then:

```bash
sudo TAILSCALE_AUTHKEY=tskey-auth-... \
  DEVICE_HOSTNAME=rhapsodi-site2-pi5 \
  IMAGE_TAG=abc1234 \
  DOCKERHUB_USER=iserenity \
  DOCKERHUB_TOKEN=... \
  POSTGRES_PASSWORD=... \
  bash provision_device.sh
```

This joins Tailscale (with `--ssh`), installs Docker, shallow-clones the slim `deploy-<sha>` tag (or floating `deploy` branch), starts `node_exporter` (:9100) + `cadvisor` (:9190), and boots `docker-compose.robot-prod.yml`.

Edit `config/device.yaml` on the device for `device_id` / `robot_id` / `site_id`.

## Deploy updates (Ansible) — always manual

```bash
cd ansible
# ensure .vault_pass exists and vault secrets are real
ansible-inventory -i inventory/tailscale.py --list
ansible-playbook -i inventory/tailscale.py deploy.yml \
  -e image_tag=$(git -C .. rev-parse --short HEAD) \
  -e serial_batch=1          # canary one device
# then
ansible-playbook -i inventory/tailscale.py deploy.yml \
  -e image_tag=$(git -C .. rev-parse --short HEAD) \
  -e serial_batch=25%
```

Each deploy:

1. Checks out `deploy-<image_tag>` on the device
2. `docker login` with vaulted credentials
3. Pins image refs + Postgres password in `robot-prod.env`
4. `docker compose pull && up -d`
5. Waits for `http://127.0.0.1:8000/health`
6. Rolls back to the previous tag if health fails

## CI auto-build (optional)

[`.github/workflows/build-and-release.yml`](../.github/workflows/build-and-release.yml) runs on push to `main` / `workflow_dispatch`:

1. Tailscale join (so the runner can reach the Jetson builder)
2. Multi-arch `buildx_push_images.sh` (including `condor-agent`)
3. `publish_deploy_bundle.sh`

Configure GitHub secrets: `DOCKERHUB_USERNAME`, `DOCKERHUB_TOKEN`, `TAILSCALE_AUTHKEY`, `JETSON_SSH_HOST`, `JETSON_SSH_USER`.

Fleet rollout remains the manual Ansible command above.

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
