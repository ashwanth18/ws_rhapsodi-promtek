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

### Path A — Fleet Console flash install (preferred after Tailscale join)

1. Publish images + deploy bundle for the sha you want.
2. Join the device to Tailscale with `tag:robot` (one-time; ACL tag ownership).
3. Open the Fleet Console (`http://<jetson>:8090`) → select the device → **Flash install**.
4. Pick `robot_type` (`niryo` | `jaka`), `site_id`, and `image_tag`, then confirm.

This runs `ansible/provision.yml`, which:

1. Installs Docker (if needed) and ensures the deploy user is in the `docker` group
2. Shallow-checks out `deploy-<image_tag>`
3. Templates `config/device.yaml` with the chosen `robot_type` / `site_id` / `device_id`
4. Pins images, starts exporters + the robot stack, waits for `/health`
5. Writes `.rhapsodi-version` so `/host_info` reports the running version

**Jaka / arm64 constraint:** `robot_type=jaka` is rejected on `aarch64`/`arm64` hosts. The vendor `libjakaAPI.so` is x86_64-only; ARM64 ROS images skip `jaka_driver` / `jaka_planner`. Use `niryo` on Raspberry Pi / Jetson, or provision Jaka on an amd64 host.

CLI equivalent:

```bash
cd ansible
ansible-playbook -i inventory/tailscale.py provision.yml \
  --limit rhapsodi-pi5 \
  -e robot_type=niryo -e site_id=site-1 -e image_tag=abc1234
```

### Path B — Manual bootstrap (before Tailscale)

Use only for the first Tailscale join on a brand-new image:

```bash
sudo TAILSCALE_AUTHKEY=tskey-auth-... \
  DEVICE_HOSTNAME=rhapsodi-site2-pi5 \
  IMAGE_TAG=abc1234 \
  ROBOT_TYPE=niryo \
  SITE_ID=site-1 \
  DOCKERHUB_USER=iserenity \
  DOCKERHUB_TOKEN=... \
  POSTGRES_PASSWORD=... \
  bash scripts/provision_device.sh
```

After the device appears in Tailscale inventory, prefer Path A / Fleet Console for all further installs and updates.

## Deploy updates (Ansible or Fleet Console)

Fleet Console → device detail → **Deploy update** (streams Ansible logs over SSE).

CLI:

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
4. Writes `.rhapsodi-version` for Fleet Console `/host_info` polling
5. `docker compose pull && up -d`
6. Waits for `http://127.0.0.1:8000/health`
7. Rolls back to the previous tag if health fails

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
  - **Rhapsodi Fleet Health** — all devices overview
  - **Rhapsodi Pi Overview** — per-device RAM / temp / disk / CPU / container restarts (pick device in the top dropdown)
- Prometheus: `http://<jetson>:9091`
- Alertmanager: `http://<jetson>:9093`

## Fleet Console (Jetson)

Central web UI + API for device inventory, flash install, updates, live Ansible logs, and deploy history. See [`src/fleet_console/README.md`](../src/fleet_console/README.md).

```bash
cd monitoring
cp fleet-console.env.example fleet-console.env   # optional token / paths
# Requires: ansible/.vault_pass, SSH keys to robots, Tailscale on the host
docker compose -f docker-compose.fleet-console.yml --env-file fleet-console.env up -d --build
```

- UI / API: `http://<jetson>:8090`
- Status model:
  - **Alive** — Prometheus `up{job="node"}` (falls back to Tailscale online + reachable `/host_info`)
  - **Active** — robot currently has an active weightment run (`GET /robot_weightment_runs/active`)
  - **Version** — `GET /host_info` → `image_tag` / `robot_type` / `site_id` / `device_id`
- Deployment history is stored in a SQLite DB inside the `fleet_console_data` volume.

## Fault checks / Cursor agent

```bash
bash scripts/fleet_health_check.sh
bash scripts/fleet_health_check.sh rhapsodi-pi5
```

See [cursor-fleet-fault-check.md](cursor-fleet-fault-check.md) for a scheduled Cursor Automation draft.
