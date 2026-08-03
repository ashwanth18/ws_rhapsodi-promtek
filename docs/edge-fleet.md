# Edge build & fleet operations

This repo uses a three-layer edge strategy:

1. **Fast builds** — native ARM64 on the Jetson Orin Nano via `docker buildx`, BuildKit **registry** cache (`:buildcache-<role>` on Docker Hub) so CI does not re-download apt/npm/pip every run, plus Dockerfile cache mounts + `ccache` so ROS package changes do not rebuild the whole workspace from scratch.
2. **Verified Releases** — GitHub Actions builds images + slim deploy bundle for a branch, then reports a Release row to the Fleet Console. Only successful Releases are selectable.
3. **Pull-based fleet** — each robot runs `fleet-agent`, which polls the Fleet Console for desired `release_id` + `profile_id`, applies locally, health-checks, and rolls back without SSH.

Ansible is used for **first-boot provisioning only** (Docker, slim checkout, device identity, agent install). Routine updates do not push over SSH. `ansible/deploy.yml` remains as a documented break-glass fallback.

Pi / robot devices run a **slim deploy bundle** (compose + env example + device config example + exporters + profiles). They do **not** clone the full ROS monorepo.

## Architecture

```
CI (build-and-release.yml)
  → buildx multi-arch images + deploy-<sha> tag
  → POST /api/releases/report  (CI_REPORT_TOKEN over Tailscale)

Fleet Console (Jetson)
  → stores Release + DeviceTarget(desired release_id, profile_id)
  → UI: pick Release + Profile → "Deploy (set desired)"

fleet-agent (each robot, systemd)
  → GET /api/agent/target
  → checkout deploy-<sha>, render robot-prod.env, compose pull/up
  → health check → local rollback to .rhapsodi-last-good on failure
  → POST /api/agent/report
```

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

CI does the same via [`.github/workflows/build-and-release.yml`](../.github/workflows/build-and-release.yml) on push to `main` or `workflow_dispatch` with an arbitrary branch. After a successful build it reports a Release to the Fleet Console (secrets: `FLEET_CONSOLE_URL`, `CI_REPORT_TOKEN`).

## Slim deploy bundle

`scripts/publish_deploy_bundle.sh` force-pushes an orphan `deploy` branch and a `deploy-<git-sha>` tag containing only:

- `docker-compose.robot-prod.yml`
- `robot-prod.env.example`
- `config/device.yaml.example`
- `config/profiles.yaml` + `config/profiles/**`
- `config/recording_profiles.yaml` (phase → topic map for RecorderV2)
- `monitoring/exporters/docker-compose.exporters.yml`

Edge devices shallow-checkout that branch/tag. Per-device state (`robot-prod.env`, `config/device.yaml`) is created once and never overwritten by bundle updates except by the agent when converging.

## Secrets (Ansible Vault)

Fleet secrets live in `ansible/group_vars/all/vault.yml` (AES256 encrypted). See `vault.yml.example` for required keys:

- `vault_dockerhub_username` / `vault_dockerhub_token` — private image pulls
- `vault_postgres_password` — seeded into each device's `robot-prod.env` at provision

```bash
cd ansible
echo 'changeme' > .vault_pass   # placeholder vault password; change after editing secrets
chmod 600 .vault_pass
ansible-vault edit group_vars/all/vault.yml
```

## Profiles

Runtime behavior is selected via [`config/profiles.yaml`](../config/profiles.yaml) (e.g. `prod-niryo`, `lightsout-training`, `jaka-site2-layout`). Profiles choose compose file + env (`BT_TREE`, pose/scene YAML paths, data root). The Fleet Console filters profiles by the device's `robot_type`. Layout overrides live under `config/profiles/<id>/`.

## Provision a new device

### Path A — Fleet Console flash install (preferred after Tailscale join)

1. Ensure CI (or a local build + `POST /api/releases/report`) has produced a successful Release.
2. Join the device to Tailscale with `tag:robot`.
3. Open Fleet Console → device → pick **robot type** (from profiles catalog), **profile**, **Release**, site.
4. **Flash install** — runs `ansible/provision.yml`, which:
   1. Installs Docker (if needed)
   2. Shallow-checks out `deploy-<sha>`
   3. Templates `config/device.yaml`
   4. Seeds Postgres password + Docker Hub login
   5. Installs `fleet-agent` (systemd) with a minted per-device token
   6. Starts the agent — it pulls the desired Release on first reconcile

**Jaka / arm64 constraint:** `robot_type=jaka` is rejected on `aarch64`/`arm64` hosts.

CLI equivalent:

```bash
cd ansible
ansible-playbook -i inventory/tailscale.py provision.yml \
  --limit rhapsodi-pi5 \
  -e robot_type=niryo -e site_id=site-1 -e image_tag=abc1234 \
  -e agent_token=<minted> -e fleet_console_url=http://jetson:8090
```

(Prefer the Fleet Console path — it mints and injects `agent_token` automatically.)

### Path B — Manual bootstrap (before Tailscale)

Use only for the first Tailscale join on a brand-new image (`scripts/provision_device.sh`), then finish with Path A so the agent is installed.

## Deploy updates (normal path — pull)

1. Develop on a branch → **Build branch (CI)** in the console (or push to `main`).
2. Wait for a successful Release to appear in the Release picker.
3. Select Release + Profile → **Deploy (set desired)**.
4. Watch **Agent** status: `applying` → `success` / `converged`, or `rolled_back` on health failure.

No Ansible SSH is involved. Drift and agent reports show on the devices list.

## How to read a Release (what build is what)

Each successful CI run becomes one **Release** row in Fleet Console:

| Field | Meaning |
|-------|---------|
| **`#N (abcdef0)`** | Deployable version label. `N` is the console Release id; `abcdef0` is the **git short sha** of the commit that was built. Image tags are `…:dashboard-abcdef0`, `…:backend-abcdef0`, etc. |
| **Subject** | Commit subject of that sha (`git log -1`) — best one-line “what’s in this build”. |
| **Branch** | Git branch CI checked out (`main`, `feature/…`). |
| **Workflow URL** | Link to the GitHub Actions run (full log, which jobs ran). |
| **Device running** | Device detail shows **desired** vs **running** sha; drift means agent has not converged yet. |

**Robot dashboard SemVer** (`v1.1.0 · abcdef0` in the sidebar) is separate: product UI version from `src/dashboard/package.json`, plus the same git sha baked at image build. Use SemVer to see which *UI generation* you have; use Release `#N (sha)` to pick *what the fleet should pull*.

### Break-glass push (optional)

```bash
cd ansible
ansible-playbook -i inventory/tailscale.py deploy.yml \
  --limit rhapsodi-pi5 \
  -e image_tag=abc1234 -e profile=lightsout-training -e serial_batch=1
```

## CI auto-build

[`.github/workflows/build-and-release.yml`](../.github/workflows/build-and-release.yml):

1. Checkout the requested branch (`workflow_dispatch.inputs.branch` or push ref)
2. Tailscale join → Jetson buildx node
3. `buildx_push_images.sh` + `publish_deploy_bundle.sh`
4. `POST /api/releases/report` success (or failure) to Fleet Console

GitHub secrets: `DOCKERHUB_USERNAME`, `DOCKERHUB_TOKEN`, `TAILSCALE_AUTHKEY`, `JETSON_SSH_HOST`, `JETSON_SSH_USER`, `FLEET_CONSOLE_URL`, `CI_REPORT_TOKEN`.

## Monitoring (Jetson)

```bash
bash scripts/generate_prom_targets.sh
cd monitoring
docker compose -f docker-compose.monitoring.yml up -d
```

- Grafana: `http://<jetson>:3001`
- Prometheus: `http://<jetson>:9091`
- Alertmanager: `http://<jetson>:9093`

## Fleet Console (Jetson)

```bash
cd monitoring
cp fleet-console.env.example fleet-console.env
docker compose -f docker-compose.fleet-console.yml --env-file fleet-console.env up -d --build
```

- UI / API: `http://<jetson>:8090`
- Status model:
  - **Alive** — Prometheus `up{job="node"}` (falls back to Tailscale + `/host_info`)
  - **Active** — active weightment run
  - **Agent** — last reconcile report (`converged` / `applying` / `rolled_back` / `failed`)
  - **Version / profile** — desired vs running (+ drift)
- Builds are triggered via GitHub Actions `workflow_dispatch` (not local buildx on the console host).

See [`src/fleet_console/README.md`](../src/fleet_console/README.md) and [`src/fleet_agent/README.md`](../src/fleet_agent/README.md).

## Migrating already-provisioned devices

Re-run flash install / provision (idempotent) so `fleet-agent` is installed and an `agent_token` is written. After that, stop using Ansible for routine deploys.

## Fault checks / Cursor agent

```bash
bash scripts/fleet_health_check.sh
bash scripts/fleet_health_check.sh rhapsodi-pi5
```

See [cursor-fleet-fault-check.md](cursor-fleet-fault-check.md) for a scheduled Cursor Automation draft.
