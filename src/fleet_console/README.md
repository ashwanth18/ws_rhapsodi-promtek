# Rhapsodi Fleet Console

Central UI + API for provisioning and updating Rhapsodi edge devices over Tailscale.

Runs on the Jetson (alongside Prometheus/Grafana). Operators open it over the Tailnet, pick a device, flash-install (robot type + site + image tag) or deploy an update, and watch Ansible logs stream live.

## Local development

```bash
# API
cd src/fleet_console
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
export REPO_ROOT="$(git rev-parse --show-toplevel)"
export ANSIBLE_DIR="$REPO_ROOT/ansible"
export FLEET_DATA_DIR=/tmp/fleet-data
uvicorn app.main:app --reload --port 8090

# UI (separate terminal)
cd src/fleet_console/web
npm install
npm run dev   # http://localhost:5174  (proxies /api → :8090)
```

## Production (Jetson)

```bash
cd monitoring
# ensure ../ansible/.vault_pass exists
docker compose -f docker-compose.fleet-console.yml --env-file fleet-console.env up -d --build
```

Open `http://<jetson-tailscale-ip>:8090`.

## Version + profile axes

Each device has a **desired target** (SQLite `device_targets`):

- `tracked_branch` — which git branch to follow for updates
- `profile_id` — runtime behavior from `config/profiles.yaml` (prod, lights-out, site layout, …)
- `pinned_image_tag` — last successfully deployed SHA

These are independent: e.g. track `main` with profile `lightsout-training`.

## API surface

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/api/devices` | Tailscale robots + Prometheus + `/host_info` + drift + update badges |
| GET | `/api/devices/{id}` | Device detail + target + version_check + history |
| PUT | `/api/devices/{id}/target` | Set tracked_branch / profile_id |
| GET | `/api/devices/{id}/version_check` | Compare deployed SHA vs GitHub branch HEAD |
| GET | `/api/profiles` | Runtime profile catalog |
| POST | `/api/devices/{id}/provision` | `ansible-playbook provision.yml` (+ profile) |
| POST | `/api/devices/{id}/deploy` | `ansible-playbook deploy.yml` (+ profile) |
| POST | `/api/devices/{id}/build` | buildx + publish bundle for a branch (optional deploy_after) |
| GET | `/api/deployments` | Global history |
| GET | `/api/deployments/{id}/logs/stream` | SSE job log tail |

Auth: optional `FLEET_API_TOKEN` bearer (also `?access_token=` for EventSource).
Set `GITHUB_TOKEN` for update-available / build-from-branch.
