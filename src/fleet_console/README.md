# Rhapsodi Fleet Console

Central UI + API for a **pull-based** fleet: CI produces verified Releases,
operators set desired `release_id` + `profile_id` per device, and each device's
`fleet-agent` pulls, applies, health-checks, and rolls back locally.

Ansible is used only for **first-boot flash install** (Docker + slim bundle +
agent). Routine updates do not SSH-push.

## Local development

```bash
# API
cd src/fleet_console
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
export REPO_ROOT="$(git rev-parse --show-toplevel)"
export ANSIBLE_DIR="$REPO_ROOT/ansible"
export FLEET_DATA_DIR=/tmp/fleet-data
export CI_REPORT_TOKEN=dev-ci-token
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
cp fleet-console.env.example fleet-console.env   # set tokens / GITHUB_TOKEN
docker compose -f docker-compose.fleet-console.yml --env-file fleet-console.env up -d --build
```

Open `http://<jetson-tailscale-ip>:8090`.

The console is the **fleet management control plane** (provision, desired
release/profile, deployments, monitoring). Each robot’s operator SPA
(weighment UI / Cell Signal Deck) is linked from Devices as
`http://<hostname>:8080` (override with `ROBOT_DASHBOARD_PORT`).

### Runtime mode visibility

Devices list + detail show each robot’s active **mode** and **environment**
(from live `GET /runtime/mode` on the robot backend, with fleet-agent
heartbeat as fallback). See [docs/MODES.md](../../docs/MODES.md) Phase 8.

### Console SemVer

Operator-facing console version lives in `web/package.json` and appears in
the sidebar footer (`Fleet Console vX.Y.Z`). Bump on operator-visible changes
per `.cursor/rules/fleet-console-versioning.mdc`.

## Desired state

Each device has a SQLite `device_targets` row:

- `tracked_branch` — which git branch to follow for new CI builds
- `profile_id` — runtime behavior from `config/profiles.yaml`
- `release_id` — FK to a **verified** `releases` row (never a free-text SHA)
- `agent_token` — minted at provision; used by on-device fleet-agent

Robot type is chosen once at flash install (from distinct `robot_type` values
in `config/profiles.yaml`) and is immutable afterward.

## API surface

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/api/devices` | Tailscale robots + metrics + desired/running + agent status |
| GET | `/api/devices/{id}` | Device detail + target + history |
| PUT | `/api/devices/{id}/target` | Set branch / profile / release_id |
| GET | `/api/releases` | Verified CI builds (selectable) |
| POST | `/api/releases/report` | CI reports a build (`CI_REPORT_TOKEN`) |
| GET | `/api/branches` | Real GitHub branches |
| GET | `/api/robot_types` | From `config/profiles.yaml` |
| GET | `/api/profiles` | Runtime profile catalog |
| POST | `/api/devices/{id}/provision` | Ansible flash install + agent |
| POST | `/api/devices/{id}/deploy` | Set desired state (agent converges) |
| POST | `/api/devices/{id}/build` | `workflow_dispatch` CI build |
| GET | `/api/agent/target` | Agent polls desired release+profile |
| POST | `/api/agent/report` | Agent reports reconcile outcome |

Auth:

- Browser: optional `FLEET_API_TOKEN` bearer
- CI: `CI_REPORT_TOKEN` (falls back to `FLEET_API_TOKEN`)
- Agent: per-device `agent_token`

Set `GITHUB_TOKEN` with Actions:write for branch listing + CI dispatch.
