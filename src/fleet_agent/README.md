# fleet-agent

Host-level pull agent that converges a robot device to the desired
`release_id` + `profile_id` stored in the Fleet Console.

## Loop

1. `GET /api/agent/target` (Bearer device token)
2. Compare to local `.rhapsodi-version`
3. On drift: checkout `deploy-<sha>`, render `robot-prod.env`, `docker compose pull && up -d`, wait for `/health`
4. On health failure: roll back to `.rhapsodi-last-good` locally
5. `POST /api/agent/report` with outcome (includes `active_mode` +
   `environment` from local `GET /runtime/mode` when reachable)

Installed as a systemd unit by `ansible/provision.yml` (not inside docker compose,
so it can manage compose without a chicken-and-egg restart).

## Manual run

```bash
export FLEET_CONSOLE_URL=http://jetson:8090
export AGENT_TOKEN=...
export WORKSPACE_DIR=/opt/rhapsodi/ws_rhapsodi-promtek
python3 agent.py
```
