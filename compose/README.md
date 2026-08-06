# Hardware compose layout

## Axes

| Axis | Owns | Config |
|------|------|--------|
| `device_class` | Compose + device mounts | `config/device_classes.yaml` → `compose/devices/<class>.yml` |
| `platform` | OCI arch pull | `linux/arm64`, `linux/amd64` on device + Release |
| `robot_type` | OEM / stack | `niryo`, `jaka` on profiles |
| `profile` | Behavior (BT, poses, data root) | `config/profiles.yaml` env only |

## Resolution

1. Device reports / is assigned `device_class` (e.g. `pi5`).
2. Fleet Console sets agent `compose_file` from `device_classes.yaml`.
3. Profile contributes `env` overrides only (not compose).
4. Release must match device `platform`; if Release lists `device_classes`, device class must be included.

## Files

- `devices/pi5.yml` — production Pi5 stack (`SIM_ALLOWED=0`, `DEVICE_CLASS=pi5`)
- `devices/jetson.yml` — stub (not production)
- `devices/x86.yml` — amd64 / JAKA laptop cell (includes `pi5.yml` + launch overlays)
- Root `docker-compose.robot-prod.yml` — compatibility include of `pi5.yml`
- Root `docker-compose.sim.yml` — laptop-only sim (`SIM_ALLOWED=1`, `ENVIRONMENT=sim`; see `docs/MODES.md`)

Laptop / Lexium bring-up (direct ethernet to the controller):

```bash
cp robot-prod.env.example robot-prod.laptop.env   # then edit Laptop / Lexium keys
# Prefer the Makefile so --project-directory cannot be omitted:
make laptop-up
# equivalent:
docker compose --project-directory . --env-file robot-prod.laptop.env \
  -f compose/devices/x86.yml up -d
```

See `docs/lexium-laptop-networking.md`.

## Project directory (required)

Compose files under `devices/` use workspace-root binds such as `./data`,
`./config`, and `./docker/nginx-dashboard.conf`. Those paths are resolved
against **`--project-directory`**, not against `compose/devices/`.

**Always** pass `--project-directory <workspace>` (fleet-agent and Ansible
do this). From the repo root that is `--project-directory .`.

```bash
# Laptop
docker compose --project-directory . --env-file robot-prod.laptop.env \
  -f compose/devices/x86.yml up -d

# Pi (manual; fleet-agent already sets project-directory)
docker compose --project-directory . --env-file robot-prod.env \
  -f compose/devices/pi5.yml up -d

# Root shim (file lives at workspace root, so cwd alone is enough)
docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml up -d
```

### Do not "fix" bind paths with `../../`

Omitting `--project-directory` makes `./docker/nginx-dashboard.conf` look under
`compose/devices/docker/` (missing). Docker then creates a **directory** stub
and the dashboard fails to start ("not a directory").

Changing the compose volume to `../../docker/...` can paper over a bad laptop
invoke, but **breaks Pi/fleet-agent**: with
`--project-directory=/opt/rhapsodi/ws_rhapsodi-promtek` that path resolves to
`/opt/docker/...` and the same stub failure appears on deploy.

Keep `./docker/...` in `pi5.yml`. Fix the invoke (use `--project-directory` or
`make laptop-up`), and remove any stub dirs before retrying:

```bash
# Laptop (if you once ran compose without project-directory)
rm -rf compose/devices/docker

# Pi (if a bad ../../ deploy created /opt/docker)
sudo rm -rf /opt/docker/nginx-dashboard.conf
sudo rmdir /opt/docker 2>/dev/null || true
```
