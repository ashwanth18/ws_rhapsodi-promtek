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

- `devices/pi5.yml` — production Pi5 stack
- `devices/jetson.yml` — stub (not production)
- `devices/x86.yml` — stub (not production)
- Root `docker-compose.robot-prod.yml` — compatibility include of `pi5.yml`

## Project directory

Compose files under `devices/` use `./data`, `./config`, and `./docker/...`
paths that must resolve at the **workspace root**. Always pass
`--project-directory <workspace>` (fleet-agent / Ansible do this), or use the
root shim `docker-compose.robot-prod.yml` from the workspace root:

```bash
docker compose --project-directory . --env-file robot-prod.env \
  -f compose/devices/pi5.yml up -d
# or
docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml up -d
```
