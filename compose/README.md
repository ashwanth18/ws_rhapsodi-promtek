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

Compose files under `devices/` still use `./data` and `./config` paths that
must resolve at the **workspace root**. The fleet-agent always passes
`--project-directory <workspace>` when invoking `docker compose`.
