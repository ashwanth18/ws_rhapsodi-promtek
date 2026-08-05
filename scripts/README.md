# Scripts

Thin wrappers and CI helpers. Prefer **`make help`** (or Cursor → Run Task) over
invoking these by hand. See [`docs/CELL_LAYOUT_DEV_LOOP.md`](../docs/CELL_LAYOUT_DEV_LOOP.md).

| Script | Make target | Touches real hardware? | Purpose |
|--------|-------------|------------------------|---------|
| `dev_author.sh` | `make author` | No | Bench + interactive cell-layout editor |
| `dev_bench.sh` | `make bench` | No | Laptop mock-arm authoring (`ROS_DOMAIN_ID=42`, `ROBOT=` / `LAYOUT=`) |
| `dev_sim.sh` | `make sim` | No | Gazebo sim with layout-composed world (`ROS_DOMAIN_ID=43`) |
| `dev_arm_session.sh` | `make arm-session` | **Yes** (stops Pi stack; you jog) | Exclusive real-arm authoring handoff (`layout_edit:=true`) |
| `dev_status.sh` | `make status` | No | Doctor: dirty tree, pose freshness, Pi layout, next step |
| `export_scoop_poses.sh` | `make export-poses` | No | Write `config/layouts/<id>/poses.yaml` via ROS service |
| `validate_layouts.py` | `make validate-layouts` | No | Schema + catalog validation |
| `test_layout_sim_parity.py` | `make layout-parity` | No | Generated Gazebo world completeness check |
| `test_touch_off_parity.py` | `make touch-off-parity` | No | C++ / Python touch-off fit parity |
| `gen_vscode_tasks.py` | `make gen-vscode-tasks` / `check-vscode-tasks` | No | Keep `.vscode/tasks.json` pickers in sync |
| `ci_layout_reachability.sh` | (CI) | No | Schema + structural reachability gate |
| `push_layout.sh` | `make push-layout` | **Yes** (SSH + apply) | **Dev fast-path only** — bypasses release provenance |
| `calibrate_container_pose.py` | — | Optional | Offline touch-off YAML helper (panel uses live TF) |
| `generate_layout_world.py` | — | No | Compose Gazebo worlds from a layout |
| `deploy_jetson_fleet_console.sh` | `make deploy-jetson` | Hub only | Redeploy Fleet Console on Jetson |
| `publish_deploy_bundle.sh` | (CI) | No | Slim deploy branch (layouts, models, robots) |
| `buildx_push_images.sh` | (CI) | No | Multi-arch image build |

`make api-sandbox-up` / `api-sandbox-down` start only the mode/API sandbox
(`docker-compose.sim.yml`: backend + Postgres). That is **not** Gazebo.

## Preferred ship path for layouts

1. Author with `make author` / `make sim` / `make arm-session` (Cell Layout tab:
   drag containers, touch-off, Save Layout; Scoop Motion tab: markers + Export)
2. `make status` → `make validate-layouts` → commit `config/layouts/**`
3. CI validates on PR (schema, catalog, touch-off parity, VS Code pickers)
4. Release → Fleet Console sets desired release / layout → fleet-agent applies

Use `push_layout.sh` only for laptop→Pi iteration before a release, analogous to
`LOCAL_SYNC=1` on the Jetson deploy script.
