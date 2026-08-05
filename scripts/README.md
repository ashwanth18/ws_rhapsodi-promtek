# Scripts

Thin wrappers and CI helpers. Prefer **`make help`** (or Cursor → Run Task) over
invoking these by hand.

| Script | Make target | Touches real hardware? | Purpose |
|--------|-------------|------------------------|---------|
| `dev_bench.sh` | `make bench` | No | Laptop mock-arm authoring (`ROS_DOMAIN_ID=42`) |
| `dev_arm_session.sh` | `make arm-session` | **Yes** (stops Pi stack; you jog) | Exclusive real-arm authoring handoff |
| `export_scoop_poses.sh` | `make export-poses` | No | Write `config/layouts/<id>/poses.yaml` via ROS service |
| `validate_layouts.py` | `make validate-layouts` | No | JSON Schema validation |
| `test_layout_sim_parity.py` | `make layout-parity` | No | Sim/real world parity check |
| `ci_layout_reachability.sh` | (CI) | No | Schema + structural reachability gate |
| `push_layout.sh` | `make push-layout` | **Yes** (SSH + apply) | **Dev fast-path only** — bypasses release provenance |
| `calibrate_container_pose.py` | — | Optional | Touch-off calibration helper |
| `generate_layout_world.py` | — | No | Generate Gazebo includes from a layout |
| `deploy_jetson_fleet_console.sh` | `make deploy-jetson` | Hub only | Redeploy Fleet Console on Jetson |
| `publish_deploy_bundle.sh` | (CI) | No | Slim deploy branch for fleet-agent |
| `buildx_push_images.sh` | (CI) | No | Multi-arch image build |

## Preferred ship path for layouts

1. Author with `make bench` / `make arm-session`
2. `make export-poses` → commit `config/layouts/**`
3. CI validates on PR
4. Release → Fleet Console sets desired release / layout → fleet-agent applies

Use `push_layout.sh` only for laptop→Pi iteration before a release, analogous to
`LOCAL_SYNC=1` on the Jetson deploy script.
