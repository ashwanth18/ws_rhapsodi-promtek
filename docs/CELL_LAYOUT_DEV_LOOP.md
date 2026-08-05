# Cell-layout development loop

Cell layouts are versioned YAML in `config/layouts/` (schema v2); applying one
updates the scene, per-robot targets, poses, and task-container contract as one
unit. Scoop poses are layout-scoped (`poses_<env>_<layout_id>.yaml` on device)
and versioned under `config/layouts/<id>/poses.yaml` (container-frame; robot-
agnostic). Named MoveTo targets live under
`config/layouts/<id>/robots/<robot_key>/targets.yaml`.

## Discoverable entry points

```bash
make help                                    # list tasks
make bench LAYOUT=dual-container ROBOT=niryo # mock arm, no Gazebo
make sim ROBOT=jaka LAYOUT=dual-container    # Gazebo + layout-composed world
make arm-session                             # real-arm handoff; never commands motion
make export-poses                            # write config/layouts/<id>/poses.yaml
make validate-layouts
make layout-parity
make push-layout LAYOUT=…                    # DEV FAST-PATH only
```

Cursor: **Run Task** → Bench / Sim / Arm session / Export scoop poses / Validate.

See also [`scripts/README.md`](../scripts/README.md).

## Four loops

1. **Bench (domain 42):** `make bench LAYOUT=<id> ROBOT=<niryo|jaka>`.
   Mock hardware via `robots.yaml` `mock:` profiles, isolated from a Pi ROS
   graph. Fast authoring of markers; then `make export-poses`.
2. **Sim (domain 43):** `make sim ROBOT=<id> LAYOUT=<id>`. Gazebo with a world
   composed at launch from the layout YAML (template + generated objects).
   `cell_layout_manager` publishes `/cell_layout/active` so layout-scoped poses
   engage. Use this to check a layout against another arm (e.g. Schneider/Jaka)
   before touching hardware.
3. **Arm handoff (domain 0):** `make arm-session` only with an idle robot. Stops
   Pi `scooping_stack`, launches the supervised RViz stack, restarts on exit.
   The script never commands motion; you jog. Export poses the same way.
4. **Ship (preferred):** commit → CI validates layouts → release (bundle
   includes `config/layouts/**` and `config/robots/<type>.yaml`) → Fleet Console
   sets desired release and/or **Apply layout** → on-device `fleet-agent` calls
   `POST :8010/apply_cell_layout` (edge-triggered).

**Dev fast-path:** `make push-layout` / `scripts/push_layout.sh` rsyncs YAML to
the Pi and curls the adapter. It bypasses release provenance (like
`LOCAL_SYNC=1` for Jetson). Prefer Fleet Console after a release for anything
that should stick.

`make api-sandbox-up` is only the mode/API sandbox (`docker-compose.sim.yml`);
it is not Gazebo.

## Single-robot Pi contract

Each Pi is connected to exactly one arm. Provision writes
`config/device.yaml` with `robot_type`. The deploy bundle emits
`config/robots/<type>.yaml` from `src/scooping_controller/config/robots.yaml`.
`scooping_real.launch.py` resolves the robot from `robot:=` / `ROBOT_TYPE` /
`device.yaml` and refuses to start when `PROFILE_ID`'s `robot_type` disagrees.
Laptop and Fleet Console keep the full multi-robot catalog.

## What requires a rebuild

Layout YAML, targets, and poses ride the deploy bundle — no image rebuild.
Robot meshes, URDF collision geometry, MTC stage logic, and the tool TCP are
image artifacts: rebuild and redeploy `ros-prod` after changing them.

`GET /layouts` and layout apply via fleet-agent need a `backend` image (and
updated `fleet-agent` systemd unit) that includes those features.

## Provenance and simulation

Run start is refused when the mode-selected layout is not successfully applied,
when layout/pose provenance is incomplete, or when a real production mode uses
poses not authored in `real`. Poses stamp `robot_key` alongside `layout_id` /
`authored_in`. On layout apply, the marker server prefers the layout-scoped
device-local cache; on mismatch it reseeds from the versioned
`config/layouts/<id>/poses.yaml`. Targets whose `frame_id` does not match the
active robot `base_frame` are hard-refused (prevents Niryo `base_link` targets
silently loading on Jaka `link0`).

A layout without an entry under `targets_by_robot` for the active robot is
**not commissioned** for that arm — apply fails rather than inventing poses.

Parity has three tiers: YAML/schema + structural reachability in CI
(`scripts/ci_layout_reachability.sh`, wired into `build-and-release.yml` on
PRs), generated Gazebo world completeness (`make layout-parity`), and full
MoveIt plan-only preflight on a bench/sim. The latter is intentionally not a
headless CI requirement.
