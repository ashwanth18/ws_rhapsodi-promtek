# Cell-layout development loop

Cell layouts are versioned YAML in `config/layouts/` (schema v2); applying one
updates the scene, per-robot targets, poses, and task-container contract as one
unit. Scoop poses are layout-scoped (`poses_<env>_<layout_id>.yaml` on device).
The **default seed** is `config/layouts/<id>/poses.yaml` (container-frame;
robot-agnostic) and is never overwritten by Save/Export. Timestamped labeled
sets go under `config/layouts/<id>/poses/sets/` for traceability; load
`default` or any set from the Cell Layout → Scoop Pose Sets panel. Named
MoveTo targets live under `config/layouts/<id>/robots/<robot_key>/targets.yaml`.

## Discoverable entry points

```bash
make help                                    # grouped task list
make status                                  # doctor: dirty tree, pose freshness, Pi layout
make author LAYOUT=dual-container ROBOT=niryo  # bench + interactive layout editor
make bench LAYOUT=dual-container ROBOT=niryo # mock arm, no Gazebo
make sim ROBOT=jaka LAYOUT=dual-container    # Gazebo + layout-composed world
make arm-session                             # real-arm handoff; never commands motion
make export-poses                            # save a timestamped set under poses/sets/ (default intact)
make validate-layouts
make layout-parity
make push-layout LAYOUT=…                    # DEV FAST-PATH only
```

Cursor: **Run Task** → Author / Bench / Sim / Arm session / Dev status / Export / Validate.
Layout and robot pickers are generated from `config/layouts/*.yaml` and
`robots.yaml` (`make gen-vscode-tasks` / CI `--check`).

See also [`scripts/README.md`](../scripts/README.md).

## Authoring containers (no YAML guessing)

Selectable CAD lives in [`config/models/catalog.yaml`](../config/models/catalog.yaml).
`make author` (or `make bench` / `make sim` with `layout_edit:=true`) launches
`cell_layout_editor`: drag containers in RViz (planar XY+yaw by default), add
from the catalog, capture 3 TCP touch points, Fit, then **Save Layout**. The
ScoopingPanel **Cell Layout** tab drives those services. Preview publishes
`/cell_layout/active` with `preview=true` / `layout_hash=preview` so production
runs refuse unsaved geometry. Saves only write into the git tree
(`config/layouts/`), never `/ws/config/layouts`.

## Four loops

1. **Author / Bench (domain 42):** `make author LAYOUT=<id> ROBOT=<niryo|jaka>`
   (or `make bench`). Mock hardware via `robots.yaml` `mock:` profiles, isolated
   from a Pi ROS graph. Drag containers + scoop markers; Save Layout from the
   panel. Save Pose Set writes `poses/sets/<UTC>_<note>.yaml` without changing
   the default `poses.yaml` (or `make export-poses`). Load Pose Set → `default`
   restores the seed.
2. **Sim (domain 43):** `make sim ROBOT=<id> LAYOUT=<id>`. Gazebo with a world
   composed at launch from the layout YAML (template + generated objects).
   `cell_layout_manager` publishes `/cell_layout/active` so layout-scoped poses
   engage. Use this to check a layout against another arm (e.g. Schneider/Jaka)
   before touching hardware.
3. **Arm handoff (domain 0):** `make arm-session` only with an idle robot. Stops
   Pi `scooping_stack`, launches the supervised RViz stack with `layout_edit:=true`,
   restarts on exit. The script never commands motion; you jog for touch-off.
   Export poses the same way.
4. **Ship (preferred):** commit → CI validates layouts (+ catalog + task pickers)
   → release (bundle includes `config/layouts/**`, `config/models/**`, and
   `config/robots/<type>.yaml`) → Fleet Console sets desired release and/or
   **Apply layout** → on-device `fleet-agent` calls
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
when layout/pose provenance is incomplete, when `/cell_layout/active` is a
preview (`layout_hash=preview`), or when a real production mode uses poses not
authored in `real`. Poses stamp `robot_key` alongside `layout_id` /
`authored_in`. On layout apply, the marker server prefers the layout-scoped
device-local cache; on mismatch it reseeds from the versioned default
`config/layouts/<id>/poses.yaml`. Labeled history lives in
`config/layouts/<id>/poses/sets/` (Save Pose Set / `make export-poses`); Load
Pose Set with `default` restores the seed without promoting any set over it.
Targets whose `frame_id` does not match the active robot `base_frame` are
hard-refused (prevents Niryo `base_link` targets silently loading on Jaka
`link0`).

A layout without an entry under `targets_by_robot` for the active robot is
**not commissioned** for that arm — apply fails rather than inventing poses.

Parity has three tiers: YAML/schema + structural reachability in CI
(`scripts/ci_layout_reachability.sh`, wired into `build-and-release.yml` on
PRs), generated Gazebo world completeness (`make layout-parity`), and full
MoveIt plan-only preflight on a bench/sim. The latter is intentionally not a
headless CI requirement.
