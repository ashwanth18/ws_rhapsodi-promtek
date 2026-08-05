# Cell-layout development loop

Cell layouts are versioned YAML in `config/layouts/`; applying one updates the
scene, targets, poses, and task-container contract as one unit. Scoop poses are
layout-scoped (`poses_<env>_<layout_id>.yaml` on device) and versioned under
`config/layouts/<id>/poses.yaml`.

## Discoverable entry points

```bash
make help                 # list tasks
make bench LAYOUT=dual-container
make arm-session          # real-arm handoff; never commands motion itself
make export-poses         # write config/layouts/<id>/poses.yaml via ROS service
make validate-layouts
make push-layout LAYOUT=… # DEV FAST-PATH only
```

Cursor: **Run Task** → Bench / Arm session / Export scoop poses / Validate.

See also [`scripts/README.md`](../scripts/README.md).

## Three loops

1. **Bench (domain 42):** `make bench LAYOUT=<id>` (or `scripts/dev_bench.sh`).
   Mock hardware, isolated from a Pi ROS graph. Author scene geometry and scoop
   markers, then `make export-poses` to land git-tracked poses.
2. **Arm handoff (domain 0):** `make arm-session` only with an idle robot. Stops
   Pi `scooping_stack`, launches the supervised RViz stack, restarts on exit.
   The script never commands motion; you jog. Export poses the same way.
3. **Ship (preferred):** commit → CI validates layouts → release (bundle
   already includes `config/layouts/**`) → Fleet Console sets desired release
   and/or **Apply layout** → on-device `fleet-agent` calls
   `POST :8010/apply_cell_layout` (edge-triggered) with a `layout_apply`
   Deployment row and logs.

**Dev fast-path:** `make push-layout` / `scripts/push_layout.sh` rsyncs YAML to
the Pi and curls the adapter. It bypasses release provenance (like
`LOCAL_SYNC=1` for Jetson). Prefer Fleet Console after a release for anything
that should stick.

## What requires a rebuild

Layout YAML, targets, and poses ride the deploy bundle — no image rebuild.
Robot meshes, URDF collision geometry, MTC stage logic, and the tool TCP are
image artifacts: rebuild and redeploy `ros-prod` after changing them.

`GET /layouts` and layout apply via fleet-agent need a `backend` image (and
updated `fleet-agent` systemd unit) that includes those features.

## Provenance and simulation

Run start is refused when the mode-selected layout is not successfully applied,
when layout/pose provenance is incomplete, or when a real production mode uses
poses not authored in `real`. On layout apply, the marker server prefers the
layout-scoped device-local cache; on mismatch it reseeds from the versioned
`config/layouts/<id>/poses.yaml` when that file's `layout_id` /
`container_spec_hash` match. Unstamped or mismatched files still hard-refuse.

Parity has three tiers: YAML/schema + structural reachability in CI
(`scripts/ci_layout_reachability.sh`, wired into `build-and-release.yml` on
PRs), scene/visual parity in simulation, and full MoveIt plan-only preflight on
a bench. The latter is intentionally not a headless CI requirement.
