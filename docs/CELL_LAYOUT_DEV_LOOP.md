# Cell-layout development loop

Cell layouts are versioned YAML in `config/layouts/`; applying one updates the
scene, targets, poses, and task-container contract as one unit.

## Three loops

1. **Bench (domain 42):** run `scripts/dev_bench.sh <layout-id>`. It is
   isolated from a Pi ROS graph and is for scene/pose authoring.
2. **Arm handoff (domain 0):** run `scripts/dev_arm_session.sh` only with an
   idle robot. It stops `scooping_stack`, launches the supervised RViz stack,
   and restarts the stack on exit. The script never commands motion.
3. **Push:** run `scripts/push_layout.sh <layout-id>` to copy layouts and
   `profiles.yaml` to the Pi and call `/apply_cell_layout`. Use
   `scripts/push_layout.sh --revert` to restore the previous layout directory.

## What requires a rebuild

Layout YAML, targets, and poses are mounted configuration and can be pushed
without rebuilding a robot image. Robot meshes, URDF collision geometry, and
the tool TCP are image artifacts: rebuild and redeploy the ROS image after
changing them.

## Provenance and simulation

Run start is refused when the mode-selected layout is not successfully applied,
when layout/pose provenance is incomplete, or when a real production mode uses
poses not authored in `real`. Existing unstamped pose files need one supervised
re-save before real production use.

Parity has three tiers: YAML/schema + structural reachability in CI
(`scripts/ci_layout_reachability.sh`), scene/visual parity in simulation, and
full MoveIt plan-only preflight on a bench. The latter is intentionally not a
headless CI requirement.
