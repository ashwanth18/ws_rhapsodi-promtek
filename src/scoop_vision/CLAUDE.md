# scoop_vision: notes for agents

D455 (table-mounted, eye-on-base) → powder height map → the **next scoop as a pure
shift** `(dx, dy, dz)` of the operator's authored 5-pose scoop. The shift is applied as
`pattern_offset_x/y/z` on `scooping_mtc_node`. The camera also calibrates the task
container's pose. Human-oriented explanation: `docs/SCOOP_VISION.md`. Operator
reference: `README.md` (same folder).

Built 2026-10-01 on branch `feature/camera-robot-calibration`. **Not committed** as of
writing. Ask before committing.

## Rules (do not break)

1. **Shift-only.** The operator explicitly wants the authored scoop shape kept.
   Never reshape poses (no sweep-scale, yaw or pitch changes) without asking.
2. **Safety checks live here, not in MoveIt.** `scooping_mtc_node`'s scene allows
   `tool_link`, `tcp_link`, `hand_link`, `wrist_link` and `forearm_link` to touch the
   task vessel, so MoveIt will not stop a scoop hitting a wall. The planner's clearance
   field is the guard: walls grown sideways by `wall_margin_xy_m` (12 mm) plus
   `wall_clearance_m` (8 mm), floor 8 mm. The forearm is **not** modelled.
3. **Every plan must be reachable.** Candidates are IK-checked best-first via MoveIt
   `/compute_ik` (`check_reachability`). Without this, a full bin lifts the approach
   out of the Niryo's workspace (seen live: `approach=no_ik`).
4. **Capture gates:** refuse when the arm occludes the bin (projected `elbow_link…tcp_link`),
   and when the bin rim differs from the layout by more than 10 mm (`require_alignment`).
5. **The map goes stale** when `tcp_link` enters the bin, when the layout hash changes,
   or after 900 s. Plan refuses stale maps, and the BT falls back to the authored scoop.
6. **Container calibration proposes, it never silently applies.** `fit_container_pose`
   writes `~/.ros/scoop_vision/proposals/<stamp>_<layout>/`. Only `apply_container_fit`
   (panel button with a confirm dialog) overwrites `config/layouts/<layout>.yaml`
   (backed up) and calls `/cell_layout/apply`. Poses are stored in the container frame,
   so applying moves the real scoop with the bin. That was the operator's choice
   ("option A"). `poses_keep_world_path.yaml` exists for the other case; it scraped the
   back wall here, so do not use it for RS6.
7. **Never move the robot without the operator.** The panel's Run and Apply buttons
   have confirm dialogs. Agents may use read-only MoveIt services (`/compute_ik`,
   `/compute_fk`, `/check_state_validity`, `/diagnose_scoop_poses`) freely.
   `ros2 param set` on `scooping_mtc_node` changes what the next Run does: restore it.

## Current state of the cell (2026-10-01)

- **TEMPORARY** `planner.surface_correction_m: 0.03` (config; the operator tuned it live to
  ~0.045 via the panel). It compensates a hand-eye error (the scoop only scraped). After
  recalibration: set it to 0, then **Fit pose → Apply fit**, because the RS6 pose came from
  the old calibration.
- `config/layouts/dual-container.yaml` RS6 was re-fitted from the camera:
  `[0.4064, -0.1251, 0.0057]`, yaw 179.99° (was `[0.460, -0.119, 0.0]`, 180°). The backup
  is in the proposals dir.
- New MoveTo target `CameraClear` (`config/layouts/dual-container/robots/niryo/targets.yaml`)
  was chosen by an IK/FK search: joints ≥ 0.52 rad from their limits, ≥ 15 cm outside the
  camera's view.
- `MoveToWeighingContainer` puts `joint_2` exactly on its URDF limit (0.44). Encoder noise
  then makes MoveIt reject the start state (`CheckStartStateBounds`). This is unresolved.
  Options: MoveIt `<pipeline>.fix_start_state: true`, or move the target. The operator
  has not chosen.
- `fill_efficiency = 0.5` is an unfitted guess. The next valuable work is logging
  `/scoop_vision/last_plan` together with the scooped grams.

## File map

| File | Role |
|------|------|
| `scoop_vision/mesh.py` | STL load (binary/ascii), top-down rasterise, surface sampling, voxel dedupe |
| `scoop_vision/transforms.py` | Quaternions, `apply_mtc_shape` (**mirrors C++ `apply_pattern_offset` order**), path interpolation |
| `scoop_vision/container.py` | `Grid2D`, `ContainerModel`: floor map, rim/interior, two distance fields (walls grown in XY; floor + table) |
| `scoop_vision/scoop_tool.py` | Scoop in the `tcp_link` frame; bowl capacity by priority-flood "trapped water" at **1 mm** (2 mm leaks on the decimated mesh) |
| `scoop_vision/heightmap.py` | Unproject depth; per-cell median per frame, then median across frames; nearest-fill holes |
| `scoop_vision/planner.py` | `ScoopPlanner`: trench `B(c)`, volume solve, constraints, score, clearance with raising, IK callback |
| `scoop_vision/alignment.py` | Rim registration: `check_alignment` (X/Y) and `fit_container_offset` (X/Y/yaw, parabolic refine, Z from flat rim) |
| `scoop_vision/layout.py` | Layout path fallback (`/ws/config` → `layouts_dir`), task-container mesh, proposal writer, pose re-anchoring |
| `scoop_vision/scoop_vision_node.py` | ROS node: services, occlusion, staleness, IK, markers/animation, apply-fit |
| `launch/scoop_vision.launch.py` | D455 (848×480@15, spatial + temporal filters), hand-eye TF publisher, node, optional RViz |
| `launch/scoop_vision_niryo.launch.py` | Native one-shot: `scooping_real` (RS6, driver without `/tf`) + the above |
| `rviz/scoop_vision.rviz` | Panel + displays. **Do not add an Image display** (segfault, see gotchas) |
| `../scooping_controller/src/scoop_vision_panel.cpp` | RViz panel `scooping_controller/ScoopVisionPanel` |
| `../robot_orchestrator/include/robot_orchestrator/scoop_vision_nodes.hpp` | BT nodes `CaptureScoopSurface`, `PlanScoopFromVision` |
| `../robot_orchestrator/src/execute_scoop_node.cpp` | `ExecuteScoop`: optional `pattern_offset_x/z` (only written when set) |
| `../robot_common_msgs/srv/PlanScoop.srv` | Plan service |
| `../../compose/devices/scoop-vision.yml` | Opt-in compose profile `vision` (untested in a prod image) |

## Gotchas found the hard way

- **Two stacks on one DDS graph:** the Pi's `scooping_stack` (lights-out) published `/tf`,
  `/robot_description` (`/ws/install/...` paths, so RViz can't load meshes) and
  `/cell_layout/active` (`/ws/config/...` paths). Run native sessions with
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` and stop other stacks.
- **Stale meshes in `install/`:** `install/` once held the old hi-res scoop STL. Rebuild
  `niryo_robot_description` after mesh changes.
- **RViz2 segfault** in `RenderWindow::exposeEvent` with Image dock + custom panel + MTC
  display and no saved window state. Also never give the RViz `Node` a `name=`: `__node`
  remaps *every* node in the process (panels, MoveIt displays).
- **PyYAML can't dump numpy scalars:** cast to `float` before `yaml.safe_dump`.
- **Executor patterns:** blocking service handlers wait on futures by polling, and run in
  a `ReentrantCallbackGroup` with a `MultiThreadedExecutor`; clients get their own
  `MutuallyExclusiveCallbackGroup`. In scripts, do not use
  `TransformListener(spin_thread=True)` and also `spin_once` the same node (deadlock).
- **`ros2 service call` timeouts:** responses can take longer than the call timeout. Check
  the node log before assuming a hang.
- **Killing test processes:** `pkill -f <pattern>` / `pgrep -f` inside a bash `-c` also
  matches *your own shell's command line* (exit code 144). Use `ps -eo pid,args` and kill
  PIDs. `ros2 run` leaves the child alive when killed: run the installed executable directly.
- **Latched topic echo:** `ros2 topic echo` truncates long strings. Use `--full-length`
  plus explicit type and QoS (`--qos-durability transient_local --qos-reliability reliable`).

## How to test

```bash
python3 -m pytest src/scoop_vision/test -q            # 33 tests, no ROS needed
python3 -m flake8 --select F,E9 src/scoop_vision        # style lint is noisy; check real errors only
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --packages-select robot_common_msgs robot_orchestrator scooping_controller scoop_vision
```

No-hardware ROS smoke test: run a fake cell on an **isolated domain**
(`ROS_DOMAIN_ID=77`). It publishes a ray-marched synthetic D455 depth of a flour bed in
RS6, static TFs (camera straight above the container frame), the layout, `/scoop_poses`,
and a stand-in `scooping_mtc_node` with the shape params. Then call capture, plan and
fit. **Never call `apply_container_fit` against a fake cell:** it writes the real
`config/layouts`.

BT XML validation: build a tiny program that calls `robot_orchestrator::RegisterNodes`
and `createTreeFromFile` with a blackboard holding `ros_node` and `phase_topic`.

Live, read-only diagnosis that worked well: read the per-node logs in `~/.ros/log/*.log`
(the launch log only has process start and stop), then use `/diagnose_scoop_poses`
(per-waypoint `no_ik` / `collision_at_goal`) and `/check_state_validity` with
`group_name: ''` (the whole robot; `'arm'` misses `tool_link`).
