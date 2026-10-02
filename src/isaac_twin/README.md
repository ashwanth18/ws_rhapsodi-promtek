# isaac_twin

Isaac Sim digital twin of the Niryo scooping cell (dual-container layout), plus a
gymnasium env over the same cell model.

The twin runs the **real** cell software against simulated hardware:

| Real cell | Twin |
|---|---|
| Niryo Ned3 Pro + ros2_control | Isaac articulation + `topic_based_ros2_control` (`/isaac_joint_commands`, `/isaac_joint_states`) |
| D455 on the table | RTX camera at the hand-eye calibration pose (`niryo_d455_eob`), same topics and frames |
| Flour in RS6 | Red PhysX granular particles (default), or a 5 mm height-field bed carved by the scoop mesh |
| Scale under RS3 | `twin_scale_node` publishing `/weight` from the powder poured into RS3 |
| Vibration motor | `/vibration/intensity` drives flow over the scoop lip |

MoveIt, MTC (`scooping_mtc_node`), `cell_layout_manager`, `scoop_vision`, `pour_server`,
`incline_control_node` and the orchestrator BT (`webhook_weightment.xml`) are the cell's own
nodes and configs. Only `config/twin_nodes.yaml` overrides them, and only where hardware differs.

Everything is on `ROS_DOMAIN_ID=77` with `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`
(`scripts/twin_env.sh`), so it never sees the real robot. The launch refuses any other domain
unless `allow_any_domain:=true`.

## Requirements

- Isaac Sim 5.1 standalone at `~/isaacsim/isaac-sim-standalone-5.1.0-linux-x86_64`, or set `ISAAC_SIM_ROOT`.
- `topic_based_ros2_control` built from source (pinned in `src/ros2.repos`; the NVIDIA apt
  package crashes against Jazzy's `hardware_interface`):

  ```bash
  vcs import src < src/ros2.repos   # or clone just topic_based_ros2_control at the pinned SHA
  colcon build --packages-select topic_based_ros2_control isaac_twin --symlink-install \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DBUILD_TESTING=OFF
  ```

- For the gym env: `pip install --user gymnasium`.

## Run the twin

Terminal 1, Isaac (renders the cell, publishes `/clock`, joints, camera, powder):

```bash
src/isaac_twin/scripts/run_isaac_twin.sh            # add --headless for no GUI
# options: --layout-id dual-container --fill-depth 0.04 --powder particles|heightfield
```

Debug options (Isaac only, no ROS stack needed):

- `--replay-scoop 2`: at sim t = 2 s, drive the arm through the authored scoop with the MTC
  shake-off, logging tracking error and grams every second.
- `--max-seconds N`: quit after N sim seconds.
- `--dump-particles f.npz`: on exit, save the grain positions and the TCP pose (base frame).

Terminal 2, the ROS stack (start it after Isaac publishes `/isaac_joint_states`; the launch
waits up to 30 s and seeds the arm's command buffer from Isaac's pose so it does not jump):

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source src/isaac_twin/scripts/twin_env.sh
ros2 launch isaac_twin scooping_isaac.launch.py     # use_rviz:=false, with_flow:=false
```

Launch arguments:

- `with_flow` (default `true`): `pour_server`, `incline_control_node` and the orchestrator with
  `webhook_weightment.xml`.
- `with_scale` (default `true`): `twin_scale_node`.
- `layout_id`, `layouts_dir`, `targets_yaml`, `calibration_name`, `use_rviz`, `rviz_config`.

Any shell that talks to the twin needs `source src/isaac_twin/scripts/twin_env.sh` first.

### Powder services (Isaac side)

```bash
ros2 service call /isaac_twin/reset_powder std_srvs/srv/Trigger    # refill RS6, empty scoop and RS3
ros2 topic echo /isaac_twin/payload_g                              # also bed_mass_g, rs3_mass_g
```

### Flow checks (sim only)

```bash
ros2 service call /scoop_vision/capture std_srvs/srv/Trigger   # arm must be clear of the camera
ros2 service call /scoop_vision/plan robot_common_msgs/srv/PlanScoop "{}"
ros2 service call /bt_start_webhook_weightment std_srvs/srv/Trigger
```

The marker server keeps the twin's authored poses in
`~/.ros/scooping_controller/poses_isaac_<layout>.yaml`, separate from the Gazebo (`sim`) and
real caches.

## Powder model

Settings are in `config/twin.yaml` (`powder`, `scoop`). They are **not fitted** to the real
cell yet. `powder.model` (or `--powder`) picks one of two models. Both publish the same
`/isaac_twin/*_g` topics, `/weight` and reset service, so the ROS stack and BT do not
change.

### Particles (default)

PhysX PBD granular grains on the GPU, rendered red, in `scene/particles.py`. The bookkeeping
that does not need Isaac (seeding, bed / scoop / RS3 / table labels) is in `particle_powder.py`.

- **Bed:** a 4 mm lattice in RS6, about 110k grains of 23 mg (2546 g for a 40 mm bed). PBD
  compresses the bed under its own weight, so it is seeded to `fill_depth_m / settle_ratio`
  and each grain weighs `spacing³ × settle_ratio × density`. The settled bed then matches the
  heightfield's depth and grams. The status log prints the measured depth; re-measure
  `settle_ratio` if you change `spacing_m` or `solver_iterations`.
- **Colliders:**
  - RS6, RS3 and the table use convex decomposition (a triangle mesh leaks at RS6's
    floor/ramp crease).
  - The robot is excluded from the cell colliders with `FilteredPairsAPI`; a collision
    group makes the arm drift.
  - The importer's box collider on `tool_link` fills the bowl, so it is turned off and
    replaced with 3 mm cubes on the scoop surface (`scoop_collider: voxels`).
- **Physics:** gravity is on in the scene and off on each robot link. The arm still tracks
  its commands exactly. The scene runs at 120 Hz with 48 solver iterations, at RTF ~0.5 on
  an RTX 5080 laptop.
- **Speed:** the status line prints FPS and the time per frame in `sim.step` and the powder
  readback. FPS is `render_hz` × RTF. On the RTX 5080 laptop, headless:
  - A frame takes about 60 ms. About 50 ms of that is the main thread waiting on PhysX's
    GPU particle solve (4 substeps), about 9 ms is RTX rendering, and the powder readback
    takes 3 ms.
  - The GPU is not the limit. Without NVIDIA Dynamic Boost (`nvidia-powerd`, not run by
    default on Ubuntu) it sat at its 80 W cap. With it the cap is 175 W, but the GPU only
    draws about 100 W at 2.6 GHz and 66 °C (65–70% busy), and FPS goes from about 15 to 16.
    The PhysX particle steps run in sequence, so a faster or more powerful GPU barely helps.
  - What did not help: `/physics/updateParticlesToUsd=false` (positions still update),
    hiding the grains, and dropping the D455.
  - Trade-offs:
    - A `sdf` scoop collider saves about 11 ms per frame, but leaks about 1 g/s from a held
      scoop.
    - 60 Hz physics compresses the bed to 34 mm and halves retention.
    - 5 mm grains run at about 0.75 RTF.
    - The Isaac viewport and RViz cost another ~0.1 RTF.
- **Grams:** read back from USD at `readback_hz`. A grain counts as scoop if it is inside the
  scoop's TCP-frame box, as RS3 or bed if it is over that container below its rim + 3 cm,
  and as table otherwise.
- **Vibration:** while `/vibration/intensity` > 0, the grains in the scoop get a random
  velocity kick of `vibration_kick_m_s` × intensity every frame. The default 0.15 shakes
  off very little; 1.0 empties the scoop in about 5 s.
- **Authored scoop** (`--replay-scoop 2`): the bowl holds about 77 g in the bed but keeps
  23 g after the lift and shake-off. It leaves the bed tilted about 30° lip-down and the
  grains slide out, although the bowl would hold about 50 g level at the transport pose.
  On a softer bed (16 solver iterations), raising `cohesion` to 0.5 and `friction` to 1.5
  did not change the retained grams. The heightfield model keeps 64 g for the same scoop.
- **Twin flow check**, 20 g webhook target (sim only): the vision-planned scoop held 21 g
  after the shake-off. `pour_server` dosed 19.9 g and the BT finished SUCCESS in one scoop.
  5.8 g stayed in the scoop and about 5 g spilled onto the table. Bed, scoop, RS3 and table
  add up to the starting 2546 g.
- **Camera:** scoop_vision's capture of the settled bed measures 4602 ml, against 4629 ml
  in the bed, and the planner plans on it as usual.

### Heightfield

The gym env always uses this model.

- The scoop points carve the bed; carved powder goes into the bowl (`capture_ratio`).
- **At rest**, the bowl holds a liquid's level fill (`ScoopTool.capacity_m3` on the scoop
  mesh) with the surface allowed to slope `repose_deg` toward the bowl's best-holding angle
  (about 25° back for the Niryo scoop, 109 ml), times `heap_factor`. Anything above that
  slides off over `spill_tau_s` once the scoop leaves the bed.
- **While vibrating**, the slope drops to `repose_vibrating_deg` with no heap. Powder above
  that flows at `vibration_flow_g_per_s` × intensity. While the scoop is tipped toward its
  lip (it holds less than level) by at least `min_pour_tilt_deg`, the flow continues until
  the scoop is empty.
- Spilled powder lands in RS3, in RS6, or on the table, depending on what is under the lip.
- Capacities are cached per scoop mesh in `~/.cache/isaac_twin/scoop_capacity_*.json`. The
  first run spends about 15 s searching for the best-holding angle. Isaac precomputes the
  capacities along the authored scoop in the background; until a value is ready, the
  payload is held.

## Gym env (phase 2)

`isaac_twin.gym.ScoopEnv` (`gymnasium.make("IsaacTwin/Scoop-v0")` after `import isaac_twin.gym`)
uses the same layout, scoop mesh, authored poses and powder model, without Isaac or ROS.
It reads the cell's `scoop_vision.yaml` with the twin overrides on top.

- **Step:** one scoop, the authored 5-pose scoop shifted by the action.
  - The path interpolates joints between the IK solutions, like MTC's pipeline-planned
    segments. Without IK it uses straight lines.
  - The MTC post-lift shake-off (5 s at 0.75, then 1.5 s settle) runs at the lift pose.
  - For the authored scoop the env keeps 64.6 g; the Isaac BT scoop kept 64.1 g
    (21.0 g poured + 43.1 g left in the scoop).
- **Action:** `(dx, dy, dz)`, scoop_vision's `pattern_offset`. Bounds come from
  `ScoopPlanner.shift_window()` and `planner.dz_min_m` / `dz_max_m`.
- **Observation:**
  - `height`: powder depth above the floor per 5 mm interior cell;
  - `joints`: IK at the contact pose.
- **Reward:** `-|scooped_g - target_g| / target_g`. The default target is
  `target_fill_ratio` × the bowl's level capacity.
- **Violations:** shifts that break wall/floor clearance (`ScoopPlanner.clearance_ok`), or
  that IK cannot reach, are not executed. They cost `violation_penalty`, plus
  `violation_per_mm` for each mm short of the clearance limit.
- **Episode end:** when the bed is nearly empty, or truncated at `max_scoops`.
- **Reachability:** numeric IK on `~/.cache/isaac_twin/niryo_ned3pro.urdf`, written by
  `run_isaac_twin.sh`. Pass `check_reachability=False` to skip it.
- `env.heuristic_plan()` runs scoop_vision's planner on the ground-truth surface.

Compare the heuristic planner with authored/random shifts, and fit `fill_efficiency`:

```bash
ros2 run isaac_twin scoop_env_compare --episodes 2 --max-scoops 12 --json /tmp/scoop_compare.json
```

## Known gaps

- The Niryo dual-container targets (`config/layouts/dual-container/robots/niryo/targets.yaml`)
  have no `PourTiltAtWeighingContainer`, which `webhook_weightment.xml` needs; the real cell
  stops at that MoveTo.
  - The twin uses a **draft** from `config/draft_targets/dual-container_niryo.yaml`:
    PourStart's TCP pitched +10° about tool Y (15° total). `draft_targets_node` merges it over
    the layout targets into `~/.cache/isaac_twin/` and points `move_to` there
    (`draft_targets:=false` to disable).
  - Copy it into `config/layouts` only after checking it on the real cell.
- Particle mode logs "Non-GPU-compatible convex mesh is not able to collide with particle
  system" once at start. Some robot-link collider does not collide with the grains; the scoop,
  vessels and table do.
- The powder parameters (heightfield `heap_factor`, both repose angles and flow rate;
  particle friction, cohesion and vibration kick) are unfitted guesses for flour. Fit them from real scoop and pour logs before trusting absolute grams.
  - Twin flow check, 20 g webhook target: one scoop of about 64 g held through PourStart
    and the draft PourTilt. `pour_server` dosed 21.0 g in about 10 s and the BT finished
    SUCCESS.
  - `scoop_env_compare`: heuristic 52 ± 14 g per scoop against the 48 g target, fitted
    `fill_efficiency` ≈ 0.79 (planner 0.5). Re-planning with 0.79 cuts the error from 27% to
    23%, but the next fit gives about 1.0. Retention is not proportional to the engaged
    volume, so a single `fill_efficiency` cannot match it.
- The BT's `ComputeRemaining` clamped the remaining weight at 0, so an overshoot (37.5 g for
  a 20 g target in an earlier twin run) ended the weightment as SUCCESS. That is fixed in
  `robot_orchestrator` on its own branch.

## Tests

```bash
python3 -m pytest src/isaac_twin/test -q
```
