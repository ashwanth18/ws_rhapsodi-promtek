# isaac_twin

Isaac Sim digital twin of the Niryo scooping cell (dual-container layout), plus a
gymnasium env over the same cell model.

The twin runs the **real** cell software against simulated hardware:

| Real cell | Twin |
|---|---|
| Niryo Ned3 Pro + ros2_control | Isaac articulation + `topic_based_ros2_control` (`/isaac_joint_commands`, `/isaac_joint_states`) |
| D455 on the table | RTX camera at the hand-eye calibration pose (`niryo_d455_eob`), same topics and frames |
| Flour in RS6 | 5 mm height-field bed carved by the scoop mesh |
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
# options: --layout-id dual-container --fill-depth 0.04
```

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
ros2 topic echo /isaac_twin/powder_status                          # bed / scoop / RS3 / table grams
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
cell yet.

- The scoop points carve the bed; carved powder goes into the bowl (`capture_ratio`).
- Once the scoop leaves the bed, anything above the bowl's level capacity at the current
  tilt times `heap_factor` slides off over `spill_tau_s`. Capacity comes from the scoop
  mesh (`ScoopTool.capacity_m3`).
- Vibration knocks the heap off (only the level fill stays). It feeds powder over the lip
  at `vibration_flow_g_per_s` × intensity, but only while the scoop is tipped toward the lip
  (it holds less than level) by at least `min_pour_tilt_deg`.
- Spilled powder lands in RS3, in RS6, or on the table, depending on what is under the lip.

## Gym env (phase 2)

`isaac_twin.gym.ScoopEnv` (`gymnasium.make("IsaacTwin/Scoop-v0")` after `import isaac_twin.gym`)
uses the same layout, scoop mesh, authored poses and powder model, without Isaac or ROS.
It reads the cell's `scoop_vision.yaml` with the twin overrides on top.

- **Step:** one scoop, the authored 5-pose scoop shifted by the action.
  - The path interpolates joints between the IK solutions, like MTC's pipeline-planned
    segments. Without IK it uses straight lines.
  - The MTC post-lift shake-off (5 s at 0.75, then 1.5 s settle) runs at the lift pose.
  - For the authored scoop this matches the Isaac twin's scooped mass (40 g).
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
  have no `PourTiltAtWeighingContainer`. `webhook_weightment.xml` needs it, so the twin BT
  stops at that MoveTo; the real cell will too.
- The powder parameters are unfitted; fit them from real scoop logs before trusting absolute
  grams. In the twin a scoop carves about 98 g, and over half slides off as the scoop leaves
  the bed lip-down. The shake-off then leaves the level fill at the lift tilt: about 40 g
  in Isaac and the env for the authored scoop.
  - `scoop_env_compare`: heuristic 34 ± 3 g per scoop, fitted `fill_efficiency` ≈ 0.44
    (planner 0.5).
  - `target_fill_ratio` 1.2 (48 g) is above what the shaken bowl holds, so the twin always
    under-fills against it.
- The lip-down exit spill depends on arm speed. Isaac precomputes bowl capacity along the
  authored scoop at startup. Otherwise the first fast scoop holds its payload until the
  capacity worker catches up.
- With `min_pour_tilt_deg: 10`, the PourStart pose (5°) does not pour, so TRICKLE-phase
  dosing never flows.

## Tests

```bash
python3 -m pytest src/isaac_twin/test -q
```
