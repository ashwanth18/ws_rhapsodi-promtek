# isaac_twin: notes for agents

Isaac Sim 5.1 digital twin of the Niryo Ned3 Pro scooping cell (layout `dual-container`).
Isaac simulates the arm, the D455, the powder and the scale input. The **real cell
software** (ros2_control, MoveIt, MTC, scoop_vision, pour_server, the orchestrator BT)
runs unchanged against it. A gymnasium env (`isaac_twin.gym.ScoopEnv`) reuses the same
cell model without Isaac or ROS.

- Learner / engineer explanation: `docs/ISAAC_TWIN.md`. Operator reference: `README.md`
  (this folder).
- Branch `feature/isaac-sim-twin` (pushed, not merged). Main commits: `222c88a` twin +
  gym env, `1fd96b9` draft PourTilt, `7e7b078` angle of repose, `226b133` particle powder,
  `9c33276` / `77e09b4` performance notes.

## Rules (do not break)

1. **Isolation.** The twin only runs on `ROS_DOMAIN_ID=77` with
   `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Source `scripts/twin_env.sh` in every
   shell. It also unsets the Fast DDS profile files, because the laptop's
   `so101_fastdds_eth.xml` disables the builtin transports. The launch refuses any other
   domain. Never use `allow_any_domain:=true` outside CI.
2. **One Isaac at a time.** Two Isaac processes on domain 77 both publish `/clock`. Sim
   time then jumps back and forth: RViz resets (679 times in one incident), TF clears,
   and pours fail mid-run. Before starting a headless run, check
   `ps -eo pid,args | rg "kit/python/bin/python3"`. If the user's twin is running, ask
   them first, and never start a second instance next to it. Stop the twin with:
   `pkill -INT -f "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch isaac_twin"`, then
   `pkill -INT -f "kit/python/bin/python3 .*build_cell.py"`. `pgrep -f` inside
   `bash -c` also matches your own shell's command line, so confirm with `ps -eo pid,args`.
3. **Twin runs move only the simulated arm.** `bt_start_webhook_weightment`, MoveTo and
   `--replay-scoop` on domain 77 are sim-only and allowed. Do not run them inside a twin
   the user is driving unless they ask. Never call run-start services on domain 0 (the
   real cell), per the root rules.
4. **No twin edits to `config/layouts`.** Twin-only targets go in
   `config/draft_targets/<layout>_niryo.yaml`. `draft_targets_node` merges them into
   `~/.cache/isaac_twin/targets_<layout>_draft.yaml` and points `move_to_server` there.
   The `PourTiltAtWeighingContainer` draft still needs an operator check on the real
   cell before it can move into the layout.
5. **Pose caches.** The twin's marker server uses `poses_env: isaac`, so it reads and
   writes `~/.ros/scooping_controller/poses_isaac_<layout>.yaml`. Never touch
   `poses_sim_*` (Gazebo) or the real/bench caches.
6. **Grain size is fixed at 4 mm** (the user's decision on 2026-10-02: "don't change grain
   size"). If `spacing_m` or `solver_iterations` changes for any reason, re-measure
   `settle_ratio`: check that the status log's `depth=` reads 40 mm for
   `fill_depth_m: 0.04`.
7. **Scoop collider stays `voxels`** unless retention is re-validated with
   `--replay-scoop 2`. `sdf` leaks about 1 g/s from a held scoop; `convexDecomposition`
   leaks more.
8. **The heightfield model must keep working.** The gym env uses it. After touching
   `build_cell.py`, check `--powder heightfield --headless --max-seconds 12`: the bed
   should read 2555 g.
9. **Commits** exclude `robot-prod.env` (it has an unrelated local change), `data/`,
   `compose/devices/data/` and the Condor agent zip/folder. Remove temporary env-var
   experiment hooks before committing.

## Current state (2026-10-02)

- `powder.model: particles` is the default; `--powder heightfield` switches.
- Particle bed: 109,604 grains of 4 mm, 23.2 mg each. That is 2546 g, settling to 40 mm
  (seeded 61 mm, `settle_ratio` 0.66 at 48 solver iterations).
- Authored scoop (`--replay-scoop 2`): 77 g in the bed, 23 g after the lift and shake. The
  heightfield model keeps 64 g. Retention is not sensitive to cohesion/friction.
- Twin webhook BT, 20 g target: one scoop, 21 g after the shake, `pour_server` dosed 19.9 g,
  SUCCESS. The mass balance closes to 0.1 g.
- scoop_vision capture of the particle bed: 4602 ml measured vs 4629 ml in the bed.
- Speed: headless 15–16 FPS / 0.5 RTF; GUI + RViz about 12–14 FPS.
  - Per frame, about 50 ms is PhysX's GPU particle solve (4 substeps of 120 Hz). It is
    sequential, so the GPU is never saturated.
  - Dynamic Boost (`nvidia-powerd`) was installed on the laptop host:
    `/etc/systemd/system/nvidia-powerd.service` and `/etc/dbus-1/system.d/nvidia-dbus.conf`.
    That is host-local, not in the repo. It raises the GPU cap from 80 W to 175 W but gives
    only about 5% FPS.
- **Open:**
  - The PhysX error "Non-GPU-compatible convex mesh is not able to collide with particle
    system" appears once at start. Some robot-link collider is not particle-compatible.
    Harmless for scoop, vessels and table.
  - Vibration is a random velocity kick on the grains inside the scoop's box. A more
    physical alternative is an oscillating scoop collider, which needs the motor's
    frequency and amplitude from the user.
  - All powder parameters are unfitted guesses for flour.
  - `joint_5` sits at -1.86 rad (limit -1.92) on the draft PourTilt.
  - The `ComputeRemaining` overshoot fix (`CheckOvershoot` BT node) is on branch
    `fix/weightment-overshoot`, built in worktree `/home/ashwanth/ws_overshoot`. Its PR is
    not opened.
  - Earlier, the twin's first launch overwrote the Gazebo cache `poses_sim_dual-container.yaml`.

## Architecture in one screen

```
Isaac process (Isaac python 3.11 + bundled Jazzy rclpy)      Native ROS 2 Jazzy (python 3.12)
 build_cell.py                                                scooping_isaac.launch.py
  URDF articulation (stiff position drives)  <- /isaac_joint_commands <- ros2_control TopicBasedSystem <- JTC <- MoveIt / MTC / move_to
                                             -> /isaac_joint_states  ->            (100 Hz)
  OmniGraph: /clock (sim time)               -> every node (use_sim_time)
  D455 render products                       -> /isaac/camera/depth/* -> depth_to_mm -> /camera/depth/* -> scoop_vision
                                             -> /camera/color/*                         sim_camera_tf: camera_link -> optical frames
  Powder (particles | heightfield)           -> /isaac_twin/{rs3_mass_g,payload_g,bed_mass_g}
                                             <- /vibration/intensity <- pour_server, scooping_mtc_node (post-lift shake)
                                             <- /isaac_twin/reset_powder (Trigger)
                                             rs3_mass_g -> twin_scale_node -> /weight -> pour_server, BT
```

- The launch reads one `/isaac_joint_states` message and passes it as `initial_joint_N`
  into the xacro. TopicBasedSystem publishes its command buffer even with no controller
  active, so without this seed the arm snaps to zero.
- Sim time is held to wall time (never faster). When Isaac is slow, the whole stack
  slows with it.

## File map

| File | Role |
|------|------|
| `scripts/run_isaac_twin.sh` | xacro expand to `~/.cache/isaac_twin/niryo_ned3pro.urdf`; symlink mesh names that are invalid USD paths (`-`); source `twin_env.sh`; exec Isaac's `python.sh build_cell.py` with Isaac's bundled Jazzy libs |
| `scripts/twin_env.sh` | Domain 77, LOCALHOST discovery, unset Fast DDS profiles, `rmw_fastrtps_cpp` |
| `isaac_twin/scene/build_cell.py` | Isaac entry point: scene, robot import + drives, D455, joint bridge, powder model switch, main loop with wall-clock pacing, status log (`rtf`, `fps`, `step`/`powder` ms, grams, depth), `--replay-scoop`, `--dump-particles` |
| `isaac_twin/scene/particles.py` | PhysX side of particle mode: GPU scene, link gravity off, static vessel colliders, `FilteredPairsAPI`, scoop collider swap (de-instancing), `ParticlePowder` (PBD system, material, red points, readback, vibration kick) |
| `isaac_twin/particle_powder.py` | Isaac-free bookkeeping: `seed_bed`, `particle_grams` (settle ratio), `bed_depth_m`, `ParticleAccounting` (bed/scoop/RS3/table labels) |
| `isaac_twin/powder.py` | Heightfield model: `PowderBed` carve/deposit, `BowlCapacity` (level-fill vs gravity, disk cache), `ScoopParams`, `PowderCell` (repose, heap, vibration flow, toward-lip pour, landing) |
| `isaac_twin/scene/joint_bridge.py` | OmniGraph: joint state pub, joint command sub → articulation controller, `/clock` |
| `isaac_twin/scene/d455.py` | Two RTX cameras (depth 848×480, colour 1280×720) and ROS camera helpers; `frameSkipCount` only skips publishing |
| `isaac_twin/cell.py` | Isaac-safe (numpy + yaml) layout parsing, URIs, robot tool block, calibration loading, D455 frame offsets, scoop_vision param merge |
| `isaac_twin/kinematics.py` | Numpy URDF chain: FK, geometric Jacobian, damped least-squares IK (one seed) |
| `isaac_twin/scoop_path.py` | Authored poses, chained IK, cartesian/joint paths, `timed_scoop` + `sample_knots` (replay) |
| `isaac_twin/depth_to_mm_node.py` | 32FC1 m → 16UC1 mm, noise `σ = 0.8 mm + 0.002·z²`, 1% holes, decimated cloud |
| `isaac_twin/twin_scale_node.py` | RS3 grams → `/weight` with first-order lag (τ 0.4 s), 0.05 g noise, 0.01 g resolution, `~/tare` |
| `isaac_twin/sim_camera_tf.py` | realsense-style static frames under `camera_link` |
| `isaac_twin/draft_targets_node.py` | Merge draft targets over the layout's, keep `move_to_server.targets_yaml` on the merged file |
| `isaac_twin/gym/scoop_env.py`, `gym/compare.py` | `IsaacTwin/Scoop-v0` env; heuristic vs authored/random plus `fill_efficiency` fit (`ros2 run isaac_twin scoop_env_compare`) |
| `launch/scooping_isaac.launch.py` | Isolation check, Isaac pose seed, the cell's nodes with twin params, staged start |
| `urdf/niryo_ned3pro_isaac.urdf.xacro` | Niryo URDF plus the `TopicBasedSystem` ros2_control block |
| `config/twin.yaml` | Isaac-side config (not ROS params): rates, drives, camera, powder (both models), scoop model |
| `config/twin_nodes.yaml` | ROS params for the twin nodes, pour nodes and the scoop_vision overrides (`surface_correction_m: 0`) |
| `config/ros2_controllers_isaac.yaml` | JTC at 100 Hz, loose goal tolerances (Isaac drives lag slightly) |
| `config/draft_targets/dual-container_niryo.yaml` | Twin-only `PourTiltAtWeighingContainer` (PourStart + 10° about tool Y) |
| `test/` | 41 pytest tests (cell, kinematics, heightfield powder, particle bookkeeping, timed scoop, gym env) |

## Gotchas found the hard way

- **`topic_based_ros2_control` from NVIDIA's apt package segfaults** `ros2_control_node`. It
  was built against an older `hardware_interface`. Build it from source, pinned in
  `src/ros2.repos`.
- **USD path names:** the URDF importer names prims after mesh files, and
  `niryo_scoop_v4-ros.STL` is invalid. `run_isaac_twin.sh` symlinks sanitised names.
- **DLSS** rendered the 848×480 depth at 424×240. `build_cell.py` sets `/rtx/post/aa/op = 0`.
- **The first launch spends minutes compiling shaders.** The main loop drops pacing debt
  above 0.5 s so the sim doesn't then sprint to catch up.
- **Isaac's ROS needs Isaac's Python.** Run `build_cell.py` only through `python.sh` with
  the bridge's Jazzy libs; `run_isaac_twin.sh` strips the native ROS env.
- **Particle seed blow-out:** seeding grains overlapping (rest offset ≥ s/2) fires them
  through the floor. Use `rest_ratio` 0.45, jitter 0.04, CCD, `max_velocity` 1 m/s and
  depenetration 0.2 m/s.
- **Triangle-mesh vessels leak** at RS6's floor/ramp crease, and SDF was worse. Use
  `convexDecomposition`.
- **A `CollisionGroup` for the robot made the position-driven arm drift** off its targets.
  Use `FilteredPairsAPI` on each cell collider instead.
- **The importer's box collider on `tool_link` fills the bowl.** It sits under
  instanceable prims, which `GetAllChildren()` never enters. Walk
  `Usd.PrimRange(..., Usd.TraverseInstanceProxies())`, de-instance, then disable. Until
  this was fixed, no grain ever entered the bowl.
- **`np.asarray(attr.Get())` from USD is read-only.** Writing into it raised in
  `_shake`, and Kit then crashed on exit, which looked like a physics segfault. Use
  `np.array(...)`.
- **PBD compresses under load:** at the first iteration count, a 40 mm seed settled to
  25 mm. 48 iterations is much stiffer, and `settle_ratio` absorbs the rest. Iterations
  cost no measurable speed (32 → 48 changed nothing).
- **`/physics/updateParticlesToUsd=false` does not stop particle position updates**, and
  gives no speed-up.
- **`sim.step(render=False)` advances one physics step, not a full render frame.** Timings
  measured that way are not comparable.
- **Kit's "CPU performance profile is set to powersave" warning is a false alarm here.**
  `intel_pstate`'s `powersave` is the dynamic governor, and EPP is `performance`.
- **Discovery is slow with about 60 nodes.** Use
  `ros2 topic list --no-daemon --spin-time 12`. The gram topics are `/isaac_twin/*_g`;
  there is no `powder_status`.
- **The marker server refuses pose caches whose `authored_in` doesn't match** ("provenance
  refuse") and reseeds from `config/layouts/<layout>/poses.yaml`. That is how the twin's
  first launch overwrote the Gazebo `poses_sim_*` cache, and why the twin uses
  `poses_env`/`authored_in: isaac`. The provenance errors at startup are expected.

## How to test

```bash
python3 -m pytest src/isaac_twin/test -q                 # 41 tests, no Isaac/ROS
python3 -m pyflakes src/isaac_twin/isaac_twin src/isaac_twin/test   # only test_scoop_env's gym-registration import
colcon build --packages-select isaac_twin --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# Isaac only, no ROS stack: settle, then the authored scoop + shake (check the user isn't running a twin)
bash src/isaac_twin/scripts/run_isaac_twin.sh --headless --max-seconds 21                 # depth=40mm, ~2546 g
bash src/isaac_twin/scripts/run_isaac_twin.sh --headless --replay-scoop 2 --max-seconds 17 --dump-particles /tmp/p.npz
bash src/isaac_twin/scripts/run_isaac_twin.sh --headless --powder heightfield --max-seconds 12
```

Full flow (sim only): Isaac, then the launch (`use_rviz:=false` for speed). Then
`/move_to {target_name: CameraClear}`, `/scoop_vision/capture`, `/scoop_vision/plan`,
`/twin_scale/tare`, then
`/bt_start_webhook_weightment robot_common_msgs/srv/StartWebhookWeightment "{run_id: x, target_weight_g: 20.0, tolerance_g: 1.0}"`.
Watch the Isaac status lines in
`~/isaacsim/isaac-sim-standalone-5.1.0-linux-x86_64/kit/logs/Kit/Isaac-Sim Python/5.1/kit_*.log`.
Find the active log with `ls -l /proc/<pid>/fd | rg kit_`.

Profiling: pass Kit args through `SimulationApp({... "extra_args": [...]})`:
`--/app/profilerBackend=cpu --/app/profileFromStart=1 --/profiler/enabled=1
--/plugins/carb.profiler-cpu.plugin/saveProfile=1 --/plugins/carb.profiler-cpu.plugin/compressProfile=0
--/plugins/carb.profiler-cpu.plugin/filePath=/tmp/kit_trace.json`. That writes a Chrome
trace (600 MB for 8 s). Aggregate the main thread's `X` events by name.
