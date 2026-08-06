# robot_orchestrator

BehaviorTree.CPP v4–based orchestrator for powder scooping → transport → pouring.

## Overview

- Main node: `orchestrator_node` (see `src/orchestrator_main.cpp`)
  - Creates a shared `rclcpp::Node` and `BT::Blackboard`
  - Loads XML (`bt_trees/main.xml`)
  - Publishes to Groot2 (port 1666)
  - Exposes services to control execution (start batch, pause, resume, stop)
- Custom BT nodes (registered in `src/register_nodes.cpp`):
  - `MoveTo` (action client to MoveTo server)
  - `ScanQr` (action client to ScanQr server)
  - `PourToTarget` (action client to pouring_controller)
  - `ProjectContainer` (sync; projects a `ContainerSpec` to blackboard keys)
  - `Vibrate` (stateful; publishes Float64 0..1 to `/vibration/intensity` for a duration at ~10 Hz keepalive)
  - `SetIncline` (sync; publishes Float64 degrees to `/incline_control`)
  - `QueueStatus` (sync; logs and publishes remaining queue count)

## Data flow

- `/bt_start_batch` (`robot_common_msgs/srv/StartBatch`) provides MES batch data:
  - `containers[]`: each has `name`, `expected_lot`, `target_weight`, `weight_tolerance`, optional `pose`.
- The service callback stores `containers` on the blackboard and resets an index. No `FetchBatch` node is used.
- The XML loops over `containers` (queue-style). For each container:
  - `ProjectContainer` sets: `container_name`, `expected_lot`, `container_target_g`, `batch_weight_tolerance`, and derived names like `qr_target_name`.
  - Flow: `MoveTo` → QR area → `ScanQr` → `MoveTo` container → `MoveTo` scale → `PourToTarget`.
  - The pour decision logic (baseline capture, net weight, settle time, phase control, overshoot handling) lives inside the `pouring_controller` action server.
  - After pour completes (achieved OR overshoot), a cleanup sequence runs: go to container → go to `tap_off` → `Vibrate` → go `home`.

## MoveTo integration

The `MoveTo` BT node sends goals to `robot_common_msgs/action/MoveTo`.
It supports:
- `target_name` (named pose from `targets.yaml`) or literal pose (`frame_id`, `target_x/y/z`, `qx/qy/qz/qw`)
- Cartesian vs joint-space (`use_cartesian`, `eef_step`, `jump_threshold`)
- Waypoint names (`waypoint_names`: vector<string>)
- Speed scaling (per-goal `velocity_scaling`, `acceleration_scaling`, and per-waypoint arrays for joint-space)

Example XML (semicolon-separated lists):
```xml
<Sequence name="MoveViaWaypoints">
  <!-- Joint-space with per-waypoint speeds -->
  <MoveTo waypoint_names="pre_pick;pick;retreat"
          use_cartesian="false"
          velocity_scaling="0.15"
          acceleration_scaling="0.15"
          waypoint_velocity_scaling="0.15;0.05;0.10"
          waypoint_acceleration_scaling="0.15;0.05;0.10" />

  <!-- Cartesian with global speed -->
  <MoveTo waypoint_names="approach;place"
          use_cartesian="true"
          eef_step="0.01"
          jump_threshold="0.0"
          velocity_scaling="0.10"
          acceleration_scaling="0.10" />

  <!-- Single named target -->
  <MoveTo target_name="home" use_cartesian="false"
          velocity_scaling="0.2" acceleration_scaling="0.2" />

  <!-- Literal pose -->
  <MoveTo frame_id="base_link" target_x="0.30" target_y="0.20" target_z="0.15"
          qx="0" qy="0" qz="0" qw="1" use_cartesian="false" />
</Sequence>
```

## Running

1) Build deps and orchestrator:
```
colcon build --packages-select robot_common_msgs robot_moveit robot_orchestrator
source install/setup.bash
```
2) Start MoveIt action server and QR/pour servers.
   Required action servers and how to start them:
   - MoveTo (robot_moveit):
     ```bash
     ros2 run robot_moveit move_to_server_node --ros-args -p targets_yaml:=/path/to/targets.yaml
     ```
     Provides action `/move_to` (robot_common_msgs/action/MoveTo)
   - ScanQr (qr_scanner):
     ```bash
     ros2 run qr_scanner scan_qr_server
     ```
     Provides action `/scan_qr` (robot_common_msgs/action/ScanQr)
   - PourToTarget (pouring_controller):
     ```bash
     ros2 run pouring_controller pour_server_node --ros-args \
       -p weight_topic:=/weight -p vibration_topic:=/vibration/intensity -p joint_state_topic:=/joint_states
     ```
     Provides action `/pour_to_target` (robot_common_msgs/action/PourToTarget)
3) Run orchestrator:
```
ros2 run robot_orchestrator orchestrator_node --ros-args -p tree_file:=/home/ashwanth/rhapsodi-promtek/src/robot_orchestrator/bt_trees/main.xml
```
4) Open Groot2 (connect to port 1666) to visualize.

## Scooping -> weighing -> pouring demo tree

A dedicated test tree is available at:

* `bt_trees/scoop_weigh_pour.xml`

This tree executes:

1. Move to a named scooping approach pose from `robot_moveit/config/targets.yaml`
2. Trigger the scooping MTC primitive via `ExecuteScoop`
3. Move to a named weighing-container pose from `robot_moveit/config/targets.yaml`
4. Run the existing weight-driven `PourToTarget` action
5. Return home using a named home pose from `robot_moveit/config/targets.yaml`

Required named targets in `robot_moveit/config/targets.yaml`:

* `MoveToScoopingContainer`
* `MoveToWeighingContainer`
* `ReturnHome`

## Webhook weightment tree

For backend-driven webhook execution there is a dedicated tree at:

* `bt_trees/webhook_weightment.xml`

It starts from the explicit service:

* `/bt_start_webhook_weightment` (`robot_common_msgs/srv/StartWebhookWeightment`)

This tree expects the backend adapter to resolve the pickup/weigh/home target names and convert kilograms to grams before the run starts. The backend calls this service through the central rosbridge server instead of a custom ROS-side bridge node.

### End-to-end test commands

Build everything once:

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scooping_controller robot_moveit robot_orchestrator pouring_controller data_collection_manager --symlink-install
source install/setup.bash
```

Terminal 1: start the scooping simulation stack.
This now also starts:
`move_group`, `scooping_mtc_node`, RViz, interactive markers,
collision-aware planning scene, `move_to_server_node`, and `target_recorder_node`.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch scooping_controller scooping_simulation.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml
```

Terminal 2: start the weight simulator.
This configuration keeps the simulated weight at the baseline until the BT publishes
`pour_start`, then ramps only during the pouring phase.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run data_collection_manager weight_sim --ros-args \
  -p topic:=/weight \
  -p rate_hz:=20.0 \
  -p baseline:=0.0 \
  -p target:=400.0 \
  -p gate_on_phase:=true \
  -p phase_topic:=/lightsout_training/phase \
  -p phase_start:=pour_start \
  -p phase_end:=pour_end \
  -p use_metadata_target:=false
```

Terminal 3: start the pouring action server.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pouring_controller pour_server_node --ros-args \
  -p weight_topic:=/weight \
  -p vibration_topic:=/vibration/intensity \
  -p joint_state_topic:=/joint_states
```

Terminal 4: start the orchestrator with the scooping demo tree.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_orchestrator orchestrator_node --ros-args \
  -p tree_file:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_orchestrator/bt_trees/scoop_weigh_pour.xml
```

Terminal 5: trigger one test episode.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /bt_start_lightsout robot_common_msgs/srv/StartLightsOut \
"{powder_name: 'scooping_demo', cycle_end_limit: '1 episode', target_weight_g: 30.0, episodes: 1, batch_id: 'demo-run', enable_scoop: true}"
```

Notes:

* `ExecuteScoop` calls `/execute_scoop_continuous` by default
* the scoop pattern offset is still controlled through the `scooping_mtc_node` parameter `pattern_offset_y`
* `move_to_server_node` reloads `targets.yaml` automatically when a goal uses `target_name` or `waypoint_names`
* `weight_sim` publishes an absolute scale reading. If you want to simulate pouring `30 g` into an already loaded container at `230 g`, use `baseline:=230.0` and `target:=260.0`

### Real-hardware bring-up

The hardware path is intentionally separate from `scooping_simulation.launch.py`.
Use the dedicated launch below so the stack starts without Gazebo, `/clock`, or the sim-only static `world` frame.

Before starting:

* set `ROBOT_IP` if the default value in `niryo_ned_ros2_driver/config/drivers_list.yaml` is not your robot
* calibrate `scooping_controller/config/container_scene_real.yaml` for the physical scooping and weighing containers
* choose a real target file, for example `src/robot_moveit/config/targets.yaml`

Terminal 1: start the real Niryo + MoveIt + scooping authoring stack.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROBOT_IP=169.254.200.200
ros2 launch scooping_controller scooping_real.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml \
  container_scene_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/scooping_controller/config/container_scene_real.yaml
```

Terminal 2: start the physical scale on `/weight`.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch weighing_scale_driver weighing_scale.launch.py \
  port:=/dev/ttyUSB0 \
  baud:=115200 \
  topic:=/weight
```

Terminal 3: start the pouring action server in grams.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pouring_controller pour_server_node --ros-args \
  -p weight_topic:=/weight \
  -p vibration_topic:=/vibration/intensity \
  -p joint_state_topic:=/joint_states
```

Terminal 4: start the orchestrator with the scooping demo tree.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_orchestrator orchestrator_node --ros-args \
  -p tree_file:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_orchestrator/bt_trees/scoop_weigh_pour.xml
```

### Real validation order

Use this order on hardware before you run the full BT:

1. Verify joint states and MoveIt are live, then send a slow `ReturnHome` with `/move_to`.
2. Re-record `ReturnHome`, `MoveToScoopingContainer`, and `MoveToWeighingContainer` with `/record_target`.
3. Adjust `container_scene_real.yaml` until the RViz meshes and MoveIt collision scene match the physical containers.
4. Save the five scoop markers into `poses_real.yaml`, then run `/plan_scoop` before any execution.
5. Execute the scoop in free space first, then near the real container, then enable the scale and `PourToTarget`.
6. Run the BT for a single cycle only after `MoveTo`, `ExecuteScoop`, and `PourToTarget` have each passed on their own.

## Lights-out training (repeat episodes)

Use the dedicated lights-out tree to repeat scoop → weigh → pour for a fixed
number of episodes. Start it with the `/bt_start_lightsout` service.

1) Run orchestrator with the lights-out tree:
```
ros2 run robot_orchestrator orchestrator_node --ros-args -p tree_file:=/home/ashwanth/rhapsodi-promtek/src/robot_orchestrator/bt_trees/lightsout.xml
```

2) Start a lights-out run:
```
ros2 service call /bt_start_lightsout robot_common_msgs/srv/StartLightsOut "{powder_name: 'alumina', cycle_end_limit: '10 episodes', target_weight_g: 125.0, episodes: 10, batch_id: 'batch-2026-01-19', enable_scoop: false}"
```

Lights-out topics:
- `/lightsout_training/active` (`std_msgs/Bool`)
- `/lightsout_training/metadata` (`std_msgs/String`, JSON payload)

### Scoop residue purge (end of episode)

After `RecordPourOutcome` the tree closes the episode recorder
(`EpisodeEndMarker`), then optionally dumps leftover powder from the scoop
back into the vessel before parking:

1. optional `MoveTo` to `lightsout_purge_target` (empty = purge in place at the pour pose)
2. `SetIncline` to `lightsout_purge_incline_deg` (default 20°) via `/incline_control` (joint_5)
3. `Vibrate` at `lightsout_purge_vibration` (default 0.8) for `lightsout_purge_duration_s` (default 10 s), then settle
4. restore upright tilt (`SetIncline 0`)
5. `MoveTo` back to the scooping container

Purge is wrapped in `ForceSuccess` so a purge hiccup does not abort an
otherwise good session. Cell tuning (not per-run service fields):

```
-p lightsout_purge_enabled:=true
-p lightsout_purge_incline_deg:=20.0
-p lightsout_purge_vibration:=0.8
-p lightsout_purge_duration_s:=10.0
-p lightsout_purge_target:=   # empty, or a taught MoveTo name
```

Compose / `robot-prod.env` mirrors these as `LIGHTSOUT_PURGE_*`.

### Control services

- Start batch: `/bt_start_batch` (`robot_common_msgs/srv/StartBatch`)
- Pause: `/bt_pause` (`std_srvs/srv/Trigger`)
- Resume: `/bt_resume` (`std_srvs/srv/Trigger`)
- Stop: `/bt_stop` (`std_srvs/srv/Trigger`)
- ASCII dump: `/bt_dump` (`std_srvs/srv/Trigger`) → prints the tree to node log

### Monitoring topics

- `/system_status` (`robot_common_msgs/msg/SystemStatus`): phase, queue remaining, current container, targets, tolerance, and scale weight
- `/orchestrator/run_state` (`std_msgs/String`): `idle`, `running`, `succeeded`, `failed`, or `stopped`

Cleanup/tap-off targets:
- Define `tap_off` (and any approach) in `robot_moveit/config/targets.yaml`.
  Example:
  ```yaml
  tap_off:
    header: {frame_id: base_link}
    pose: {position: {x: 0.25, y: 0.0, z: 0.35}, orientation: {x: 0, y: 0, z: 0, w: 1}}
  ```

## Demo: start a batch

Call the service with two containers, one with a known pose and one without:

```
ros2 service call /bt_start_batch robot_common_msgs/srv/StartBatch "{containers: [
  {name: 'boxA', expected_lot: 'https://en.wikipedia.org/wiki/URL', has_pose: true,
   pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.30, y: 0.20, z: 0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}},
   target_weight: 100.0, weight_tolerance: 1.0},
  {name: 'boxB', expected_lot: 'https://en.wikipedia.org/wiki/URL', has_pose: false,
   target_weight: 50.0, weight_tolerance: 0.5}
]}"
```

Notes:
- Ensure the `MoveTo` action server is running and `targets.yaml` contains required named targets (e.g., `scale`, container names if you use `target_name`).
- `move_to_server_node` reloads `targets.yaml` automatically when a goal uses `target_name` or `waypoint_names`.

## Notes
- `targets.yaml` changes require restarting the `move_to_server_node`.
- Ensure frames are consistent (generally `base_link`) for named targets.
- For Cartesian paths, per-waypoint speed arrays are ignored (single time-parameterized trajectory).

### Timing and settling delays (important)

Small waits between motions are critical for stable execution and sensing. We observed that adding short delays (we use 0.5 s) between steps eliminates intermittent controller start-state mismatches and ensures sensors (e.g., weight, QR) have time to update before the next action.

- Where delays are placed in `bt_trees/main.xml`:
  - After `FetchBatch`
  - After `ProjectContainer`/`QueueStatus`
  - Between `MoveToQr` and `ScanQr`, and after `ScanQr`
  - Inside the re-scoop cycle: after `MoveToContainer`, after `MoveToScale`, before and after `PourToTarget`
  - Before cleanup moves
- Recommended starting value: 0.5 s (`<WaitSeconds seconds="0.5"/>`). Tune to your hardware/network latency and sensor update rates.
- Symptoms if missing: sporadic `Invalid Trajectory: start point deviates...`, stale sensor readings, skipped visual updates.

## Simplified BT flow (high level)

- Loop containers:
  - `ProjectContainer` → `MoveTo` QR → `ScanQr` → `MoveTo` container → `MoveTo` scale → `PourToTarget` → Cleanup (`Vibrate`, home)
- `PourToTarget` returns SUCCESS on overshoot as well, to still trigger Cleanup.






