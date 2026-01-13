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
  - `Vibrate` (sync; publishes Int32 to vibration topic for a duration)
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
       -p weight_topic:=/weight -p vibration_topic:=/motor_speed -p joint_state_topic:=/joint_states
     ```
     Provides action `/pour_to_target` (robot_common_msgs/action/PourToTarget)
3) Run orchestrator:
```
ros2 run robot_orchestrator orchestrator_node --ros-args -p tree_file:=/home/ashwanth/rhapsodi-promtek/src/robot_orchestrator/bt_trees/main.xml
```
4) Open Groot2 (connect to port 1666) to visualize.

### Control services

- Start batch: `/bt_start_batch` (`robot_common_msgs/srv/StartBatch`)
- Pause: `/bt_pause` (`std_srvs/srv/Trigger`)
- Resume: `/bt_resume` (`std_srvs/srv/Trigger`)
- Stop: `/bt_stop` (`std_srvs/srv/Trigger`)
- ASCII dump: `/bt_dump` (`std_srvs/srv/Trigger`) → prints the tree to node log

### Monitoring topics

- `/system_status` (`robot_common_msgs/msg/SystemStatus`): phase, queue remaining, current container, targets, tolerance, and scale weight

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
- If you record new targets with the `RecordTarget` service, restart the `move_to_server_node` to reload the YAML.

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






