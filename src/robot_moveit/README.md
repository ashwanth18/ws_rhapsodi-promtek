# robot_moveit

MoveIt-based utilities and action server for point-to-point and waypoint motion, plus a helper to record named targets.

## Nodes

- MoveTo action server
  - Executable: `move_to_server_node`
  - Action: `/move_to` (`robot_common_msgs/action/MoveTo`)
  - Plans and executes joint-space or Cartesian motions to:
    - a named target from `targets.yaml`
    - a literal PoseStamped
    - a sequence of waypoints (literal and/or named), with optional per-waypoint speed scaling (joint-space)

- Target recorder
  - Executable: `target_recorder_node`
  - Service: `/record_target` (`robot_common_msgs/srv/RecordTarget`)
  - Captures the current end-effector pose (or joint state) and appends it to `targets.yaml` as a named target.

## Build

```bash
colcon build --packages-select robot_common_msgs robot_moveit
source install/setup.bash
```

## Configuration

- `targets.yaml`: named targets in the planning frame (typically `base_link`).
  Example:
```yaml
home:
  header: {frame_id: base_link}
  pose:
    position: {x: 0.25, y: 0.0, z: 0.30}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
scale:
  header: {frame_id: base_link}
  pose:
    position: {x: 0.40, y: 0.00, z: 0.20}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
```
Note: restart `move_to_server_node` after editing `targets.yaml` so it reloads.

## MoveTo action server

Start:
```bash
ros2 run robot_moveit move_to_server_node --ros-args \
  -p planning_group:=arm \
  -p targets_yaml:=/absolute/path/to/targets.yaml \
  -p velocity_scaling:=1.0 -p acceleration_scaling:=1.0 \
  -p planning_time:=5.0 -p cartesian_avoid_collisions:=false
```

Node parameters:
- `planning_group` (string, default `arm`)
- `targets_yaml` (string, required): path to YAML with named targets
- `velocity_scaling` (double, default 1.0) global cap (0..1)
- `acceleration_scaling` (double, default 1.0) global cap (0..1)
- `planning_time` (double, default 5.0)
- `cartesian_avoid_collisions` (bool, default false)

Action goal (selected fields):
- `target_name` (string) OR `target_pose` (PoseStamped)
- `waypoints` (PoseStamped[]), `waypoint_names` (string[])
- `use_cartesian` (bool), `eef_step` (float), `jump_threshold` (float)
- Per-goal scaling (0..1): `velocity_scaling`, `acceleration_scaling`
- Per-waypoint scaling (joint-space only): `waypoint_velocity_scaling[]`, `waypoint_acceleration_scaling[]`

Examples:
- Move to a named target:
```bash
ros2 action send_goal /move_to robot_common_msgs/action/MoveTo "{target_name: 'home'}"
```
- Move to a literal pose (base_link):
```bash
ros2 action send_goal /move_to robot_common_msgs/action/MoveTo \
"{target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.30, y: 0.20, z: 0.25}, orientation: {x: 0, y: 0, z: 0, w: 1}}}, \
  use_cartesian: false, velocity_scaling: 0.2, acceleration_scaling: 0.2}"
```
- Cartesian through two named waypoints:
```bash
ros2 action send_goal /move_to robot_common_msgs/action/MoveTo \
"{waypoint_names: ['approach','place'], use_cartesian: true, eef_step: 0.01, jump_threshold: 0.0, velocity_scaling: 0.1, acceleration_scaling: 0.1}"
```
Notes:
- Cartesian: a single time-parameterized trajectory is generated; per-waypoint arrays are ignored in this mode.
- Joint-space: per-waypoint scaling arrays apply to segment speeds.

## Recording targets (TargetRecorder)

Start:
```bash
ros2 run robot_moveit target_recorder_node --ros-args \
  -p planning_group:=arm \
  -p targets_yaml:=/absolute/path/to/targets.yaml \
  -p eef_link:=tool_link \
  -p pose_source:=auto   # tf | moveit | auto
```
Parameters:
- `planning_group` (string, default `arm`)
- `targets_yaml` (string, required): file to append new entries
- `eef_link` (string): end-effector link name
- `pose_source` (string: `tf` | `moveit` | `auto`, default `auto`)
  - `tf`: use TF (base_link→eef_link)
  - `moveit`: use MoveIt current state FK
  - `auto`: try TF first, fallback to MoveIt

Record a pose:
```bash
ros2 service call /record_target robot_common_msgs/srv/RecordTarget \
"{name: 'tap_off', joints: false}"
```
Record joints:
```bash
ros2 service call /record_target robot_common_msgs/srv/RecordTarget \
"{name: 'home_joints', joints: true}"
```

## BT integration

- The orchestrator uses `/move_to` to navigate among named targets (e.g., containers, `scale`, `tap_off`).
- The post‑pour cleanup uses a `Vibrate` BT node (publishes Int32 to `/motor_speed`), independent of MoveIt.

## Troubleshooting

- kinematics: ensure `kinematics.yaml` is properly keyed by the planning group (e.g., `arm`).
- `/joint_states`: must be fresh; otherwise MoveIt reports stale state.
- Multiple servers: ensure only one `/move_to` exists to avoid goal/result confusion.
