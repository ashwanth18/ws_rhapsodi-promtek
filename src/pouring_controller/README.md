# pouring_controller

Action server that controls powder pouring using a weight feedback loop, per-phase vibration, and optional tilt via joint trajectory. Works with a load cell publishing weight and a motor driver topic for vibration.

## Node: pour_server_node

- Action: `/pour_to_target` (robot_common_msgs/action/PourToTarget)
- Subscriptions:
  - Float64 `weight_topic` (default `/weight`)
  - JointState `joint_state_topic` (default `/joint_states`)
- Publications:
  - Int32 `vibration_topic` raw 0..255 (default `/motor_speed`)
  - Float64 `valve_topic` (default `/valve_control`)
  - Float64 `incline_topic` (default `/incline_control`)
- Optional: FollowJointTrajectory client to tilt a configured joint

## Action interface

Goal
- `target_weight` (kg), `tolerance` (kg), `max_time_s` (s)

Result
- `achieved`, `overshoot`, `timeout`, `final_weight`, `message`

Feedback
- `current_weight` (kg)
- `phase`: COARSE | SETTLE | FINE | TRICKLE
- `error_to_next_band` (kg), `band_threshold` (kg)
- `hold_time_remaining` (s)

## Phase logic (percentage bands)

- COARSE → SETTLE when `|target - filtered| ≤ coarse_threshold × target`
- SETTLE lasts `settle_time_s` seconds
- SETTLE → FINE after `settle_time_s` elapses
- FINE → TRICKLE when `|target - filtered| ≤ fine_threshold × target`
- Success after `hold_within_tol_count` consecutive cycles within tolerance, then wait
  `final_settle_time_s` and report stabilized `final_weight`

## Parameters (key)

- `weight_topic` (string, `/weight`)
- `vibration_topic` (string, `/motor_speed`) [Int32]
- `valve_topic` (string, `/valve_control`), `incline_topic` (string, `/incline_control`) [Float64]
- `joint_state_topic` (string, `/joint_states`)
- `ema_alpha` (double, 0.2), `sample_rate_hz` (double, 12.0), `stale_ms` (double, 500.0)
- `coarse_threshold` (double, e.g. 0.10), `fine_threshold` (double, e.g. 0.02)
- `settle_time_s` (double), `hold_within_tol_count` (int), `final_settle_time_s` (double)
- Per-phase vibration raw (Int32 0..255):
  - `coarse_vibration_raw`, `settle_vibration_raw`, `fine_vibration_raw`, `trickle_vibration_raw`
- Optional tilt (FollowJointTrajectory):
  - `tilt_joint_name`, `traj_action_server`, `coarse_tilt_deg`, `fine_tilt_deg`, `trickle_tilt_deg`, `joint_move_time_s`

## Run

Build and source:
```bash
colcon build --packages-select robot_common_msgs pouring_controller
source install/setup.bash
```

Start server (example):
```bash
ros2 run pouring_controller pour_server_node --ros-args \
  -p weight_topic:=/weight -p vibration_topic:=/motor_speed -p joint_state_topic:=/joint_states \
  -p coarse_threshold:=0.10 -p fine_threshold:=0.02 -p settle_time_s:=0.8 -p hold_within_tol_count:=10 -p ema_alpha:=0.2 \
  -p coarse_vibration_raw:=210 -p settle_vibration_raw:=178 -p fine_vibration_raw:=128 -p trickle_vibration_raw:=90 \
  -p tilt_joint_name:=joint_5 -p coarse_tilt_deg:=6 -p fine_tilt_deg:=3 -p trickle_tilt_deg:=1 -p joint_move_time_s:=0.5
```

Send a goal:
```bash
ros2 action send_goal /pour_to_target robot_common_msgs/action/PourToTarget \
"{target_weight: 1200.0, tolerance: 0.5, max_time_s: 30.0}" --feedback
```

## Testing tips

- Simulate weight:
```bash
python3 - <<'PY'
import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Float64
rclpy.init(); n=Node('sim_scale'); p=n.create_publisher(Float64,'/weight',10)
w=0.0
while rclpy.ok():
  p.publish(Float64(data=w)); w=min(2000.0, w+5.0); time.sleep(1/12)
PY
```

- Tune:
  - Bands: `coarse_threshold`, `fine_threshold`
  - Stabilization: `settle_time_s`, `hold_within_tol_count`, `final_settle_time_s`
  - Per-phase vibration and tilt

## Notes

- SETTLE can be disabled with `-p settle_time_s:=0`.
- On cancel: vibration=0 (Int32), valve=incline=0.0.
