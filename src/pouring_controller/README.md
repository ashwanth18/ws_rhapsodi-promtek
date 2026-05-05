# pouring_controller

Action server that controls powder pouring using a weight feedback loop and normalized vibration
intensity output. It keeps the existing `/pour_to_target` action contract while moving the actuator
side to `/vibration/intensity`.

## Node: pour_server_node

- Action: `/pour_to_target` (robot_common_msgs/action/PourToTarget)
- Subscriptions:
  - Float64 `weight_topic` (default `/weight`)
  - JointState `joint_state_topic` (default `/joint_states`)
- Publications:
  - Float64 `vibration_topic` normalized 0..1 (default `/vibration/intensity`)
  - Float64 `valve_topic` (default `/valve_control`)
  - Float64 `incline_topic` (default `/incline_control`)
- Optional: FollowJointTrajectory client to tilt a configured joint

## Action interface

Goal
- `target_weight` (g), `tolerance` (g), `max_time_s` (s)

Result
- `achieved`, `overshoot`, `timeout`, `final_weight`, `message`

Feedback
- `current_weight` (g)
- `phase`: COARSE | SETTLE | FINE | TRICKLE
- `error_to_next_band` (g), `band_threshold` (g)
- `hold_time_remaining` (s)

## Phase logic (percentage bands)

- Initial phase is target-size aware:
  - start in `TRICKLE` when `target_weight <= start_in_trickle_below_g`
  - else start in `FINE` when `target_weight <= start_in_fine_below_g`
  - else start in `COARSE`
- COARSE → SETTLE when `|target - filtered| ≤ coarse_threshold × target`
- SETTLE lasts `settle_time_s` seconds
- SETTLE → FINE after `settle_time_s` elapses
- FINE → TRICKLE when `|target - filtered| ≤ fine_threshold × target`
- Success after `hold_within_tol_count` consecutive cycles within tolerance, then wait
  `final_settle_time_s` and report stabilized `final_weight`

## Parameters (key)

- `weight_topic` (string, `/weight`)
- `vibration_topic` (string, `/vibration/intensity`) [Float64, 0..1]
- `valve_topic` (string, `/valve_control`), `incline_topic` (string, `/incline_control`) [Float64]
- `joint_state_topic` (string, `/joint_states`)
- `ema_alpha` (double, 0.2), `sample_rate_hz` (double, 12.0), `stale_ms` (double, 500.0)
- `coarse_threshold` (double, default `0.40`), `fine_threshold` (double, default `0.05`)
- `start_in_fine_below_g` (double, default `40.0`): skip coarse and begin in `FINE`
  for smaller pour targets
- `start_in_trickle_below_g` (double, default `10.0`): skip straight to `TRICKLE`
  for very small top-up pours
- `settle_time_s` (double), `hold_within_tol_count` (int), `final_settle_time_s` (double)
- `min_progress_g` (double, default `0.5`): minimum increase in net poured mass that counts as progress
- `no_progress_timeout_s` (double, default `3.0`): abort if progress is below `min_progress_g`
  for longer than this during `COARSE` or `FINE`
- Dynamic incline recovery before rescoop:
  - `no_progress_incline_step_deg` (double, default `5.0`): add this much incline each
    time the no-progress watchdog fires
  - `max_incline_deg` (double, default `20.0`): cap for commanded incline
- Per-phase normalized vibration tuning:
  - `coarse_vibration_intensity`, `settle_vibration_intensity`, `fine_vibration_intensity`,
    `trickle_vibration_intensity`
  - `trickle_pulse_ms`, `trickle_pause_ms`
- PI tuning when `control_law_type:=pid`:
  - `pid_kp`, `pid_ki`, `pid_kd`, `pid_feedforward_intensity`, `pid_integral_limit`
- Optional tilt (FollowJointTrajectory):
  - `tilt_joint_name`, `traj_action_server`, `coarse_tilt_deg`, `fine_tilt_deg`, `trickle_tilt_deg`, `joint_move_time_s`

The runtime server and `/pour_status` UI topic both operate in grams.

### Control behavior

The action goal still uses grams of net mass to add on top of the current baseline. Internally the
controller now publishes normalized vibration intensity and uses the same live `/weight` stream to
drive phase transitions and stop conditions:

* `COARSE`: high approach intensity to get flow started quickly on larger pour targets
* `SETTLE`: low or zero intensity while the scale settles
* `FINE`: controller output capped by `fine_vibration_intensity`; can also be the starting phase for
  medium-sized remainder pours
* `TRICKLE`: controller output capped by `trickle_vibration_intensity` and optionally pulsed using
  `trickle_pulse_ms` and `trickle_pause_ms`; can also be the starting phase for very small top-ups

If `COARSE` or `FINE` makes no progress, the controller now raises `/incline_control` by
`no_progress_incline_step_deg` up to `max_incline_deg` before returning `need_rescoop=true`.
The boost counter is scoped to one `/pour_to_target` goal, so a re-scoop starts from the base
phase incline again.

`incline_control_node` can subscribe to `/incline_control` and apply it to a configured robot joint
through `FollowJointTrajectory`. It captures the current `tilt_joint_name` position as zero incline
when a pour starts, then commands `base + incline_direction * incline_deg` while preserving the
latest positions of the other controller joints. The robot-prod dev compose service starts this node
alongside `pour_server_node`.

When `control_law_type:=bangbang`, the per-phase intensities act as a simple robust default.
When `control_law_type:=pid`, the controller uses feedforward plus PI on the net poured mass while
still respecting the phase caps above.

Because the controller publishes every control cycle, it also satisfies a micro-ROS actuator
watchdog that expects repeated keepalive messages while vibration is active.

## Run

Build and source:
```bash
colcon build --packages-select robot_common_msgs pouring_controller
source install/setup.bash
```

Start server (example):
```bash
ros2 run pouring_controller pour_server_node --ros-args \
  -p weight_topic:=/weight -p vibration_topic:=/vibration/intensity -p joint_state_topic:=/joint_states \
  -p coarse_threshold:=0.40 -p fine_threshold:=0.05 -p settle_time_s:=0.8 -p hold_within_tol_count:=10 -p ema_alpha:=0.2 \
  -p coarse_vibration_intensity:=0.9 -p settle_vibration_intensity:=0.0 -p fine_vibration_intensity:=0.70 -p trickle_vibration_intensity:=0.5 \
  -p trickle_pulse_ms:=180 -p trickle_pause_ms:=160 \
  -p tilt_joint_name:=joint_5 -p coarse_tilt_deg:=6 -p fine_tilt_deg:=3 -p trickle_tilt_deg:=1 -p joint_move_time_s:=0.5
```

Send a goal:
```bash
ros2 action send_goal /pour_to_target robot_common_msgs/action/PourToTarget \
"{target_weight: 120.0, tolerance: 0.5, max_time_s: 30.0}" --feedback
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
  - Stall detection: `min_progress_g`, `no_progress_timeout_s`
  - Per-phase vibration intensity and trickle pulsing
  - PI gains and feedforward if you switch to `control_law_type:=pid`

## Notes

- SETTLE can be disabled with `-p settle_time_s:=0`.
- On cancel, success, abort, and timeout: vibration returns to `0.0`.
- If net poured mass does not increase enough for `no_progress_timeout_s`, the action exits early with
  `message="No progress timeout"` and `need_rescoop=true`.
- `pouring_controller` should be treated as the authoritative vibration owner while a pour is active.
