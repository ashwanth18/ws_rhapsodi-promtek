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
- `start_in_fine_below_g` (double, default `80.0`): skip coarse and begin in `FINE`
  for smaller pour targets / rescoop remainders
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
  - `vibration_cmd_max` (default `0.7`): PID/inflight command ceiling (was hard-coded `1.0`)
  - `min_pour_vibration` (default `0.40`): bump nonzero cmds below this up (powder deadband)
  - `trickle_pulse_ms`, `trickle_pause_ms`
- PI tuning when `control_law_type:=pid` or `pid_inflight`:
  - `pid_kp` (default `0.7`), `pid_ki` (default `0.05`), `pid_kd`,
    `pid_feedforward_intensity`, `pid_integral_limit` (default `5.0`)
  - `pid_error_norm_g` (default `100.0`): **fixed** grams of remaining error that map to
    normalized error = 1.0. Command is `u ≈ kp * (err_g / pid_error_norm_g) + …`.
    Do **not** normalize by this goal’s `target_weight` — that would re-saturate every
    rescoop (and a 10 g batch would always look like 100% error). A 500 g first pour
    still saturates early; a ~56 g top-up starts near `0.4` duty instead of max vib.
- In-flight compensation when `control_law_type:=pid_inflight`:
  - `inflight_s` (default `0.80`): vib→scale transport delay
  - `inflight_flow_gain` (default `8.0` in code; **prefer ~4.0** on this cell — see
    “In-flight law” notes below)
  - `inflight_flow_gain_alpha`, `inflight_early_stop_margin_g`
- Optional tilt (FollowJointTrajectory):
  - `tilt_joint_name`, `traj_action_server`, `coarse_tilt_deg`, `fine_tilt_deg`, `trickle_tilt_deg`, `joint_move_time_s`

The runtime server and `/pour_status` UI topic both operate in grams.

### Control behavior

The action goal still uses grams of net mass to add on top of the current baseline. Internally the
controller now publishes normalized vibration intensity and uses the same live `/weight` stream to
drive phase transitions and stop conditions:

* `COARSE`: controller output capped by `coarse_vibration_intensity` (and `vibration_cmd_max`)
* `SETTLE`: low or zero intensity while the scale settles
* `FINE`: controller output capped by `fine_vibration_intensity`; can also be the starting phase for
  medium-sized remainder pours
* `TRICKLE`: controller output capped by `trickle_vibration_intensity` and optionally pulsed using
  `trickle_pulse_ms` and `trickle_pause_ms`; can also be the starting phase for very small top-ups

Nonzero commands below `min_pour_vibration` are raised to that floor (except settle / trickle pause
zeros) so the controller does not dwell in a no-flow deadband.

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
When `control_law_type:=pid_smooth`, the same PI is published directly. The phase machine does
**not** zero vibration in settle, step the coarse/fine/trickle caps, or pulse trickle.
`pid_slew_per_s` defaults to 0, so this node does not ramp the command. Error is normalized by
`pid_smooth_error_norm_g` (default 250 g) so duty follows remaining mass instead of sitting at the
cap until the last 100 g. `pid` is unchanged.
When `control_law_type:=pid_flow`, the flow cascade below runs for the whole pour.
When `control_law_type:=pid_flow_80`, `pid_smooth` runs until 80 g remain, then
the same cascade. The name is the version stored on the run. Phase caps stay off,
and the status phase is `flow` for both.
When `control_law_type:=pid_inflight`, the same PI runs on **predicted** weight
(`scale + pending in-flight mass`) instead of raw scale — see below.

Because the controller publishes every control cycle, it also satisfies a micro-ROS actuator
watchdog that expects repeated keepalive messages while vibration is active.

### In-flight law (`pid_inflight`) — why it matters and what to improve

Powder leaves the scoop before the scale registers it. On the laptop/Jaka cell the vib→scale
transport delay was measured at roughly **0.76–0.80 s**. For precise pouring, a good controller
should stop (or cut vib) based on **mass already committed in the air**, not only what the scale
shows now — otherwise the delayed arrival shows up as overshoot. That is the point of
`pid_inflight`.

**Model (simplified):**

```text
pending_g ≈ flow_gain × Σ(u · dt)   over commands younger than inflight_s (θ)
predicted  = filtered_weight + pending_g
error      = target − predicted
```

`flow_gain` is g/s at vibration duty `u = 1.0` (adapts online from delayed `Δweight / Σ(u·dt)`).
When `predicted` reaches `target − early_stop_margin_g`, vibration is forced to zero.

**Cell evidence (2026-09-16, 100 g webhook pours):**

| Run | Law | Net | Error | Notes |
|-----|-----|-----|-------|-------|
| 21 | bangbang | 105.0 g | +5.0 g | Fast, overshoots |
| 22 | pid | 102.0 g | +2.0 g | Best so far |
| 23 | pid_inflight | 104.5 g | +4.5 g | Worse than PID |

Run 23 diagnosis:

- `θ = 0.80 s` matched the measured delay (keep).
- Initial `inflight_flow_gain = 8.0` **over-credited** pending mass → predicted near target while
  scale was still ~91 g → vib collapsed into the **no-flow deadband** (~0.15–0.35 intensity
  produced ~0 g/s median flow) → ~9.6 s stall → trickle catch-up → overshoot.
- Mean pour vib dropped to ~0.29 vs ~0.62 on plain PID; trickle at ~0.55 still flowed well
  (~5–13 g/s).

**Default for ops:** prefer `control_law_type:=pid` until the pending-mass model is trustworthy
on this powder/vib band. Keep `pid_inflight` in tree — it is the right architecture for
sub-gram precision once gain/deadband are fixed.

**Improvement checklist before re-enabling `pid_inflight` as default:**

1. **Lower / gate `inflight_flow_gain`** — start nearer **3–5** (not 8); only adapt gain when
   recent `u` was above `min_pour_vibration` so deadband windows do not poison the estimate.
2. **Respect the pour deadband in the predictor** — treat `u < ~0.40` as ~0 flow contribution
   (same threshold as `min_pour_vibration`), or freeze pending growth while vib is ineffective.
3. **Do not early-stop into deadband** — if cutting vib, cut to 0; avoid commanding 0.15–0.35
   for long stretches (run 23’s stall). Already partially helped by `min_pour_vibration` on the
   command path.
4. **Keep `vibration_cmd_max` ≤ 0.7** — opening at 1.0 makes jerky coarse bursts and makes
   pending-mass spikes worse.
5. **Validate on one cell** — compare signed error and pour duration vs plain PID on the same
   target/powder; only promote when inflight is consistently ≤ PID error without long stalls.
6. **Log predicted vs scale** (future) — publish or record `predicted` / `pending_g` /
   `flow_gain` so the next mis-tune is obvious from the bag, not only from final net.

Env knobs (compose / `robot-prod*.env`): `CONTROL_LAW_TYPE`, `INFLIGHT_S`,
`INFLIGHT_FLOW_GAIN`, `INFLIGHT_FLOW_GAIN_ALPHA`, `INFLIGHT_EARLY_STOP_MARGIN_G`,
`VIBRATION_CMD_MAX`, `MIN_POUR_VIBRATION`.

### Flow-rate law (`pid_flow`)

Vibration duty to flow is a deadband: below a threshold `u*` nothing leaves the scoop, and
`u*` drifts during a pour as the bed compacts, bridges, or runs out. A mass-error PID in the
endgame commands a small duty, falls under `u*`, and stalls until the slow integral climbs
back out. That is the same failure run 23 showed for `pid_inflight` (duty stuck in the
no-flow band, then a late catch-up).

`pid_flow` is this cascade for the entire pour. `pid_flow_80` uses `pid_smooth`
until 80 g remain, then this cascade. It:

* outer loop sets a flow setpoint that tapers to zero at the target,
  `flow_ref = clamp((err_g - stop_margin) / land_time_s, 0, flow_max)`
* inner loop holds duty at `u*_hat + flow_ref / gain_hat` plus a PI correction on the
  flow error, so the command tracks grams per second instead of grams
* flow is the change in net mass over `pid_flow_window_s` (the scale sample is unfiltered)
* if flow stays under `pid_flow_stall_g_s` for `pid_flow_stall_time_s`, duty ramps up at
  `pid_flow_seek_rate` (default 0.06 duty/s) until powder moves
* when flow resumes, `u*_hat` takes an EMA step toward that duty (`pid_flow_gain_alpha`)
  and the command drops to `u*_hat` plus a small margin on that tick, then slews. A broken
  bridge does not stay at the seek peak
* duty increases while tracking the cascade are limited by `pid_flow_ramp_up_rate`
  (default 0.08 duty/s). Decreases use `pid_flow_ramp_down_rate` (default 0.40
  duty/s) so the command can fall with the flow setpoint instead of coasting
  past the target
* `pid_flow_dither_amp` defaults to 0

The seek cap is `pid_flow_seek_max_duty` while more than `pid_flow_seek_taper_g` remains,
then falls linearly to `u*_hat`. The last few grams are not chased at full duty.

**Rescoop.** The law sets `scoop_empty` after it has held the seek cap with no flow for
`pid_flow_exhausted_time_s`. `pour_server` returns `need_rescoop=true` immediately
(`message="Scoop empty"`). Reaching the cap is the empty-scoop test: flow coming back means
the bed was bridged, no flow means the scoop is empty. The no-progress watchdog is still the
backstop. Stall detect plus a 0.20→0.70 ramp at 0.06 duty/s plus the exhausted wait
is about 11 s, so this cell sets `NO_PROGRESS_TIMEOUT_S=20`.

Select the whole-pour cascade with `CONTROL_LAW_TYPE=pid_flow`. Select the
80 g split with `CONTROL_LAW_TYPE=pid_flow_80`.

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
