# lexium_driver

ROS 2 driver for the **Schneider Lexium Cobot** using its native JSON/TCP
protocol (LexiumCobotCommunication), instead of the JAKA SDK
(`libjakaAPI.so`) which fails to log in on Lexium firmware.

It is a drop-in replacement for `jaka_planner`'s `moveit_server`: it publishes
`/joint_states` and hosts the same `FollowJointTrajectory` action server name
MoveIt expects, so the existing `jaka_zu5_moveit_config` is reused unchanged.

> **Design & safety reference:** for the full architecture, per-module
> walkthrough, and safety model, see [docs/DESIGN.md](docs/DESIGN.md). Read its
> safety section before commanding motion on hardware.
>
> **Porting into another workspace:** see the repo-root handoff
> [LEXIUM_COBOT_PORTING.md](../../LEXIUM_COBOT_PORTING.md) — what to copy, what
> to skip, interfaces, conflicts, and a verification checklist for another agent.

## Protocol

| Channel | Port | Use |
|---------|------|-----|
| Commands | 10001 | request/response JSON (`moveJ`, `power_on`, `enable_robot`, `stop_program`, `set_speed_rate`, ...) |
| Feedback | 10000 | ~50 Hz JSON stream (`joint_position`, fault flags, `command_id`, ...) |

The controller must have its **control source delegated to Remote** in
EcoStruxure Cobot Expert before it will accept motion.

## Architecture

```
move_group (MoveIt2)
   |  FollowJointTrajectory goal  -> /jaka_zu5_controller/follow_joint_trajectory
   v
lexium_driver  --(moveJ JSON, deg)-->  TCP :10001  -->  Lexium @ <ip>
   ^  /joint_states (rad), /lexium/status
   |
   +--(joint_position deg, faults)--   TCP :10000  <--
```

Trajectories are converted rad->deg, downsampled by an angular threshold, and
streamed as `moveJ` waypoints with `arc_transition` blending (or one-at-a-time
in `sequential` mode). Progress is tracked via the feedback `command_id` /
`in_position`.

## Build

```bash
cd ~/jaka_ros2
source /opt/ros/jazzy/setup.bash
colcon build --packages-select lexium_msgs lexium_rviz_plugins lexium_driver
source install/setup.bash
```

## Run

Driver + MoveIt + RViz (one command):

```bash
ros2 launch lexium_driver lexium_moveit.launch.py ip:=192.168.88.82
```

Driver only:

```bash
ros2 launch lexium_driver lexium_driver.launch.py ip:=192.168.88.82
```

Do **not** also run `jaka_driver` or `jaka_planner moveit_server` — only one
process may own the controller connection.

## RViz safety panel

`lexium_moveit.launch.py` loads `config/moveit_lexium.rviz`, which includes the
**Lexium Safety** dockable panel (`lexium_rviz_plugins/SafetyPanel`). It
subscribes to `/lexium/status` and drives the arm through two one-click cycles:

| Button | What it does |
|--------|----------------|
| **Bring Up** | `power_on` → settle → `enable_robot` (ready for MoveIt) |
| **Shut Down** | `disable_robot` → `power_off` (safe shutdown) |
| Clear Error | clear controller faults |
| STOP | `stop_program` (emergency stop) |

Bring Up / Shut Down may take up to ~1 minute; the panel shows progress and
disables both cycle buttons while running.

If the panel is missing, add it manually: **Panels → Add New Panel →
lexium_rviz_plugins/SafetyPanel**.

## Services

All under the `lexium_driver` namespace, type `std_srvs/srv/Trigger`:

```bash
ros2 service call /lexium_driver/bring_up     std_srvs/srv/Trigger
ros2 service call /lexium_driver/shut_down    std_srvs/srv/Trigger
ros2 service call /lexium_driver/clear_error  std_srvs/srv/Trigger
ros2 service call /lexium_driver/stop         std_srvs/srv/Trigger
```

Low-level steps (debugging only):

```bash
ros2 service call /lexium_driver/power_on     std_srvs/srv/Trigger
ros2 service call /lexium_driver/enable       std_srvs/srv/Trigger
ros2 service call /lexium_driver/stop        std_srvs/srv/Trigger
ros2 service call /lexium_driver/disable     std_srvs/srv/Trigger
ros2 service call /lexium_driver/power_off   std_srvs/srv/Trigger
```

Speed override (0..1) is the `speed_rate` parameter; setting it sends
`set_speed_rate`:

```bash
ros2 param set /lexium_driver speed_rate 0.2
```

## Per-move speed

### Through MoveIt (automatic)

Two cooperating mechanisms make the RViz **velocity/acceleration scaling
sliders** affect real motion (re-Plan after changing a slider — scaling is
applied at plan time):

1. **`speed_rate_from_trajectory: true`** (default, recommended): before
   executing, the driver derives the controller's global override
   (`set_speed_rate`) from the trajectory's planned peak velocity
   (`peak / max_joint_speed_degps`, floored at `min_speed_rate`). This scales
   **acceleration too**, so the velocity slider stays effective even on short,
   acceleration-limited segments. Each `moveJ` is then sent at the full
   `move_speed_deg` / `move_acc_deg` caps and the global rate does the scaling.
   The configured `speed_rate` is restored after the trajectory finishes.
2. **`speed_from_trajectory: true`** (fallback when #1 is off): each `moveJ`
   `speed`/`acc` is derived per-segment from the planned velocities, capped by
   `move_speed_deg` / `move_acc_deg`, floored by `min_move_speed_deg`.

`downsample_threshold_deg` (default 15) controls waypoint density: larger =
fewer, longer `moveJ` segments where the arm reaches cruise speed (velocity
slider more visible); smaller = finer path tracking.

`moveJ` has no duration argument, so timing is still approximate — the path and
relative speeds are honoured, the exact time parameterization is not.

### Direct list of moves (each with its own speed)

For "go to these poses, each at its own speed" use the `MoveSequence` action at
`/lexium_driver/move_sequence`. Each move carries `move_type`, `target`,
`speed`, `acc`, and `arc_transition` (0 = stop at the point, a clean place to
change speed; > 0 = blend into the next move). Values are protocol-native
(degrees / mm) and sent verbatim (not remapped by `joint_sign`/`offset`).

```bash
ros2 action send_goal /lexium_driver/move_sequence lexium_msgs/action/MoveSequence "{
  moves: [
    {move_type: 'moveJ',   target: [0,0,90,0,90,0], speed: 20, acc: 40, arc_transition: 0},
    {move_type: 'moveJ',   target: [30,0,90,0,90,0], speed: 5,  acc: 20, arc_transition: 0},
    {move_type: 'moveTCP', target: [400,0,300,180,0,0], speed: 50, acc: 100, arc_transition: 0}
  ]
}" --feedback
```

Move types: `moveJ` (target = 6 joint deg), `moveL` / `moveTCP`
(target = `[x,y,z,rx,ry,rz]` mm/deg), `moveC` (`circ` = via pose, `target` =
end pose). The same safety gating, fault monitor, watchdog, and cancel apply.

## Safety

- Motion is **refused** unless: command channel connected, feedback fresh,
  control source = Remote, arm powered + enabled, and no active faults.
- A watchdog aborts the active goal if the feedback stream goes stale.
- Any `protective_stop` / `emergency_stop` / `collision_stop` / `on_soft_limit`
  during motion triggers `stop_program` and aborts the goal.
- Cancel (MoveIt cancel or Ctrl-C) sends `stop_program`.
- Startup applies a conservative `speed_rate` (default 0.1).

## IMPORTANT: validate the joint mapping before trusting motion

The URDF joint zero/direction may not match the Lexium's. Before running any
MoveIt trajectory:

1. Power on + enable with control source = Remote.
2. Compare `/joint_states` against the pose shown in RViz and on the pendant.
3. If a joint is mirrored or offset, set `joint_sign` / `joint_offset_deg` in
   [config/lexium_driver.yaml](config/lexium_driver.yaml) (per joint).
4. Start with a very low `speed_rate` and small jogs.

## Known limitations

- `moveJ` has no duration argument, so MoveIt's exact time parameterization is
  approximated by `move_speed_deg` / `move_acc_deg`; the path is preserved, the
  velocity profile is not exact.
- Plain TCP `Connect` is used; TLS (`ConnectTLS`) is not yet implemented.
