# Robot Integration Guide

This document records the robot abstraction and the steps for adding another
robot to the scooping stack.

## Architecture

Robot-specific details live in declarative profiles. Runtime C++ receives them
as parameters; it must not assume Niryo or Jaka frame names.

Main files:

- `src/scooping_controller/config/robots.yaml` — full multi-robot catalog (laptop / image)
- `config/robots/<type>.yaml` — per-robot split emitted into the deploy bundle
- `src/scooping_controller/launch/robot_profiles.py` — `robot_profile()` + `resolve_robot()`
- `scooping_simulation.launch.py`, `scooping_bench.launch.py`, `scooping_real.launch.py` — all profile-driven
- `src/scoop_description/urdf/scoop_tool.xacro` — shared scoop tool macro
- `config/layouts/<id>/robots/<robot_key>/targets.yaml` — per-robot named targets
- `config/layouts/<id>/poses.yaml` — scoop strokes in `scooping_container_frame` (shared)

## Robot resolution

```text
explicit robot:=  >  ROBOT_TYPE env  >  device.yaml robot_type  >  niryo fallback
```

`resolve_robot()` prefers `$ROBOT_PROFILES_DIR` / `/ws/config/robots/<type>.yaml`
(single-robot on the Pi) and falls back to the image `robots.yaml` on a laptop.
It refuses to start when `PROFILE_ID`'s `robot_type` disagrees with
`device.yaml`.

## Current Robot Profiles

Niryo (`niryo` / `niryo_ned3pro`):

- Base frame: `base_link`
- Flange: `hand_link`, TCP: `tcp_link`
- Default planner: `stomp`
- Has `sim:`, `real:`, and `mock:` blocks

Jaka / Schneider (`jaka` / `jaka_zu5`):

- Base frame: `link0`
- Flange: `Link6`, TCP: `tcp_link`
- Default planner: `ompl`
- Has `sim:`, `real:`, and `mock:` blocks

Launch commands:

```bash
ros2 launch scooping_controller scooping_simulation.launch.py robot:=niryo layout_id:=dual-container
ros2 launch scooping_controller scooping_simulation.launch.py robot:=jaka layout_id:=dual-container
ros2 launch scooping_controller scooping_bench.launch.py robot:=niryo layout_id:=dual-container
ros2 launch scooping_controller scooping_bench.launch.py robot:=jaka layout_id:=dual-container
ros2 launch scooping_controller scooping_real.launch.py robot:=niryo
ros2 launch scooping_controller scooping_real.launch.py robot:=jaka
# On a provisioned Pi, omit robot:= — it resolves from device.yaml.
```

Make wrappers:

```bash
make bench ROBOT=niryo LAYOUT=dual-container
make sim ROBOT=jaka LAYOUT=lightsout-single-vessel
```

## Single-robot Pi deploy contract

Each Pi is connected to one arm. Ansible / provision writes
`config/device.yaml` with `robot_type`. `scripts/publish_deploy_bundle.sh`
splits `robots.yaml` into `config/robots/<type>.yaml` in the slim deploy
branch. Compose mounts `./config` at `/ws/config` and sets
`ROBOT_PROFILES_DIR=/ws/config/robots`. The Pi therefore has an effective
single-robot config even though the `ros-prod` image remains one multi-arch
build (per-robot images are a future option).

Laptop and Fleet Console keep the full catalog so you can author layouts for
any arm and ship only the matching profile + layout to a given device.

## Adding A New Robot

The new robot must already have a valid ROS 2 description, MoveIt config, and
controller setup. The abstraction is configuration-driven; it does not generate
a MoveIt package or ros2_control driver automatically.

1. **Import packages** — URDF/xacro, MoveIt config (SRDF, kinematics, joint
   limits, controllers), sim controller YAML, real driver launch.

2. **Attach the scoop tool** via `scoop_description/urdf/scoop_tool.xacro` on
   the flange; tip link must be `tcp_link` in the planning group.

3. **Add a profile** in `robots.yaml` with `sim:`, `real:`, and `mock:` blocks
   (see existing `niryo` / `jaka` entries). Regenerate `config/robots/` by
   re-running the split in `publish_deploy_bundle.sh` (or the same Python
   snippet locally).

4. **Commission each layout** — add
   `config/layouts/<id>/robots/<new_robot>/targets.yaml` with every named
   target using the new robot's `base_frame`, then list it under
   `targets_by_robot` in the layout YAML. Do not reuse another robot's
   targets. Scoop poses in `poses.yaml` stay in `scooping_container_frame`
   and usually transfer; re-author if the tool TCP differs.

5. **Scene calibration** — `config/container_scene/<robot>_sim.yaml` and
   `_real.yaml` (or rely on the cell layout objects once those fully replace
   the legacy scene files).

6. **Dependencies** — `exec_depend` in `scooping_controller/package.xml` when
   the launch includes the new driver/MoveIt package directly. Optional OEM
   packages that are arch-limited (like Jaka on arm64) should stay optional.

7. **Fleet profile** — add a `profiles:` entry in `config/profiles.yaml` with
   `robot_type: <new>` and the correct `device_classes`.

## Validation Checklist

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scoop_description scooping_controller robot_moveit NEW_ROBOT_MOVEIT_CONFIG --symlink-install
source install/setup.bash
make validate-layouts
make layout-parity
ros2 launch scooping_controller scooping_bench.launch.py robot:=new_robot --show-args
ros2 launch scooping_controller scooping_simulation.launch.py robot:=new_robot layout_id:=dual-container --show-args
ros2 launch scooping_controller scooping_real.launch.py robot:=new_robot --show-args
```

Confirm:

- Generated URDF contains `base_frame`, flange, `tool_link`, `tcp_link`
- Bench and sim start without duplicate robot nodes
- TF contains `base_frame -> ... -> tcp_link`
- Layout apply refuses targets whose `frame_id` ≠ `base_frame`
- MoveTo plan-only succeeds for commissioned targets
- MTC scooping task plans with the layout scene

## Common Failure Points

- Wrong `base_frame`: targets, scene objects, markers appear shifted
- Loading another robot's targets: hard-refused by `cell_layout_manager`
- Missing `tcp_link` in SRDF group
- Incorrect controller name: plan succeeds, execution never reaches the action
- Bad mount transform: marker pose and scoop tip consistently offset
- Profile / device `robot_type` mismatch: launch refuses to start

## Rule Of Thumb

If a value differs by robot, put it in `robots.yaml` or
`config/layouts/<id>/robots/<robot_key>/`. Runtime C++ receives it as a
parameter. Cell geometry and scoop strokes in the container frame stay
robot-agnostic; approach targets do not.
