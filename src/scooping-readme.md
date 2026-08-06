# MoveIt Task Constructor Scooping Simulation (Manual Waypoint System)

## Goal

Build a simulation pipeline where a user can manually place scooping poses in RViz and execute a scooping trajectory using MoveIt Task Constructor (MTC).

This first version does NOT use vision. The scooping poses are placed manually using interactive markers in RViz.

Later this system can be extended to:

* automatically generate poses from a pointcloud
* train reinforcement learning policies
* collect human demonstration trajectories

---

# System Architecture

Robot (URDF)
↓
MoveIt
↓
MoveIt Task Constructor
↓
Interactive Marker Node
↓
RViz GUI (drag scooping poses)
↓
Scooping trajectory execution

---

# Components

The system consists of the following nodes:

1. robot_simulation
2. moveit_config
3. scooping_marker_server
4. mtc_scooping_planner
5. rviz_visualization

---



## 2. Required packages

Install dependencies if nneeded if not no need:

```
sudo apt install ros-jazzy-moveit
sudo apt install ros-jazzy-moveit-task-constructor
sudo apt install ros-jazzy-interactive-markers
```

---

# Package Structure

Create a package:

```
ros2 pkg create scooping_demo --build-type ament_cmake
```

Directory layout:

```
scooping_demo

src/
   interactive_marker_server.cpp
   scooping_mtc_node.cpp

launch/
   simulation.launch.py

config/
   rviz_config.rviz

include/
```

---

# Scooping Waypoint Design

The scoop trajectory consists of four poses.

1. Approach pose
2. Contact pose
3. Scoop motion pose
4. Lift pose

These poses are defined as interactive markers.

RViz markers:

approach_marker
contact_marker
scoop_marker
lift_marker

Each marker represents a 6D pose.

---

# Interactive Marker Server

The marker server publishes draggable markers that allow the user to place scoop poses.

Example markers:

```
approach_marker
contact_marker
scoop_marker
lift_marker
```

The node publishes:

```
/scoop_poses
```

Message type:

```
geometry_msgs/PoseArray
```

---

# Example Marker Node Logic

Pseudo pipeline:

```
create_interactive_marker_server()

create_marker("approach")
create_marker("contact")
create_marker("scoop")
create_marker("lift")

on_marker_moved():

    update_pose_list()

    publish_pose_array()
```

The user can drag the markers in RViz.

---

# MoveIt Task Constructor Pipeline

The planner subscribes to the marker poses and builds an MTC task.

Task stages:

```
CurrentState

MoveTo(approach_pose)

CartesianPath(contact_pose)

CartesianPath(scoop_pose)

CartesianPath(lift_pose)
```

---

# MTC Node Logic

Pseudo logic:

```
subscribe /scoop_poses

wait until 4 poses received

build MTC task

execute task
```

Example stage pipeline:

```
task

 stage 1  current_state
 stage 2  move_to_approach
 stage 3  cartesian_contact
 stage 4  cartesian_scoop
 stage 5  cartesian_lift
```

---

# RViz Setup

Add the following displays:

* RobotModel
* PlanningScene
* InteractiveMarkers
* TF

Tools enabled:

* MoveIt MotionPlanning
* Interactive marker manipulation

---

# Launch File

Launch sequence:

1. robot_state_publisher
2. move_group
3. marker_server
4. mtc_node
5. rviz

Example launch:

```
ros2 launch scooping_demo simulation.launch.py
```

---

# Typical Workflow

Step 1

Launch simulation.

Step 2

Open RViz.

Step 3

Drag the markers to desired scoop poses.

Step 4

Press execute in the MTC node.

Step 5

Robot executes scooping trajectory.

---

# Data Collection Extension

Later this system can log demonstration data.

Log data:

```
approach_pose
contact_pose
scoop_pose
lift_pose
joint_trajectory
execution_time
```

Saved format:

```
JSON
CSV
ROS bag
```

This dataset can be used for:

* imitation learning
* reinforcement learning
* trajectory optimization

---

# Future Extensions

Add vision system:

pointcloud

```
↓
```

surface detection

```
↓
```

pose candidate generation

```
↓
```

publish markers

```
↓
```

execute MTC task

Other extensions:

* automatic scoop orientation from surface normals
* learning scoop trajectories
* RL policy to optimize scoop depth
* force feedback

---

# Testing Strategy

Test in three stages.

Stage 1

Verify markers move correctly in RViz.

Stage 2

Verify poses are published.

Stage 3

Verify MTC executes the trajectory.

---

# Expected Output

Robot performs motion:

approach → contact → scoop → lift

This provides a basic scooping primitive that can later be improved.

---

# Notes for AI Implementation Agent

The implementation agent should:

1. Create interactive marker server
2. Publish pose array
3. Build MTC task dynamically
4. Convert poses to MoveTo and CartesianPath stages
5. Provide launch file
6. Provide RViz config

Do NOT integrate perception in the first version.

Focus only on manual scoop pose placement.

---

# Current Integration Notes

## Current scooping stack

The current `scooping_controller` workflow is a dedicated scooping experiment stack built around:

* interactive scoop markers in RViz
* a four-pose scooping primitive
* MoveIt Task Constructor for the scoop sequence
* a custom Gazebo world and container model for simulation

The four scoop poses are:

1. `approach`
2. `contact`
3. `scoop`
4. `lift`

The planner expects exactly these four poses and then executes them as a structured multi-stage task.

## Important implementation update

The original design in this document described the scoop stages as Cartesian path segments.

The current implementation has been changed to use `MoveTo` stages for all four scooping poses:

* `approach`
* `contact`
* `scoop`
* `lift`

This means the scooping task now behaves as a sequence of absolute pose goals rather than strict straight-line Cartesian segments between poses.

In practice:

* the motion is easier to get planning reliably
* the path between poses may differ from a strict Cartesian line
* the pose sequence still defines the scoop primitive

## Relation to `lightsout.xml`

The behavior tree in `robot_orchestrator/bt_trees/lightsout.xml` does not use the scooping MTC node directly.

Right now the lights-out tree uses the existing `MoveTo` action server from `robot_moveit`:

* move to container
* move to scale
* pour
* move back

So the current BT and the current scooping MTC workflow are separate systems.

## Recommended integration pattern

Do not replace the whole BT motion stack with MTC.

Recommended split:

* use `robot_moveit` `MoveTo` for gross motion between stations
* use the scooping MTC node only for the scoop primitive itself

Recommended BT flow:

1. `MoveToContainer`
2. `ExecuteScoop`
3. `MoveToScale`
4. `PourToTarget`

This keeps named-target navigation in the existing orchestrator while using MTC only where it provides clear value.

## Why MTC is different from the current BT `MoveTo`

`robot_moveit` `MoveTo`:

* generic motion action server
* good for named targets and simple waypoint moves
* fits well with behavior trees and reusable station-to-station moves

Scooping MTC node:

* specialized manipulation primitive
* fixed multi-stage scoop task
* better introspection for stage failures
* better fit for a structured scoop sequence than a single generic target move

## Simulation vs real robot

The current scooping launch file is simulation-specific.

It currently includes:

* Gazebo startup
* custom world loading
* simulated robot spawning
* simulation controllers
* simulation time
* RViz tools for manual scoop authoring

So it should be treated as a scooping simulation workflow, not a direct real-robot launch.

## Using the scooping logic on a real Niryo NED

The scooping logic itself can still be used on a real robot, but the launch composition must be different.

For real hardware, the recommended stack is now available directly through:

* `scooping_controller/launch/scooping_real.launch.py`

That launch starts:

* Niryo driver / hardware stack
* `robot_state_publisher`
* real `move_group`
* `move_to_server_node`
* `target_recorder_node`
* scooping marker server
* scooping MTC node
* planning-scene collision publisher
* RViz

And you would remove the Gazebo-specific parts:

* no Gazebo world
* no simulated robot spawn
* no sim clock
* no sim-only static world assumptions unless they are replaced with calibrated real-world frames

## Real cell calibration files

Three files must be treated as real-cell calibration, not portable simulation defaults:

* `src/robot_moveit/config/targets.yaml`
  Record `ReturnHome`, `MoveToScoopingContainer`, and `MoveToWeighingContainer` again on the real robot with `/record_target`.
* `~/.ros/scooping_controller/poses_real.yaml`
  Save the five scoop-marker poses here after tuning them in RViz on hardware.
* `src/scooping_controller/config/container_scene_real.yaml`
  Update the scooping and weighing container positions, orientations, and scales until the RViz meshes and MoveIt collision objects line up with the physical stations.

## Real bring-up command

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROBOT_IP=169.254.200.200
ros2 launch scooping_controller scooping_real.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml \
  container_scene_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/scooping_controller/config/container_scene_real.yaml
```

## Main real-robot concerns

Compared to simulation, the main issues on real hardware are:

* TCP / tool calibration accuracy
* container pose calibration
* collision scene accuracy
* controller timing and execution tolerance
* more conservative speed settings

A scoop that works in simulation may miss or collide on hardware if the robot frame, tool frame, or container frame are not calibrated correctly.

## Practical recommendation

Near-term recommendation:

* keep using the current scooping simulation launch for scoop development
* keep using the current BT + `MoveTo` stack for orchestration
* integrate them later by adding a dedicated BT node that triggers the scooping MTC execution

This gives a clean architecture:

* BT orchestrates the process
* `MoveTo` handles station navigation
* MTC handles the scoop primitive

## Real validation sequence

Use this order on hardware:

1. Confirm joint states are live and `move_group` is planning with `use_sim_time:=false`.
2. Run one slow `MoveTo` to `ReturnHome`.
3. Re-record the named station targets in `targets.yaml`.
4. Tune `container_scene_real.yaml` until the RViz container markers and collision scene match reality.
5. Save the five scoop poses to `poses_real.yaml`.
6. Call `/plan_scoop` first, then execute free-space tests before approaching the real scooping container.

## Manual 5-pose parameterization

The manual 5-pose workflow now supports a small set of shape controls so you can reuse one good
hand-authored scoop pattern without re-recording every pose.

### Manual tuning parameters

The RViz panel exposes:

* `offset_x`
* `offset_y`
* `offset_z`
* `sweep_scale`
* `pitch_offset`
* `lift_offset_z`

The corresponding node parameters are:

* `pattern_offset_x`
* `pattern_offset_y`
* `pattern_offset_z`
* `manual_sweep_scale`
* `manual_pitch_offset_rad`
* `manual_lift_offset_z`

### How they affect the 5 poses

The authored poses are still:

1. `Approach`
2. `Contact`
3. `Scoop`
4. `Lift`
5. `Transport Ready`

The adjustments are applied like this:

* `offset_x`, `offset_y`, `offset_z` translate all five poses together in `base_link`
* `sweep_scale` rescales the motion from `Contact` onward, so `Scoop`, `Lift`, and
  `Transport Ready` move farther or shorter relative to `Contact`
* `pitch_offset` adds a common TCP pitch bias to all five poses
* `lift_offset_z` raises or lowers `Lift` and `Transport Ready`

This keeps the overall style of the manually tuned scoop while letting you test nearby variants.

### RViz preview

The `Manual Scoop Preview` display now reflects the same manual tuning parameters used by
`/plan_scoop` and `/execute_scoop`, so the preview line should match the motion the planner sees.

### Post-lift shake-off

The manual scoop executor can now trigger a short vibration burst after `Lift` and before
`Transport Ready`.

Parameters:

* `post_lift_vibration_enabled`
* `post_lift_vibration_duration_s`
* `post_lift_vibration_intensity`
* `post_lift_vibration_publish_rate_hz`
* `post_lift_vibration_settle_s`
* `post_lift_vibration_topic`

Behavior:

* The staged `/execute_scoop` flow pauses after the `Lift` stage and streams the configured
  normalized intensity on `/vibration/intensity` for the configured duration before publishing
  `0.0` and continuing to `Transport Ready`
* The waypoint-style manual execution path does the same pause between its contact/scoop/lift block
  and the final transport move
* The continuous manual execution mode now runs continuously through `Lift`, performs the same
  shake-off pause, then executes the final `Transport Ready` move
* The keepalive stream defaults to `10 Hz`, which is suitable for a micro-ROS vibration watchdog
  like the Teensy firmware timeout described in `scoop_microros`

## Parameterized scooping workflow

The scooping panel now includes a second workflow called `Scooping Parameter Workflow`.

This mode does not replace the manual 5-marker editor. It adds a parameterized template that
generates and executes a scooping motion from a compact set of parameters in `base_link`.

### Template definition

The parameterized sequence is:

1. Hover above `(x, y)` at `hover_height`
2. Plunge vertically to `(x, y, z_i)`
3. Sweep forward to `(x + L, y, z_f)`
4. Tilt upward in place to the retain pitch
5. Lift vertically to `lift_height`

This is a forward scoop in the robot frame. The sweep is along `+X`, not `+Y`.

The TCP/tool point used by planning is `tcp_link`, but the target coordinates are expressed in
`base_link`.

The pitch fields are interpreted as the actual `tcp_link` pitch angle used for planning and
preview. There is no hidden `90 deg` conversion.

### Parameters

The panel exposes:

* `x`, `y`
* `z_i`
* `z_f`
* `L`
* `tcp_pitch`
* `hover_height`
* `retain_tcp_pitch`
* `lift_height`

Current node parameters in `scooping_mtc_node`:

* `template_x`
* `template_y`
* `template_z_initial`
* `template_z_final`
* `template_sweep_length`
* `template_pitch_rad`
* `template_hover_height`
* `template_transport_pitch_rad`
* `template_lift_height`

### RViz preview

The panel publishes a live preview on:

* `/parameterized_scoop_preview`
* `/manual_scoop_preview`
* `/display_planned_path`
* `/solution`

The default RViz config displays these automatically as:

* `Parameterized Scoop Preview` for parameterized template preview lines
* `Manual Scoop Preview` for manual 5-pose preview lines
* `MoveTo Planned Path` for normal `MoveTo` plan previews
* `Motion Planning Tasks` for MTC task solutions on `/solution`

The preview shows:

* the full template path as a line
* stage points for `Hover`, `Plunge`, `Sweep End`, `Retain`, and `Lift`
* arrows showing the working and retain orientations

`Preview Template` also pushes the generated poses into the existing 5 scoop markers so the manual
workflow and parameterized workflow can be compared visually.

In `Target Authoring`, `Plan MoveTo Preview` requests a normal `MoveTo` plan without execution, while
`Plan Constrained Test` applies the panel's constraint test settings first so transport/upright
constraint failures can be debugged visually in RViz.

### Services

New services on `scooping_mtc_node`:

* `/plan_parameterized_scoop`
* `/execute_parameterized_scoop`

Existing manual services are unchanged:

* `/plan_scoop`
* `/execute_scoop`
* `/execute_scoop_continuous`
* `/execute_scoop_waypoint_motion`

### Starter parameter set

For the current scooping container placement in simulation, start with:

* `x = 0.300`
* `y = 0.000`
* `z_i = 0.060`
* `z_f = 0.080`
* `L = 0.100`
* `tcp_pitch = 25 deg`
* `hover_height = 0.240`
* `retain_tcp_pitch = 15 deg`
* `lift_height = 0.240`

This is intended as a conservative free-space test that scoops straight forward in front of the
robot.

### Recommended usage

1. Launch the scooping stack and RViz.
2. Look at the `Parameterized Scoop Preview` display in RViz.
3. Adjust the template fields in the panel.
4. Confirm the preview line stays inside the scooping container.
5. Click `Plan Parameterized Scoop`.
6. Only then click `Execute Parameterized Scoop`.

If the first hover pose is still reported as `GOAL_STATE_INVALID`, reduce the reach or pitch:

* move `x` closer to the robot
* reduce `L`
* reduce `theta`
* increase `hover_height`
