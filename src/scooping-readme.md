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
