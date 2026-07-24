# Robot Integration Guide

This document records the robot abstraction refactor and the steps for adding another robot to the scooping stack.

## What Changed

Robot-specific details now live in declarative profiles instead of being scattered through launch files and C++ defaults.

Main files:

- `src/scooping_controller/config/robots.yaml`: robot profiles for Niryo and Jaka.
- `src/scooping_controller/launch/robot_profiles.py`: shared launch helper that loads profiles.
- `src/scooping_controller/launch/scooping_simulation.launch.py`: simulation launch now consumes profiles.
- `src/scooping_controller/launch/scooping_real.launch.py`: real/hardware launch now consumes profiles.
- `src/scoop_description/urdf/scoop_tool.xacro`: shared scoop tool macro.
- `src/scooping_controller/config/container_scene/`: per-robot scene calibration files.
- `src/robot_moveit/config/targets/`: per-robot target files.

Runtime nodes now receive robot-specific values from launch parameters:

- `move_to_server`: planning group, end-effector link, controller action, planner, and position-only behavior.
- `target_recorder`: records target poses in the profile's base frame instead of always `base_link`.
- `scooping_marker_server`: gets the tool mesh and TCP visual offset from the profile.
- `scooping_mtc_node`: gets group, IK frame, planning scene frame, controller, and action from the profile.
- scene and marker publishers: use the profile base frame.

## Current Robot Profiles

The active profiles are in `src/scooping_controller/config/robots.yaml`.

Niryo:

- Alias: `niryo`
- Canonical name: `niryo_ned3pro`
- Base frame: `base_link`
- Flange link: `hand_link`
- TCP link: `tcp_link`
- Default planner: `stomp`

Jaka:

- Alias: `jaka`
- Canonical name: `jaka_zu5`
- Base frame: `link0`
- Flange link: `Link6`
- TCP link: `tcp_link`
- Default planner: `ompl`

Launch commands stay stable:

```bash
ros2 launch scooping_controller scooping_simulation.launch.py robot:=niryo
ros2 launch scooping_controller scooping_simulation.launch.py robot:=jaka
ros2 launch scooping_controller scooping_real.launch.py robot:=niryo
ros2 launch scooping_controller scooping_real.launch.py robot:=jaka
```

## Adding A New Robot

The new robot must already have a valid ROS 2 description, MoveIt config, and controller setup. The abstraction makes integration configuration-driven, but it does not generate a MoveIt package or ros2_control driver automatically.

1. Add or import the robot packages.

   Required pieces:

   - URDF/xacro package.
   - MoveIt config package with SRDF, kinematics, joint limits, and controllers.
   - ros2_control controller YAML for simulation if using Gazebo.
   - real driver launch file if using hardware.

2. Attach the scoop tool.

   Include the shared macro in the robot's MoveIt xacro:

   ```xml
   <xacro:include filename="$(find scoop_description)/urdf/scoop_tool.xacro" />
   ```

   Then instantiate it from the robot flange:

   ```xml
   <xacro:scoop_tool
       parent_link="ROBOT_FLANGE_LINK"
       mount_joint_name="robot_scoop_mount_joint"
       tcp_joint_name="robot_scoop_tcp_joint"
       mesh_resource="file://$(find niryo_robot_description)/meshes/ned3pro/stl/niryo_scoop_v4-ros.STL"
       mount_xyz="0 0 0"
       mount_rpy="0 0 0"
       tcp_xyz="0.15825 0 -0.09356" />
   ```

   Replace `ROBOT_FLANGE_LINK`, `mount_xyz`, `mount_rpy`, and `tcp_xyz` with the real adapter calibration.

3. Update the SRDF.

   The MoveIt planning group should include the fixed tool joints and `tcp_link`, and the end-effector tip used by the stack should be `tcp_link`.

   Check adjacent collisions around the flange/tool/scoop. Disable only the collisions that are genuinely adjacent or always in contact.

4. Add a robot profile in `robots.yaml`.

   Example skeleton:

   ```yaml
   new_robot:
     canonical_name: new_robot_model
     aliases: [new_robot, new_robot_model]
     moveit_robot_name: new_robot_model
     planning_group: arm
     eef_link: tcp_link
     base_frame: base_link
     flange_link: tool0
     follow_joint_trajectory_controller: arm_controller
     planning_pipeline: ompl
     position_only_goal: false
     targets:
       package: robot_moveit
       path: targets/new_robot_model.yaml
     tool:
       mesh_resource: package://niryo_robot_description/meshes/ned3pro/stl/niryo_scoop_v4-ros.STL
       tcp_visual_offset_xyz: [0.15825, 0.0, -0.09356]
     sim:
       scene:
         package: scooping_controller
         path: config/container_scene/new_robot_sim.yaml
       moveit_package: new_robot_moveit_config
       urdf:
         package: new_robot_moveit_config
         path: config/new_robot.urdf.xacro
         xacro_args: ["use_gazebo:=true"]
       controllers:
         package: new_robot_moveit_config
         path: config/ros2_controllers.yaml
       spawn_model_name: new_robot
       publish_world_to_base_tf: false
       planning_pipelines: [ompl]
     real:
       scene:
         package: scooping_controller
         path: config/container_scene/new_robot_real.yaml
       moveit_package: new_robot_moveit_config
       urdf:
         package: new_robot_moveit_config
         path: config/new_robot.urdf.xacro
         xacro_args: []
       controllers:
         package: new_robot_moveit_config
         path: config/ros2_controllers.yaml
       driver:
         package: new_robot_driver
         path: launch/robot_start.launch.py
       timing:
         move_group_delay: 5.0
         move_to_delay: 7.0
         target_recorder_delay: 7.2
         task_frame_delay: 7.8
         marker_delay: 8.0
         container_delay: 8.5
         collisions_delay: 8.7
         mtc_delay: 9.0
         rviz_delay: 10.0
   ```

5. Add target files.

   Create `src/robot_moveit/config/targets/new_robot_model.yaml`:

   ```yaml
   targets:
     home:
       frame_id: base_link
       position: {x: 0.3, y: 0.0, z: 0.3}
       orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
   ```

   The `frame_id` must match the new robot profile's `base_frame`.

6. Add scene calibration files.

   Create:

   - `src/scooping_controller/config/container_scene/new_robot_sim.yaml`
   - `src/scooping_controller/config/container_scene/new_robot_real.yaml`

   Start from the Niryo or Jaka scene files, then calibrate container and table positions for the new robot base frame.

7. Add package dependencies.

   If the new robot xacro includes the scoop macro, its MoveIt config package should depend on `scoop_description`.

   If `scooping_controller` launches the new robot driver or MoveIt config directly, add the needed `exec_depend` entries to `src/scooping_controller/package.xml`.

## Validation Checklist

Run these before calling the robot integrated:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scoop_description scooping_controller robot_moveit NEW_ROBOT_MOVEIT_CONFIG --symlink-install
source install/setup.bash
```

Check xacro:

```bash
xacro $(ros2 pkg prefix NEW_ROBOT_MOVEIT_CONFIG)/share/NEW_ROBOT_MOVEIT_CONFIG/config/new_robot.urdf.xacro > /tmp/new_robot.urdf
```

Confirm the generated URDF contains:

- the profile `base_frame`
- the robot flange link
- `tool_link`
- `tcp_link`
- the scoop mount joint
- the scoop TCP joint

Check launch files load:

```bash
ros2 launch scooping_controller scooping_simulation.launch.py robot:=new_robot --show-args
ros2 launch scooping_controller scooping_real.launch.py robot:=new_robot --show-args
```

Check simulation:

- Gazebo starts without duplicate robot nodes.
- `/clock` has one publisher.
- TF contains `base_frame -> ... -> tool_link -> tcp_link`.
- RViz marker pose matches the real `tcp_link` pose.
- `MoveTo` succeeds for a recorded target.
- MTC scooping task plans with the selected scene file.

## Common Failure Points

- Wrong `base_frame`: targets, scene objects, markers, and target recording will appear shifted or rejected.
- Wrong `eef_link`: MoveIt may plan to the flange instead of the scoop tip.
- Missing `tcp_link` in SRDF group: planning to the scoop tip may fail.
- Incorrect controller name: `MoveTo` will plan but execution will not reach the joint trajectory action.
- Bad mount transform: marker pose and scoop pose will look consistently offset.
- Reusing another robot's target file: poses may be in the wrong frame.

## Rule Of Thumb

If a value differs by robot, put it in `robots.yaml` or a per-robot data file. Runtime C++ should receive it as a parameter, not assume Niryo or Jaka frame names.
