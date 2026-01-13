# ROS 2 Humble to Jazzy Migration Guide

This document outlines the changes made to migrate the project from ROS 2 Humble to ROS 2 Jazzy.

## Overview

The migration involved updating package names, message types, and configuration file structures to be compatible with ROS 2 Jazzy. The main areas affected were:

1. **Gazebo/Simulation packages** (Ignition → Gazebo)
2. **Controller configuration files** (YAML structure)
3. **Launch files** (package and message type updates)

---

## 1. Gazebo Package Name Changes

### Package Renames

| Humble (Ignition) | Jazzy (Gazebo) |
|-------------------|----------------|
| `ros_ign_gazebo` | `ros_gz_sim` |
| `ros_ign_bridge` | `ros_gz_bridge` |
| `ign_gazebo.launch.py` | `gz_sim.launch.py` |

### Message Type Changes

| Humble | Jazzy |
|--------|-------|
| `ignition.msgs.Clock` | `gz.msgs.Clock` |

### Launch File Updates

**Files Modified:**
- `src/niryo_robot_simulation_client/launch/niryo_ned3pro_complete.launch.py`
- `src/niryo_robot_simulation_client/launch/simulation_brigup.launch.py`

**Example Changes:**

```python
# Before (Humble)
bridge = Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
)

gz_spawn_entity = Node(
    package='ros_ign_gazebo',
    executable='create',
    ...
)

gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare('ros_ign_gazebo'),
                               'launch',
                               'ign_gazebo.launch.py'])]
    ),
    launch_arguments=[('gz_args', [' -r -v 4 /usr/share/ignition/ignition-gazebo6/worlds/empty.sdf'])]
)

# After (Jazzy)
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
)

gz_spawn_entity = Node(
    package='ros_gz_sim',
    executable='create',
    ...
)

gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                               'launch',
                               'gz_sim.launch.py'])]
    ),
    launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]  # Use default empty world
)
```

---

## 2. Controller Configuration File Structure

### Critical Change: YAML File Format

ROS 2 Jazzy requires a specific YAML structure for controller configuration files. The key requirement is that **all parameters must be under `ros__parameters`**, and controllers must be defined at the root level.

### File Modified
- `src/niryo_robot_moveit_interface/config/ros2_controllers.yaml`

### Correct Structure for ROS 2 Jazzy

```yaml
/**:
  ros__parameters:
    # Global parameters (controller_manager, trajectory_execution)
    controller_manager:
      ros__parameters:
        update_rate: 100  # Hz

    # Trajectory execution parameters (for MoveIt)
    trajectory_execution:
      allowed_start_tolerance: 0.05
      execution_duration_monitoring: true
      allowed_execution_duration_scaling: 1.2
      allowed_goal_duration_margin: 0.5

# Controllers at root level with ros__parameters
joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster

niryo_robot_follow_joint_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    command_interfaces:
      - position
    state_interfaces:
      - position
    allow_nonzero_velocity_at_trajectory_end: true
```

### Key Points:

1. **`/**` wildcard**: Makes parameters available to all nodes
2. **`ros__parameters` wrapper**: Required for all ROS 2 Jazzy parameter files
3. **Controllers at root level**: Controllers must be defined at the root level (not nested under `/**`) so the spawner can find them when loading with `--param-file`
4. **Separate sections**: 
   - Global parameters (`controller_manager`, `trajectory_execution`) go under `/**`
   - Controller-specific parameters go at the root level

### Common Errors and Solutions

#### Error: "Cannot have a value before ros__parameters"
**Cause**: Parameters defined at root level without `ros__parameters` wrapper  
**Solution**: Wrap all parameters under `ros__parameters`

#### Error: "The 'type' param was not defined for 'controller_name'"
**Cause**: Controller parameters nested incorrectly or missing `ros__parameters`  
**Solution**: Define controllers at root level with their own `ros__parameters` section

#### Error: "Could not initialize the controller"
**Cause**: Controller parameters not accessible when spawner loads with `--param-file`  
**Solution**: Ensure controllers are at root level, not nested under `/**`

---

## 3. Launch File Updates for Controller Spawners

### Changes Required

Both controller spawners now need the `--param-file` argument:

```python
# Before (Humble)
joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=['joint_state_broadcaster'],
    ...
)

# After (Jazzy)
joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
        'joint_state_broadcaster',
        '--param-file',
        robot_controllers,  # Path to ros2_controllers.yaml
    ],
    ...
)
```

---

## 4. Summary of Changes

### Files Modified

1. **Launch Files:**
   - `src/niryo_robot_simulation_client/launch/niryo_ned3pro_complete.launch.py`
   - `src/niryo_robot_simulation_client/launch/simulation_brigup.launch.py`

2. **Configuration Files:**
   - `src/niryo_robot_moveit_interface/config/ros2_controllers.yaml`

### Key Changes Summary

| Component | Humble | Jazzy |
|----------|--------|-------|
| Gazebo package | `ros_ign_gazebo` | `ros_gz_sim` |
| Bridge package | `ros_ign_bridge` | `ros_gz_bridge` |
| Launch file | `ign_gazebo.launch.py` | `gz_sim.launch.py` |
| Clock message | `ignition.msgs.Clock` | `gz.msgs.Clock` |
| YAML structure | Mixed format | All under `ros__parameters` |
| Controller location | Nested | Root level |
| Spawner args | Optional `--param-file` | Required `--param-file` |

---

## 5. Testing the Migration

After making these changes, verify the migration by:

1. **Building the workspace:**
   ```bash
   cd ~/ws_rhapsodi-promtek
   colcon build
   source install/setup.bash
   ```

2. **Launching the simulation:**
   ```bash
   ros2 launch niryo_robot_simulation_client niryo_ned3pro_complete.launch.py
   ```

3. **Verifying controllers are loaded:**
   ```bash
   ros2 control list_controllers
   ```

4. **Checking joint states:**
   ```bash
   ros2 topic echo /joint_states
   ```

---

## 6. Additional Notes

### Why These Changes Were Necessary

1. **Ignition → Gazebo**: The Ignition Gazebo project was renamed to Gazebo Sim, and all related ROS packages were updated accordingly.

2. **YAML Structure**: ROS 2 Jazzy introduced stricter requirements for parameter file structure to ensure consistency and prevent parsing errors.

3. **Controller Loading**: The spawner now requires explicit parameter file specification to ensure controllers can find their configuration.

### Compatibility

- **ROS 2 Humble**: These changes are **not** backward compatible with Humble
- **ROS 2 Jazzy**: All changes are required for Jazzy compatibility
- **Future versions**: This structure should be compatible with future ROS 2 distributions

---

## 7. References

- [ROS 2 Jazzy Release Notes](https://docs.ros.org/en/jazzy/Releases/Release-Jazzy-Jalisco.html)
- [Gazebo Sim Documentation](https://gazebosim.org/docs)
- [ROS 2 Control Documentation](https://control.ros.org/)

---

## 8. Troubleshooting

### Issue: Gazebo crashes on startup
**Solution**: Check that `ros_gz_sim` and `ros_gz_bridge` packages are installed:
```bash
ros2 pkg list | grep ros_gz
```

### Issue: Controllers fail to load
**Solution**: Verify the YAML file structure matches the format shown above, especially:
- All parameters under `ros__parameters`
- Controllers at root level
- Correct indentation

### Issue: Joint states not publishing
**Solution**: Ensure `joint_state_broadcaster` is loaded:
```bash
ros2 control list_controllers
```

---

**Last Updated**: November 2025  
**ROS 2 Distribution**: Jazzy Jalisco

