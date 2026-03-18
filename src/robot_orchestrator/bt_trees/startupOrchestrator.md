# Orchestrator Startup Guide

This file collects the commands to test the `scoop_weigh_pour.xml` orchestrator flow in both simulation and real-robot mode.

## Build Once

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scooping_controller robot_moveit robot_orchestrator pouring_controller data_collection_manager weighing_scale_driver --symlink-install
source install/setup.bash
```

## Simulation Startup

### Terminal 1: scooping simulation stack

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch scooping_controller scooping_simulation.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml
```

### Terminal 2: weight simulator

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run data_collection_manager weight_sim --ros-args \
  -p topic:=/weight \
  -p rate_hz:=20.0 \
  -p baseline:=0.0 \
  -p target:=30.0 \
  -p gate_on_phase:=true \
  -p phase_topic:=/lightsout_training/phase \
  -p phase_start:=pour_start \
  -p phase_end:=pour_end \
  -p use_metadata_target:=false
```

### Terminal 3: pouring action server

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pouring_controller pour_server_node --ros-args \
  -p weight_topic:=/weight \
  -p vibration_topic:=/motor_speed \
  -p joint_state_topic:=/joint_states
```

### Terminal 4: orchestrator

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_orchestrator orchestrator_node --ros-args \
  -p tree_file:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_orchestrator/bt_trees/scoop_weigh_pour.xml
```

### Terminal 5: trigger one episode

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /bt_start_lightsout robot_common_msgs/srv/StartLightsOut \
"{powder_name: 'scooping_demo', cycle_end_limit: '1 episode', target_weight_g: 30.0, episodes: 1, batch_id: 'demo-run'}"
```

## Real-Robot Startup

### Terminal 1: real scooping stack

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROBOT_IP=169.254.200.200
ros2 launch scooping_controller scooping_real.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml \
  container_scene_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/scooping_controller/config/container_scene_real.yaml
```

### Terminal 2: physical scale

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch weighing_scale_driver weighing_scale.launch.py \
  port:=/dev/ttyUSB0 \
  baud:=115200 \
  topic:=/weight
```

### Terminal 3: pouring action server

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pouring_controller pour_server_node --ros-args \
  -p weight_topic:=/weight \
  -p vibration_topic:=/motor_speed \
  -p joint_state_topic:=/joint_states
```

### Terminal 4: orchestrator

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_orchestrator orchestrator_node --ros-args \
  -p tree_file:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_orchestrator/bt_trees/scoop_weigh_pour.xml
```

### Terminal 5: trigger one episode

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call /bt_start_lightsout robot_common_msgs/srv/StartLightsOut \
"{powder_name: 'scooping_demo', cycle_end_limit: '1 episode', target_weight_g: 30.0, episodes: 1, batch_id: 'real-demo-run'}"
```

## Required Named Targets

Make sure these names exist in `src/robot_moveit/config/targets.yaml` before starting the flow:

- `MoveToScoopingContainer`
- `MoveToWeighingContainer`
- `ReturnHome`

## Notes

- Use the simulation startup when testing marker placement, MTC planning, and BT flow without hardware.
- Use the real startup only after re-recording the named targets and validating the real container collision scene.
- Stop the current stack before switching between sim and real mode so you do not end up with multiple `move_group` or `/move_to` servers.

## Webhook Integration Startup

Use this path when the dashboard/backend should start one webhook weightment run through the central rosbridge server.

### Terminal 4: orchestrator with webhook tree

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_orchestrator orchestrator_node --ros-args \
  -p tree_file:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_orchestrator/bt_trees/webhook_weightment.xml
```

### Terminal 5: central rosbridge server

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch niryo_robot_simulation_client rosbridge_websocket_fixed.launch.py
```

Once both are running, use the dashboard `Run Robot` button on a webhook weightment row. The backend will call `/bt_start_webhook_weightment` through rosbridge, watch `/orchestrator/run_state`, and post completion into the database automatically.
