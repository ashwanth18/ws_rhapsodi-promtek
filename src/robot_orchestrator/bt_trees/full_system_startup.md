# Full System Startup Guide

This file collects the commands to run the full webhook-driven system:

- dashboard
- backend
- processing backend
- data collection manager
- webhook ingestion service
- robot start adapter
- rosbridge
- scooping stack
- pouring controller
- BT orchestrator
- either the physical weighing scale node or the simulated weight node

Use this guide when you want the end-to-end `Run Robot` flow from the dashboard to drive
`webhook_weightment.xml`.

If you want the Pi to bring up the full webhook physical robot stack with minimal manual commands,
prefer `docker-compose.robot-prod.yml` instead of the mixed Docker + manual-terminal flow below.

## One-Command Pi Deployment

Use this when the Pi should run the webhook physical robot runtime stack itself.

### Build and push from laptop

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
docker login
COLCON_PARALLEL_WORKERS=24 bash scripts/buildx_push_images.sh
```

### Pull and run on the Pi

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
cp robot-prod.env.example robot-prod.env
# edit robot-prod.env for this Pi / robot
docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml pull
docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml up -d
```

This starts:

- `db`
- `backend`
- `processing`
- `webhook_service`
- `rosbridge`
- `robot_start_adapter`
- `scooping_stack`
- `scale_launcher`
- `micro_ros_launcher`
- `pouring_controller`
- `data_collection`
- `orchestrator`

Per-robot configuration is passed by environment variables such as:

- `ROBOT_IP`
- `SCALE_DEVICE`
- `SCALE_BAUD`
- `MICRO_ROS_DEVICE`
- `ROS_DOMAIN_ID`
- optional `TARGETS_YAML` / `CONTAINER_SCENE_YAML`

The recommended place to keep those per-Pi values is `robot-prod.env`, created from
`robot-prod.env.example`.

Recommended dashboard deployment:

- Best default: run the dashboard on your laptop and point it at the Pi backend and rosbridge
- `docker-compose.robot-prod.yml` does not start the dashboard on the Pi
- Only host the dashboard on the Pi when you specifically want a separate appliance/kiosk setup
- If you host it on the Pi, build the dashboard image with Pi-reachable `VITE_API_BASE` and `VITE_ROSBRIDGE_URL` values because the static app bakes those at build time

## Build ROS Packages Once

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
colcon build --packages-select scooping_controller robot_moveit robot_orchestrator pouring_controller data_collection_manager weighing_scale_driver --symlink-install
source install/setup.bash
```

## Start App Services

Run these from the workspace root:

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
docker compose up -d db backend processing webhook_service dashboard rosbridge robot_start_adapter
```

This gives you:

- dashboard on `http://localhost:8080`
- backend on `http://localhost:8000`
- webhook service on `http://localhost:5000`
- processing service on `http://localhost:8002`
- rosbridge on `ws://localhost:9090`
- robot start adapter on `http://localhost:8010`

Important:

- `processing` is the HTTP backend that post-processes recorded runs
- `data_collection_manager` is a separate ROS node that records webhook MCAP traces and triggers `processing`

If you prefer to run the dashboard locally instead of Docker:

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev/src/dashboard
npm install
npm run dev
```

## ROS Robot Stack

Open separate terminals for the ROS-side runtime.

### Terminal 1: Scooping Stack

#### Simulation

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch scooping_controller scooping_simulation.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml
```

#### Real Robot

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROBOT_IP=169.254.200.200
ros2 launch scooping_controller scooping_real.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml \
  container_scene_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/scooping_controller/config/container_scene_real.yaml
```

This starts `move_group`, `scooping_mtc_node`, RViz, the marker tooling, and `move_to_server_node`.

### Terminal 2: Weighing Source

Choose one of these, not both.

#### Option A: Weight Simulator

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
  -p phase_topic:=/webhook_run/phase \
  -p phase_start:=pour_start \
  -p phase_end:=pour_end \
  -p use_metadata_target:=false
```

#### Option B: Physical Weighing Scale

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch weighing_scale_driver weighing_scale.launch.py \
  port:=/dev/ttyUSB0 \
  baud:=115200 \
  topic:=/weight
```

### Terminal 3: Data Collection Manager

For webhook runs, start the recorder node as well. It listens to `/webhook_run/active`,
`/webhook_run/metadata`, and `/webhook_run/phase`, writes a dedicated webhook trace, and then calls
the processing backend.

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run data_collection_manager data_collection_manager --ros-args \
  -p output_root:=/home/ashwanth/ws_rhapsodi-promtek-dev/data/webhook \
  -p processing_url:=http://localhost:8002/process
```

### Terminal 4: Pouring Controller

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run pouring_controller pour_server_node --ros-args \
  -p weight_topic:=/weight \
  -p vibration_topic:=/vibration/intensity \
  -p joint_state_topic:=/joint_states
```

## Vibration Control

The default scooping launch files now start with post-lift shake-off enabled.
If you want to test without scoop vibration, or without any real motor output, use one of the
options below.

### Scoop shake-off on or off

The scooping launch files default to:

- `post_lift_vibration_enabled:=true`
- `post_lift_vibration_duration_s:=5.0`
- `post_lift_vibration_intensity:=0.5`
- `post_lift_vibration_publish_rate_hz:=10.0`

To disable shake-off at launch time:

```bash
ros2 launch scooping_controller scooping_simulation.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml \
  post_lift_vibration_enabled:=false
```

or on real hardware:

```bash
ros2 launch scooping_controller scooping_real.launch.py \
  targets_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_moveit/config/targets.yaml \
  container_scene_yaml:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/scooping_controller/config/container_scene_real.yaml \
  post_lift_vibration_enabled:=false
```

To change it live after the node is already running:

```bash
ros2 param set /scooping_mtc_node post_lift_vibration_enabled false
ros2 param set /scooping_mtc_node post_lift_vibration_duration_s 5.0
ros2 param set /scooping_mtc_node post_lift_vibration_intensity 0.5
ros2 param set /scooping_mtc_node post_lift_vibration_publish_rate_hz 10.0
```

### Pouring controller without real vibration output

`pour_server_node` will always publish normalized commands to `/vibration/intensity` while a pour
is active. If you want to test the logic without a real actuator moving:

- do not start the micro-ROS / Teensy vibration path
- or run in simulation only, without the real motor connected

The controller can still run, read `/weight`, and execute `/pour_to_target`; there just will not be
any physical subscriber applying the motor command.

### Real vibration hardware path

For real motor output you also need the micro-ROS / Teensy path active.

With Docker:

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
docker compose up -d micro_ros_launcher
```

Without Docker, start your micro-ROS agent / Teensy path separately so `/vibration/intensity`
actually reaches the vibration motor firmware.

### Quick rules

- Want full real vibration behavior: run `micro_ros_launcher`
- Want the stack but no real vibration: do not run `micro_ros_launcher`
- Want scoop motions without shake-off: set `post_lift_vibration_enabled:=false`

### Terminal 5: Orchestrator With Webhook Tree

```bash
cd /home/ashwanth/ws_rhapsodi-promtek-dev
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run robot_orchestrator orchestrator_node --ros-args \
  -p tree_file:=/home/ashwanth/ws_rhapsodi-promtek-dev/src/robot_orchestrator/bt_trees/webhook_weightment.xml
```

## Use The Dashboard

Once the app services and ROS terminals above are running:

1. Open the dashboard.
2. Go to the batch or webhook weightment page.
3. Press `Run Robot` on the desired row.

The expected path is:

1. dashboard calls backend `POST /webhook_weightments/{id}/run_robot`
2. backend resolves location and target mapping
3. backend calls the robot start adapter on port `8010`
4. the adapter calls `/bt_start_webhook_weightment`
5. orchestrator runs `webhook_weightment.xml`
6. backend watches run state and completion over rosbridge
7. `data_collection_manager` records the webhook run from `/webhook_run/*`
8. `data_collection_manager` triggers the processing backend at `http://localhost:8002/process`

## Recommended Startup Order

1. Build the ROS workspace once.
2. Start Docker services: `db`, `backend`, `processing`, `webhook_service`, `dashboard`, `rosbridge`, `robot_start_adapter`.
3. Start the scooping stack.
4. Start either the weighing scale node or the weight simulator.
5. Start `data_collection_manager` for webhook recording + processing.
6. Start `pour_server_node`.
7. Start the orchestrator with `webhook_weightment.xml`.
8. Use the dashboard `Run Robot` button.

## Notes

- Do not run both the physical scale and `weight_sim` at the same time on `/weight`.
- The scooping launch files now default post-lift shake-off to enabled.
- The current shake-off defaults are `enabled=true`, `duration=5.0 s`, `intensity=0.5`.
- If you use the weight simulator with webhook runs, the `phase_topic` should follow webhook phases, not the old lights-out phase topic.
- For webhook runs, `data_collection_manager` is not optional if you want MCAP capture and automatic processing results in the backend.
- The current webhook tree uses a separate `PourAtWeighingContainer` target for the tilted pour pose and a `RecoverFromPourAtWeighingContainer` target to retreat before re-establishing the upright weighing pose and returning home.
- If the dashboard is up but `Run Robot` fails immediately, check:
  - backend logs
  - `robot_start_adapter` logs
  - whether `/bt_start_webhook_weightment` exists
  - whether rosbridge is reachable on `ws://localhost:9090`
