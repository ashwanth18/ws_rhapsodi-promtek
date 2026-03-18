# data_collection_manager

Lightweight recorder for lights-out experiment runs. It listens for:

- `/lightsout_training/active` (`std_msgs/Bool`) to start/stop a run
- `/lightsout_training/run_id` (`std_msgs/String`) run ID (latched)
- `/lightsout_training/batch_id` (`std_msgs/String`) batch ID (latched)
- `/lightsout_training/ingredient_id` (`std_msgs/String`) ingredient ID (latched)
- `/lightsout_training/target_weight_g` (`std_msgs/Float64`) target weight (latched)
- `/lightsout_training/mode` (`std_msgs/String`) mode (latched)
- `/lightsout_training/episode` (`std_msgs/Int32`) to start a new MCAP file per episode
- `/lightsout_training/episode_end` (`std_msgs/Int32`) to finalize the MCAP and trigger ingest

It also supports dedicated webhook-driven robot runs using:

- `/webhook_run/active` (`std_msgs/Bool`) to start and stop one webhook trace
- `/webhook_run/metadata` (`std_msgs/String`) JSON metadata for the webhook run
- `/webhook_run/phase` (`std_msgs/String`) phase markers for scoop, transport, and pour

## Run

```bash
ros2 run data_collection_manager data_collection_manager --ros-args -p output_root:=/home/ashwanth/ws_rhapsodi-promtek/data/lightsout
```

Parameters:

- `output_root` (default: `data/lightsout`)
- `topics` (default: includes `/weight`, `/system_status`, `/lightsout_training/*`, `/tf_static`, `/joint_states`)
- `lightsout_active_topic` (default: `/lightsout_training/active`)
- `lightsout_episode_topic` (default: `/lightsout_training/episode`)
- `lightsout_episode_end_topic` (default: `/lightsout_training/episode_end`)
- `run_id_topic` (default: `/lightsout_training/run_id`)
- `batch_id_topic` (default: `/lightsout_training/batch_id`)
- `ingredient_id_topic` (default: `/lightsout_training/ingredient_id`)
- `target_weight_topic` (default: `/lightsout_training/target_weight_g`)
- `mode_topic` (default: `/lightsout_training/mode`)
- `robot_id` (default: `robot-1`)
- `mode` (default: `lightsout`)
- `processing_url` (default: `http://localhost:8002/process`)

## Simulation (no hardware)

You can simulate lights-out episodes and the weighing stream to test
MCAP recording + processing without the robot:

1. Start backend services (`db`, `backend`, and `processing`).
1. Run the lights-out signal simulator:

```bash
ros2 run data_collection_manager lightsout_sim --ros-args -p episodes:=3 -p episode_duration:=6.0 -p batch_id:=sim-001 -p target_weight:=50.0
```

1. Run the weight simulator:

```bash
ros2 run data_collection_manager weight_sim --ros-args -p rate_hz:=20.0 -p target:=50.0
```

1. Run the data collection manager:

```bash
ros2 run data_collection_manager data_collection_manager --ros-args \
  -p output_root:=/home/ashwanth/ws_rhapsodi-promtek/data/lightsout \
  -p processing_url:=http://localhost:8002/process
```

Notes:

- Ensure the topics you want for Foxglove playback are included in `topics`.

## Output layout

Each run creates:

- `robotID_runID_batchID_date_mode/`
  - `episode_1/` (MCAP bag directory)
  - `episode_2/`
  - ...

For webhook runs, the same folder also includes:

- `metadata.json`
  - contains `run_id`, `weightment_id`, `batch_id`, `ingredient_id`, `location_id`, `target_weight_g`, and bag path
- `webhook_run/`
  - MCAP bag directory for the single webhook robot run
