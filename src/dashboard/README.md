# Dashboard (ROS 2 Web UI)

A lightweight React (Vite) frontend that connects directly to ROS 2 via rosbridge (WebSocket) using roslibjs.

## Features

- Single shared ROS connection (see `src/ros/RosContext.tsx`).
- Controls page to record named targets via the `/record_target` service.
- Pluggable architecture to add more tools (topic viewers, action clients). MoveTo was intentionally removed from the UI per design (use BT/CLI for motions).

## Prerequisites

- Node.js ≥ 18
- ROS 2 (Humble or compatible)
- rosbridge server (WebSocket)
  - Install: `sudo apt install ros-humble-rosbridge-server`
  - Run (recommended flags shown below)
  - Default URL: `ws://localhost:9090`

### Recommended rosbridge flags

To avoid UI stalls when calling services/actions via rosbridge and to set a sane timeout, run:

```bash
ros2 run rosbridge_server rosbridge_websocket --ros-args \
  -p default_call_service_timeout:=5.0 \
  -p call_services_in_new_thread:=true \
  -p send_action_goals_in_new_thread:=true
```

- `default_call_service_timeout` (seconds): prevents indefinite blocking when a service doesn’t respond (default is 0.0)
- `call_services_in_new_thread`: executes service calls without blocking the main thread
- `send_action_goals_in_new_thread`: sends action goals without blocking the main thread

Expose rosbridge publicly (optional, for remote dashboards):
```bash
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9090 -- --address 0.0.0.0
```

## Back-end expectations

- `robot_moveit` TargetRecorder service `/record_target` (`robot_common_msgs/srv/RecordTarget`)
  - Ensure `target_recorder_node` is running with `targets_yaml` set
- Optional: `pouring_controller` publishes `/pour_status` (`robot_common_msgs/msg/PourStatus`) for the weighing widget’s target/tolerance band

## Configuration

The dashboard reads endpoints from env vars. Create `.env.local` in this folder:

```
VITE_ROSBRIDGE_URL=ws://localhost:9090
# Optional if you expose a custom status topic for pouring
VITE_POUR_STATUS_TOPIC=/pour_status
# Optional custom weight topic
VITE_WEIGHT_TOPIC=/weight
```

If not set, defaults are used.

## Install & Run

```bash
# From this directory
npm install
npm run dev
# or build/preview
npm run build && npm run preview
```

Open the URL printed by Vite (e.g., http://localhost:5173).

## How it works

- `src/ros/RosContext.tsx` creates a single `ROSLIB.Ros` connection and provides it via React Context.
- Components use `useRos()` to construct `ROSLIB.Service`/`ROSLIB.Topic` objects.
- Example: call `/record_target` service
```ts
import ROSLIB from 'roslib';
import { useRos } from '../ros/RosContext';

const ros = useRos();
const srv = new ROSLIB.Service({
  ros,
  name: '/record_target',
  serviceType: 'robot_common_msgs/srv/RecordTarget',
});

srv.callService({ name: 'tap_off', joints: false }, (resp: any) => {
  console.log('record_target response:', resp);
});
```

## Message/Service schemas

`robot_common_msgs/srv/RecordTarget`:
```
string name
bool joints
---
bool success
string message
```

`robot_common_msgs/msg/PourStatus`:
```
float32 target_g
float32 band_threshold_g
float32 remaining_to_band_g
bool active
string phase
```

## Troubleshooting

- Import errors in rosbridge:
  - Run rosbridge in a shell where your workspace is sourced (`source install/setup.bash`), or use the distro package
- UI connects but calls fail:
  - Verify services and topics exist:
    - `ros2 service list | grep record_target`
    - `ros2 topic list | grep pour_status`
- WebSocket connectivity:
  - Ensure port 9090 is reachable from the browser host
  - If the dashboard runs over https, use `wss://` via a TLS proxy (nginx/caddy)

## Extending

- Topic viewer: create a `ROSLIB.Topic` and subscribe/unsubscribe in a React effect
- Action clients: use `ROSLIB.ActionClient` and `ROSLIB.Goal` where appropriate

## Production

```bash
npm run build
# Serve ./dist with a static web server
```
