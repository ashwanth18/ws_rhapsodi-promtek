# ROS 2 Jazzy Docker (Raspberry Pi 5 / Raspberry Pi OS)

This workspace can be built into a container image that includes the full colcon overlay at `/ws/install`.

For a complete Laptop + Raspberry Pi guide, see the repo root `README.md`.

## Prereqs on the Pi

- Install Docker Engine + Compose plugin on Raspberry Pi OS (64-bit).
- Use **ARM64** userspace (recommended on Pi 5).

## Build the image

From the repo root:

```bash
docker build -t rhapsodi-promtek:jazzy .
```

If the build fails with `cc1plus ... Killed` / `cannot allocate memory`, rebuild with lower parallelism:

```bash
docker build --build-arg COLCON_PARALLEL_WORKERS=1 -t rhapsodi-promtek:jazzy .
```

## Run (recommended for ROS 2 discovery)

```bash
docker run --rm -it --net=host --ipc=host rhapsodi-promtek:jazzy bash
```

Inside the container your environment is already sourced (ROS + `/ws/install` overlay). Example:

```bash
ros2 pkg list | head
```

## Using docker compose

- Runtime (no source mounts):

```bash
docker compose run --rm runtime
```

- Dev (mounts your local `./src` so you can edit and rebuild):

```bash
docker compose run --rm dev
```

If you change code in dev mode, rebuild the overlay inside the container:

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --merge-install
source /ws/install/setup.bash
```

## Dashboard (React + rosbridge)

Your React dashboard lives under `src/dashboard` and uses `roslib` to connect to rosbridge.

- **Default URL**: `ws://localhost:9090`
- **Override**: set `VITE_ROSBRIDGE_URL` at build time (it gets baked into the bundle)

Build + run the dashboard:

```bash
VITE_ROSBRIDGE_URL=ws://localhost:9090 DASHBOARD_PORT=8080 docker compose up --build dashboard
```

Run rosbridge in the ROS container (port 9090):

```bash
docker compose run --rm --service-ports runtime bash -lc "ros2 launch niryo_robot_simulation_client rosbridge_websocket_fixed.launch.py address:=0.0.0.0 port:=9090"
```

If you open the dashboard from another machine (not the Pi), set:

```bash
VITE_ROSBRIDGE_URL=ws://<pi-ip>:9090 docker compose up --build dashboard
```

## Hardware access (serial/USB/cameras)

For drivers that need device access on the Pi, add one of the following when running:

- Serial device example:

```bash
docker run --rm -it --net=host --ipc=host --device=/dev/ttyACM0 rhapsodi-promtek:jazzy bash
```

- Broad device access (use only if needed):

```bash
docker run --rm -it --net=host --ipc=host --privileged rhapsodi-promtek:jazzy bash
```
