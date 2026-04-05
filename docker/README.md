# ROS 2 Jazzy Docker

This repo now distinguishes between:

- `docker/ros/Dockerfile.dev`: local development image
- `docker/ros/Dockerfile.prod`: production/runtime image for registry push and Pi consumption
- `docker-compose.yml`: local build/dev workflow
- `docker-compose.prod.yml`: production counterpart to `docker-compose.yml` using prebuilt images
- `docker-compose.robot-prod.yml`: full webhook physical robot stack for Pi deployment
- `docker-compose.lightsout.yml`: special-purpose lightsout/runtime stack for the training/robot-run flow

MoveIt and MoveIt Task Constructor are expected from the Jazzy apt underlay on both `amd64` and
`arm64`; they are not vendored from source in this repo.

## Prereqs

- Docker Engine + Compose plugin
- For Raspberry Pi 5, use a 64-bit userspace and prefer pulling prebuilt multi-arch images instead
  of building large ROS images directly on the Pi

## Build local ROS images

Production-style runtime image:

```bash
docker build \
  --build-arg COLCON_PARALLEL_WORKERS=1 \
  -f docker/ros/Dockerfile.prod \
  -t rhapsodi-promtek:ros-prod-local .
```

Development shell image:

```bash
docker build \
  --build-arg COLCON_PARALLEL_WORKERS=8 \
  -f docker/ros/Dockerfile.dev \
  -t rhapsodi-promtek:ros-dev-local .
```

## Run with compose

Runtime shell (local build, baked `/ws/install` overlay):

```bash
docker compose run --rm runtime
```

Dev shell (bind-mounts the repo at `/workspace`):

```bash
docker compose run --rm dev
```

If you rebuild from the mounted repo inside `dev`:

```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
vcs import src < src/ros2.repos
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --merge-install --parallel-workers 1 --packages-skip micro_ros_agent
source install/setup.bash
```

## Dashboard

The dashboard is built from `docker/dashboard.Dockerfile` so its build args stay consistent with the
registry image.

```bash
VITE_API_BASE=http://localhost:8000 \
VITE_ROSBRIDGE_URL=ws://localhost:9090 \
VITE_MICROROS_HEARTBEAT_TOPIC=/microros/heartbeat \
docker compose up --build dashboard
```

## Pi / production pull model

The image-backed stack in `docker-compose.prod.yml` mirrors the same service architecture as
`docker-compose.yml`, but expects prebuilt tags such as:

- `iserenity/rhapsodi-promtek:ros-prod`
- `iserenity/rhapsodi-promtek:backend`
- `iserenity/rhapsodi-promtek:processing`
- `iserenity/rhapsodi-promtek:webhook`

Start it on a Pi with:

```bash
docker compose -f docker-compose.prod.yml pull
docker compose -f docker-compose.prod.yml up -d
```

## Lightsout stack

`docker-compose.lightsout.yml` is intentionally different. It is a special-purpose runtime stack for
the lightsout / robot-run workflow and includes services like `pouring_controller`, `orchestrator`,
`weight_sim`, and `data_collection` that are not part of the generic app-stack architecture.

## Robot-prod stack

`docker-compose.robot-prod.yml` is the one-command Pi deployment for the webhook physical robot flow.
It starts:

- the Pi-side app services (`db`, `backend`, `processing`, `webhook_service`)
- `rosbridge`
- `robot_start_adapter`
- `scooping_stack`
- `scale_launcher`
- `micro_ros_launcher`
- `pouring_controller`
- `data_collection`
- `orchestrator`

Use it on the Pi with:

```bash
cp robot-prod.env.example robot-prod.env
# edit robot-prod.env for this Pi / robot
docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml pull
docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml up -d
```

The recommended per-robot settings file is `robot-prod.env`, created from
`robot-prod.env.example`.

Recommended operator setup:

- run the dashboard on your laptop and point it to the Pi backend/rosbridge
- keep the Pi focused on robot runtime services
- if you separately host the dashboard on the Pi, build the dashboard image with Pi-reachable `VITE_API_BASE` and `VITE_ROSBRIDGE_URL` values because they are baked into the static app at build time

## Multi-arch push

Use the helper:

```bash
bash scripts/buildx_push_images.sh
```

or build manually with:

```bash
docker buildx build --platform linux/amd64,linux/arm64 \
  -f docker/ros/Dockerfile.prod \
  -t iserenity/rhapsodi-promtek:ros-prod --push .
```
