## ws_rhapsodi-promtek (ROS 2 Jazzy) — Docker on Laptop + Raspberry Pi 5

This repo contains a ROS 2 Jazzy workspace plus Docker assets to build/run it consistently on:
- **Laptop/desktop** (x86_64)
- **Raspberry Pi 5** (ARM64 on Raspberry Pi OS 64-bit)

It also includes an optional **React dashboard** (`src/dashboard`) that talks to ROS via **rosbridge websocket**.

The container builds assume `MoveIt` and `MoveIt Task Constructor` come from the ROS 2 Jazzy apt
underlay on both `amd64` and `arm64`; they are not fetched from `src/ros2.repos`.

### What’s in here

- **ROS workspace**: `src/` (your packages)
- **External source deps**: `src/ros2.repos` (fetched during Docker build)
- **ROS dev image**: `docker/ros/Dockerfile.dev` (local build + interactive dev shell)
- **ROS prod image**: `docker/ros/Dockerfile.prod` (multi-stage runtime image for registry / Pi)
- **Compose**:
  - `docker-compose.yml` for local build/dev
  - `docker-compose.prod.yml` for the same architecture using prebuilt images
  - `docker-compose.robot-prod.yml` for the full webhook physical robot stack on a Pi
  - `docker-compose.lightsout.yml` for the special-purpose lightsout/runtime stack

---

## Prerequisites

### On a laptop (Ubuntu/etc.)
- Docker Engine + Compose plugin

### On Raspberry Pi 5
- Raspberry Pi OS **64-bit** (recommended)
- Docker Engine + Compose plugin installed
- Expect long builds on Pi; use `COLCON_PARALLEL_WORKERS=1`

---

## Build the ROS image

### Laptop build (x86_64)

```bash
cd ws_rhapsodi-promtek
docker build \
  --build-arg COLCON_PARALLEL_WORKERS=1 \
  -f docker/ros/Dockerfile.prod \
  -t rhapsodi-promtek:ros-prod-local .
```

### Faster builds on a strong laptop

If you have a high‑end laptop, you can speed up builds with more colcon workers
and BuildKit caching:

```bash
cd ws_rhapsodi-promtek
DOCKER_BUILDKIT=1 docker build \
  --build-arg COLCON_PARALLEL_WORKERS=24 \
  -f docker/ros/Dockerfile.prod \
  -t rhapsodi-promtek:ros-prod-local .
```

If you build multi‑arch often, enable a local buildx cache:

```bash
docker buildx create --use
docker buildx build \
  --cache-from=type=local,src=./.buildx-cache \
  --cache-to=type=local,dest=./.buildx-cache-new \
  --platform linux/amd64,linux/arm64 \
  --build-arg COLCON_PARALLEL_WORKERS=24 \
  -f docker/ros/Dockerfile.prod \
  -t iserenity/rhapsodi-promtek:ros-prod --push .
mv ./.buildx-cache-new ./.buildx-cache
```

Cached multi‑arch builds for all images:

```bash
docker buildx create --use

docker buildx build \
  --cache-from=type=local,src=./.buildx-cache \
  --cache-to=type=local,dest=./.buildx-cache-new \
  --platform linux/amd64,linux/arm64 \
  --build-arg COLCON_PARALLEL_WORKERS=24 \
  -t iserenity/rhapsodi-promtek:backend --push ./src/backend
mv ./.buildx-cache-new ./.buildx-cache

docker buildx build \
  --cache-from=type=local,src=./.buildx-cache \
  --cache-to=type=local,dest=./.buildx-cache-new \
  --platform linux/amd64,linux/arm64 \
  --build-arg COLCON_PARALLEL_WORKERS=24 \
  -t iserenity/rhapsodi-promtek:processing --push ./src/backend/processing
mv ./.buildx-cache-new ./.buildx-cache

docker buildx build \
  --cache-from=type=local,src=./.buildx-cache \
  --cache-to=type=local,dest=./.buildx-cache-new \
  --platform linux/amd64,linux/arm64 \
  -t iserenity/rhapsodi-promtek:webhook --push ./src/backend/webhook_service
mv ./.buildx-cache-new ./.buildx-cache

docker buildx build \
  --cache-from=type=local,src=./.buildx-cache \
  --cache-to=type=local,dest=./.buildx-cache-new \
  --platform linux/amd64,linux/arm64 \
  --build-arg COLCON_PARALLEL_WORKERS=24 \
  -t iserenity/rhapsodi-promtek:dashboard --push -f docker/dashboard.Dockerfile .
mv ./.buildx-cache-new ./.buildx-cache
```

Notes:
- Avoid `--no-cache` unless needed.
- Keep the build context small (use `.dockerignore` to exclude `data/`, `log/`).

### Raspberry Pi 5 build (ARM64)

```bash
cd ws_rhapsodi-promtek
docker build \
  --build-arg COLCON_PARALLEL_WORKERS=1 \
  -f docker/ros/Dockerfile.prod \
  -t rhapsodi-promtek:ros-prod-local .
```

Notes:
- `COLCON_PARALLEL_WORKERS=1` avoids OOM during `colcon build` on low-RAM systems and Docker Desktop defaults.
- The Docker build will automatically fetch external repos listed in `src/ros2.repos` using `vcs import`.

---

## Run ROS container

### Interactive shell (recommended first test)

```bash
docker run --rm -it --net=host --ipc=host rhapsodi-promtek:ros-prod-local bash
```

Inside the container, ROS + the overlay are already sourced. Quick check:

```bash
ros2 pkg list | head
```

### Using docker compose

- Runtime (local build of the production ROS image, no source mounts):

```bash
docker compose run --rm runtime
```

- Dev shell (bind-mounts the repo at `/workspace` so you can edit locally and rebuild inside the container):

```bash
docker compose run --rm dev
```

Rebuild overlay inside `dev`:

```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
vcs import src < src/ros2.repos
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --merge-install --parallel-workers 1 --packages-skip micro_ros_agent
source install/setup.bash
```

---

## Dashboard (React) + rosbridge

### 1) Run rosbridge on the machine that runs ROS (Pi or laptop)

This launch file already exists in the workspace:

```bash
docker compose run --rm --service-ports runtime bash -lc "ros2 launch niryo_robot_simulation_client rosbridge_websocket_fixed.launch.py address:=0.0.0.0 port:=9090"
```

### 2) Run the dashboard web server

If you open the dashboard **on the same machine** that runs the dashboard container:

```bash
VITE_ROSBRIDGE_URL=ws://localhost:9090 DASHBOARD_PORT=8080 docker compose up --build dashboard
```

Then open:
- `http://localhost:8080`

If ROS+rosbridge are on the **Pi** but you open the dashboard from your **laptop browser**, set the Pi IP:

```bash
VITE_ROSBRIDGE_URL=ws://<pi-ip>:9090 DASHBOARD_PORT=8080 docker compose up --build dashboard
```

Then open:
- `http://<pi-ip>:8080` (if dashboard runs on the Pi)
- or `http://localhost:8080` (if dashboard runs on your laptop)

---

## Run dashboard on laptop (point to a Pi)

Use this if you want the **dashboard container on your laptop** but the **ROS stack on the Pi**.

1) Build the dashboard locally with the Pi IP:

```bash
VITE_API_BASE=http://<pi-ip>:8000 \
VITE_ROSBRIDGE_URL=ws://<pi-ip>:9090 \
VITE_MICROROS_HEARTBEAT_TOPIC=/microros/heartbeat \
docker compose build --no-cache dashboard
```

2) Start the dashboard container:

```bash
docker compose up -d dashboard
```

3) Open:

- `http://localhost:8080`

To point at a different Pi, rebuild with the new IPs.

---

## Docker Hub push (ARM64 for Pi)

If you use Docker Hub (single private repo), build and push ARM64 images from your laptop:

```bash
docker login
docker buildx create --use
```

ROS production image:

```bash
docker buildx build --platform linux/arm64 -f docker/ros/Dockerfile.prod -t iserenity/rhapsodi-promtek:ros-prod --push .
```

Backend:

```bash
docker buildx build --platform linux/arm64 -t iserenity/rhapsodi-promtek:backend --push ./src/backend
```

Processing:

```bash
docker buildx build --platform linux/arm64 -t iserenity/rhapsodi-promtek:processing --push ./src/backend/processing
```

Webhook:

```bash
docker buildx build --platform linux/arm64 -t iserenity/rhapsodi-promtek:webhook --push ./src/backend/webhook_service
```

Dashboard:

```bash
docker buildx build --platform linux/arm64 -t iserenity/rhapsodi-promtek:dashboard --push -f docker/dashboard.Dockerfile .
```

Multi‑arch (amd64 + arm64) from the laptop:

```bash
docker buildx build --platform linux/amd64,linux/arm64 -f docker/ros/Dockerfile.prod -t iserenity/rhapsodi-promtek:ros-prod --push .
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:backend --push ./src/backend
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:processing --push ./src/backend/processing
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:webhook --push ./src/backend/webhook_service
docker buildx build --platform linux/amd64,linux/arm64 -t iserenity/rhapsodi-promtek:dashboard --push -f docker/dashboard.Dockerfile .
```

On each Pi for the generic production app-stack:

```bash
docker compose -f docker-compose.prod.yml pull
docker compose -f docker-compose.prod.yml up -d
```

`docker-compose.lightsout.yml` remains available as a different stack shape for the lightsout /
robot-run workflow and is not the production equivalent of `docker-compose.yml`.

For the full webhook physical robot runtime stack on a Pi with minimal manual steps:

```bash
cp robot-prod.env.example robot-prod.env
# edit robot-prod.env for this Pi / robot
docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml pull
docker compose --env-file robot-prod.env -f docker-compose.robot-prod.yml up -d
```

The per-Pi settings live in:

```bash
robot-prod.env
```

Start from:

```bash
robot-prod.env.example
```

If you do not need per-Pi overrides, plain compose also works:

```bash
docker compose -f docker-compose.robot-prod.yml pull
docker compose -f docker-compose.robot-prod.yml up -d
```

This stack includes the Pi-side app and robot runtime services:

- `db`
- `backend`
- `processing`
- `webhook_service`
- `dashboard`
- `rosbridge`
- `robot_start_adapter`
- `scooping_stack`
- `scale_launcher`
- `micro_ros_agent`
- `pouring_controller`
- `data_collection`
- `orchestrator`

Recommended dashboard deployment:

- run the dashboard on your laptop and point it to the Pi backend + rosbridge for normal operation
- only run the dashboard on the Pi if you specifically want a separate kiosk-style setup outside `docker-compose.robot-prod.yml`
- the current dashboard image bakes `VITE_API_BASE` and `VITE_ROSBRIDGE_URL` at build time, so a Pi-hosted dashboard should be built with the Pi-reachable URLs

---

## Laptop ↔ Pi Ethernet + Internet sharing

If your Pi is connected to your laptop over Ethernet and you want the Pi to use
the laptop’s internet (Wi‑Fi or another uplink), use NetworkManager sharing.

### 1) Enable sharing on the laptop (Linux)

- Open `nm-connection-editor`
- Select the **Ethernet** connection to the Pi
- IPv4 Settings → **Method: Shared to other computers** → Save
- Disconnect/reconnect that Ethernet interface

### 2) Verify the laptop side

```bash
ip a | grep 10.42
```

You should see `10.42.0.1/24` on the Ethernet interface to the Pi.

### 3) Verify the Pi side

```bash
ip r
ping -c 3 10.42.0.1
ping -c 3 8.8.8.8
curl -I https://files.pythonhosted.org/
```

If the Pi can ping `10.42.0.1` but not `8.8.8.8`, enable forwarding/NAT on the
laptop (replace `<UPLINK_IFACE>` with your internet interface from
`ip r | grep default`):

```bash
sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -A POSTROUTING -o <UPLINK_IFACE> -j MASQUERADE
sudo iptables -A FORWARD -i <UPLINK_IFACE> -o <PI_ETH_IFACE> -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i <PI_ETH_IFACE> -o <UPLINK_IFACE> -j ACCEPT
```

## EXAMPLE

```bash
sudo sysctl -w net.ipv4.ip_forward=1

sudo iptables -t nat -A POSTROUTING -o wlp130s0f0 -j MASQUERADE
sudo iptables -A FORWARD -i wlp130s0f0 -o enp129s0 -m state --state RELATED,ESTABLISHED -j ACCEPT
sudo iptables -A FORWARD -i enp129s0 -o wlp130s0f0 -j ACCEPT
```

Typical interface names:

- `<UPLINK_IFACE>`: `wlan0` or `wlp*` (Wi‑Fi) or `enx*` (USB‑Ethernet)
- `<PI_ETH_IFACE>`: the Ethernet interface to the Pi (e.g., `enp*`)

---

## Dual‑NIC network setup (Pi ↔ Niryo + Pi ↔ Laptop)

Use this if the **Pi connects directly to the Niryo** *and* to your **laptop**.

### Topology

- `eth0` (Pi built‑in) → **Niryo** (link‑local)
- `enx…` (USB‑Ethernet) → **Laptop** (shared network)

### IP plan

- **Niryo**: `169.254.200.200/16`
- **Pi (eth0)**: `169.254.200.201/16`
- **Pi (USB‑Ethernet)**: `10.42.0.72/24`
- **Laptop (shared)**: `10.42.0.1/24`

### Laptop settings

Set the laptop Ethernet (to Pi) as **“Shared to other computers”**.  
This assigns `10.42.0.1` and provides internet to the Pi.

### Pi (Ubuntu Netplan) config

Edit `/etc/netplan/50-cloud-init.yaml`:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses: [169.254.200.201/16]
    enx9cebe8b7ccea:
      dhcp4: no
      addresses: [10.42.0.72/24]
      routes:
        - to: default
          via: 10.42.0.1
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
  wifis:
    wlan0:
      # keep your existing Wi‑Fi config (if any)
```

Apply:

```bash
sudo netplan apply
```

Verify:

```bash
ping -c 3 169.254.200.200
ping -c 3 10.42.0.1
```

---

## Pi Ubuntu static IP (Netplan)

If your Pi runs **Ubuntu**, configure a static IP using Netplan.

1) Find the Ethernet interface (e.g., `eth0`):

```bash
ip link
```

2) Edit the Netplan config (example file name):

```bash
sudo nano /etc/netplan/01-netcfg.yaml
```

3) Example static IP config:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses: [10.42.0.72/24]
      gateway4: 10.42.0.1
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
```

4) Apply:

```bash
sudo netplan apply
```

5) Verify:

```bash
ip a | grep 10.42
ip r
ping -c 3 10.42.0.1
```

---

## Hardware access (serial/USB/camera)

Example serial device:

```bash
docker run --rm -it --net=host --ipc=host --device=/dev/ttyACM0 rhapsodi-promtek:ros-prod-local bash
```

If you truly need broad access (use sparingly):

```bash
docker run --rm -it --net=host --ipc=host --privileged rhapsodi-promtek:ros-prod-local bash
```

---

## More details

See `docker/README.md` for additional notes.

## Git workflow

See `GIT_WORKFLOW.md` for the commands to push to GitHub and pull/update on the laptop/Pi.


