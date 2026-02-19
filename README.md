## ws_rhapsodi-promtek (ROS 2 Jazzy) — Docker on Laptop + Raspberry Pi 5

This repo contains a ROS 2 Jazzy workspace plus Docker assets to build/run it consistently on:
- **Laptop/desktop** (x86_64)
- **Raspberry Pi 5** (ARM64 on Raspberry Pi OS 64-bit)

It also includes an optional **React dashboard** (`src/dashboard`) that talks to ROS via **rosbridge websocket**.

### What’s in here

- **ROS workspace**: `src/` (your packages)
- **External source deps**: `src/ros2.repos` (fetched during Docker build)
- **ROS image**: `Dockerfile` (builds the workspace into `/ws/install`)
- **Compose**: `docker-compose.yml` (`runtime`, `dev`, `dashboard`)

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
docker build --build-arg COLCON_PARALLEL_WORKERS=1 -t rhapsodi-promtek:jazzy .
```

### Raspberry Pi 5 build (ARM64)

```bash
cd ws_rhapsodi-promtek
docker build --build-arg COLCON_PARALLEL_WORKERS=1 -t rhapsodi-promtek:jazzy .
```

Notes:
- `COLCON_PARALLEL_WORKERS=1` avoids OOM during `colcon build` on low-RAM systems and Docker Desktop defaults.
- The Docker build will automatically fetch external repos listed in `src/ros2.repos` using `vcs import`.

---

## Run ROS container

### Interactive shell (recommended first test)

```bash
docker run --rm -it --net=host --ipc=host rhapsodi-promtek:jazzy bash
```

Inside the container, ROS + the overlay are already sourced. Quick check:

```bash
ros2 pkg list | head
```

### Using docker compose

- Runtime (no source mounts):

```bash
docker compose run --rm runtime
```

- Dev (mounts your local `./src` so you can edit and rebuild inside the container):

```bash
docker compose run --rm dev
```

Rebuild overlay inside `dev`:

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --merge-install --parallel-workers 1
source /ws/install/setup.bash
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

Typical interface names:

- `<UPLINK_IFACE>`: `wlan0` or `wlp*` (Wi‑Fi) or `enx*` (USB‑Ethernet)
- `<PI_ETH_IFACE>`: the Ethernet interface to the Pi (e.g., `enp*`)

---

## Hardware access (serial/USB/camera)

Example serial device:

```bash
docker run --rm -it --net=host --ipc=host --device=/dev/ttyACM0 rhapsodi-promtek:jazzy bash
```

If you truly need broad access (use sparingly):

```bash
docker run --rm -it --net=host --ipc=host --privileged rhapsodi-promtek:jazzy bash
```

---

## More details

See `docker/README.md` for additional notes.

## Git workflow

See `GIT_WORKFLOW.md` for the commands to push to GitHub and pull/update on the laptop/Pi.


