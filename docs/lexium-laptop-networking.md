# Laptop ↔ Schneider Lexium Cobot (JAKA Zu5 kinematic twin)

Host-level networking so an **amd64 laptop** can run the full cell stack
(`compose/devices/x86.yml`) and talk to the Schneider Lexium Cobot controller
over a direct Ethernet cable. This is **not** inside Docker; ROS containers use
`network_mode: host`, so the laptop NIC must already reach the controller.

The Lexium firmware speaks **JSON/TCP** (commands on port **10001**, feedback on
**10000**). The JAKA SDK (`jaka_driver` / `jaka_planner/moveit_server`, port
10004) **cannot** log into Lexium firmware — use `lexium_driver` instead.
See [LEXIUM_COBOT_PORTING.md](LEXIUM_COBOT_PORTING.md).

Canonical IPs for **this cell**:

| Host | Interface | Address |
|------|-----------|---------|
| Lexium controller | robot eth | `192.168.88.82/24` |
| Laptop | eth (Lexium profile) | `192.168.88.10/24` |
| Laptop | eth (JAKA profile alt) | `192.168.88.101/24` |
| Laptop | Wi‑Fi / upstream | DHCP / site network (optional) |

Set `ROBOT_IP=192.168.88.82` in `robot-prod.laptop.env` and put the laptop on
the same subnet.

## 1. Static address on the laptop NIC

Identify the interface plugged into the arm (`ip link` / `nmcli device`).

**Preferred:** use the switcher script so you can keep a Niryo/link-local
profile and flip to Lexium without rewriting the NIC by hand:

```bash
# Once — creates:
#   rhapsodi-niryo-link   → 169.254.99.187/16
#   rhapsodi-jaka-link    → 192.168.88.101/24
#   rhapsodi-lexium-link  → 192.168.88.10/24
bash scripts/switch_arm_ethernet.sh setup

# Then switch as needed:
bash scripts/switch_arm_ethernet.sh lexium
bash scripts/switch_arm_ethernet.sh niryo
bash scripts/switch_arm_ethernet.sh status
```

Override interface/addresses if needed:

```bash
IFACE=enp129s0 LEXIUM_ADDR=192.168.88.10/24 LEXIUM_ROBOT_IP=192.168.88.82 \
  bash scripts/switch_arm_ethernet.sh setup
```

Do **not** put a default route on this link if Wi‑Fi already provides upstream
internet (Condor agent needs outbound HTTPS). The switcher sets
`ipv4.never-default=yes` for that reason.

## 2. Verify reachability

```bash
ping -c 3 192.168.88.82
nc -vz 192.168.88.82 10001   # Lexium command channel
nc -vz 192.168.88.82 10000   # Lexium feedback stream (~50 Hz)
```

On the teach pendant / controller UI, set **control source = Remote (3)** before
expecting motion from ROS.

## 3. Compose env for the laptop cell

Build a local `ros-prod` image that includes `lexium_driver` / `lexium_msgs`
(ament_python + generated msgs cannot be bind-mounted):

```bash
make ros-image-local
# → iserenity/rhapsodi-promtek:ros-prod-lexium
```

```bash
cp robot-prod.env.example robot-prod.laptop.env
# Edit the "Laptop / Lexium" keys:
#   DEVICE_CLASS=x86
#   PROFILE_ID=prod-jaka
#   ROBOT_IP=192.168.88.82
#   TRAJ_ACTION_SERVER=/jaka_zu5_controller/follow_joint_trajectory
#   RHAPSODI_DEVICE_CONFIG=/ws/config/device.jaka-laptop.yaml
#   ROS_PROD_IMAGE=iserenity/rhapsodi-promtek:ros-prod-lexium
#   COMPOSE_FILE=compose/devices/x86.yml

# Required: --project-directory . (workspace root). Prefer:
make laptop-up
# equivalent:
docker compose --project-directory . --env-file robot-prod.laptop.env \
  -f compose/devices/x86.yml up -d
```

Do **not** omit `--project-directory` and do **not** change
`compose/devices/pi5.yml` dashboard binds to `../../docker/...` to work around
a missing path. That paper-fix breaks Pi/fleet-agent deploys (paths resolve
to `/opt/docker/...`). See [compose/README.md](../compose/README.md)
"Project directory".

Device identity: `config/device.jaka-laptop.yaml` (`robot_type: jaka`). Keep
`config/device.yaml` as the Niryo/Pi default so a shared checkout does not
break Pi deploy.

## 4. Operator bring-up (RViz Lexium Safety panel)

Compose stays headless. On the laptop host (same ROS graph via host networking):

```bash
# Once: build the Safety panel into the host install tree
colcon build --packages-select lexium_msgs lexium_rviz_plugins
source install/setup.bash

make lexium-session
# → ros2 launch scooping_controller scooping_rviz_only.launch.py robot:=jaka
```

Use the **Lexium Safety** panel: **Bring Up** (power + enable), **STOP**,
**Shut Down**. The panel talks to `/lexium/status` and `/lexium_driver/*`
Trigger services.

### Headless fallback

```bash
docker compose --project-directory . --env-file robot-prod.laptop.env \
  -f compose/devices/x86.yml exec scooping_stack \
  ros2 service call /lexium_driver/bring_up std_srvs/srv/Trigger

docker compose --project-directory . --env-file robot-prod.laptop.env \
  -f compose/devices/x86.yml exec scooping_stack \
  ros2 service call /lexium_driver/stop std_srvs/srv/Trigger
```

## 5. Read-only bring-up checks

```bash
# From a host ROS install, or: docker compose exec scooping_stack bash
ros2 node list | grep -E 'lexium_driver|move_group'
# Expect lexium_driver; do NOT expect moveit_server

ros2 topic info /joint_states -v
# Exactly one publisher: lexium_driver

ros2 topic echo /joint_states --once
# Plausible radians (not SDK garbage like 5.2e-310)

ros2 topic echo /lexium/status --once
# connected, feedback_fresh, control_source: 3, no faults

ros2 action list | grep jaka_zu5_controller
```

**Single `/joint_states` publisher rule:** if anything else (mock_components,
stale SDK node) also publishes `/joint_states`, MoveIt will see inconsistent
state. Stop competing publishers before authoring or running.

Do **not** command motion until those look healthy. Layout apply for
`mes-condor` (`dual-container`) still needs `targets_by_robot.jaka`
commissioned — that is a separate milestone. Verify `joint_sign` /
`joint_offset_deg` with slow jogs before trusting scoop trajectories.
