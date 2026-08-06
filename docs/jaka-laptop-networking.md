# Laptop ↔ JAKA ZU5 direct ethernet

Host-level networking so an **amd64 laptop** can run the full cell stack
(`compose/devices/x86.yml`) and talk to a JAKA ZU5 controller over a direct
Ethernet cable. This is **not** inside Docker; ROS containers use
`network_mode: host`, so the laptop NIC must already reach the controller.

Canonical IPs (JAKA factory default):

| Host | Interface | Address |
|------|-----------|---------|
| JAKA controller | robot eth | `10.5.5.100/24` |
| Laptop | eth (to arm) | `10.5.5.101/24` (or any free `10.5.5.x`) |
| Laptop | Wi‑Fi / upstream | DHCP / site network (optional) |

If your controller was re-addressed, set `ROBOT_IP` in `robot-prod.laptop.env`
to that address and put the laptop on the same subnet.

## 1. Static address on the laptop NIC

Identify the interface plugged into the arm (`ip link` / `nmcli device`).

**Preferred:** use the switcher script so you can keep a Niryo/link-local
profile and flip to JAKA without rewriting the NIC by hand:

```bash
# Once — creates:
#   rhapsodi-niryo-link  → 169.254.99.187/16  (your current other-robot IP)
#   rhapsodi-jaka-link   → 10.5.5.101/24      (JAKA subnet)
bash scripts/switch_arm_ethernet.sh setup

# Then switch as needed:
bash scripts/switch_arm_ethernet.sh jaka
bash scripts/switch_arm_ethernet.sh niryo
bash scripts/switch_arm_ethernet.sh status
```

Override interface/addresses if needed:

```bash
IFACE=enp129s0 NIRYO_ADDR=169.254.99.187/16 JAKA_ADDR=10.5.5.101/24 \
  bash scripts/switch_arm_ethernet.sh setup
```

Manual NetworkManager example (same idea):

```bash
# Replace enp129s0 with your ethernet NIC name
sudo nmcli con add type ethernet ifname enp129s0 con-name rhapsodi-jaka-link \
  ipv4.method manual ipv4.addresses 10.5.5.101/24 ipv4.gateway "" \
  ipv4.never-default yes ipv6.method disabled
sudo nmcli con up rhapsodi-jaka-link
```

Do **not** put a default route on this link if Wi‑Fi already provides upstream
internet (Condor agent needs outbound HTTPS). The switcher sets
`ipv4.never-default=yes` for that reason.

## 2. Verify reachability

```bash
ping -c 3 10.5.5.100
# Optional: controller status port mentioned in the JAKA SDK (10004)
nc -vz 10.5.5.100 10004
```

`scooping_real` launches `jaka_planner/moveit_server` with `ip:=${ROBOT_IP}`
and `model:=zu5`. The action server appears as
`/jaka_zu5_controller/follow_joint_trajectory` and joint feedback on
`/joint_states`.

## 3. Compose env for the laptop cell

```bash
cp robot-prod.env.example robot-prod.laptop.env
# Edit the "Laptop / JAKA" keys (DEVICE_CLASS=x86, PROFILE_ID=prod-jaka,
# ROBOT_IP, TRAJ_ACTION_SERVER, RHAPSODI_DEVICE_CONFIG, USB scale/micro-ROS).

docker compose --project-directory . --env-file robot-prod.laptop.env \
  -f compose/devices/x86.yml up -d
```

Device identity: `config/device.jaka-laptop.yaml` (`robot_type: jaka`). Keep
`config/device.yaml` as the Niryo/Pi default so a shared checkout does not
break Pi deploy.

## 4. Read-only bring-up checks

```bash
# From a host ROS install, or: docker compose exec scooping_stack bash
ros2 node list | grep -E 'moveit_server|move_group'
ros2 action list | grep jaka_zu5_controller
ros2 topic echo /joint_states --once
ros2 param get /move_group moveit_simple_controller_manager.controller_names
```

Do **not** command motion until those look healthy. Layout apply for
`mes-condor` (`dual-container`) still needs `targets_by_robot.jaka`
commissioned — that is a separate milestone.

## Related

- [ROBOT_INTEGRATION.md](ROBOT_INTEGRATION.md) — JAKA vs Niryo profiles
- [pi-niryo-link-networking.md](pi-niryo-link-networking.md) — Pi ↔ Niryo link-local (different subnet)
- [compose/README.md](../compose/README.md) — `device_class` → compose resolution
