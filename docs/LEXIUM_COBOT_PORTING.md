# Schneider Lexium Cobot — Porting & Handoff Guide

**Audience:** a coding agent (or engineer) integrating this work into a *different* ROS 2 workspace that already has a robotic tech stack.  
**Source workspace:** `~/jaka_ros2` (this repo).  
**Goal:** copy only what is needed for the Schneider Lexium Cobot (Zu5-class) + MoveIt2, not the entire upstream JAKA SDK stack.

> Read this document end-to-end before copying packages. Wrong packages + wrong publishers on `/joint_states` will produce garbage joint values and false MoveIt collisions.

---

## 1. One-paragraph summary

The Schneider **Lexium Cobot** is kinematically a **JAKA Zu5**, so we reuse the Zu5 URDF/SRDF/MoveIt config. The upstream **JAKA SDK** (`libjakaAPI.so`, used by `jaka_driver` / `jaka_planner/moveit_server`) **cannot log into Lexium firmware** (SDK error 2 / gRPC 10014). We built a new ROS 2 driver that speaks Schneider’s native **JSON/TCP** protocol (commands on **TCP 10001**, feedback on **TCP 10000**) and presents the **same MoveIt interface** as the old `moveit_server`: `/joint_states` + `/jaka_zu5_controller/follow_joint_trajectory`.

**Hardware used in development:** controller IP `192.168.88.82`.  
**ROS distro:** ROS 2 **Jazzy** (Ubuntu 24.04).

---

## 2. What to copy vs what to leave behind

### 2.1 MUST copy (new work — Lexium driver)

Copy these three packages **verbatim** from `src/`:

| Package | Path | Build type | Role |
|---------|------|------------|------|
| `lexium_msgs` | `src/lexium_msgs/` | ament_cmake (rosidl) | `LexiumStatus.msg`, `MoveCommand.msg`, `MoveSequence.action` |
| `lexium_driver` | `src/lexium_driver/` | ament_python | TCP client + driver node + launch + yaml + RViz config + DESIGN.md |
| `lexium_rviz_plugins` | `src/lexium_rviz_plugins/` | ament_cmake (C++/Qt) | RViz **Lexium Safety** panel (Bring Up / Shut Down / STOP) |

Suggested copy command from this repo into the target workspace:

```bash
SRC=~/jaka_ros2/src
DST=~/<your_other_ws>/src

cp -a "$SRC/lexium_msgs" "$SRC/lexium_driver" "$SRC/lexium_rviz_plugins" "$DST/"
```

Also copy this handoff doc and keep the design reference:

- `LEXIUM_COBOT_PORTING.md` (this file)
- Inside `lexium_driver`: `README.md` + `docs/DESIGN.md` (deep architecture / safety)

### 2.2 MUST copy (upstream JAKA assets required by MoveIt — not the SDK driver)

The Lexium launch reuses Zu5 MoveIt config and launch helpers. Copy:

| Package | Path | Why |
|---------|------|-----|
| `jaka_zu5_moveit_config` | `src/jaka_zu5_moveit_config/` | URDF xacro wrapper, SRDF, kinematics, joint limits, `moveit_controllers.yaml` (`jaka_zu5_controller`) |
| `jaka_moveit_launches` | `src/jaka_moveit_launches/` | `generate_rsp_launch` / `generate_move_group_launch` used by `lexium_moveit.launch.py` |
| `jaka_description` | `src/jaka_description/` | `urdf/jaka_zu5.urdf` + `meshes/jaka_zu5_meshes/` (and package install rules) |

```bash
cp -a "$SRC/jaka_zu5_moveit_config" "$SRC/jaka_moveit_launches" "$SRC/jaka_description" "$DST/"
```

**Note on size:** `jaka_description` is ~139 MB because it contains meshes for *all* JAKA models. For a lean port you may slim the package to only:

- `urdf/jaka_zu5.urdf`
- `meshes/jaka_zu5_meshes/`
- plus `CMakeLists.txt` / `package.xml` / any shared `config`/`launch` the package installs

…but the simplest reliable port is to copy the whole `jaka_description` package and ignore unused meshes.

### 2.3 Do NOT copy (not needed for Lexium; will conflict)

| Package / area | Why skip |
|----------------|----------|
| `jaka_driver` | JAKA SDK driver; fails on Lexium; fights for robot connection |
| `jaka_planner` (esp. `moveit_server`) | Same SDK; publishes garbage `/joint_states` on Lexium |
| `jaka_msgs` | SDK-era services; Lexium stack does not use them |
| All other `jaka_*_moveit_config` (`zu3`, `zu7`, `pro5`, `a12`, …) | Wrong models |
| Root `README.md` JAKA SDK quickstart | Documents the *broken-on-Lexium* path |
| `build/`, `install/`, `log/` | Rebuild in the target workspace |

**Hard rule:** never run `jaka_driver`, `jaka_planner/moveit_server`, or any other node that publishes `/joint_states` for this arm **at the same time** as `lexium_driver`.

### 2.4 Optional slim-copy checklist

```
target_ws/src/
├── lexium_msgs/                 # REQUIRED (new)
├── lexium_driver/               # REQUIRED (new)
├── lexium_rviz_plugins/         # REQUIRED (RViz safety UI)
├── jaka_zu5_moveit_config/      # REQUIRED (MoveIt)
├── jaka_moveit_launches/        # REQUIRED (launch helpers)
└── jaka_description/            # REQUIRED (URDF + meshes; may slim to zu5)
```

That’s the entire Lexium bring-up footprint. Everything else in `jaka_ros2/src/` is unused for this arm.

---

## 3. Why the new driver exists

| Approach | Transport | Result on Lexium |
|----------|-----------|------------------|
| Upstream `jaka_driver` / `moveit_server` | JAKA SDK `libjakaAPI.so` → typically port **10004** / gRPC | Login rejected (error 2 / 10014) |
| **`lexium_driver` (this work)** | Schneider **LexiumCobotCommunication** JSON/TCP | Works |

Protocol channels:

| Channel | Port | Pattern | Purpose |
|---------|------|---------|---------|
| Commands | **10001** | request/response JSON | `moveJ`, `power_on`, `enable_robot`, `stop_program`, `set_speed_rate`, `get_control_source`, … |
| Feedback | **10000** | ~50 Hz concatenated JSON stream | `joint_position` (deg), power/enable, faults, `command_id`, `in_position` |

**Critical protocol fact:** `control_source` is **not** in the feedback stream. It is only available via the `get_control_source` command. The driver polls it ~1 Hz and re-queries before accepting motion. Control source must be **Remote (3)** (set in EcoStruxure Cobot Expert / pendant).

Units on the wire: **degrees** and **mm**. ROS `/joint_states` are published in **radians**.

---

## 4. Architecture (how it plugs into MoveIt)

```
RViz / your app
      │
      ▼
move_group (MoveIt2, jaka_zu5_moveit_config)
      │  FollowJointTrajectory goal
      ▼
/jaka_zu5_controller/follow_joint_trajectory   ← hosted by lexium_driver
      │
lexium_driver
  ├── lexium_tcp.py          TCP/JSON client (thread + reconnect)
  ├── /joint_states          rad, ~50 Hz
  ├── /lexium/status         lexium_msgs/LexiumStatus
  ├── services               bring_up, shut_down, stop, …
  └── move_sequence action   low-level per-move speeds
      │
      ├── TCP :10001  →  Lexium controller
      └── TCP :10000  ←  feedback stream
```

MoveIt needs **no controller-name changes**:  
`jaka_zu5_moveit_config/config/moveit_controllers.yaml` already points at `jaka_zu5_controller` / `follow_joint_trajectory`. The Lexium driver is a **drop-in replacement** for `jaka_planner/moveit_server`.

Trajectory path:

1. MoveIt plans (OMPL + time parameterization).
2. Driver downsamples waypoints (`downsample_threshold_deg`, default 15°).
3. Converts rad → Lexium deg (`joint_sign` / `joint_offset_deg`).
4. Streams `moveJ` commands (blended or sequential).
5. Tracks progress via feedback `command_id` / `in_position`.
6. Optionally derives global `set_speed_rate` from planned peak velocity so RViz velocity scaling is visible (`speed_rate_from_trajectory: true`).

---

## 5. Package internals (file map)

### 5.1 `lexium_msgs`

```
lexium_msgs/
├── msg/LexiumStatus.msg      # connection, control_source, power, faults, …
├── msg/MoveCommand.msg       # one protocol move (deg/mm, not remapped)
├── action/MoveSequence.action
├── CMakeLists.txt
└── package.xml
```

### 5.2 `lexium_driver`

```
lexium_driver/
├── lexium_driver/
│   ├── lexium_tcp.py              # pure Python TCP/JSON client (~230 LOC)
│   └── lexium_driver_node.py      # ROS node (~1100 LOC)
├── config/
│   ├── lexium_driver.yaml         # ALL tunable defaults (IP, speeds, safety)
│   └── moveit_lexium.rviz         # RViz layout + Safety panel
├── launch/
│   ├── lexium_driver.launch.py        # driver only
│   ├── lexium_moveit.launch.py        # driver + MoveIt + RViz (main entry)
│   └── lexium_moveit_rviz.launch.py   # RViz with Lexium config
├── docs/DESIGN.md                 # deep design + safety (READ BEFORE HARDWARE)
├── README.md                      # operator quickstart
├── package.xml / setup.py / setup.cfg
└── resource/lexium_driver
```

Entry point: console script `lexium_driver` → `lexium_driver.lexium_driver_node:main`.

### 5.3 `lexium_rviz_plugins`

```
lexium_rviz_plugins/
├── src/safety_panel.cpp
├── include/lexium_rviz_plugins/{safety_panel.hpp,error_code_utils.hpp}
├── plugin_description.xml
└── CMakeLists.txt / package.xml
```

Panel class: `lexium_rviz_plugins/SafetyPanel`. Subscribes to `/lexium/status`, calls Trigger services.

---

## 6. ROS interfaces (contract for the target stack)

### Topics (published by `lexium_driver`)

| Topic | Type | Rate / notes |
|-------|------|--------------|
| `/joint_states` | `sensor_msgs/JointState` | ~50 Hz, radians, names `joint_1`…`joint_6` |
| `/lexium/status` | `lexium_msgs/LexiumStatus` | ~50 Hz structured status |

### Actions (servers)

| Action | Type | Who uses it |
|--------|------|-------------|
| `/jaka_zu5_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | MoveIt `move_group` |
| `/lexium_driver/move_sequence` | `lexium_msgs/action/MoveSequence` | Scripts needing per-move speed |

### Services (`std_srvs/srv/Trigger` under `/lexium_driver/`)

| Service | Purpose |
|---------|---------|
| `bring_up` | **Preferred:** `power_on` → settle → `enable_robot` (retries + power-cycle recovery) |
| `shut_down` | **Preferred:** `disable_robot` → `power_off` |
| `clear_error` | Clear controller faults |
| `stop` | `stop_program` (immediate stop) |
| `power_on` / `power_off` / `enable` / `disable` | Low-level steps (debug only) |

### Important parameters (`config/lexium_driver.yaml`)

| Param | Default | Meaning |
|-------|---------|---------|
| `ip` | `192.168.88.82` | Controller IP |
| `command_port` / `feedback_port` | `10001` / `10000` | Protocol ports |
| `controller_name` | `jaka_zu5_controller` | Action namespace prefix |
| `joints` | `joint_1`…`joint_6` | Order must match MoveIt |
| `joint_sign` / `joint_offset_deg` | identity / zeros | **Calibrate on hardware** |
| `speed_rate` | `0.1` | Conservative global override on startup |
| `speed_rate_from_trajectory` | `true` | Map MoveIt velocity scaling → `set_speed_rate` |
| `downsample_threshold_deg` | `15.0` | Waypoint density (larger ⇒ longer segments) |
| `require_remote_control` | `true` | Require control_source == 3 |
| `auto_power_on` / `auto_enable` | `false` | Operator must Bring Up |
| `feedback_timeout` | `1.0` | Stale-feedback watchdog (s) |
| `state_command_timeout` | `45.0` | Power/enable can be slow |

---

## 7. Safety model (must preserve when porting)

Motion is **rejected** unless all of:

1. Command channel connected  
2. Feedback fresh (`age ≤ feedback_timeout`)  
3. `control_source == 3` (Remote), if `require_remote_control`  
4. `powered_on` and `enabled`  
5. No faults: `protective_stop`, `emergency_stop`, `collision_stop`, `on_soft_limit`

During motion, the same checks run continuously; any failure → `stop_program` + abort goal.  
Cancel / shutdown also send `stop_program`.  
Only **one** active goal across both action servers.

**Residual risks (not enforced by the driver):**

- Unverified `joint_sign` / `joint_offset_deg` (defaults are identity — **verify with slow jogs**)
- No TLS (plain TCP; use on trusted network only)
- `moveJ` has no duration → MoveIt timing is approximate
- Planning-scene / URDF / payload mistakes are application-level

Full details: `src/lexium_driver/docs/DESIGN.md` §8.

---

## 8. Build & run (in the target workspace)

### Dependencies (system / ROS)

- ROS 2 **Jazzy**
- MoveIt 2 (`moveit_ros_move_group`, `moveit_configs_utils`, planners, RViz plugins)
- `rclpy`, `control_msgs`, `sensor_msgs`, `std_srvs`, `trajectory_msgs`
- For RViz panel: Qt5 + `rviz_common` / `pluginlib`
- Python 3 (stdlib sockets/json/threading only for TCP client — no pip extras)

No `libjakaAPI.so` dependency for Lexium packages.

### Build

```bash
cd ~/<your_other_ws>
source /opt/ros/jazzy/setup.bash
# if the target ws already has an overlay, source it too

colcon build --packages-select \
  lexium_msgs \
  lexium_rviz_plugins \
  lexium_driver \
  jaka_description \
  jaka_moveit_launches \
  jaka_zu5_moveit_config

source install/setup.bash
```

Build order note: `lexium_msgs` first (interfaces), then plugins + driver.

### Launch (main entry)

```bash
ros2 launch lexium_driver lexium_moveit.launch.py ip:=192.168.88.82
```

Driver only (no MoveIt):

```bash
ros2 launch lexium_driver lexium_driver.launch.py ip:=192.168.88.82
```

### Operator sequence on hardware

1. On the pendant / EcoStruxure: set **control source = Remote**.  
2. Launch `lexium_moveit.launch.py`.  
3. In RViz **Lexium Safety** panel: click **Bring Up** (or call `/lexium_driver/bring_up`). Wait until success (can take up to ~1 min).  
4. Verify:

```bash
ros2 topic info /joint_states -v
# Publisher Node name MUST be: lexium_driver

ros2 topic echo /joint_states --once
# Positions should look like real radians (~ ±π), NOT 1e-310 garbage

ros2 topic echo /lexium/status --once
# connected, feedback_fresh, control_source: 3, powered_on, enabled, faults false
```

5. Plan & Execute in MoveIt at **low** velocity/accel scaling first. Re-Plan after changing sliders.

### Shut down

RViz **Shut Down**, or:

```bash
ros2 service call /lexium_driver/shut_down std_srvs/srv/Trigger
```

---

## 9. Integration notes for an existing tech stack

### 9.1 Controllers / action names

If your other workspace already defines a MoveIt controller name other than `jaka_zu5_controller`, you have two options:

1. **Preferred for least change:** keep `jaka_zu5_moveit_config` and `controller_name: jaka_zu5_controller` as-is.  
2. Or change **both**:
   - `lexium_driver.yaml` → `controller_name`
   - MoveIt `moveit_controllers.yaml` → matching name  
   so the action path stays consistent.

Joint names must remain `joint_1`…`joint_6` (or update both URDF/SRDF and driver `joints`).

### 9.2 Do not double-publish `/joint_states`

Your stack must ensure **exactly one** publisher of `/joint_states` for this robot. Candidates that conflicted in practice:

- `jaka_planner/moveit_server` (garbage values when SDK can’t read Lexium)
- Other cell stacks / Docker Compose ROS domains sharing `ROS_DOMAIN_ID`

**Symptom of wrong publisher:** positions like `5.213e-310`, `6.4e-323` → MoveIt reports `START_STATE_IN_COLLISION` with a twisted robot.

**Debug:**

```bash
ros2 topic info /joint_states -v
ros2 node list | grep -E 'lexium_driver|moveit_server|jaka'
```

Direct sanity check of the robot (bypasses ROS):

```bash
python3 - <<'PY'
import socket, json, time
s = socket.create_connection(("192.168.88.82", 10000), timeout=3)
buf = b""; t = time.time()
while time.time() - t < 2:
    buf += s.recv(4096)
    try:
        d = json.loads(buf.decode())
        print(d.get("joint_position")); break
    except json.JSONDecodeError:
        pass
s.close()
PY
# Expect ~[-0.07, 89.8, -90.3, 89.4, 89.9, 0.0] degrees when upright
```

### 9.3 Docker / other ROS domains on the same laptop

On the development machine, a **Docker** container (`ws_rhapsodi-promtek-dev-scooping_stack-1`, image `iserenity/rhapsodi-promtek:ros-prod`) launched a scooping cell stack that included `moveit_server` with params `ip:=10.5.5.100`, `model:=zu5`. That was invisible in local terminals but polluted the ROS graph. **Stop such containers** before Lexium bring-up:

```bash
docker ps | grep -i scooping
docker stop <container>
```

When porting: document any compose projects in the target repo that might also launch Zu5/`moveit_server`.

### 9.4 Planning scene obstacles

Collision objects like `table` / `rs6` seen in RViz are **not** part of the Lexium packages — they come from other apps (e.g. scooping cell). Clear them if they cause false collisions after joint states are fixed.

### 9.5 Launch composition tip

`lexium_moveit.launch.py` sets `use_rviz_sim:=false` so MoveIt does **not** start mock `ros2_control` hardware. If you write a custom launch in the target stack, keep that: real execution goes through the Lexium action server only.

### 9.6 Bring Up quirks learned on hardware

- After `shut_down`, feedback can briefly still show `powered_on: true` — naive “already powered” skips of `power_on` then fail `enable_robot` (`errorCode=2`). The driver’s `bring_up` does a full power-cycle recovery + enable retries. Prefer `bring_up` over raw `power_on`/`enable`.
- Treat `error_code: "0x0"` / `"0"` as **no fault** (string comparison pitfalls).
- State commands need long timeouts (~45 s); don’t use the short move timeout for power/enable.

---

## 10. Known limitations

- No TLS (`ConnectTLS` not implemented).
- No gripper / IO / force control in this driver.
- No streaming Cartesian velocity control (protocol limitation for MoveIt servo-style use).
- MoveIt time parameterization is approximated via `moveJ` speed/acc + global `speed_rate`.
- Joint mapping defaults are uncalibrated until verified on the specific cell.

---

## 11. Verification checklist for the porting agent

After copying packages into the target workspace:

- [ ] Only the six packages from §2.4 are added (or slimmed `jaka_description`).
- [ ] `jaka_driver` / `jaka_planner` are **not** launched for this robot.
- [ ] `colcon build` of the six packages succeeds on Jazzy.
- [ ] `ros2 launch lexium_driver lexium_moveit.launch.py ip:=<controller_ip>` starts without missing-package errors.
- [ ] `ros2 topic info /joint_states -v` shows **only** `lexium_driver` as publisher.
- [ ] `/joint_states` positions are sane radians (not scientific-notation near-zero garbage).
- [ ] `/lexium/status` shows `control_source: 3` after Remote is selected on the pendant.
- [ ] **Bring Up** succeeds; then a tiny MoveIt move works at low speed.
- [ ] RViz Lexium Safety panel appears (or add via Panels → `lexium_rviz_plugins/SafetyPanel`).
- [ ] No second stack (Docker/other workspace) shares the same `ROS_DOMAIN_ID` with a competing `/joint_states` publisher.

---

## 12. Suggested commit / PR message for the target repo

```
Add Schneider Lexium Cobot driver (JSON/TCP) and Zu5 MoveIt reuse.

Port lexium_msgs, lexium_driver, lexium_rviz_plugins from jaka_ros2, plus
jaka_zu5_moveit_config / jaka_moveit_launches / jaka_description. Drop-in
replacement for JAKA SDK moveit_server, which cannot login on Lexium firmware.
```

---

## 13. Where to read more in the source tree

| Doc | Path |
|-----|------|
| Operator quickstart | `src/lexium_driver/README.md` |
| Architecture, protocol, safety (authoritative) | `src/lexium_driver/docs/DESIGN.md` |
| Tunables | `src/lexium_driver/config/lexium_driver.yaml` |
| Combined launch | `src/lexium_driver/launch/lexium_moveit.launch.py` |
| MoveIt controller binding | `src/jaka_zu5_moveit_config/config/moveit_controllers.yaml` |
| This handoff | `LEXIUM_COBOT_PORTING.md` (repo root) |

---

## 14. Copy-paste: minimal rsync of exactly what’s needed

```bash
SRC=~/jaka_ros2/src
DST=~/<your_other_ws>/src

mkdir -p "$DST"
rsync -a \
  "$SRC/lexium_msgs/" \
  "$DST/lexium_msgs/"
rsync -a \
  "$SRC/lexium_driver/" \
  "$DST/lexium_driver/"
rsync -a \
  "$SRC/lexium_rviz_plugins/" \
  "$DST/lexium_rviz_plugins/"
rsync -a \
  "$SRC/jaka_zu5_moveit_config/" \
  "$DST/jaka_zu5_moveit_config/"
rsync -a \
  "$SRC/jaka_moveit_launches/" \
  "$DST/jaka_moveit_launches/"
rsync -a \
  "$SRC/jaka_description/" \
  "$DST/jaka_description/"

cp ~/jaka_ros2/LEXIUM_COBOT_PORTING.md "$DST/../"

# Explicitly do NOT copy:
# jaka_driver, jaka_planner, jaka_msgs, other jaka_*_moveit_config
```

Then build (§8) and verify (§11).

---

*End of handoff. If something is unclear, prefer `lexium_driver/docs/DESIGN.md` over guessing — especially for safety gating and `control_source` polling.*
