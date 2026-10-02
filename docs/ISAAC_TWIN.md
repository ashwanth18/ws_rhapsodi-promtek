# Isaac Sim digital twin of the scooping cell

A guide for robotics engineers who know ROS 2, MoveIt and the cell, but have not worked
with Isaac Sim, USD, PhysX particles or simulated sensors. It explains what was built,
why each design choice was made, the maths behind the powder models, and the problems
we hit. It ends with a step-by-step recipe to rebuild the twin from scratch.

- Operator reference (commands, flags, numbers): `src/isaac_twin/README.md`
- Notes for AI agents (rules, file map, gotchas): `src/isaac_twin/CLAUDE.md`
- The vision side the twin feeds: `docs/SCOOP_VISION.md`

## Contents

1. [What the twin is, in one page](#1-what-the-twin-is-in-one-page)
2. [Background you need](#2-background-you-need)
3. [System architecture](#3-system-architecture)
4. [Frames and geometry](#4-frames-and-geometry)
5. [Building the Isaac scene](#5-building-the-isaac-scene)
6. [Controlling the simulated arm](#6-controlling-the-simulated-arm)
7. [Simulated sensors: D455 and scale](#7-simulated-sensors-d455-and-scale)
8. [Powder model A: heightfield](#8-powder-model-a-heightfield)
9. [Powder model B: PhysX particles](#9-powder-model-b-physx-particles)
10. [How vibration is modelled](#10-how-vibration-is-modelled)
11. [Running the real behaviour tree on the twin](#11-running-the-real-behaviour-tree-on-the-twin)
12. [The gym environment](#12-the-gym-environment)
13. [Validation results](#13-validation-results)
14. [Performance: where the time goes](#14-performance-where-the-time-goes)
15. [Debugging and tuning methods](#15-debugging-and-tuning-methods)
16. [Problems we hit and what fixed them](#16-problems-we-hit-and-what-fixed-them)
17. [Rebuilding the twin from scratch](#17-rebuilding-the-twin-from-scratch)
18. [Limitations and next steps](#18-limitations-and-next-steps)
19. [Glossary](#19-glossary)
20. [Code map](#20-code-map)

---

## 1. What the twin is, in one page

The real cell is a Niryo Ned3 Pro with a scoop, a D455 depth camera looking down, a source
container (RS6) full of powder, and a weighing container (RS3) on a scale. Software:
ros2_control, MoveIt, the MTC scooping task, scoop_vision (camera-guided scoop planning),
the pouring controller and the orchestrator behaviour tree (BT).

The twin replaces only the **physical world**: the arm hardware, the camera, the powder and
the scale. Everything that decides what to do is the cell's own code with the cell's own
config. The BT, MoveIt and scoop_vision cannot tell they are in simulation, apart from
sim time.

```
               REAL CELL                                  TWIN
  Niryo driver (ros2_control HW)           Isaac articulation  + TopicBasedSystem
  RealSense driver (D455)                  Isaac RTX cameras   + depth_to_mm + sim_camera_tf
  Powder                                   heightfield model  OR  PhysX GPU particles
  Scale on /weight                         Isaac RS3 grams     + twin_scale_node
  --------------------------------------------------------------------------------
  Same in both: MoveIt, MTC scooping, move_to, scoop_vision, pour_server,
                incline_control, orchestrator BT, layout manager, RViz
```

What the twin is for:

- **Integration testing without hardware:** run the full webhook-weightment BT, with
  capture, plan, scoop, shake, pour and weigh, on a laptop.
- **Catching config bugs.** The twin found that the Niryo layout had no
  `PourTiltAtWeighingContainer` target, which the BT needs. It also found that
  `ComputeRemaining` treated an overshoot as success.
- **A safe place to try planner changes** before an operator runs them on the cell.
- **A data source for learning:** the gym env (section 12) runs thousands of scoops on the
  same cell model, with no rendering.

What it is **not**: a calibrated powder simulator. The powder parameters are educated
guesses for flour. Trust trends and integration behaviour, not absolute grams, until the
parameters are fitted to real logs (section 18).

### Design choices at a glance

| Question | Choice | Why |
|----------|--------|-----|
| Simulator | Isaac Sim 5.1 standalone (`python.sh`) | RTX depth rendering, GPU PhysX particles, a ROS 2 bridge, and URDF import |
| How ROS controls the arm | `topic_based_ros2_control` `TopicBasedSystem` | The cell's JTC, MoveIt and MTC run unchanged; only the hardware plugin changes |
| Arm physics | Stiff position drives, gravity off on the links | The real Niryo is position-controlled; we need tracking, not dynamics |
| Time | Isaac publishes `/clock`; every node uses `use_sim_time` | Controllers and the BT see consistent time even when Isaac runs slower than real time |
| Isolation | `ROS_DOMAIN_ID=77`, discovery LOCALHOST, enforced by the launch | The twin can never join the real cell's ROS graph |
| Camera | Two RTX render products at the hand-eye calibration pose | scoop_vision gets the same topics, encodings, frames and intrinsics as with a real D455 |
| Scale | A node with lag, noise and resolution on RS3's grams | Pour control laws see a scale, not a perfect integer |
| Powder | Two models behind one interface | Particles for realism in Isaac; the heightfield for speed (gym env) and as a fallback |
| Config | Read the same layout, calibration, robot profile and scoop poses as the cell | Twin and cell cannot drift apart silently |

---

## 2. Background you need

This section is a short primer. Skip the parts you already know.

### 2.1 Omniverse, Kit, USD

- **USD (Universal Scene Description)** is Pixar's scene format. A scene is a **stage**
  holding a tree of **prims** (primitives), addressed by paths like
  `/World/Cell/rs6/mesh`. Prims have typed **attributes** (`points`, `size`, `xformOp`)
  and **relationships** (links to other prims). Behaviour is added with **API schemas**
  applied to a prim. Applying `UsdPhysics.CollisionAPI` makes it a collider, and
  `UsdPhysics.RigidBodyAPI` makes it a rigid body.
- **Instancing:** a prim marked `instanceable` shares one copy of its sub-tree with others.
  Its children are **instance proxies**: they are read-only, and normal child iteration
  does not enter them. This caused one of our hardest bugs (section 16).
- **Kit** is the Omniverse app framework, and Isaac Sim is a Kit app. Python scripts start
  it through `SimulationApp`. It is configured with **carb settings** (key paths like
  `/rtx/post/aa/op`).
- **OmniGraph** is Kit's node-graph system. The ROS 2 bridge is a set of OmniGraph nodes:
  "on every tick, publish joint states", "subscribe to joint commands and drive the
  articulation". We build those graphs in Python with `og.Controller.edit`.

### 2.2 PhysX: articulations, drives, colliders, the GPU

- An **articulation** is a tree of rigid links joined by joints and solved as one unit
  (reduced coordinates). That is how robots are simulated. The URDF importer turns the
  Niryo URDF into an articulation.
- A **joint drive** is a PD controller per joint:
  `torque = stiffness·(q_target − q) + damping·(q̇_target − q̇)`, clamped to `max_force`.
  In USD, **angular drive gains are per degree**, not per radian. We use stiffness 1e5,
  damping 2e3 and max force 1e4, so the joints track commands with small lag.
- **Collision approximations** for meshes:
  - `convexHull`: one convex shape;
  - `convexDecomposition`: many convex pieces, which a concave bowl needs;
  - `sdf`: a signed-distance field;
  - `none`: the raw triangle mesh.

  Particles collide with all of them, but each behaves differently (section 9.4).
- **GPU dynamics:** with `enableGPUDynamics` and a GPU broadphase on the physics scene,
  PhysX solves rigid bodies and particles in CUDA. Particles require it.
- **Substeps:** Isaac renders at `render_hz` (30) and steps physics at `physics_hz`, so
  each rendered frame runs `physics_hz / render_hz` physics steps (120/30 = 4).

### 2.3 Position-based dynamics (PBD) particles

PhysX particles use **position-based dynamics**. Each physics step:

1. **Predict:** `x* = x + v·dt + g·dt²`, which moves every particle as if nothing collides.
2. **Project constraints, repeated `solver_iterations` times:** move positions so that
   - no two particles are closer than twice the **rest offset** (their radius);
   - no particle is inside a collider;
   - friction, cohesion and adhesion constraints hold.
3. **Update velocities from the position change:** `v = (x_new − x)/dt`, then apply damping.

Two consequences matter for us:

- With too few iterations the constraints are not fully met, so a tall pile **compresses**
  under its own weight. PBD is "soft" at low iteration counts. Our bed calibration
  (section 9.3) exists because of this.
- Mass in PBD only weights how corrections are shared between bodies. The grams we report
  come from our own **accounting** (counting grains × grams per grain), not from PhysX mass.

Particle parameters you will see:

| Parameter | Meaning |
|-----------|---------|
| `rest_offset` | Particle "radius" for collisions with shapes |
| `solid_rest_offset` | Radius for particle–particle collisions (granular mode) |
| `contact_offset` | Distance at which a contact starts to be considered (must be > rest offset) |
| `friction`, `particle_friction_scale` | Grain–surface friction, and its scale for grain–grain |
| `cohesion` | Grains stick to each other (powders are cohesive; sand is not) |
| `adhesion` | Grains stick to surfaces |
| `damping` | Velocity damping |
| `max_velocity`, `max_depenetration_velocity` | Speed caps that stop grains from being fired out of overlaps |
| CCD | Continuous collision detection, so fast grains don't tunnel through thin walls |

### 2.4 ROS 2 pieces

- **`use_sim_time` and `/clock`:** every node takes "now" from the `/clock` topic instead of
  the wall clock. Isaac publishes `/clock` each frame. If **two** publishers exist, time
  jumps backwards and forwards, and TF, RViz and controllers reset (section 16).
- **ros2_control:** a `ros2_control_node` loads a **hardware plugin**, which reads states and
  writes commands. **Controllers** run on top of it. The cell uses the
  `JointTrajectoryController` (JTC) that MoveIt sends trajectories to.
  `topic_based_ros2_control/TopicBasedSystem` is a hardware plugin whose "hardware" is two
  topics: it publishes commands on one and reads states from the other. That is exactly
  what Isaac's OmniGraph bridge speaks.
- **DDS discovery:** nodes find each other on a `ROS_DOMAIN_ID`. With
  `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`, discovery stays on this machine.

---

## 3. System architecture

Two processes, two Python interpreters, one ROS graph on domain 77.

```
┌────────────────────────── Isaac process ───────────────────────────┐
│ Isaac python 3.11 + Isaac's bundled ROS 2 Jazzy (rclpy)             │
│ build_cell.py                                                       │
│   stage: table, RS6, RS3, floor, lights (from config/layouts)       │
│   Niryo articulation (URDF import, stiff drives)                    │
│   OmniGraph joint bridge ─ /isaac_joint_states (pub)                │
│                          ─ /isaac_joint_commands (sub)              │
│                          ─ /clock (pub)                             │
│   D455: 2 RTX render products ─ /isaac/camera/depth/* (32FC1, m)    │
│                               ─ /camera/color/* (rgb8)              │
│   powder model ─ /isaac_twin/{rs3_mass_g,payload_g,bed_mass_g}      │
│                ─ /vibration/intensity (sub) ─ /isaac_twin/reset_powder│
└──────────────────────────────────────────────────────────────────────┘
                  ▲                         │
                  │ DDS, domain 77, localhost│
                  │                         ▼
┌────────────────── Native ROS 2 Jazzy (python 3.12) ─────────────────┐
│ scooping_isaac.launch.py                                            │
│  robot_state_publisher, ros2_control_node (TopicBasedSystem), JTC   │
│  move_group, scooping_mtc_node, move_to_server, target_recorder     │
│  task frame, markers, container markers, collisions, layout manager │
│  sim_camera_tf ─ static D455 frames under camera_link               │
│  depth_to_mm   ─ /camera/depth/image_rect_raw (16UC1 mm, noisy)     │
│  scoop_vision (+ hand-eye camera_link publisher)                    │
│  twin_scale    ─ /weight                                            │
│  incline_control_node, pour_server_node, orchestrator (BT)          │
│  draft_targets_node, RViz                                           │
└──────────────────────────────────────────────────────────────────────┘
```

**Why two interpreters?** Isaac ships its own Python 3.11 and a ROS 2 Jazzy built for it.
The system ROS is built for Python 3.12. Mixing them crashes on import. `run_isaac_twin.sh`
strips the native ROS environment (`PYTHONPATH`, `AMENT_PREFIX_PATH`...) and points
`LD_LIBRARY_PATH` at Isaac's bridge libraries before starting `python.sh`. The two sides
talk only through DDS, so they never share Python packages.

**Why the Isaac side imports `scoop_vision`:** it reuses pure-numpy modules
(`ContainerModel`, `ScoopTool`, mesh loading) so both sides use identical container and
scoop geometry. `cell.py` is deliberately numpy + yaml only, so it imports under Isaac's
Python.

### Topic map

| Topic / service | Type | From → to | Notes |
|-----------------|------|-----------|-------|
| `/isaac_joint_states` | JointState | Isaac → TopicBasedSystem | 30 Hz (one per rendered frame) |
| `/isaac_joint_commands` | JointState | TopicBasedSystem → Isaac | Position targets |
| `/clock` | Clock | Isaac → all | Sim time, held at or below wall time |
| `/isaac/camera/depth/image_raw` | Image 32FC1 | Isaac → depth_to_mm | Metres, 848×480, 15 Hz |
| `/camera/depth/image_rect_raw` | Image 16UC1 | depth_to_mm → scoop_vision | mm + noise + holes, like realsense-ros |
| `/camera/color/image_raw` | Image rgb8 | Isaac → RViz/vision | 1280×720, 15 Hz |
| `/isaac_twin/rs3_mass_g` | Float64 | Isaac → twin_scale | Grams in RS3, 10 Hz |
| `/isaac_twin/payload_g`, `/isaac_twin/bed_mass_g` | Float64 | Isaac → you | Diagnostics |
| `/weight` | Float64 | twin_scale → pour_server, BT | What the real scale publishes |
| `/vibration/intensity` | Float64 0–1 | MTC, pour_server → Isaac | Drives the vibration model |
| `/isaac_twin/reset_powder` | Trigger | you → Isaac | Refill the bed |
| `/twin_scale/tare` | Trigger | BT/you → twin_scale | Zero the display |

### Start order

1. `run_isaac_twin.sh` builds the scene and starts publishing joint states and `/clock`.
2. `ros2 launch isaac_twin scooping_isaac.launch.py`:
   - refuses to run unless domain 77 + LOCALHOST (`_check_isolation`);
   - reads **one** `/isaac_joint_states` message and passes the positions as xacro args
     `initial_joint_N` (section 6.2 explains why);
   - starts nodes in stages with `TimerAction`, like the real launch: controllers, then
     move_group at 2 s, scene publishers, move_to at 5 s, MTC at 5.5 s, vision at 6 s,
     then pour nodes and the BT.

---

## 4. Frames and geometry

All geometry comes from the files the cell uses:

| Source | Used for |
|--------|----------|
| `config/layouts/<layout>.yaml` | Table, RS6, RS3 poses, meshes, scales, colours (`cell.layout_objects`, same rules as the C++ scene loader) |
| `config/layouts/<layout>/poses.yaml` | The 5 authored scoop poses in `scooping_container_frame` |
| `src/scooping_controller/config/robots.yaml` (`niryo.tool`) | Scoop STL and the TCP offset |
| `src/camera_robot_calibration/calibrations/niryo_d455_eob.calib` | `base_link → camera_color_optical_frame` (eye-on-base hand-eye result) |
| `niryo_robot_description` xacro | Arm links, joints, limits |

Key frames:

```
world = base_link (Niryo base, z up)
 ├─ rs6 (layout pose)  ─ scooping_container_frame (task frame publisher)
 ├─ rs3 (layout pose)
 ├─ tcp_link (scoop TCP; FK from the URDF)
 └─ camera_link (computed from the calib)
      ├─ camera_depth_frame ─ camera_depth_optical_frame
      └─ camera_color_frame (y −59 mm) ─ camera_color_optical_frame
```

**The camera chain, step by step.** The calibration gives
`T_base_colorOptical`. realsense-ros publishes the frames under `camera_link`:

- the colour frame sits 59 mm along −y from `camera_link` (`D455_COLOR_OFFSET_M`);
- each optical frame is the body frame turned so that z points forward and y down, using
  the quaternion `(-0.5, 0.5, -0.5, 0.5)`.

So:

```
T_link_colorOptical = T(0,-0.059,0) · R_optical
T_base_link         = T_base_colorOptical · inverse(T_link_colorOptical)      # cell.camera_link_in_base
T_base_depthOptical = T_base_link · R_optical                                 # depth sits at camera_link
```

On the ROS side, `sim_camera_tf` publishes exactly those static frames under
`camera_link`. The cell's hand-eye publisher then composes `base_link → camera_link` the
same way as on the real cell. Isaac places its cameras at the same poses, so a pixel in the
twin's depth image back-projects to the same `base_link` point as on the real cell.

**USD cameras look down −z with y up**, whereas ROS optical frames look down +z with y down.
`d455.py` converts with `diag(1, −1, −1)`. The pinhole intrinsics map to USD's physical
camera model as:

```
focal_length_mm = fx · horizontal_aperture_mm / width_px      (aperture fixed at 20.955 mm)
vertical_aperture_mm = horizontal_aperture_mm · height / width
```

Example: depth `fx = 428` at 848 px gives a focal length of 10.58 mm.

---

## 5. Building the Isaac scene

### 5.1 The launcher script (`scripts/run_isaac_twin.sh`)

1. **Expand the URDF** with the system ROS (`xacro` is not in Isaac's Python) into
   `~/.cache/isaac_twin/niryo_ned3pro.urdf`.
2. **Sanitise mesh names.** The URDF importer names prims after mesh file names, and a name
   like `niryo_scoop_v4-ros.STL` is not a valid USD path (no `-`). The script symlinks each
   such mesh under a clean name and rewrites the URDF.
3. `source twin_env.sh`: domain 77, LOCALHOST, Fast DDS, and **unset Fast DDS profile
   files** (the laptop had one that pinned DDS to an ethernet interface and disabled the
   builtin transports, so Isaac and ROS never discovered each other).
4. `exec` Isaac's `python.sh build_cell.py` with a clean environment.

### 5.2 `build_cell.py`, top to bottom

1. **Start Kit:** `SimulationApp({"headless": ..., "width": 1280, "height": 720})`. All
   `omni.*`/`pxr` imports must come **after** this line.
2. **Turn off DLSS:** `carb.settings ... set("/rtx/post/aa/op", 0)`. DLSS renders at half
   resolution and upscales. Our 848×480 depth was really 424×240.
3. **Enable extensions:** `isaacsim.ros2.bridge` (also makes Isaac's rclpy importable) and
   `isaacsim.asset.importer.urdf`.
4. **`SimulationContext(physics_dt=1/physics_hz, rendering_dt=1/render_hz)`**, with stage
   up-axis z and units in metres.
5. **Layout** (`build_layout`): for each enabled layout object, an Xform with its pose, plus
   the STL as a `UsdGeom.Mesh`, or a cube for box objects. Add a room floor 0.75 m below the
   base, so the depth image has a background. In particle mode, also add static colliders.
6. **Robot** (`import_robot`):
   - `URDFParseAndImportFile` with a fixed base, no merged fixed joints, no self-collision,
     and imported inertia;
   - set the stiff angular drives on every revolute joint.
7. **Particle-only physics setup** (section 9): GPU dynamics, link gravity off, the robot
   filtered from cell colliders, and the scoop collider swap.
8. **D455** (section 7.1) and the **joint bridge** (section 6.1).
9. **Powder:** `ContainerModel` for RS6 and RS3 from the same STLs, then `ParticlePowder` or
   `PowderCell`.
10. **ROS node in Isaac** (`PowderRos`): gram publishers, the vibration subscriber, the reset
    service.
11. **The main loop:**

```python
while APP.is_running():
    sim.step(render=True)                  # physics substeps + render + OmniGraph (ROS pubs/subs)
    sim_t += rendering_dt
    q = robot.get_joint_positions()
    powder.step(rendering_dt, chain.tip_pose(q), ros.vibration)   # our powder logic
    publish grams at 10 Hz; status line every 10 s sim
    rclpy.spin_once(ros.node, timeout_sec=0)
    # pacing: never let sim time run ahead of wall time
    ahead = sim_t - (wall_now - wall_start)
    if ahead > 0: sleep(ahead)
    elif ahead < -0.5: wall_start = wall_now - sim_t   # drop the debt after a slow frame
```

**Why hold sim time to wall time?** ros2_control, the BT's timeouts and the pour controller
were all tuned at real rates. If Isaac ran faster than real time, the stack would be
starved. When Isaac is slower (particles: RTF about 0.5), sim time simply runs slower.
Everything uses `/clock`, so it all slows down together and stays consistent.

**Why drop the debt?** The first frame can take over a minute (shader compilation). Without
the reset, the loop would then run flat out to "catch up" and blast the stack with time.

**TCP pose without asking PhysX:** `kinematics.Chain` does FK from the same URDF:
`chain.tip_pose(q)` gives `base_link → tcp_link`. It is cheap, exact, and identical to what
MoveIt uses.

---

## 6. Controlling the simulated arm

### 6.1 The joint bridge (OmniGraph)

`scene/joint_bridge.py` builds one graph that runs on every playback tick:

```
OnPlaybackTick ──► ROS2PublishJointState   (articulation → /isaac_joint_states, stamped with sim time)
               ──► ROS2SubscribeJointState (/isaac_joint_commands) ──► IsaacArticulationController
               ──► ROS2PublishClock        (sim time → /clock)
```

The articulation controller writes the commanded positions as drive targets, and the stiff
drives pull the joints there.

### 6.2 ros2_control side

`urdf/niryo_ned3pro_isaac.urdf.xacro` includes the normal Niryo xacro and adds:

```xml
<ros2_control name="IsaacSystem" type="system">
  <hardware>
    <plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
    <param name="joint_commands_topic">/isaac_joint_commands</param>
    <param name="joint_states_topic">/isaac_joint_states</param>
  </hardware>
  <joint name="joint_1"> position command; position (initial_value) + velocity state </joint>
  ...
</ros2_control>
```

Then the usual `ros2_control_node`, `joint_state_broadcaster` and
`niryo_robot_follow_joint_trajectory_controller` (JTC, 100 Hz) come up with the same name
the cell uses, so MoveIt's controller config does not change.

**The initial-pose seed.** TopicBasedSystem keeps a command buffer, seeded from each
joint's `initial_value`. It **publishes that buffer even before any controller is active**.
With `initial_value = 0`, every relaunch drove the Isaac arm to the zero pose, which is a
hazard on real hardware and confusing in the twin. The launch therefore reads Isaac's
current joints first and passes them in:

```python
for joint, position in _isaac_joint_positions("/isaac_joint_states", 30.0).items():
    xacro_cmd.extend([" ", f"initial_{joint}:={position:.6f}"])
```

**Tolerances.** Isaac's drives lag the command slightly, as the real driver does. JTC uses
`stopped_velocity_tolerance: 0.05` and `goal_time: 1.0`, and move_group uses
`allowed_start_tolerance: 0.05`.

**Plugin build.** NVIDIA's apt build of `topic_based_ros2_control` was compiled against an
older `hardware_interface` and segfaulted `ros2_control_node`. Build it from source; it is
pinned in `src/ros2.repos`.

### 6.3 Why position drives and no gravity on the links

The Niryo is position-controlled with its own servo loops. We care that the TCP follows
MoveIt's trajectory, not about motor torques. Gravity on the links only adds steady-state
error to a PD drive. In heightfield mode the whole scene has gravity off. In particle mode
the scene needs gravity for the grains, so each robot link gets
`PhysxRigidBodyAPI.disableGravity = True` (`disable_link_gravity`).

---

## 7. Simulated sensors: D455 and scale

### 7.1 D455 (`scene/d455.py`, `depth_to_mm_node.py`, `sim_camera_tf.py`)

Isaac renders two cameras, one per stream, each a **render product** (an offscreen RTX
render target) feeding `ROS2CameraHelper` and `ROS2CameraInfoHelper` OmniGraph nodes:

| Stream | Resolution | fx | Rate | Topic | Frame |
|--------|-----------|----|------|-------|-------|
| depth | 848×480 | 428 | 15 Hz | `/isaac/camera/depth/image_raw` (32FC1 m) | `camera_depth_optical_frame` |
| colour | 1280×720 | 640 | 15 Hz | `/camera/color/image_raw` | `camera_color_optical_frame` |

Rates: `frameSkipCount = render_hz/rate − 1 = 1`, so every second frame is **published**.
Note that the render product still **renders every frame**; the skip only affects
publishing.

**Why depth goes through `depth_to_mm_node`:** realsense-ros publishes 16-bit millimetres
(`16UC1`), and scoop_vision reads exactly that with `depth_scale = 0.001`. Rather than
change scoop_vision, the twin converts and adds realistic imperfections:

```
valid = finite(z) and 0.4 m ≤ z ≤ 6 m
z_noisy = z + N(0,1) · (σ0 + σ2·z²)        σ0 = 0.8 mm, σ2 = 0.002 /m
1% of the valid pixels become holes (0)
mm = round(z_noisy · 1000)  as uint16
```

At the bed (z ≈ 0.7 m), σ ≈ 0.8 + 0.002·0.49·1000 ≈ 1.8 mm. Stereo depth error grows
roughly with z², and the noise and holes exercise scoop_vision's multi-frame median and
its `min_measured_fraction` check. The node also publishes a decimated point cloud at
2 Hz for RViz.

The twin overrides one scoop_vision parameter, `planner.surface_correction_m: 0`. The real
cell has a temporary +3 cm workaround for a hand-eye error. The twin camera sits exactly
at the calibration, so the workaround would only make sim scoops too shallow.

### 7.2 Scale (`twin_scale_node.py`)

Isaac knows the exact grams in RS3. A real scale is slow, noisy and quantised, and pour
control laws are sensitive to all three. The node models a first-order lag:

```
shown += (m − shown) · (1 − exp(−dt/τ))        τ = 0.4 s   (exact discretisation of ẋ = (m−x)/τ)
value  = shown − tare + N(0, 0.05 g)
value  = round(value / 0.01 g) · 0.01 g
```

It publishes on `/weight` at 20 Hz, and `~/tare` zeroes it. Using `1 − exp(−dt/τ)`
instead of `dt/τ` keeps it stable for any `dt`, which matters because sim-time ticks are
irregular when Isaac is slow.

---

## 8. Powder model A: heightfield

File: `isaac_twin/powder.py`. It is fast (RTF 1.0), deterministic and pure numpy. The gym
env always uses it.

### 8.1 The bed as a 2.5D surface

The RS6 interior is a grid of 5 mm cells: `ContainerModel`, the same grid scoop_vision
plans on. The bed is one height per cell:

```
surface[i,j] = max(floor[i,j], floor_z + fill_depth)        (level fill after reset)
volume = Σ (surface − floor) · cell²
grams  = volume · 10⁶ · density            (density 0.55 g/ml for flour, bulk)
```

**Carving.** The scoop mesh is reduced to a cloud of points (deduplicated to 3 mm) in the
TCP frame. Each frame, they are transformed into the RS6 frame with FK. For each cell, the
lowest scoop point there is found, and if it is below the surface, the surface drops to it:

```
lowest[c] = min z of scoop points in cell c
cut       = lowest < surface
removed   = Σ_cut (surface − max(floor, lowest)) · cell²
```

This is the scoop's **lower envelope** sweeping through the powder. The carved volume goes
into the scoop (`capture_ratio` 1.0; anything else falls back evenly onto the bed).

### 8.2 How much the scoop holds: level fill and the angle of repose

A liquid in a tilted bowl fills up to the lowest point of the rim: the **level-fill
capacity** for that tilt. `ScoopTool.capacity_m3(orientation)` (scoop_vision) computes it
by voxelising the bowl and flooding it.

Powder is not a liquid. Its free surface can slope up to the **angle of repose** (about 35°
for flour) before it avalanches. We model that as follows. The held powder behaves as if
gravity were tilted toward the bowl's **best-holding direction**, by up to the repose
angle:

```
g_tcp      = R_tcpᵀ · (0, 0, −1)                         gravity in the scoop frame
hold       = argmax over gravity of capacity(g)          (found once: about 25° back, 109 ml)
g_held     = g_tcp rotated toward hold by ≤ repose_deg   (rotate_toward)
holds      = capacity(g_held) · heap_factor              (heap_factor 1.3: a heap above the rim)
```

When the scoop is level, the powder can heap. As it tilts, the held amount drops smoothly.
Past the repose angle it drops as fast as a liquid's would.

**Capacities are expensive** (about 0.3 s each). They are memoised per quantised gravity
vector (0.03 per component, about 1.7°) in memory and on disk
(`~/.cache/isaac_twin/scoop_capacity_<mesh hash>.json`). In Isaac they are computed on a
background thread (`CapacityWorker`) and **prewarmed** along the authored scoop path, so
the render loop never blocks. Until a value is ready, the payload is simply held.

### 8.3 Spilling at rest, while vibrating, and pouring

Each frame, while the scoop is out of the bed (`touching` is false):

- **At rest:** `excess = payload − holds`. It slides off with a time constant:
  `spill = excess · min(1, dt/τ_spill)`, with τ = 0.3 s.
- **Vibrating at intensity v:** the repose angle drops to `repose_vibrating_deg` (10°, so
  vibration fluidises powder) with no heap. Powder above that flows out at a feed rate:
  `flow = v · 4 g/s`.
- **Pouring:** if the scoop is tilted at least `min_pour_tilt_deg` (10°) **and** it holds
  less there than level (`capacity(g_tcp) < capacity(upright)`, so it is tipped toward its
  lip), the flow continues **without** the repose floor, until the scoop is empty. That is
  how `pour_server`'s bang-bang vibration doses into RS3.

**Where spilled powder lands:** at the scoop's lowest point (the lip), projected down:

- over the RS3 interior → `rs3_g` (the scale sees it);
- else over the RS6 interior → back onto the bed;
- else → `table_g`.

**What it cannot do:** grains do not flow on the bed (craters stay), there is no pile
inside the scoop, and spills land instantly. That is good enough for integration and fast
enough for learning.

---

## 9. Powder model B: PhysX particles

Files:

- `isaac_twin/scene/particles.py`: PhysX setup, needs Isaac;
- `isaac_twin/particle_powder.py`: seeding and accounting, pure numpy, unit-tested.

It is selected by `powder.model: particles` (the default) or `--powder particles`. The
grains are red so they are easy to see.

### 9.1 Why particles

The heightfield cannot represent powder moving: avalanches inside the scoop, grains
sliding off the lip during the exit, the bed flowing into a crater. Those decide how many
grams one scoop really keeps. PBD granular particles get them for free, at a GPU cost.

### 9.2 Grain size and the particle system

- `spacing_m = 0.004`. Each grain stands for a 4 mm cube of bulk powder. Real flour grains
  are about 0.1 mm, so a grain here is a **parcel** of powder. The user chose to keep 4 mm
  as the best match to the real powder, so it is fixed (smaller grains are much slower;
  larger ones change the scoop behaviour).
- Radius: `rest_offset = rest_ratio · s = 0.45 · 4 mm = 1.8 mm`, which is **less than s/2**.
  Seeded on an s-lattice, neighbours start 0.4 mm apart, not overlapping. Overlapping seeds
  are pushed apart violently and fire grains through the floor.
- `contact_offset = rest + 0.1·s`, `solid_rest_offset = rest`,
  `fluid_rest_offset = 0.6·rest` (unused, since the grains are not a fluid).
- Material: friction 0.6, `particle_friction_scale` 1.0, cohesion 0.02, adhesion 0,
  damping 0.1.
- Solver: 48 position iterations, CCD on, `max_velocity` 1 m/s, `max_depenetration` 0.2 m/s.
- Physics at 120 Hz (4 substeps per frame). At 60 Hz the bed compressed to 34 mm and the
  scoop kept only 13 g, with no speed gain.
- PhysX mass per grain is `s³ · 550 kg/m³`. It only matters for PBD correction weighting;
  see section 9.5 for the grams we report.

### 9.3 Seeding and bed calibration (the settle ratio)

`seed_bed(container, depth, s, jitter)`:

1. Lay an x–y lattice at spacing s over the interior. Keep a point only if it and its four
   half-spacing neighbours are inside, so grains start half a grain off the walls.
2. For each column, stack grains from `floor + s/2` up to `level − s/2`.
3. Add uniform jitter of ±0.04·s. A perfect crystal stacks unrealistically and avalanches
   oddly.

**The compression problem.** Even at 48 iterations, PBD packs a tall pile down. Seeded to
40 mm, the bed settled to about 25 mm at the first iteration count. The heightfield model,
the camera and the grams would then all disagree.

**The fix: seed taller, and give each grain less mass.** Define
`settle_ratio = settled height / seeded height`, measured as 0.66 at 48 iterations.

```
seeded depth   H_seed = fill_depth / settle_ratio        = 40 mm / 0.66 = 61 mm
grain grams    m      = s³ · settle_ratio · ρ            = (0.4 cm)³ · 0.66 · 0.55 g/ml = 23.2 mg
total grams    N · m  = (N·s³) · settle · ρ  =  V_seed · settle · ρ  =  V_settled · ρ
```

So after settling, the bed is 40 mm deep **and** weighs what 40 mm of powder weighs at bulk
density: 109,604 grains × 23.2 mg = 2546 g, against 2555 g for the heightfield. The status
line prints the measured depth. `bed_depth_m` takes the top grain centre in each 1 cm
column, takes the median, and the twin adds the grain radius. The camera agrees: scoop_vision
measured 4602 ml, against 4629 ml in the bed.

> If you change `spacing_m` or `solver_iterations`, re-measure `settle_ratio`. Run headless
> for about 20 s and read `depth=` in the status line.

### 9.4 Colliders: the part that took the longest

Grains must collide with the vessels, table and scoop. The robot must **not** collide with
the vessels: MTC deliberately lets the scoop touch RS6, and stiff position drives would
fight the contact.

| Collider | Choice | What went wrong with the alternatives |
|----------|--------|----------------------------------------|
| RS6 / RS3 / table | `convexDecomposition` (64 hulls, 32 verts, shrink-wrap) | Triangle mesh (`none`): grains leaked through the floor/ramp crease. `sdf`: also leaked |
| Robot vs cell | `FilteredPairsAPI` on every cell collider, listing every robot link | A `CollisionGroup` for the robot made the arm drift off its targets |
| Scoop | 5460 cubes of 3 mm on the bowl surface (`voxels`) | The importer's box filled the bowl (nothing could get in). `sdf`: leaked ~1 g/s while held. `convexDecomposition`: leaked more |

**The scoop collider in detail** (`scoop_collider`):

1. Find the `tool_link` rigid body under the robot.
2. **De-instance:** the importer puts colliders under instanceable prims. Walk
   `Usd.PrimRange(link, Usd.TraverseInstanceProxies())`; for any collider that is an
   instance proxy, find its instance ancestor and `SetInstanceable(False)`. Repeat until
   none remain.
3. Disable every old collider (`collisionEnabled = False`).
4. Voxelise the scoop STL's surface (`shell_voxels`): sample points every voxel/3 and keep
   the unique 3 mm cells. Add one cube per cell (×1.05 so neighbours overlap) as children
   of `tool_link`. They move with the link, and PhysX treats them as one compound shape.

Why cubes and not the mesh? The STL has non-manifold edges and thin walls. Convex pieces
bridge the bowl and SDF has finite resolution, but a shell of small cubes is watertight
for 4 mm grains.

### 9.5 Accounting: from grain positions to grams

At 10 Hz (`readback_hz`), and on every frame while vibrating, the twin reads all positions
from USD and labels each grain (`ParticleAccounting.classify`, later labels win):

1. default **TABLE** (spilled);
2. **BED** if inside RS6: x–y over the interior or rim cells, and z between floor − 1 cm and
   rim + 3 cm (a heap still counts);
3. **RS3** with the same rule for RS3;
4. **SCOOP** if inside the scoop's TCP-frame bounding box + 4 mm. Each grain is transformed
   into the TCP frame with the FK pose, so the box moves with the scoop.

Grams = count per label × 23.2 mg. That gives the same four numbers as the heightfield
model (`bed_g`, `payload_g`, `rs3_g`, `table_g`), so `build_cell.py`, the ROS topics and the
scale don't care which model runs. Non-finite positions are logged and treated as table.

### 9.6 Rendering

The grains are a `UsdGeom.Points` prim with a width per grain, bound to a red material. RTX
draws them as spheres, and the D455 render product sees them, which is how scoop_vision
"sees" the particle bed.

---

## 10. How vibration is modelled

On the cell, a vibration motor on the scoop is driven by `/vibration/intensity` (0–1). Two
publishers:

- `scooping_mtc_node`, after the lift: 5 s at 0.75, then 1.5 s settle. This shakes off the
  loose heap so the scoop carries a repeatable amount.
- `pour_server`: bang-bang on `/weight` while dosing into RS3.

Isaac subscribes to the same topic. Neither model shakes the scoop itself.

- **Heightfield:** vibration changes the rules (section 8.3). The repose angle drops from
  35° to 10°, the heap goes away, and excess flows at `intensity · 4 g/s`. It pours over the
  lip without limit when tipped toward it.
- **Particles:** each frame while vibrating, every grain labelled SCOOP gets a random
  velocity kick:

```python
vel[held] += rng.normal(0.0, vibration_kick_m_s * intensity, size=(n_held, 3))   # σ = 0.15·v m/s
```

PhysX then resolves the kicked grains against the scoop walls and each other. Grains near
the lip get knocked over it, and the pile loses its heap. It is like stirring the grains
rather than shaking the bowl.

**Why this and not a moving collider?** A physical model would oscillate the scoop collider
at the motor's frequency and amplitude, for example 100 Hz at 0.2 mm. That needs:

- physics at least 10× the vibration frequency (≥ 1 kHz, 8× the current cost);
- the real motor's frequency and amplitude, which we don't have yet.

The kick gives the right qualitative effect at no extra cost. `vibration_kick_m_s` should
be fitted against the cell's measured "grams kept after the post-lift shake" before
trusting it.

---

## 11. Running the real behaviour tree on the twin

`webhook_weightment.xml`, the cell's BT, runs unchanged. The flow is:

1. MoveTo `CameraClear` → `/scoop_vision/capture` (multi-frame depth → surface map) →
   `/scoop_vision/plan` (shift the authored scoop to where the powder is).
2. MTC scoop: approach, drag, lift. Then the post-lift vibration.
3. MoveTo `PourStartAtWeighingContainer` → `PourTiltAtWeighingContainer`.
4. `pour_server`: tilts with `incline_control` (joint_5), vibrates bang-bang on `/weight`
   until it reaches target − tolerance.
5. `ComputeRemaining`: rescoop if more is needed, otherwise recover and finish.

Two twin-only pieces:

- **The `PourTiltAtWeighingContainer` draft.** The Niryo layout does not define it, so on
  the real cell the BT would stop there. The twin adds a **draft** in
  `config/draft_targets/dual-container_niryo.yaml`: PourStart's TCP pitched a further 10°
  about tool Y, 15° in total. joint_5 is then at −1.86 rad, near its −1.92 limit. The
  Jaka's 32° is out of reach.
  - `draft_targets_node` merges it over the layout's targets into
    `~/.cache/isaac_twin/targets_<layout>_draft.yaml`, and keeps
    `move_to_server.targets_yaml` pointing there. It re-asserts every 2 s, because the
    layout manager and move_to's respawn reset that parameter.
  - **`config/layouts` is never touched.** The draft must be checked by an operator on the
    real cell before it moves there.
- **A separate pose cache.** The marker server runs with `poses_env: isaac` and
  `authored_in: isaac`, so it uses `poses_isaac_<layout>.yaml` and never the real, bench or
  Gazebo caches.

Start it (sim only, domain 77):

```bash
ros2 service call /twin_scale/tare std_srvs/srv/Trigger
ros2 service call /bt_start_webhook_weightment robot_common_msgs/srv/StartWebhookWeightment \
  "{run_id: twin-1, target_weight_g: 20.0, tolerance_g: 1.0}"
```

---

## 12. The gym environment

`isaac_twin/gym/scoop_env.py`, registered as `IsaacTwin/Scoop-v0`. It is the same cell
model (layout, RS6 mesh, scoop mesh, authored poses, heightfield powder, scoop_vision's
planner limits) with no Isaac, no ROS and no rendering, so it runs thousands of scoops.

The problem as an RL formulation:

| | Definition |
|-|------------|
| **Action** | `(dx, dy, dz)` added to all 5 authored poses (scoop_vision's `pattern_offset`), bounded by `ScoopPlanner.shift_window()` and `dz_min/max` |
| **Observation** | `height`: powder depth per 5 mm cell (0 outside); `joints`: IK at the contact pose |
| **Transition** | IK each shifted pose (seeded from the authored IK), interpolate in joint space like MTC, step the powder along the path at a TCP speed of 0.2 m/s, run the post-lift shake (5 s at 0.75, 1.5 s settle), then empty the scoop into "RS3" |
| **Reward** | `−|scooped_g − target_g| / target_g` |
| **Constraint** | Wall/floor clearance (`ScoopPlanner.clearance_ok`) or no IK → not executed, penalty `1 + 0.1·mm_short` |
| **Episode** | Ends when the bed is nearly empty; truncated at `max_scoops` |

`env.heuristic_plan()` runs scoop_vision's real planner on the ground-truth surface: the
baseline to beat. `ros2 run isaac_twin scoop_env_compare` compares heuristic, authored and
random shifts, and fits the planner's `fill_efficiency`. Result: the heuristic gets
52 ± 14 g against a 48 g target, and the fitted `fill_efficiency` is about 0.79 (the
planner assumes 0.5). Retention is not proportional to the engaged volume, so a single
`fill_efficiency` cannot model it. A learned policy could.

**Why 0.2 m/s matters:** how much spills on the lip-down exit depends on exit speed. That
value came from replaying the MTC trajectory the twin recorded (0.12–0.28 m/s).

**Numeric IK** (`kinematics.py`): damped least squares on the URDF chain,
`Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ e`, with λ = 0.02, clamped to the joint limits, from one seed. It
is fast and dependency-free. A miss does not prove the pose is unreachable: MoveIt's
`/compute_ik` uses random restarts.

---

## 13. Validation results

| Check | Heightfield | Particles |
|-------|-------------|-----------|
| Bed at 40 mm | 2555 g | 2546 g (109,604 × 23.2 mg), depth 40 mm |
| Camera capture vs bed volume | n/a | 4602 ml vs 4629 ml |
| Authored scoop, after lift + shake | 64 g | 23 g (77 g held while in the bed) |
| 20 g webhook BT | SUCCESS, 21.0 g dosed | SUCCESS, one scoop of 21 g, 19.9 g dosed |
| Mass balance (bed + scoop + RS3 + table) | exact | closes to 0.1 g |
| Real-time factor | 1.0 | 0.5 headless, about 0.4 with GUI + RViz |

**Why do the two models disagree on retention (64 vs 23 g)?** The authored scoop leaves
the bed about 30° lip-down. In the particle model the grains slide out over the lip
during the exit, even though the bowl would hold about 50 g level at the transport pose.
The heightfield model only spills above its repose-tilted capacity. Changing cohesion or
friction did not change the particle result. Only the real cell can say which is closer.
That measurement, grams kept after the shake for the authored scoop, is the most useful
next step.

---

## 14. Performance: where the time goes

`FPS = render_hz × RTF`. The status line prints `rtf`, `fps`, and the mean milliseconds
spent in `sim.step` and in the powder logic.

For one headless particle frame of about 60 ms (Kit CPU profiler, Chrome trace):

| Part | Time | Notes |
|------|------|-------|
| PhysX particle solve | ~50 ms | 4 substeps × (neighbour search + 48 iterations + contacts) for 110k grains. Kernels run one after another, so the GPU is never full |
| RTX render | ~9 ms | 110k points + 2 camera products |
| Readback and accounting | 3 ms (9 ms while vibrating) | USD read of 110k positions, numpy classify |

What we learned:

- **It is latency-bound, not throughput-bound.** The GPU drew about 100 W at 2.6 GHz and
  66 °C, 65–70% busy. Raising the power cap from 80 to 175 W (Dynamic Boost) gave only
  15 → 16 FPS. VRAM use is about 1.5 GB of 16. A bigger GPU would not help much.
- **What does not help:**
  - more or fewer solver iterations (32 → 48 cost nothing);
  - `/physics/updateParticlesToUsd=false` (positions still update);
  - hiding the grains;
  - removing the cameras.
- **What does help, at a price:**
  - an `sdf` scoop collider (−11 ms, but it leaks);
  - 5 mm grains (0.75 RTF, but a different powder);
  - 60 Hz physics (worse bed and retention).

  The user chose accuracy: 4 mm grains, voxel scoop, 120 Hz.
- **Benchmark trap:** `sim.step(render=False)` advances **one** physics step, not a full
  frame, so "no-render" timings look 4× faster than they are.

**Dynamic Boost on the laptop host** (not in the repo). `nvidia-powerd` was installed from
the driver's docs:

- `/etc/systemd/system/nvidia-powerd.service`;
- a D-Bus policy `/etc/dbus-1/system.d/nvidia-dbus.conf` that lets root own
  `nvidia.powerd.server`;
- `systemctl enable --now nvidia-powerd`.

It is safe: the firmware still enforces the thermal and power limits. Revert with
`sudo systemctl disable --now nvidia-powerd`. Kit's "CPU powersave" warning is a false
alarm: `intel_pstate`'s `powersave` governor is dynamic, and EPP is `performance`.

---

## 15. Debugging and tuning methods

These methods turned guesses into measurements. Reuse them.

1. **Replay without ROS** (`--replay-scoop T`). After T seconds, the twin plays the
   authored scoop itself: chained IK of the 5 poses, a timed joint path at the MTC-like
   speed, the post-lift shake inserted before the transport segment (`timed_scoop`,
   `sample_knots`). It logs tracking error, TCP z and grams every second. It takes 20 s,
   needs no stack, and is repeatable. Use it to tune colliders and parameters.
2. **Particle dumps** (`--dump-particles /tmp/p.npz`): positions plus the TCP pose at exit.
   Load them in numpy, transform to the TCP frame, and histogram the grains against the
   scoop's bounding box. That showed "no grain ever enters the bowl", which led to the
   hidden box collider.
3. **Settle check:** run headless for 20 s and read `depth=` and `bed=`. Do this after any
   particle parameter change.
4. **Status line arithmetic:** if `fps × step_ms ≈ 1000`, the frame is spent in
   `sim.step` (PhysX + render). Powder logic shows separately.
5. **Profiling:** the Kit CPU profiler writes a Chrome trace (flags in
   `src/isaac_twin/CLAUDE.md`). Aggregate the main thread's events by name and look for
   the biggest totals.
6. **ROS-side checks:**
   - `ros2 topic list --no-daemon --spin-time 12`: discovery with about 60 nodes is slow;
   - `ros2 topic echo /isaac_twin/rs3_mass_g`;
   - `~/.ros/log/latest/launch.log`;
   - Isaac's log in `kit/logs/Kit/Isaac-Sim Python/5.1/`.
7. **Check for a second Isaac** (`ps -eo pid,args | rg kit/python`) whenever RViz says
   "jump back in time".

**Tuning by symptom:**

| Symptom | Likely cause | Setting |
|---------|-------------|---------|
| Bed settles below `fill_depth_m` | PBD compression | `settle_ratio` (re-measure), `solver_iterations` |
| Grains explode at start | Seed overlap | `rest_ratio` < 0.5, `max_depenetration_m_s`, CCD |
| Grains leak through RS6 | Triangle collider crease | `vessel_collider: convexDecomposition` |
| Scoop never fills | A collider fills the bowl | Check that the de-instancing/disable log line says `N imported colliders off` |
| Held scoop slowly empties | Leaky scoop collider | `scoop_collider: voxels`, smaller `scoop_voxel_m` |
| Shake removes too much / too little | Kick strength | `vibration_kick_m_s` (fit to cell data) |
| Heightfield keeps too much | Heap or repose | `heap_factor`, `repose_deg` |
| Pour stalls | Tilt below `min_pour_tilt_deg`, or flow too low | `min_pour_tilt_deg`, `vibration_flow_g_per_s`, PourTilt |

---

## 16. Problems we hit and what fixed them

| Problem | What we saw | Root cause | Fix |
|---------|-------------|-----------|-----|
| Isaac and ROS could not see each other | No topics across | A Fast DDS profile disabled the builtin transports | `twin_env.sh` unsets the profile files |
| URDF import failed | Invalid prim path | `-` in mesh file names | Sanitised symlinks in `run_isaac_twin.sh` |
| Depth looked blocky | Half resolution | DLSS upscaling | `/rtx/post/aa/op = 0` |
| Sim sprinted after start | Huge time debt | 80 s shader compile on the first frame | Drop pacing debt beyond 0.5 s |
| `ros2_control_node` segfault | Crash on load | apt TopicBasedSystem ABI mismatch | Build from source (`src/ros2.repos`) |
| Arm snapped to zero on relaunch | Unexpected motion | TopicBasedSystem publishes `initial_value` before any controller | Seed `initial_joint_N` from Isaac |
| Marker server overwrote the Gazebo cache | `poses_sim_*` reseeded | Shared `poses_env` and provenance check | `poses_env/authored_in: isaac` |
| BT stopped at PourTilt | Missing target | The Niryo layout has no `PourTiltAtWeighingContainer` | Twin-only draft target |
| Overshoot reported as success | 37.5 g for 20 g → SUCCESS | `ComputeRemaining` clamped at 0 | `CheckOvershoot` on `fix/weightment-overshoot` |
| Gym env and Isaac disagreed on one scoop | 14 g (env) vs 40 g (Isaac) | The env drove straight lines at a guessed speed; MTC interpolates joints and exits faster. Replaying the recorded trajectory through the same model gave 36.5 g vs 37.6 g | The env interpolates joints between IK solutions at 0.2 m/s; Isaac prewarms capacities along the authored path so a cold cache cannot hold the payload |
| Grains through the floor at start | Bed vanishes | Overlapping seed + depenetration | `rest_ratio` 0.45, caps, CCD |
| Grains through the RS6 crease | Slow leak | Triangle-mesh collider | `convexDecomposition` |
| Arm drifted in particle mode | Tracking error | Robot `CollisionGroup` | `FilteredPairsAPI` |
| Scoop never collected grains | 0 g in the scoop | Importer box collider under instance proxies | De-instance + disable + voxel shell |
| "Physics segfault" in sweeps | Crash at exit | Writing to a read-only numpy view of a USD array raised, then Kit crashed | `np.array(...)` copy |
| Bed 25 mm instead of 40 | Camera and grams disagree | PBD compression | 48 iterations + `settle_ratio` |
| User's twin laggy, RViz resets | 679 "jump back in time" | A second Isaac (a leftover benchmark) publishing `/clock` on domain 77 | Kill it; one Isaac at a time |

---

## 17. Rebuilding the twin from scratch

A recipe in the order that de-risks fastest. Each step has a check.

### Step 0: prerequisites

- Isaac Sim 5.1 standalone (`~/isaacsim/isaac-sim-standalone-5.1.0-linux-x86_64`), an
  NVIDIA RTX GPU, driver 580 or newer.
- ROS 2 Jazzy, with the workspace built: `niryo_robot_description`, the MoveIt config,
  `scooping_controller`, `scoop_vision`, `pouring_controller`, `robot_orchestrator`.
- `topic_based_ros2_control` built **from source**.

### Step 1: isolation first

Write `twin_env.sh`: domain 77, LOCALHOST discovery, unset the DDS profile files. Put the
isolation check in the launch before anything else. *Check:* the launch refuses to start
from a plain shell.

### Step 2: robot in Isaac, talking ROS

1. Expand the URDF and sanitise mesh names.
2. In `build_cell.py`: start `SimulationApp`, enable the bridge and the importer, import
   the URDF with a fixed base, set stiff drives, and build the joint-bridge OmniGraph with
   `/clock`.
3. Add the pacing loop.

*Check:* `ros2 topic echo /isaac_joint_states` at 30 Hz. Publishing a JointState on
`/isaac_joint_commands` moves the arm.

### Step 3: ros2_control and MoveIt on top

1. Write the Isaac xacro (TopicBasedSystem block) and a controllers yaml with the cell's
   JTC name.
2. In the launch: read one Isaac joint state, pass `initial_joint_N`, then start RSP,
   `ros2_control_node`, the spawners and move_group, all with `use_sim_time`.

*Check:* MoveIt plans and executes in RViz, and the arm does not jump at launch.

### Step 4: the cell geometry

Parse the layout yaml the way the C++ loader does (quaternion or rpy, mm scale) and add the
meshes. Start the cell's scene publishers (task frame, markers, collisions, layout
manager). *Check:* the RViz collision objects line up with the Isaac meshes.

### Step 5: the camera

1. Compute `camera_link` from the hand-eye calib.
2. Add two USD cameras with the intrinsics → focal-length conversion and render products
   with the ROS camera helpers. Disable DLSS.
3. Write `depth_to_mm` (32FC1 → 16UC1 + noise) and `sim_camera_tf`.
4. Start scoop_vision with `start_realsense:=false`.

*Check:* the depth cloud in RViz overlays the RS6 mesh, and `/scoop_vision/capture`
succeeds.

### Step 6: a simple powder (heightfield)

1. Bed grid from `ContainerModel`; carve with the scoop's points through FK.
2. Publish `rs3_mass_g`; add the scale node.
3. Then add capacity + repose + vibration + pour (section 8).

*Check:* the authored scoop removes a sensible volume; a manual pour moves grams into RS3
and `/weight` rises with a lag.

### Step 7: the full BT

Start the pour nodes and the orchestrator with the cell's tree. Supply any missing targets
as twin-only drafts, never in `config/layouts`. *Check:* the 20 g webhook BT finishes
SUCCESS and the mass balance closes.

### Step 8: particles

1. GPU dynamics on the physics scene; gravity on the scene, off on the robot links.
2. Vessel and table colliders (`convexDecomposition`), and robot filtering
   (`FilteredPairsAPI`).
3. Swap the scoop collider: de-instance, disable the importer's collider, add the voxel
   shell.
4. Particle system + PBD material + a points prim seeded with `seed_bed`.
5. Calibrate `settle_ratio` with a 20 s headless run.
6. Accounting by bounding box and container footprint, then the vibration kick.

*Check:* depth = 40 mm, grams match the heightfield bed, the replay scoop keeps grains, the
camera volume matches, and the BT succeeds.

### Step 9: the gym env

Wrap the heightfield model, the planner limits, IK and the timed path in `gym.Env` (section
12). *Check:* `scoop_env_compare` runs and the heuristic beats random.

### Minimal particle code skeleton (Isaac 5.1)

```python
from omni.physx.scripts import particleUtils, physicsUtils
from pxr import Sdf, Vt, UsdShade

s = 0.004; rest = 0.45 * s
particleUtils.add_physx_particle_system(
    stage, Sdf.Path("/World/Powder/system"),
    contact_offset=rest + 0.1 * s, rest_offset=rest,
    particle_contact_offset=rest + 0.1 * s, solid_rest_offset=rest, fluid_rest_offset=0.6 * rest,
    enable_ccd=True, solver_position_iterations=48, max_velocity=1.0, max_depenetration_velocity=0.2)
UsdShade.Material.Define(stage, "/World/Powder/mat")
particleUtils.add_pbd_particle_material(stage, Sdf.Path("/World/Powder/mat"),
    friction=0.6, particle_friction_scale=1.0, damping=0.1, cohesion=0.02, adhesion=0.0)
physicsUtils.add_physics_material_to_prim(stage, stage.GetPrimAtPath("/World/Powder/system"),
                                          Sdf.Path("/World/Powder/mat"))
points = particleUtils.add_physx_particleset_points(
    stage, Sdf.Path("/World/Powder/grains"),
    Vt.Vec3fArray.FromNumpy(seed.astype("float32")), Vt.Vec3fArray.FromNumpy(zeros),
    Vt.FloatArray.FromNumpy(widths), Sdf.Path("/World/Powder/system"),
    self_collision=True, fluid=False, particle_group=0, particle_mass=s**3 * 550.0, density=0.0)
# each frame: pos = np.array(points.GetPointsAttr().Get())  -> classify -> grams
```

---

## 18. Limitations and next steps

1. **Fit the powder to the real cell.** That is the biggest gap. Measure on the cell:
   - grams in the scoop after the post-lift shake, for the authored scoop and for 3–4
     shifted scoops;
   - grams per second while pouring at a known intensity and tilt;
   - bed depth before and after.

   Fit `vibration_kick_m_s`, friction and cohesion (particles), and `heap_factor`, the
   repose angles and the flow rate (heightfield).
2. **A physical vibration model:** an oscillating scoop collider, once the motor's
   frequency and amplitude are known. Budget for ≥ 1 kHz physics in the scoop region, or
   accept an effective lower frequency.
3. **The non-GPU convex warning:** find which robot link has a non-particle-compatible
   collider (likely an importer convex hull). It is harmless for scoop, vessels and table.
4. **Promote the PourTilt draft** after an operator check on the real Niryo (joint_5 margin
   is small).
5. **Open the overshoot PR** (`fix/weightment-overshoot`, `CheckOvershoot` BT node).
6. **Speed:** the PhysX solve is the floor at 4 mm. Options, if accuracy allows: a smaller
   simulated region (only grains near the scoop dynamic), or newer PhysX releases.
7. **RL:** train a policy in `ScoopEnv`, validate it on particle Isaac, then on the cell.

---

## 19. Glossary

| Term | Meaning |
|------|---------|
| Twin | This simulation of the cell running the cell's real software |
| RTF | Real-time factor: sim seconds per wall second |
| Render product | An offscreen RTX render target (a camera's output) |
| OmniGraph | Kit's node graph; the ROS 2 bridge is OmniGraph nodes |
| Articulation | A reduced-coordinate multi-body (the robot) in PhysX |
| Drive | A per-joint PD controller in PhysX |
| Instance proxy | A read-only prim inside an instanced USD sub-tree |
| PBD | Position-based dynamics: constraint projection on positions |
| Rest offset | A particle's collision radius |
| Settle ratio | Settled / seeded bed height; scales the grams per grain |
| Level fill | What a bowl holds of liquid at a given tilt |
| Angle of repose | The steepest slope a powder surface holds |
| Heap factor | How much a bowl heaps above its level fill at rest |
| TopicBasedSystem | A ros2_control hardware plugin that talks over two topics |
| JTC | `JointTrajectoryController` |
| MTC | MoveIt Task Constructor; runs the scoop |
| BT | Behaviour tree (the orchestrator's `webhook_weightment.xml`) |
| RS6 / RS3 | Source container (powder bed) / weighing container on the scale |

---

## 20. Code map

All paths are under `src/isaac_twin/` unless noted.

| File | What to read it for |
|------|---------------------|
| `scripts/run_isaac_twin.sh`, `scripts/twin_env.sh` | How Isaac is started and isolated |
| `isaac_twin/scene/build_cell.py` | The whole Isaac side: scene, model switch, main loop, replay |
| `isaac_twin/scene/joint_bridge.py` | The OmniGraph ROS bridge for joints and `/clock` |
| `isaac_twin/scene/d455.py` | Cameras, intrinsics, render products |
| `isaac_twin/scene/particles.py` | GPU physics, colliders, the scoop voxel collider, `ParticlePowder` |
| `isaac_twin/particle_powder.py` | Seeding, the settle ratio, depth, accounting |
| `isaac_twin/powder.py` | The heightfield model, capacity and repose maths |
| `isaac_twin/cell.py` | Layout parsing, calibration and D455 frames, param merge |
| `isaac_twin/kinematics.py` | FK, Jacobian, DLS IK |
| `isaac_twin/scoop_path.py` | Authored scoop paths and the replay timing |
| `isaac_twin/depth_to_mm_node.py`, `twin_scale_node.py`, `sim_camera_tf.py`, `draft_targets_node.py` | The twin's ROS nodes |
| `isaac_twin/gym/scoop_env.py`, `gym/compare.py` | RL env and evaluation |
| `launch/scooping_isaac.launch.py` | How the cell stack is wired to the twin |
| `urdf/niryo_ned3pro_isaac.urdf.xacro`, `config/ros2_controllers_isaac.yaml` | ros2_control on topics |
| `config/twin.yaml`, `config/twin_nodes.yaml` | Every tunable |
| `test/` | 41 tests; read `test_particle_powder.py` and `test_powder.py` for the expected behaviour |
| `docs/SCOOP_VISION.md` (repo root) | The capture → plan pipeline the twin feeds |
