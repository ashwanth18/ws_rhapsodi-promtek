# camera_robot_calibration

Table-mounted **RealSense D455** **eye-on-base** hand-eye calibration
(ChArUco + [easy_handeye2](https://github.com/marcoesposito1988/easy_handeye2)
+ `/move_to`).

The camera is fixed in the cell. The ChArUco board is taped to the **flange**
only while sampling. After save, take the paper off; day-to-day runtime only
needs the published `base` → `camera_color_optical_frame` TF.

| Cell | Config | Calib file | Isolated bringup |
|------|--------|------------|------------------|
| Niryo Ned3 Pro | `config/robots/niryo_d455.yaml` | `calibrations/niryo_d455_eob.calib` | yes (`niryo_robot_bringup.launch.py`) |
| JAKA Zu5 / Lexium | `config/robots/jaka_d455.yaml` | `calibrations/jaka_d455_eob.calib` (after you run it) | no — attach to laptop `scooping_real` |

## Current Niryo result (2026-09-08)

Checked in at `calibrations/niryo_d455_eob.calib`.

`base_link` → `camera_color_optical_frame`:

- Translation **`[0.343, -0.017, 0.773]` m** (camera ~34 cm forward of the Niryo base, ~77 cm up, looking down; 42 mm A4-fill squares)
- Recalibrate if you move the D455 or the robot base relative to each other

How we got here (wrong `/tf`, optical-frame publish, depth vs color cloud,
ChArUco corner, **A4 42 mm squares**): [Issue log](#issue-log-niryo-d455-2026-09-08).


Copy into the easy_handeye2 home path before publish:

```bash
mkdir -p ~/.ros2/easy_handeye2/calibrations
cp src/camera_robot_calibration/calibrations/niryo_d455_eob.calib \
  ~/.ros2/easy_handeye2/calibrations/
```

## Print and mount the board

Default **PNG** is 5×7, `DICT_5X5_250`, square 30 mm, marker 22 mm at 100% print.
The Niryo sheet on the cell was filled to A4, so `config/charuco_board.yaml` is
**42 mm / 30.8 mm**. Measure a square if you reprint.

```text
boards/charuco_5x7_dict5x5_30mm.png
```

Print at **100% scale** (no fit-to-page). See `boards/PRINT_ME.txt`.

Tape the **whole sheet rigid on the flange** (Niryo `hand_link` / JAKA `Link6`),
print facing the D455. Paper on the table does **not** work with this stack.

Regenerate after editing `config/charuco_board.yaml`:

```bash
ros2 run camera_robot_calibration generate_charuco_board \
  --config $(ros2 pkg prefix camera_robot_calibration)/share/camera_robot_calibration/config/charuco_board.yaml \
  --output /tmp/charuco.png
```

## Frames

Camera frames are the same on both cells. Robot frames differ:

| Role | Niryo | JAKA |
|------|-------|------|
| Robot base | `base_link` | `link0` |
| easy_handeye2 effector (flange) | `hand_link` | `Link6` |
| MoveTo tip | `tcp_link` | `tcp_link` |
| Camera optical | `camera_color_optical_frame` | same |
| Board | `charuco_board` | same |

Topics: `/camera/color/image_raw`, `/camera/color/camera_info`, overlay `/charuco_detector/overlay`.

## Isolate the ROS graph

Do not calibrate while the **other** cell’s scooping stack is on the same
`ROS_DOMAIN_ID`. FastDDS shared memory on one laptop sees every local
container even with `LOCALHOST`.

- **Niryo session:** stop laptop JAKA compose ROS (`scooping_stack`, …) and
  stop Pi `scooping_stack` (or it will republish Niryo `/tf`).
- **JAKA session:** stop the Pi Niryo stack; do not start a second Niryo
  `niryo_d455_calibrate` bringup on this graph.

Calibrate / publish launches set `ROS_LOCALHOST_ONLY=1`. Interactive
`tf2_echo` terminals need the same:

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOCALHOST_ONLY=1
```

## Calibrate Niryo again (same robot)

Stop leftover `realsense2_camera_node` / `charuco_detector` if a previous
launch died. Park the arm so the board faces the D455 with room to tilt ~25°.

**1. Detection only (no motion)**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROBOT_IP=169.254.200.200   # Niryo rosbridge

ros2 launch camera_robot_calibration niryo_d455_calibrate.launch.py \
  dry_run:=true \
  bringup_robot:=true \
  start_sampler:=false \
  start_handeye:=true \
  start_realsense:=true \
  serial_no:=YOUR_D455_SERIAL
```

```bash
ros2 run tf2_ros tf2_echo camera_color_optical_frame charuco_board
ros2 run tf2_ros tf2_echo base_link hand_link
```

Board TF should be stable. Then Ctrl+C.

**2. Sample (this moves the arm)**

`dry_run:=false` + `start_sampler:=true` drives ~12 poses around the current
TCP (mostly ±25° wrist tilts, 15% speed), then compute + save.

```bash
ros2 launch camera_robot_calibration niryo_d455_calibrate.launch.py \
  dry_run:=false \
  bringup_robot:=true \
  start_sampler:=true \
  start_handeye:=true \
  start_realsense:=true \
  serial_no:=YOUR_D455_SERIAL
```

Wait for `Bridge node initialized`, then `Prepared 12 sample poses`.
Success looks like:

```text
Calibration saved to ~/.ros2/easy_handeye2/calibrations/niryo_d455_eob.calib
Hand-eye calibration finished successfully
```

**3. Check the file into git**

```bash
cp ~/.ros2/easy_handeye2/calibrations/niryo_d455_eob.calib \
  src/camera_robot_calibration/calibrations/niryo_d455_eob.calib
```

Update the date / translation in `calibrations/README.md`.

**4. Publish and verify** (three terminals, leave 1+2 running)

```bash
# 1 — robot + camera + detector (no sampler)
ros2 launch camera_robot_calibration niryo_d455_calibrate.launch.py \
  dry_run:=true bringup_robot:=true start_sampler:=false \
  start_handeye:=false start_realsense:=true serial_no:=YOUR_D455_SERIAL

# 2 — static TF from the .calib (keep this process alive)
ros2 launch camera_robot_calibration niryo_d455_publish.launch.py \
  name:=niryo_d455_eob

# 3
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 run tf2_ros tf2_echo hand_link charuco_board
```

`hand_link` → `charuco_board` is the rigid paper mount. Jog the arm (board
still in view). That TF should stay within a few millimetres. Then remove
the board from the flange.

RViz against an already-running Niryo calibrate bringup (do **not** start
`scooping_real` as well):

```bash
ros2 launch scooping_controller scooping_rviz_only.launch.py robot:=niryo
```

## Calibrate JAKA (different robot)

There is **no** isolated JAKA driver in this package. Use the laptop cell
stack for `/move_to` + `move_group`, then this package for camera +
easy_handeye2.

**1.** Stop Pi Niryo ROS. Start (or keep) laptop `scooping_real` for JAKA
so `tf2_echo link0 Link6` is stable.

**2.** Tape the same ChArUco print on **`Link6`**, facing the D455.

**3.** Detection + sample (no second robot bringup):

```bash
export ROBOT_IP=192.168.88.82   # Lexium, or your JAKA IP

ros2 launch camera_robot_calibration niryo_d455_calibrate.launch.py \
  robot_config:=$(ros2 pkg prefix camera_robot_calibration)/share/camera_robot_calibration/config/robots/jaka_d455.yaml \
  dry_run:=true \
  bringup_robot:=false \
  start_sampler:=false \
  start_handeye:=true \
  start_realsense:=true \
  serial_no:=YOUR_D455_SERIAL
```

Confirm `camera_color_optical_frame` → `charuco_board` and `link0` → `Link6`.
Then the same launch with `dry_run:=false start_sampler:=true` (arm will move).

Saves `~/.ros2/easy_handeye2/calibrations/jaka_d455_eob.calib`. Copy to
`src/camera_robot_calibration/calibrations/jaka_d455_eob.calib`.

**4.** Publish with `name:=jaka_d455_eob`. Verify
`tf2_echo Link6 charuco_board` stays put while you jog.

If `/move_to` or flange TF names differ on that cell, edit
`config/robots/jaka_d455.yaml` (`eef_link`, `robot_base_frame`,
`robot_effector_frame`, `calibration_name`) before launching.

## Publish at runtime (no board)

```bash
ros2 launch camera_robot_calibration niryo_d455_publish.launch.py \
  name:=niryo_d455_eob    # or jaka_d455_eob
```

Keep that node running **with RealSense up**. It publishes static
`base_link` → `camera_link` (composed with RealSense
`camera_link` → `camera_color_optical_frame`). Do **not** publish
`base_link` → `camera_color_optical_frame` while the camera driver is
running: that gives the optical frame two parents, so the depth cloud
never transforms into `base_link`.

## Click-to-move (RGB-D + RViz)

Cold-starts Niryo + D455 **aligned depth / textured cloud** + hand-eye TF +
RViz. Click a 3D point on the cloud; TCP goes to that XYZ with the **current
wrist orientation** if MoveIt can plan (otherwise the arm does not move).

```bash
export ROBOT_IP=169.254.200.200
ros2 launch camera_robot_calibration niryo_d455_click_move.launch.py \
  serial_no:=351322303477
# optional: enable_octomap:=false  and/or  enable_containers:=false
```

In RViz: **Publish Point** (toolbar), click the **point cloud** (not the 2D
image). Green sphere = plan ok / moving; red = unreachable or rejected.
A successful click **moves the real arm**.

Cloud topic: `/camera/color/points` (aligned depth, `camera_color_optical_frame`
— the same camera the ChArUco calib used). The D455 native
`/camera/depth/color/points` cloud is in the **depth** sensor, ~6 cm from
color, so the URDF will not sit on it.

The Niryo bringup does **not** bridge robot `/tf` (only `/joint_states` and
the rest). Laptop `robot_state_publisher` owns the scoop URDF (`tcp_link`
offset `0.15825, 0, -0.09356`). Leave `scooping_real` off this graph.

Click-to-move can collide with authored tubs and/or the D455 cloud. Both are
on by default:

| Arg | Default | Meaning |
|-----|---------|---------|
| `enable_containers` | `true` | RS3 / RS6 / table from `niryo_real.yaml` |
| `enable_octomap` | `true` | Occupied voxels from `/camera/color/points` |

Octomap needs `ros-jazzy-moveit-ros-perception` (`sudo apt install
ros-jazzy-moveit-ros-perception`); without it `enable_octomap:=true` is a
no-op. In RViz, **PlanningScene** shows tubs and voxels when those flags are
on. Enable **Self-filtered Cloud** to see the cloud after the arm is stripped.

## Launch args (`niryo_d455_calibrate.launch.py`)

| Arg | Default | Meaning |
|-----|---------|---------|
| `robot_config` | Niryo YAML | Frame / sample / camera profile |
| `bringup_robot` | `true` | Niryo driver + move_group + move_to only |
| `start_realsense` | `true` | `realsense2_camera` |
| `start_sampler` | `true` | Auto pose sampling |
| `start_handeye` | `true` | easy_handeye2 server |
| `dry_run` | `false` | Skip motion and save |
| `serial_no` | `""` | Pin D455 serial |
| `use_rqt` | `false` | easy_handeye2 rqt UI |

**Warning:** `dry_run:=false` and `start_sampler:=true` moves the real arm.

## Issue log (Niryo D455, 2026-09-08)

What we actually hit, in order. Several “the overlay is wrong” symptoms were
**different bugs stacked**. Fixing only the last one does not explain the
earlier RViz failures.

### 1. Two robots on one DDS graph

Pi `scooping_stack` and laptop JAKA compose both publish `/tf` on domain 0.
FastDDS shared memory on the Legion sees them even with
`ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. `tf2_echo` and MoveIt then mix
Niryo and JAKA trees.

**Do:** stop the other cell’s ROS (`scooping_stack`, fleet-agent ROS) for a
Niryo session. Calibrate / click-move launches set `ROS_LOCALHOST_ONLY=1`.
**Any extra terminal** (`tf2_echo`, `rqt`) needs the same export.

### 2. Two parents for `tcp_link` (scoop looked “wrong”)

Laptop `robot_state_publisher` publishes the scoop URDF (`tcp_link` at
`0.15825, 0, -0.09356` m on `tool_link`). The Niryo rosbridge driver also
bridges `/tf` / `/tf_static` with a **short flange TCP** (~10 cm).

RViz RobotModel then flickers between those two. It is not a bad STL origin.

**Do:** `config/niryo_driver_no_tf.yaml` — whitelist everything except `/tf`
and `/tf_static`. Joint states still come from the robot; RSP owns kinematics.
Do **not** run `scooping_real` next to `niryo_d455_click_move`.

### 3. No point cloud in `base_link` (two TF trees)

easy_handeye2’s publisher attaches `base_link` → `camera_color_optical_frame`.
RealSense already parents that optical frame under `camera_link`. Two parents
split the graph: the robot is under `base_link`, the cloud stays under
`camera_depth_optical_frame`. RViz Fixed Frame `base_link` cannot transform
the cloud.

**Do:** `handeye_camera_link_publisher` publishes **`base_link` → `camera_link`**
= calib(`base` → color optical) × inv(RealSense `camera_link` → color optical).
`niryo_d455_publish.launch.py` uses that, not stock easy_handeye2 publish.

Check: `tf2_echo base_link camera_depth_optical_frame` must succeed.

### 4. Cloud in the depth camera, calib on the color camera

D455 native `/camera/depth/color/points` is `camera_depth_optical_frame`
(built **before** align). Depth origin is ~**59 mm** from color optical
(stereo baseline). Hand-eye locates **color**. Overlay of URDF vs that cloud
is shifted even when (3) is fixed.

**Do:** `align_depth.enable:=true`, **disable** the native pointcloud, and
build `/camera/color/points` from `/camera/aligned_depth_to_color/image_raw`
with **color** `camera_info` (`aligned_color_cloud`). Same frame as ChArUco.

This still does **not** fix a wrong square size (next items). Depth *values*
can disagree with PnP even in the color frame.

### 5. ChArUco TF is a sheet **corner**, not the scoop centre

OpenCV’s board origin is a corner of the sheet. `charuco_board` in TF sits
~10 cm off the scoop midline and looks “below” the mesh. That is not a Z
error.

**Do:** look at `charuco_board_center` (detector publishes it). After a
metric-correct calib the centre sits in the scoop AABB (`tool_link`
≈ `[0.14, 0.00, -0.05]` m on this cell).

### 6. Print scale (the real “depth” mismatch) — overlooked twice

After (3)+(4)+(5), color PnP and aligned depth still disagreed on the paper:

| | Distance from color camera |
|--|--|
| solvePnP (config **30 mm** squares) | ~**0.37 m** |
| aligned depth at the same pixels | ~**0.51 m** |
| ratio | **~1.39** |

That ratio is A4 **fill-page** of a 150×210 mm PNG (5×42 mm = 210 mm).
`PRINT_ME.txt` said 100% / 30 mm; the sheet on the flange was **~42 mm**.
PnP (and therefore Tsai-Lenz) was ~28% too short. The URDF is metric from
joints; the cloud is metric from the stereo; the board TF was in “30 mm
fantasy metres”. Overlay of TF vs cloud cannot work.

**Do:** measure one square with a ruler. This cell:
`config/charuco_board.yaml` → `square_length_m: 0.042`,
`marker_length_m: 0.0308`. Then **recalibrate** (new samples). Restarting
click-to-move alone with a 42 mm detector and a 30 mm `.calib` puts the
board TF on the cloud and **off** the scoop.

Sanity check (board still on, both topics live):

```text
project PnP origin into the color image
read /camera/aligned_depth_to_color/image_raw at that pixel (mm→m)
compare to PnP z in camera_color_optical_frame
```

They must match within a couple of centimetres. If not, square size or
depth scale is wrong — do not keep tuning camera TF.

### 7. Calib files (same day)

| Run | Squares | `base` → color optical (m) | Notes |
|-----|---------|----------------------------|--------|
| 1 | 30 mm assumed | `[0.344, 0.000, 0.611]` | Dual `/tf` still on; 11/12 samples |
| 2 | 30 mm assumed | `[0.338, -0.009, 0.616]` | Driver `/tf` blocked; still wrong print size |
| 3 | **42 mm** | `[0.343, -0.017, 0.773]` | Overlay matched TF, cloud, scoop |

easy_handeye2 always writes
`~/.ros2/easy_handeye2/calibrations/niryo_d455_eob.calib`. Copy into
`src/camera_robot_calibration/calibrations/` after a good run.

### 8. Other traps (do not confuse with overlay)

- Sampler needs `tcp_link` / rosbridge up; wait up to 60 s for TF. Sequence
  must not `spin_once` on an already spinning executor (run it on a background
  thread).
- Click-to-move keeps **current wrist** and only changes TCP XYZ. STOMP
  `INVALID_GOAL_CONSTRAINTS` means that pose is unreachable, not a bad calib.
- `hand_link` → `charuco_board` staying put when you jog only proves the
  paper is rigid on the flange. It does **not** prove square size or
  camera-in-base.
- Click the **point cloud** in RViz (Publish Point), not the 2D image.

## Optional compose overlay


```bash
docker compose -f compose/devices/pi5.yml -f compose/devices/handeye-calibrate.yml \
  --profile handeye --project-directory <workspace> \
  --env-file robot-prod.env up -d handeye_calibrate
```

Needs a ros-prod image with this package, `easy_handeye2`, and
`realsense2_camera`.

## Tests (no hardware)

```bash
colcon test --packages-select camera_robot_calibration
python3 -m pytest src/camera_robot_calibration/test -q
```
