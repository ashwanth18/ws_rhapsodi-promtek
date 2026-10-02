# scoop_vision

Uses the table-mounted **RealSense D455** to measure the flour in the scoop bin
(RS6) and choose the **next scoop** for the Niryo Ned3 Pro. It only **shifts the
scoop you authored** (`pattern_offset_x/y/z` on `scooping_mtc_node`) and never
changes its shape. It can also **calibrate the bin's pose from the camera**.

> Want the algorithms, maths and code explained? Read
> [`docs/SCOOP_VISION.md`](../../docs/SCOOP_VISION.md).
> Agents: see [`CLAUDE.md`](CLAUDE.md).

```
arm out of view ──► capture: 8 depth frames → powder height map (5 mm grid)
                        │      (refuses if the arm blocks the view or the bin
                        │       is not where the layout says)
                        ▼
                plan: best shift (dx, dy, dz) of the authored scoop
                        │  fill the bowl · dig the highest flour · ≥ 20 mm from
                        │  back/side walls · ≥ 8 mm from floor · reachable (IK)
                        ▼
      pattern_offset_x/y/z ──► scooping_mtc_node /execute_scoop_continuous
```

Prerequisite: the hand-eye calibration from `camera_robot_calibration`
(`niryo_d455_eob.calib`). The launch files publish `base_link → camera_link` from it.

## Quick start (laptop, Niryo, native)

Stop anything else that drives the Niryo or publishes `/tf` first: the Pi
`scooping_stack` / `orchestrator`, and the laptop `orchestrator_dev` container.

```bash
cd ~/ws_rhapsodi-promtek-dev && source install/setup.bash
ros2 launch scoop_vision scoop_vision_niryo.launch.py serial_no:=351322303477
#   layout_id:=dual-container (default, RS6)   stack_rviz:=true (also the scooping RViz)
```

This starts `scooping_real` (RS6 layout from `config/layouts`, Niryo driver with
`/tf` blocked), the D455, the hand-eye TF, `scoop_vision` and RViz with the
**Scoop Vision panel**. Other terminals need
`export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`.

Alongside an already-running stack: `ros2 launch scoop_vision scoop_vision.launch.py
serial_no:=… layouts_dir:=$PWD/config/layouts use_rviz:=true`. `layouts_dir` is
needed when native, because the Docker stack publishes `/ws/config/...` paths.

## The Scoop Vision panel (RViz)

| Button | Does |
|--------|------|
| **Check** | Compares the bin rim seen by the camera with the layout |
| **Fit pose** | Fits the bin pose (X/Y/Z/yaw) from the rim and writes a layout *proposal*; the orange bin in RViz shows the fit |
| **Apply fit** | Confirms, backs up the layout, writes the fitted pose and calls `/cell_layout/apply`. The scoop poses follow the bin |
| **Arm to camera-clear** | `MoveTo CameraClear` at 20% (arm to the robot's left, out of the camera's view, joints ≥ 0.5 rad from their limits) |
| **Capture powder** | Builds the powder height map |
| **Plan next scoop** | Picks the shift; shows shift, fill, depth and clearance, and animates the scoop in RViz |
| **Preview in MoveIt** | Sets the offsets, then `/plan_scoop`; the whole-arm MTC plan shows in *Motion Planning Tasks* |
| **Run scoop** | Confirms, sets the offsets, `MoveTo MoveToScoopingContainer` at 20%, then `/execute_scoop_continuous` |
| **Use authored scoop** | Resets the offsets to 0 |
| **Depth correction** | `planner.surface_correction_m` live, in mm. Re-plan after changing it |

Typical loop: **Arm to camera-clear → Capture → Plan → (Preview) → Run**, then repeat.

RViz displays (Fixed Frame `base_link`):

| Display | Topic |
|---------|-------|
| D455 depth cloud | `/camera/depth/color/points` |
| Containers | `/container_marker` |
| Powder height map | `/scoop_vision/surface` |
| Planned scoop (path, ghost scoops, swept volume, summary) | `/scoop_vision/plan_markers` |
| Animated scoop | `/scoop_vision/scoop_motion` |
| Fitted container (orange) | `/scoop_vision/fitted_container` |
| Motion Planning Tasks | `/solution` |

The config deliberately has **no camera Image display**. Image dock, panel and
MTC display together, with no saved window layout, crash RViz2 under XWayland.

## Services and topics

| Name | Type | Notes |
|------|------|-------|
| `/scoop_vision/capture` | `std_srvs/Trigger` | Refuses if the arm occludes the bin or the bin differs from the layout by more than 1 cm |
| `/scoop_vision/plan` | `robot_common_msgs/PlanScoop` | Needs a fresh map (no TCP entry since capture, < `max_heightmap_age_s`) |
| `/scoop_vision/check_container_alignment` | `std_srvs/Trigger` | X/Y-only rim fit vs layout |
| `/scoop_vision/fit_container_pose` | `std_srvs/Trigger` | X/Y/Z/yaw fit; writes a proposal to `~/.ros/scoop_vision/proposals/` |
| `/scoop_vision/apply_container_fit` | `std_srvs/Trigger` | Applies this session's last proposal |
| `/scoop_vision/last_plan` | `std_msgs/String` (JSON, latched) | Every plan with its parameters and surface stats; log this to fit `fill_efficiency` |
| `/scoop_vision/surface_stats` | `std_msgs/String` (JSON, latched) | After each capture |

## Behaviour trees

In `webhook_weightment.xml` and `scoop_weigh_pour.xml`:
- **`CaptureScoopSurface`** runs during the 2.5 s scale settle at the weighing vessel, and once at tree start.
- **`PlanScoopFromVision`** runs before **`ExecuteScoop`**, which now takes optional `pattern_offset_x/z`.
- **Fallback:** if vision fails, the tree does exactly what it did before (the authored scoop, plus the `ComputeScoopOffset` Y raster in webhook mode).

`lightsout.xml` is unchanged, because the scoop vessel is also the scale vessel there.

Opt-in compose service: `compose/devices/scoop-vision.yml` (profile `vision`). It
needs a ros-prod image rebuilt with this package.

## Tuning (`config/scoop_vision.yaml`)

| Param | Default | Meaning |
|-------|---------|---------|
| `planner.surface_correction_m` | **0.03 (TEMPORARY)** | The camera reads flour this much too high, so the surface is lowered before planning. **Set to 0 after recalibrating the camera**, then re-run Fit pose → Apply fit |
| `planner.fill_efficiency` | 0.5 | Share of the trench volume that ends up in the bowl. **Fit from data** (`last_plan` vs grams ÷ ~0.55 g/ml) |
| `planner.target_fill_ratio` | 1.2 | Aim to overfill the 73 ml bowl; the pour handles precision |
| `planner.max_penetration_m` | 0.065 | Deepest point of the scoop below the (corrected) surface |
| `planner.extra_depth_m` | 0.0 | Extra depth beyond the fill model |
| `wall_margin_xy_m` | 0.012 | Walls grown sideways (bin-pose / hand-eye error). Back and side walls get ≥ 8 + 12 = 20 mm |
| `planner.wall_clearance_m`, `floor_clearance_m` | 0.008 | Clearance on top of the grown walls, and to the floor |
| `planner.level_weight` | 0.3 | Preference for the highest flour once fill is met |
| `check_reachability` | true | MoveIt `/compute_ik` on all 5 shifted poses |
| `require_alignment` | true | Gate capture on the bin rim matching the layout |
| `max_heightmap_age_s` | 900 | Older maps are refused |

## Tests (no hardware)

```bash
python3 -m pytest src/scoop_vision/test -q
```

There are 33 tests. They use the real RS6 mesh, the Niryo scoop STL and
`config/layouts/dual-container/poses.yaml`, with synthetic beds: the planner,
height map, alignment and yaw fit, layout proposal and re-anchoring, and the margins.
