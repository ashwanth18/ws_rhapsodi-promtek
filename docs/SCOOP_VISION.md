# Camera-guided scooping: how it works

How the table-mounted RealSense D455 decides **where the Niryo scoops next**, and how
it **calibrates the bin's pose**. Written for a robotics engineer who wants to
understand, tune and extend it. It covers the algorithms, the maths, the frames, the
code, and what is approximate.

- **Code:** `src/scoop_vision/` (Python, ROS 2 Jazzy). The panel is in `src/scooping_controller/`; the BT nodes are in `src/robot_orchestrator/`.
- **Operator reference:** `src/scoop_vision/README.md`
- **Agent notes:** `src/scoop_vision/CLAUDE.md`

---

## 0. One-paragraph summary

The camera looks down at the scoop bin (RS6). With the arm out of view, 8 depth frames
are turned into a **2.5D height map** of the flour (5 mm cells) in the bin's own frame.
The planner treats your taught scoop (approach → contact → scoop → lift →
transport_ready) as a **rigid template** and searches only for a **translation**
`(dx, dy, dz)`. For every candidate placement it computes:
- the **trench** the scoop would cut;
- how much flour lies above that trench, and therefore the predicted bowl fill;
- the **shallowest depth** that still overfills the bowl.

It then ranks placements: **fill first, then the highest flour (to keep the bed level),
then the smallest shift**. The best placements are checked for **clearance to the bin and
floor** and for **robot reachability (IK)**. The winner becomes
`pattern_offset_x/y/z` on your existing `scooping_mtc_node`, which runs the scoop
unchanged.

Separately, the same height data can **register the bin mesh to what the camera sees**
(rim matching). That corrects the bin's pose in the cell layout.

---

## 1. Design choices (and why)

| Choice | Why |
|---|---|
| **Shift the authored scoop, don't generate one** | The authored scoop already works mechanically on the real robot (entry angle, sweep, shake-off, transport). Shifting it keeps all that, and keeps the problem to 3 numbers. It is also safer, which is what you asked for. |
| **2.5D height map, not a point cloud or mesh** | Flour is a surface seen from above. A per-cell height `H(x,y)` is enough, cheap and robust. Everything (volume, depth, level) becomes array maths. |
| **Geometry heuristic, not a physics simulation** | It's fast (≈0.1–0.5 s), explainable and tunable. The one physical unknown, how much of the disturbed flour ends up in the bowl, is a single parameter (`fill_efficiency`) you can fit from data. |
| **Own clearance check** | MoveIt can't be relied on here: the MTC scene *allows* the scoop and wrist links to touch the vessel, because otherwise contact would be "in collision". |
| **Everything derived from your existing meshes and poses** | Changing the layout, the bin or the scoop needs no new configuration: the RS6 STL, the scoop STL, `robots.yaml` and `/scoop_poses` are the inputs. |
| **Camera calibrates the bin, but only proposes** | Your scoop poses live in the bin's frame. Moving the bin frame moves the real scoop, so a human confirms. |

---

## 2. System overview

```
             ┌──────────────── RealSense D455 (848×480 @15 Hz, spatial+temporal filters)
             │ /camera/depth/image_rect_raw + camera_info
             ▼
┌──────────────────────────────── scoop_vision_node ────────────────────────────────┐
│ capture ─► occlusion test ─► 8 frames ─► rim check vs layout ─► height map H(x,y) │
│ plan    ─► planner (trench, volume, depth, score) ─► clearance ─► IK (/compute_ik)│
│ fit_container_pose ─► rim registration ─► layout proposal ─► apply (/cell_layout) │
│ markers: surface cloud, planned path, ghost scoops, animated scoop, fitted bin    │
└─────────────┬───────────────────────────────────────────────────┬────────────────┘
              │ PlanScoop response (dx,dy,dz)                      │ /cell_layout/active,
              ▼                                                   │ /scoop_poses, TF
  BT: PlanScoopFromVision ─► ExecuteScoop(pattern_offset_x/y/z)   │
  or RViz panel "Run scoop"                                        │
              ▼                                                   │
     scooping_mtc_node /execute_scoop_continuous ◄────────────────┘
     (applies the offsets to the 5 authored poses, MTC plans, one controller goal)
```

Inputs it subscribes to or reads:
- **`/cell_layout/active`:** the task container ID and the layout YAML, giving the bin mesh and its scale.
- **`/scoop_poses`:** the 5 authored poses, in `scooping_container_frame`.
- **`scooping_mtc_node` parameters:** `manual_sweep_scale`, `manual_pitch_offset_rad`, `manual_lift_offset_z`, so the planner sees exactly the path MTC will run.
- **`robots.yaml`:** the scoop mesh and the `tool_link → tcp_link` offset.
- **TF:** `base_link → camera_link` (hand-eye), `base_link → scooping_container_frame` (layout), and the arm links (occlusion and staleness).

---

## 3. Frames and the transform chain

```
base_link ──(hand-eye calib: base→camera_link)──► camera_link ──(RealSense)──► camera_depth_optical_frame
    │
    └──(cell layout: RS6 position_xyz + yaw)──► scooping_container_frame  ◄── all planning happens here
                                                      (= RS6 mesh frame, z up, origin at the mesh origin)
tool_link ──(fixed +[0.15825, 0, −0.09356] m)──► tcp_link   (scoop STL is on tool_link, identity visual origin)
```

- **Everything is computed in `scooping_container_frame`.** That's the frame the
  authored poses and `pattern_offset_*` use, so the planner's output needs no
  conversion.
- **A depth pixel becomes a point in the bin frame** by unprojecting it in the optical
  frame, then applying one rigid transform `T_C←cam` from TF.
- **Error budget:** the hand-eye error goes straight into where the flour *appears*
  relative to the robot. The camera-fitted bin pose carries the same error. That's why the
  temporary `surface_correction_m` exists today, see §7.6.

---

## 4. Perception: from depth to a height map

### 4.1 Unprojection (`heightmap.unproject_depth`)
For a depth pixel `(u, v)` with depth `z` (metres = raw × 0.001), using the depth camera's
intrinsics `fx, fy, cx, cy`:

```
x = (u − cx) · z / fx        y = (v − cy) · z / fy        p_cam = (x, y, z)
p_C = R_C←cam · p_cam + t_C←cam
```

Every 2nd pixel is used (`stride: 2`), which gives about 100k points per frame. Depths
outside 0.15–2.0 m are dropped.

### 4.2 Gridding and robust statistics (`heightmap.cell_median`, `build_surface`)
The bin-frame XY plane is a regular grid with **5 mm cells** covering the mesh footprint
plus 15 cm.

1. **Per frame:** keep points that fall in **interior** cells (§5.1), lie above the local
   floor (minus 1 cm), and lie below the tallest wall plus 8 cm. Each cell gets the
   **median z** of its points. Implemented with one `lexsort` and grouped indexing: there
   are no Python loops over points.
2. **Across frames:** the per-cell **median over the 8 frames**, wherever at least 30% of the
   frames saw the cell. This removes D455 speckle and single-frame dropouts. White flour
   has little texture, so the IR projector and the temporal filter matter.
3. **Holes** (shadows next to walls, specular spots) are filled from the **nearest measured
   cell**, via `distance_transform_edt(..., return_indices=True)`.
4. **Clamp:** `H ≥ floor height` (flour can't be below the floor).

The result is `H(c)` for every interior cell, plus the measured fraction. Capture fails
below 50% measured.

### 4.3 Is the arm in the way? (`_occlusion`)
Before and after collecting frames:
1. Project the 8 corners of the bin's interior box into the depth image, giving a pixel
   bounding box.
2. Take the arm chain `elbow_link → forearm_link → wrist_link → hand_link → tool_link →
   tcp_link` from TF, sample 6 points per segment, and project each with a radius of 7 cm:
   `r_px = fx · 0.07 / z`.
3. If any sample's disc overlaps the bin box and the sample is *closer* to the camera than
   the bin, capture refuses and names the blocking link.

Only link origins are used; it's a fast, conservative approximation. The `CameraClear`
target was chosen to be at least 15 cm outside this test, see §8.

### 4.4 When is a height map still valid?
- **Scoop in the bin:** if `tcp_link` enters the bin's interior box (XY + 2 cm, below rim + 3 cm),
  the map is stale, because the scoop just changed the flour.
- **Layout change:** a new layout hash means the bin frame moved, so the map is stale.
- **Age:** older than `max_heightmap_age_s` (900 s) is stale, which covers refills.

`plan` refuses stale maps.

---

## 5. Geometry models built from the STLs

### 5.1 The container (`container.ContainerModel`)

**Top-surface raster `F(c)`.** For each 5 mm cell, the highest mesh surface directly
below its centre: barycentric rasterisation of every triangle, keeping the maximum z.
Vertical faces have no top area and are skipped.

**Rim and interior.**
- `rim_z` is the **lowest** wall top on the outer ring of the footprint. On RS6 that's the
  open front lip, 102 mm.
- The **interior** is the cells with `F < rim_z − 5 mm`, keeping the largest connected
  region. On RS6 that's 350 × 380 mm, floor at 10 mm, including the sloped front wall.
- **Rim cells** are the footprint cells that aren't interior (the wall tops). They're used
  by the calibration.

**Clearance fields (3D, 4 mm voxels).** There are two Euclidean distance transforms:
- **walls:** every triangle except the floor;
- **floor + table:** the triangles at floor height, plus everything below the bin's base
  plane `z < 0`.

The wall occupancy is **dilated horizontally only**, by a disc of `wall_margin_xy` (12 mm),
before the distance transform. Why horizontal only:
- The errors we're guarding against are **horizontal**: bin pose and hand-eye XY. The rim
  fit pins Z to about 1 mm.
- A scoop *beside* the back wall then gets 12 + 8 = **20 mm**.
- A scoop passing *over* the 102 mm front lip from above is judged on vertical distance,
  which isn't inflated. A uniform 20 mm would have forced every scoop up and cut predicted
  fill from 120% to 26%.

`clearances(points)` returns the `(wall, floor)` distances per point by a voxel lookup.

### 5.2 The scoop (`scoop_tool.ScoopTool`)

The scoop STL (on `tool_link`) is shifted into the `tcp_link` frame, then sampled two ways:
- **dense surface samples (about 4 mm)**, for the trench `B(c)`;
- **6 mm voxel-deduplicated samples**, for the clearance checks.

**Bowl capacity: "trapped rain water".** The question is how much a level fill holds at
the lift orientation (pose 4 of 5):
1. Rotate the mesh, then rasterise its top surface on a **1 mm** grid; cells with no mesh are open air.
2. Run a **priority flood**:
   - Seed a min-heap with all border and open-air cells (they drain).
   - Repeatedly pop the lowest "water level" `h`. For each unvisited neighbour with top `t`,
     add `max(0, h − t)` of water and push `max(h, t)`.
   - Total water × cell² is the capacity.
3. Result: **≈ 73–75 ml** for the Niryo scoop, about 40 g of level-full flour. Cohesive flour
   heaps above that, which `fill_efficiency` and the 120% target absorb.

The decimated scoop mesh has rim walls thinner than 2 mm. At a 2 mm grid the water "leaks"
and gives 42 ml, so a regression test now compares the decimated and hi-res meshes.

---

## 6. Container calibration: registering the bin to the camera

**Goal:** find the rigid correction `(dx, dy, dz, θ)` such that the real bin is
`T_layout · Trans(dx, dy, dz) · Rz(θ)`, using only what the camera sees.
Code: `alignment.fit_container_offset`.

### 6.1 What we compare
- **Measured map `M(c)`:** the **90th percentile** of points per cell, so it represents the
  top surface. The camera also sees inner wall faces, which a median would mix in.
- **Model:** `F(c)` on the **rim cells only**. Rims look the same whether the bin is empty or
  full; flour changes everything inside.

### 6.2 Cost of a candidate shift `s` (in cells)
For each rim cell `k`, compare the measured height at the shifted cell with the mesh height:

```
r_k(s) = M(c_k + s) − F(c_k)                       (NaN if the camera has no data there)
m      = median_k r_k(s)                            (a constant height offset is ignored here)
J(s)   = [ Σ_seen min(|r_k − m|, κ)  +  κ · N_unseen ] / N_rim         κ = 30 mm
```

- **Subtracting the median** makes the XY fit independent of a constant Z error.
- **The cap `κ`** stops a few cells of flour on the rim from dominating.
- **Charging unseen cells `κ`** stops the fit from sliding the rim out of view to make the error go away.

### 6.3 Search
1. **Coarse:** all shifts in **±80 mm** at 5 mm steps (33 × 33) with θ = 0.
2. **Yaw:** for θ in **±4° at 0.5°**, rotate the *measured points* by −θ about the frame
   origin, re-grid, and search shifts in a small window (±4 cells) around the coarse
   optimum. A few degrees of yaw moves the rim by only a few cells.
3. **Sub-cell refinement:** fit a parabola through the scores on each side of the best one.

   ```
   δ = ½ (J₋ − J₊) / (J₋ − 2J₀ + J₊)      (clipped to ±½ step)
   ```

   This is applied to X, Y and θ. The shift found in the de-rotated frame converts back with `d = Rz(θ) · s'`.
4. **Z:** use only **flat** rim cells, whose 4 neighbours are rim at the same height (wall
   edges mix in lower surfaces). Take the **20th percentile** of `r_k`, because flour on a
   rim only ever reads high.
5. **"At the search limit":** if the best shift is on the search edge, the result is flagged
   as untrustworthy.

Runtime is about 0.5 s on 250k subsampled points.

### 6.4 Validation
- **Synthetic:** a bin moved (+61, +18) mm and −3° was recovered to within about 3 mm and 0.1°.
- **Your cell:** RS6 came out at (+54, +6) mm, +6–7 mm Z, about 0° yaw from the old layout.
  The rim error dropped from 19.6 to 12.3 mm; the remainder is flour on the rims and the
  real bin vs the STL. Re-checking a fresh snapshot against the corrected pose gave (0, 0) mm.

### 6.5 Turning the fit into a layout, safely
With the layout's position `p` and yaw `ψ`:

```
p' = p + Rz(ψ) · d            yaw' = ψ + θ            (roll = pitch = 0 assumed)
```

`layout.write_layout_proposal` edits **only the task container's line** in the YAML. It
keeps your comments, adds a comment recording the fit, and writes to
`~/.ros/scoop_vision/proposals/<stamp>_<layout>/`.

Your scoop poses are in the bin frame, so you choose what happens to them:
- **Follow the bin (used here):** keep the poses; the real scoop moves with the
  corrected bin. Right when the layout was wrong and the scoop should sit relative to the
  *real* bin.
- **Keep the world path:** `poses_keep_world_path.yaml` is the old poses re-expressed in the
  new frame:

  ```
  p_new = Rz(−θ) · (p_old − d)        q_new = Rz(−θ) ⊗ q_old
  ```

  On your cell this would have cut into the back wall, so it wasn't used.

**Apply fit** backs up the layout, copies the proposal and calls `/cell_layout/apply`.
Your provenance system then re-stamps the poses (`container_spec_hash`).

### 6.6 The capture gate
Every capture runs the fast **X/Y-only** version (about 0.1 s). If the bin is more than 10 mm
off in XY, more than 10 mm in Z, or at the search limit, capture refuses. A moved bin
can't silently produce a plan with walls inside the "flour".

**Accuracy ceiling:** the camera finds the bin in *its estimate* of `base_link`. Hand-eye
errors carry straight through. Cross-check once with a touch-off
(`scripts/calibrate_container_pose.py`) or a tape measure.

---

## 7. The next-scoop planner (`planner.ScoopPlanner`)

### 7.1 What is decided
The decision variables are `d = (dx, dy, dz)`. The 5 poses MTC executes are
`P_i' = shape(P_i) + d`, where `shape()` (`transforms.apply_mtc_shape`) mirrors the C++
`apply_pattern_offset` exactly: the sweep scaled about contact, the lift offset on poses 4 and
5, and the pitch post-multiplied. After shaping, any offset is a **pure translation**.
That's what makes the search cheap.

### 7.2 The trench `B(c)`
Interpolate the path (≤ 5 mm or 0.05 rad per step; linear position, slerp orientation)
through the **cutting segments**: approach → contact → scoop → lift. Transform the scoop's
surface samples by each step. For every grid cell, **`B(c)` = the lowest scoop point ever
above that cell** (`np.minimum.at`).

```
B(c) = min { z(p) : p ∈ ⋃_steps T_step(scoop samples),  xy(p) ∈ cell c }
```

**A translation in whole cells just shifts this map.** `B` is computed **once**, and each
candidate is "the same trench, moved".

### 7.3 Engaged volume and predicted fill
For a candidate XY shift (in cells) and height `dz`, with `d_c = H(c+shift) − B(c)` over trench
cells that have flour, and cell area `A` = 25 mm²:

```
V(dz)    = A · Σ_c max(0, d_c − dz)              (flour above the trench)
fill(dz) = η · V(dz) / C_bowl                    (η = fill_efficiency, C_bowl ≈ 73 ml)
```

`η` lumps together everything the geometry doesn't model: flour pushed aside by the scoop
body, flour falling off during lift and shake-off, and packing. It is the parameter to
**fit from data**.

### 7.4 Choosing the depth: closed form
"Fill as much as possible" is implemented as **reach the target `ρ` = 120% with the
shallowest scoop**. Digging deeper than needed only adds drag on the Ned3. `V(dz)` is
piecewise linear and decreasing in `dz`. Sort `d_c` in descending order (`d₁ ≥ d₂ ≥ …`); on
the interval where the deepest `m` cells are engaged:

```
Σ_c max(0, d_c − dz) = (d₁+…+d_m) − m·dz   ⇒   dz_m = (cumsum_m − S*) / m,   S* = ρ·C/(η·A)
```

Pick the `m` whose `dz_m` lies in its own interval (`solve_dz_for_volume`). That's exact
and O(n log n).

### 7.5 Hard constraints (they only push the scoop **up**)

```
dz ≥ max(  dz_min,
           max_c d_c − p_max,                              (deepest point ≤ 65 mm below the surface)
           max_a [H(a) − A_min(a)] + δ_approach,           (approach pose ≥ 10 mm above the flour)
           dz_vol − extra_depth )
dz ≤ dz_max                                                (otherwise reject)
penetration = max_c d_c − dz;   if < 8 mm the predicted fill is 0 (the scoop would only skim)
```

`A_min(a)` is the scoop's lowest point per cell **at the approach pose**. The free-space
move into the approach doesn't know about flour, so the approach must be above it.

### 7.6 Depth knobs: surface correction vs extra depth
- **`surface_correction_m` (temporary, 0.03):** the measured surface becomes
  `H ← max(H − s, F)` *before* everything else. Use it when the **camera reads flour too
  high** (calibration). Fill, the depth cap and the approach rule all then reason about
  the corrected surface. The floor check uses the bin model, which is unaffected. A test
  proves that "30 mm-too-high camera + 30 mm correction" gives *exactly* the plan of a
  perfect camera.
- **`extra_depth_m` (0):** dig deeper than the fill model asks for. An operator preference,
  not a calibration fix.

### 7.7 Scoring
```
score = min(fill, ρ)/ρ  +  w_L · (H̄ − z_floor)/(z_rim − z_floor)  −  w_S · ‖(dx, dy)‖
          w_L = 0.3                                                       w_S = 0.2 /m
```
- **Fill dominates:** a lane that fills always beats one that doesn't.
- **Level term:** `H̄` is the mean flour height under the trench. Among lanes that fill,
  the planner takes from the **highest flour**. That flattens the bed, which keeps the next
  scoop predictable and avoids pits that cohesive flour collapses into.
- **Shift term:** a gentle pull toward your authored lane.

The XY search covers every 10 mm shift where the trench fits the interior, which is about
100 lanes. On RS6 your sweep spans almost the whole bin length, so **X barely moves and the
real decision is the Y lane and the depth**.

### 7.8 Clearance (best candidates first)
For the top candidates, in score order, the **full** path (all 4 segments, including
lift → transport) is swept with the scoop's collision samples (deduplicated at 4 mm) and
looked up in the two distance fields:

```
min wall distance ≥ wall_clearance (8 mm)   on walls already grown 12 mm sideways
min floor distance ≥ floor_clearance (8 mm)
```

- **If it fails:** raise `dz` in 5 mm steps (up to 12) and re-evaluate the volume. This helps
  floor and front-lip contacts.
- **If raising stops helping:** that's a side or back wall, so give up on that lane quickly.

The first 40 feasible candidates (or 600 checks) form a pool.

### 7.9 Reachability
In score order, each pooled candidate's 5 shifted poses go to MoveIt `/compute_ik` (group
`arm`, link `tcp_link`, seeded from the current state, collisions off). The **first fully
reachable** candidate wins.

This was added after a full bin lifted the approach by 8 cm out of the Niryo's workspace.
Live, the planner then chose an 18 cm-different lane, and your MTC diagnose confirmed all 5
poses OK.

### 7.10 Outputs
- **Response:** `PlanScoop` with the offsets, predicted fill, volume, capacity, depth,
  surface height, clearances, map age, and `container_empty` (true if the best reachable
  fill is below 15%).
- **Log:** `/scoop_vision/last_plan` carries the full JSON record (parameters + surface
  statistics). Use it for logging and fitting.
- **RViz:** the TCP path, a ghost scoop at each waypoint, the swept volume, a summary, and
  an animated scoop running the motion.

### 7.11 Cost
| Step | Time |
|---|---|
| Container model (two 3D distance transforms + wall dilation) | ≈0.25 s, once per bin |
| Planner build (trench, capacity) | ≈0.15 s, once per pose set |
| A plan (≈100 lanes + clearance) | ≈0.15–0.3 s |
| IK | ≈5–10 ms per pose |

---

## 8. Execution and integration

- **`scooping_mtc_node`** (unchanged) adds the offsets to the 5 authored poses
  (`apply_pattern_offset`). MTC plans the approach/contact/scoop/lift stages and sends
  **one continuous controller goal**, then the post-lift shake-off.
- **BT** (`webhook_weightment.xml`, `scoop_weigh_pour.xml`):
  - `CaptureScoopSurface` runs during the 2.5 s scale settle at the weighing vessel.
  - `PlanScoopFromVision` runs before `ExecuteScoop`, which writes `pattern_offset_x/y/z`.
  - **Fallback:** if vision fails, the old behaviour runs (authored scoop / Y raster).
- **RViz panel** (`ScoopVisionPanel`): Check, Fit, Apply, camera-clear, Capture, Plan,
  Preview (MTC plan only), Run (with confirm), Use authored, and the Depth correction box.
- **`CameraClear` target:** found by searching about 50 poses with `/compute_ik`,
  `/compute_fk` and `/check_state_validity`.
  - Every joint is at least 0.52 rad from its limits. By contrast, `MoveToWeighingContainer`
    sits on `joint_2`'s limit, which made MoveIt reject start states.
  - The arm is at least 15 cm outside the occlusion test.
  - It is collision-free.

---

## 9. Safety layers

| Layer | Protects against |
|---|---|
| Occlusion test at capture | Arm in the picture read as flour |
| Rim-vs-layout gate at capture | Moved bin: walls inside the "flour", wrong clearances |
| Staleness (TCP entry, layout hash, age) | Planning on flour that has since changed |
| Walls grown 12 mm sideways + 8 mm clearance | Scraping back and side walls despite pose or calibration error |
| Floor + table clearance 8 mm | Hitting the bin floor on a low bed |
| Depth cap (65 mm below the surface) | Excessive drag on the Ned3 |
| Approach ≥ 10 mm above the flour | The free-space approach ploughing flour |
| IK on all 5 poses | Unreachable plans (MTC `GOAL_STATE_INVALID`) |
| Proposal + confirm for layout changes | Silently moving every taught scoop |
| Confirm dialog on Run | Accidental motion from RViz |
| BT fallback | Vision outage stopping production |

---

## 10. Tuning guide (symptom → knob)

| Symptom | Likely cause | Knob |
|---|---|---|
| Scoop only scrapes the surface | The camera reads flour too high (hand-eye Z) | `surface_correction_m` (panel "Depth correction"); recalibrate the camera |
| Scoop full on a deep bed but light on a shallow one | `fill_efficiency` wrong | Fit `η` from data (§11) |
| Digs too hard, Niryo strains or faults | Too deep | lower `max_penetration_m`, `target_fill_ratio`, `extra_depth_m` |
| "None of the safe shifts is reachable" | Full bin lifts the approach out of reach | Re-teach the approach lower/flatter; empty the bin a bit |
| "All checked shifts come too close to the container" | Margins too tight for the lane, or the bin moved | Check / Fit pose; `wall_margin_xy_m` |
| Capture: "Container differs from layout" | Bin moved or layout wrong | Fit pose → Apply fit |
| Capture: "Arm (…) blocks the camera view" | Arm over the bin | Arm to camera-clear |
| Always the same lane / digs a pit | Level preference too weak | raise `level_weight` |
| Wanders far from the authored lane | Shift penalty too weak | raise `shift_weight_per_m` |

---

## 11. Making it better with data

The most valuable next step is to **log one row per scoop**:
`(H before, d, V predicted, fill predicted, grams on the scale, H after)`.

- **Fit `η`:** grams ≈ ρ_flour · η · V. Linear regression through the origin, with bulk
  density ρ_flour ≈ 0.55 g/ml.
- **Check the trench model:** `H before − H after` measures what was actually removed.
  Compare it with the predicted trench.
- **Then learn the score:** for example, regress grams on `(V, depth, H̄, lane)`, or fit a
  small model that predicts grams from local height-map patches. Keep the clearance and IK
  layers as hard constraints around any learned scorer.

---

## 12. Known limitations and ideas to refine

1. **Hand-eye accuracy** limits everything: surface, bin pose, clearances. Recalibrate,
   check depth vs PnP on the board (see the `camera_robot_calibration` issue log), then
   **set `surface_correction_m` back to 0 and re-fit the bin**.
2. **Path model:** the planner assumes straight lines between waypoints, while MTC executes
   a joint-space plan. Better: plan with MTC (plan-only), then sweep the *actual* joint
   trajectory through FK.
3. **Forearm and wrist** aren't in the clearance check. Either model them via FK, or narrow
   the MTC allowed-collision list to the scoop links only.
4. **Shift-only:** powder against the back wall is unreachable within the 20 mm margin. Yaw
   about z, a shorter sweep or a second authored scoop would widen coverage; that's a design
   decision to make with you.
5. **One scoop at a time:** no lookahead. A sequence planner (keep the bed flat, avoid
   collapse into pits) could improve consistency.
6. **No uncertainty model:** cells are treated equally whether they were measured 8 times
   or inpainted. Per-cell confidence could weight the volume and the level term.
7. **Predict instead of recapture:** subtracting the trench from `H` after a scoop would
   allow back-to-back scoops without moving to camera-clear (verify against a capture now
   and then).
8. **Registration** uses rim tops only. Adding the inner wall faces, or ICP on 3D points,
   would be more robust to flour on the rims.
9. **`MoveToWeighingContainer`** sits on `joint_2`'s limit. Move the target, or enable MoveIt
   `fix_start_state`.
10. **Production:** the compose overlay and BT integration are built and the trees load, but
    the full BT loop hasn't been run on hardware. `lightsout.xml` isn't integrated.

---

## 13. Code map

| File | What to read it for | Key functions |
|---|---|---|
| `scoop_vision/heightmap.py` | Depth → height map | `unproject_depth`, `cell_median`, `build_surface` |
| `scoop_vision/container.py` | Bin geometry and clearance fields | `ContainerModel.__init__`, `_build_sdf`, `clearances` |
| `scoop_vision/scoop_tool.py` | Scoop model, capacity | `ScoopTool`, `trapped_volume` |
| `scoop_vision/transforms.py` | MTC shape mirror, interpolation | `apply_mtc_shape`, `interpolate_path` |
| `scoop_vision/planner.py` | **The planner** | `ScoopPlanner.__init__` (trench), `plan` (search/score/clearance/IK), `solve_dz_for_volume` |
| `scoop_vision/alignment.py` | **Container registration** | `fit_container_offset`, `check_alignment`, `_score`, `_parabolic` |
| `scoop_vision/layout.py` | Layout I/O, proposal, re-anchoring | `write_layout_proposal`, `corrected_container_pose`, `reanchor_poses` |
| `scoop_vision/scoop_vision_node.py` | ROS glue, safety gates, visuals | `_capture_points`, `_occlusion`, `_watch_eef`, `_srv_plan`, `_reachable`, `_srv_fit_container`, `_srv_apply_fit` |
| `test/*.py` | Executable spec (33 tests) | Run `python3 -m pytest src/scoop_vision/test -q` |
| `scooping_controller/src/scoop_vision_panel.cpp` | RViz panel | button handlers, `setOffsets`, `moveTo` |
| `robot_orchestrator/include/.../scoop_vision_nodes.hpp` | BT nodes | `CaptureScoopSurfaceNode`, `PlanScoopFromVisionNode` |

To experiment offline, build a `ContainerModel`, a `ScoopTool` and a `ScoopPlanner` from
the STLs and `config/layouts/dual-container/poses.yaml` (see `test/conftest.py`). Feed
`SurfaceMap`s made of synthetic beds, or one rebuilt from a saved depth snapshot, and
compare parameter sets with `dataclasses.replace(PlannerParams(), ...)`.

---

## 14. Glossary

| Term | Meaning |
|---|---|
| **Height map `H(c)`** | Flour surface height per 5 mm cell, in the bin frame |
| **Trench `B(c)`** | The lowest point the scoop passes above each cell during the cut |
| **Lane** | One XY placement of the authored scoop |
| **Penetration** | Deepest point of the scoop below the local (corrected) flour surface |
| **`η` (fill_efficiency)** | Share of the trench volume that ends up in the bowl |
| **Rim cells** | Wall-top cells of the bin mesh, used for registration |
| **Proposal** | A corrected layout written for review, not applied |
| **CameraClear** | Arm pose out of the camera's view of the bin, with joints well inside their limits |
