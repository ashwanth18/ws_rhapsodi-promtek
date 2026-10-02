# Agent notes (repo root)

## Camera-guided scooping (D455 + Niryo)

- **Hand-eye calibration:** `src/camera_robot_calibration/` (its README has an issue log)
- **Next-scoop planner, container calibration from the camera, RViz panel:** `src/scoop_vision/`
  - agents: read `src/scoop_vision/CLAUDE.md` before touching scooping, the cell layouts, or
    `scooping_mtc_node` offsets
  - humans: `docs/SCOOP_VISION.md`

Rules that matter across the repo:
- `scooping_mtc_node` allows the scoop and wrist links to collide with the task vessel. The
  wall/floor safety for shifted scoops lives in `scoop_vision`. Do not remove its clearance
  or IK checks.
- `config/layouts/dual-container.yaml` RS6 pose was fitted from the camera (2026-10-01).
  `planner.surface_correction_m` (0.03) is a **temporary** calibration workaround. Revert it
  after the camera is recalibrated.
- Never move the robot or overwrite `config/layouts` without the operator's confirmation.
