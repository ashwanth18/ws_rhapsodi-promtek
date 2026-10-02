"""Gymnasium env: one step is one shifted scoop in the twin's RS6 bed.

Same cell model as the Isaac twin (layout RS6 mesh, Niryo scoop, authored
scoop poses, ``PowderCell`` carve/spill with ``config/twin.yaml``) and the same
shift action and clearance rules as scoop_vision, without Isaac or ROS.

* action: ``(dx, dy, dz)`` added to all 5 scoop poses (scoop_vision's
  ``pattern_offset``), bounded by ``ScoopPlanner.shift_window`` and
  ``planner.dz_min_m`` / ``dz_max_m``;
* observation: ``height`` (powder depth above the floor per interior cell, 0
  elsewhere) and ``joints`` (IK at the last contact pose);
* reward: ``-|scooped_g - target_g| / target_g``; a clearance violation or an
  unreachable shift is not executed and costs ``violation_penalty`` plus
  ``violation_per_mm`` per mm under the clearance limit.
"""

from __future__ import annotations

from dataclasses import fields
from pathlib import Path

import gymnasium as gym
import numpy as np
import yaml
from gymnasium import spaces

from isaac_twin import cell
from isaac_twin.kinematics import Chain
from isaac_twin.powder import PowderCell, ScoopParams
from isaac_twin.scoop_path import cartesian_path, chained_ik, joint_path, load_scoop_poses, pose_matrix
from scoop_vision.container import ContainerModel
from scoop_vision.heightmap import SurfaceMap
from scoop_vision.mesh import load_stl
from scoop_vision.planner import PlannerParams, ScoopPlan, ScoopPlanner
from scoop_vision.scoop_tool import ScoopTool
from scoop_vision.transforms import Pose, apply_mtc_shape

DEFAULT_URDF = Path.home() / ".cache" / "isaac_twin" / "niryo_ned3pro.urdf"


class ScoopEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(
        self,
        *,
        layout_id: str = "dual-container",
        layouts_dir: str | Path | None = None,
        poses_yaml: str | Path | None = None,
        twin_config: str | Path | None = None,
        nodes_yaml: str | Path | None = None,
        urdf: str | Path | None = None,
        check_reachability: bool = True,
        target_g: float | None = None,
        fill_depth_m: float | None = None,
        fill_depth_range_m: tuple[float, float] | None = None,
        max_scoops: int = 30,
        violation_penalty: float = 1.0,
        violation_per_mm: float = 0.1,
        sweep_scale: float = 1.0,
        pitch_offset_rad: float = 0.0,
        lift_offset_z: float = 0.0,
        # Spill on the lip-down exit depends on it. 0.2 matches replaying the
        # twin's recorded MTC scoop (0.12-0.28 m/s through the bed and exit).
        tcp_speed_m_s: float = 0.2,
        shake_s: float = 5.0,
        shake_intensity: float = 0.75,
        settle_s: float = 1.5,
    ) -> None:
        ws = cell.workspace_root()
        layouts_dir = Path(layouts_dir) if layouts_dir else ws / "config" / "layouts"
        layout = cell.load_layout(layouts_dir / f"{layout_id}.yaml")
        task_id = str(layout.get("task_container_id") or "rs6")
        task = {o.id: o for o in cell.layout_objects(layout)}[task_id]
        self.base_to_container = task.pose

        sv = cell.twin_scoop_vision_params(nodes_yaml)["scoop_vision"]["ros__parameters"]
        planner_cfg = sv.get("planner") or {}
        params = PlannerParams(**{
            f.name: type(f.default)(planner_cfg[f.name]) for f in fields(PlannerParams) if f.name in planner_cfg
        })
        tris = load_stl(cell.resolve_uri(task.mesh_resource)) * task.scale
        self.container = ContainerModel(tris, wall_margin_xy=float(sv.get("wall_margin_xy_m", 0.0)))
        tool_cfg = cell.robot_tool_config(str(sv.get("robot_key", "niryo")))
        self.tool = ScoopTool(load_stl(cell.resolve_uri(tool_cfg["mesh_resource"])) * 0.001, tool_cfg["tcp_visual_offset_xyz"])

        poses = load_scoop_poses(poses_yaml or layouts_dir / layout_id / "poses.yaml")
        poses = apply_mtc_shape(poses, sweep_scale=sweep_scale, pitch_offset_rad=pitch_offset_rad, lift_offset_z=lift_offset_z)
        self.planner = ScoopPlanner(self.container, self.tool, poses, params)

        twin_config = Path(twin_config) if twin_config else cell.package_dir("isaac_twin") / "config" / "twin.yaml"
        twin = yaml.safe_load(twin_config.read_text(encoding="utf-8"))
        self.fill_depth_m = float(fill_depth_m if fill_depth_m is not None else twin["powder"]["fill_depth_m"])
        self.fill_depth_range_m = fill_depth_range_m
        self.powder = PowderCell(
            self.container, np.eye(4), None, None, self.tool, self.fill_depth_m, ScoopParams.from_twin_config(twin),
            capacity_cache_dir=Path.home() / ".cache" / "isaac_twin",
        )

        self.capacity_g = self.powder.grams(self.planner.capacity_m3)
        self.target_g = float(target_g if target_g is not None else params.target_fill_ratio * self.capacity_g)
        self.max_scoops = int(max_scoops)
        self.violation_penalty = float(violation_penalty)
        self.violation_per_mm = float(violation_per_mm)
        self.tcp_speed_m_s = float(tcp_speed_m_s)
        self.shake_s, self.shake_intensity, self.settle_s = float(shake_s), float(shake_intensity), float(settle_s)

        self.chain = None
        self._seeds: list[np.ndarray] = []
        urdf = Path(urdf) if urdf else DEFAULT_URDF
        if check_reachability and urdf.is_file():
            self.chain = Chain(urdf.read_text(encoding="utf-8"), "base_link", "tcp_link")
            self._seeds = self._authored_seeds()

        x0, x1, y0, y1 = self.planner.shift_window()
        self.action_space = spaces.Box(
            low=np.array([x0, y0, params.dz_min_m], dtype=np.float32),
            high=np.array([x1, y1, params.dz_max_m], dtype=np.float32),
        )
        mask = self.powder.bed.mask
        depth_max = float(self.container.rim_z - self.container.floor_z)
        lo = self.chain.lower if self.chain else np.full(6, -np.pi)
        hi = self.chain.upper if self.chain else np.full(6, np.pi)
        self.observation_space = spaces.Dict({
            "height": spaces.Box(0.0, depth_max, shape=mask.shape, dtype=np.float32),
            "joints": spaces.Box(lo.astype(np.float32), hi.astype(np.float32), dtype=np.float32),
        })
        self._joints = np.zeros(6)
        self._scoops = 0

    def _authored_seeds(self) -> list[np.ndarray]:
        """IK of the authored poses, chained, as seeds for shifted ones."""
        seeds = chained_ik(self.chain, [self.base_to_container @ pose_matrix(p) for p in self.planner.poses])
        if seeds is None:
            raise RuntimeError("Authored scoop poses have no IK solution; check the URDF/layout")
        return seeds

    def joints_for(self, poses: list[Pose]) -> list[np.ndarray] | None:
        """IK of each pose (base_link), or None if any is unreachable."""
        if self.chain is None:
            return None
        out = []
        for pose, seed in zip(poses, self._seeds):
            sol = self.chain.ik(self.base_to_container @ pose_matrix(pose), seed)
            if sol is None:
                return None
            out.append(sol)
        return out

    def reachable(self, poses: list[Pose]) -> tuple[bool, str]:
        """``ScoopPlanner.plan`` reachability callback."""
        if self.chain is None:
            return True, ""
        return (True, "") if self.joints_for(poses) is not None else (False, "IK failed")

    def _obs(self) -> dict:
        bed = self.powder.bed
        depth = np.where(bed.mask, bed.surface - bed.floor, 0.0)
        depth = np.clip(np.nan_to_num(depth), 0.0, self.observation_space["height"].high.max())
        return {"height": depth.astype(np.float32), "joints": self._joints.astype(np.float32)}

    def surface_map(self) -> SurfaceMap:
        """Ground-truth surface as scoop_vision would capture it."""
        bed = self.powder.bed
        return SurfaceMap(height=bed.surface.copy(), measured=bed.mask.copy(), measured_fraction=1.0, frames=1)

    def heuristic_plan(self) -> ScoopPlan:
        """scoop_vision's planner on the ground-truth surface."""
        return self.planner.plan(self.surface_map(), reachable=self.reachable if self.chain else None)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        super().reset(seed=seed)
        options = options or {}
        depth = options.get("fill_depth_m")
        if depth is None and self.fill_depth_range_m is not None:
            depth = float(self.np_random.uniform(*self.fill_depth_range_m))
        self.powder.reset(depth if depth is not None else self.fill_depth_m)
        self._joints = np.zeros(6)
        self._scoops = 0
        return self._obs(), {"bed_g": self.powder.bed_g, "target_g": self.target_g}

    def _path(self, poses: list[Pose], joints: list[np.ndarray] | None) -> list[tuple[np.ndarray, int]]:
        """``(container -> tcp, segment)`` samples: joint-space with IK, else straight lines."""
        if joints is None:
            return cartesian_path(poses)
        container_from_base = np.linalg.inv(self.base_to_container)
        return [(container_from_base @ tcp, seg) for tcp, seg in joint_path(self.chain, joints)]

    def _execute(self, poses: list[Pose], joints: list[np.ndarray] | None) -> None:
        """Run the scoop through lift, the MTC post-lift shake-off, then on to transport."""
        p = self.powder
        prev = None

        def shake(tcp: np.ndarray) -> None:
            for duration, vib in ((self.shake_s, self.shake_intensity), (self.settle_s, 0.0)):
                for _ in range(int(round(duration / 0.05))):
                    p.step(0.05, tcp, vib)

        shaken = False
        for tcp, seg in self._path(poses, joints):
            if seg == 3 and not shaken:
                shake(prev)
                shaken = True
            dt = 0.0 if prev is None else float(np.linalg.norm(tcp[:3, 3] - prev[:3, 3])) / self.tcp_speed_m_s
            p.step(max(dt, 1e-3), tcp)
            prev = tcp
        if not shaken:
            shake(prev)

    def step(self, action):
        dx, dy, dz = (float(v) for v in np.clip(action, self.action_space.low, self.action_space.high))
        self._scoops += 1
        ok, wall, floor = self.planner.clearance_ok(dx, dy, dz)
        poses = self.planner.shifted_poses(dx, dy, dz)
        joints = self.joints_for(poses) if ok else None
        reachable = not ok or self.chain is None or joints is not None
        info = {
            "action": (dx, dy, dz),
            "clearance_ok": ok,
            "reachable": reachable,
            "min_wall_clearance_m": wall,
            "min_floor_clearance_m": floor,
        }
        p = self.powder
        if not ok or not reachable:
            pp = self.planner.params
            short_mm = 1000.0 * max(pp.wall_clearance_m - wall, pp.floor_clearance_m - floor, 0.0)
            reward = -self.violation_penalty - self.violation_per_mm * short_mm
            info.update(executed=False, scooped_g=0.0, carved_g=0.0, spilled_g=0.0)
        else:
            bed0, table0, carved0 = p.bed.volume_m3(), p.table_m3, p.carved_m3
            p.payload_m3 = 0.0
            self._execute(poses, joints)
            scooped = p.payload_g
            if joints is not None:
                self._joints = joints[1]
            reward = -abs(scooped - self.target_g) / self.target_g
            info.update(
                executed=True,
                scooped_g=scooped,
                carved_g=p.grams(p.carved_m3 - carved0),
                spilled_g=p.grams(p.carved_m3 - carved0) - scooped,
                table_g=p.grams(p.table_m3 - table0),
                bed_change_g=p.grams(p.bed.volume_m3() - bed0),
            )
            # The scooped powder leaves the cell (poured into RS3).
            p.payload_m3 = 0.0
        info["bed_g"] = p.bed_g
        terminated = p.bed.volume_m3() < self.planner.capacity_m3 * self.planner.params.empty_fill_ratio
        truncated = self._scoops >= self.max_scoops
        return self._obs(), float(reward), bool(terminated), bool(truncated), info
