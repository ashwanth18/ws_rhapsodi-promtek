"""Cell layout YAML: find the task container, propose a camera-fitted pose."""

from __future__ import annotations

import math
import os
import re
import time

import yaml


def resolve_scene_path(scene_yaml_path: str, layouts_dir: str = "") -> str:
    """Path to read the layout from on this machine.

    ``/cell_layout/active`` carries the path as the publisher sees it, e.g.
    ``/ws/config/layouts/<id>.yaml`` inside the ros-prod container. A node
    running natively on the laptop falls back to ``layouts_dir/<file>``.
    """
    if os.path.isfile(scene_yaml_path):
        return scene_yaml_path
    if layouts_dir:
        local = os.path.join(layouts_dir, os.path.basename(scene_yaml_path))
        if os.path.isfile(local):
            return local
    hint = f" or {layouts_dir}" if layouts_dir else " (set the layouts_dir parameter)"
    raise FileNotFoundError(f"Layout {scene_yaml_path} not found here{hint}")


def task_container_mesh(scene_yaml_path: str, task_container_id: str) -> tuple[str, float]:
    """``(mesh_resource, metres_per_mesh_unit)`` of the task container."""
    with open(scene_yaml_path, encoding="utf-8") as fh:
        root = yaml.safe_load(fh)
    obj = next((o for o in root["objects"] if o["id"] == task_container_id), None)
    if obj is None:
        raise ValueError(f"Task container {task_container_id} not in {scene_yaml_path}")
    if obj.get("geometry_type") != "mesh":
        raise ValueError(f"Task container {task_container_id} is not a mesh")
    scale = float(obj.get("scale_xyz", [1.0])[0])
    if obj.get("mesh_units") == "mm":
        scale *= 0.001
    return obj["mesh_resource"], scale


# --------------------------------------------------------------------------
# Camera-based container calibration: write a *proposal*, never apply it.

_MARKERS = (
    ("approach_marker", "Approach"),
    ("contact_marker", "Contact"),
    ("scoop_marker", "Scoop"),
    ("lift_marker", "Lift"),
    ("transport_ready_marker", "Transport Ready"),
)


def _yaw_deg(orientation: dict) -> float:
    if "rpy_deg" in orientation:
        roll, pitch, yaw = (float(v) for v in orientation["rpy_deg"])
    else:
        x, y, z, w = (float(v) for v in orientation["quat_xyzw"])
        roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
        pitch = math.degrees(math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))))
        yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    if abs(roll) > 0.5 or abs(pitch) > 0.5:
        raise ValueError("Container calibration assumes a level bin (roll = pitch = 0)")
    return yaw


def corrected_container_pose(obj: dict, dx: float, dy: float, dz: float, dyaw_deg: float):
    """New ``(position_xyz, yaw_deg)`` for a layout object.

    ``(dx, dy, dz, dyaw)`` is the real bin relative to the layout, expressed in
    the container frame: ``real = layout · Trans(d) · Rz(dyaw)``.
    """
    yaw0 = _yaw_deg(obj["orientation"])
    c, s = math.cos(math.radians(yaw0)), math.sin(math.radians(yaw0))
    x, y, z = (float(v) for v in obj["position_xyz"])
    position = [float(x + c * dx - s * dy), float(y + s * dx + c * dy), float(z + dz)]
    yaw = float((yaw0 + dyaw_deg + 180.0) % 360.0 - 180.0)
    return position, yaw


def reanchor_poses(poses, dx: float, dy: float, dz: float, dyaw_deg: float):
    """Express container-frame poses in the corrected frame, same world pose.

    ``poses`` are ``(position_xyz, quat_xyzw)`` tuples.
    """
    th = math.radians(dyaw_deg)
    c, s = math.cos(-th), math.sin(-th)
    qz = (0.0, 0.0, math.sin(-th / 2), math.cos(-th / 2))
    out = []
    for (px, py, pz), q in poses:
        ux, uy = px - dx, py - dy
        nq = _quat_mul(qz, q)
        out.append(((c * ux - s * uy, s * ux + c * uy, pz - dz), nq))
    return out


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def write_layout_proposal(
    layout_path: str,
    task_container_id: str,
    fit,
    out_dir: str,
    *,
    current_poses=None,
    poses_meta: dict | None = None,
) -> dict:
    """Write ``<out_dir>/<layout file>`` (+ re-anchored poses) and return paths.

    Only the task container's ``position_xyz`` / ``orientation`` change; every
    other line (and comment) of the layout is kept as-is.
    """
    with open(layout_path, encoding="utf-8") as fh:
        text = fh.read()
    root = yaml.safe_load(text)
    obj = next(o for o in root["objects"] if o["id"] == task_container_id)
    position, yaw = corrected_container_pose(
        obj, fit.shift_x_m, fit.shift_y_m, fit.z_offset_m, fit.yaw_offset_deg
    )
    pos_text = "[" + ", ".join(f"{v:.4f}" for v in position) + "]"
    ori_text = "{rpy_deg: [0, 0, " + f"{yaw:.2f}" + "]}"
    stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
    note = (
        f"  # scoop_vision rim fit {stamp}: {task_container_id} moved by "
        f"({fit.shift_x_m * 1000:+.0f}, {fit.shift_y_m * 1000:+.0f}, {fit.z_offset_m * 1000:+.0f}) mm, "
        f"{fit.yaw_offset_deg:+.1f} deg yaw (container frame) from position_xyz "
        f"{obj['position_xyz']}; rim error {fit.rim_mad_at_best_m * 1000:.1f} mm.\n"
    )
    line_re = re.compile(rf"^(\s*-\s*\{{\s*id:\s*{re.escape(task_container_id)}\s*,.*)$", re.M)
    m = line_re.search(text)
    if m:
        line = m.group(1)
        line = re.sub(r"position_xyz:\s*\[[^\]]*\]", f"position_xyz: {pos_text}", line)
        line = re.sub(r"orientation:\s*\{[^}]*\}", f"orientation: {ori_text}", line)
        new_text = text[: m.start()] + note + line + text[m.end():]
    else:  # block-style YAML: lose comments, keep content
        obj["position_xyz"] = position
        obj["orientation"] = {"rpy_deg": [0, 0, round(yaw, 2)]}
        new_text = note + yaml.safe_dump(root, sort_keys=False)

    os.makedirs(out_dir, exist_ok=True)
    out = {"layout": os.path.join(out_dir, os.path.basename(layout_path)),
           "position_xyz": position, "yaw_deg": yaw}
    with open(out["layout"], "w", encoding="utf-8") as fh:
        fh.write(new_text)

    if current_poses:
        meta = dict(poses_meta or {})
        moved = reanchor_poses(
            current_poses, fit.shift_x_m, fit.shift_y_m, fit.z_offset_m, fit.yaw_offset_deg
        )
        doc = {
            "layout_id": meta.get("layout_id", root.get("layout_id", "")),
            "task_container_id": task_container_id,
            "frame_id": meta.get("frame_id", "scooping_container_frame"),
            "tool_id": meta.get("tool_id", root.get("tool_id", "")),
            # Restamped by the marker server when saved against the new layout.
            "container_spec_hash": "",
            "authored_in": meta.get("authored_in", "real"),
            "markers": [
                {
                    "name": name,
                    "label": label,
                    "pose": {
                        "position": dict(zip("xyz", (round(float(v), 6) for v in p))),
                        "orientation": dict(zip("xyzw", (round(float(v), 9) for v in q))),
                    },
                }
                for (name, label), (p, q) in zip(_MARKERS, moved)
            ],
        }
        out["poses_keep_world_path"] = os.path.join(out_dir, "poses_keep_world_path.yaml")
        with open(out["poses_keep_world_path"], "w", encoding="utf-8") as fh:
            fh.write(
                "# Same scoop path in the world as before the container correction:\n"
                "# the old poses re-expressed in the corrected container frame.\n"
                "# Use only if the poses were taught while the layout was wrong.\n"
            )
            yaml.safe_dump(doc, fh, sort_keys=False)
    return out
