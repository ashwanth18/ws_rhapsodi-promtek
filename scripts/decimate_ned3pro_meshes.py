#!/usr/bin/env python3
"""Decimate Ned3pro visual STLs and emit AABB collision boxes for the URDF.

Keeps originals under meshes/ned3pro/stl/hires/. Writes decimated visuals
in-place and a YAML of per-link collision boxes next to the URDF.

Uses pure-Python voxel vertex clustering (no meshlab/trimesh dependency).
"""
from __future__ import annotations

import struct
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
STL_DIR = ROOT / "src/niryo_robot_description/meshes/ned3pro/stl"
HIRES = STL_DIR / "hires"
OUT_YAML = ROOT / "src/niryo_robot_description/urdf/ned3pro/collision_boxes.yaml"

# Links referenced by the real URDF (mesh filename stem -> target face count).
LINK_TARGETS = {
    "base_link": 8000,
    "shoulder_link": 4000,
    "arm_link": 4000,
    "elbow_link": 4000,
    "forearm_link": 4000,
    "wrist_link": 2000,
    "hand_link": 800,
    "niryo_scoop_v4-ros": 2000,
}


def read_binary_stl(path: Path) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]], list[tuple[float, float, float]]]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"STL too small: {path}")
    if data[:5].lower() == b"solid" and b"facet" in data[:512].lower():
        raise ValueError(f"ASCII STL not supported here: {path}")
    n = struct.unpack_from("<I", data, 80)[0]
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []
    normals: list[tuple[float, float, float]] = []
    off = 84
    for i in range(n):
        vals = struct.unpack_from("<12fH", data, off)
        normals.append((vals[0], vals[1], vals[2]))
        base = len(verts)
        verts.append((vals[3], vals[4], vals[5]))
        verts.append((vals[6], vals[7], vals[8]))
        verts.append((vals[9], vals[10], vals[11]))
        faces.append((base, base + 1, base + 2))
        off += 50
    return verts, faces, normals


def write_binary_stl(
    path: Path,
    verts: list[tuple[float, float, float]],
    faces: list[tuple[int, int, int]],
    normals: list[tuple[float, float, float]] | None = None,
) -> None:
    header = b"decimated by scripts/decimate_ned3pro_meshes.py".ljust(80, b"\0")
    body = [header, struct.pack("<I", len(faces))]
    for i, (a, b, c) in enumerate(faces):
        if normals is not None and i < len(normals):
            nx, ny, nz = normals[i]
        else:
            ax, ay, az = verts[a]
            bx, by, bz = verts[b]
            cx, cy, cz = verts[c]
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = cx - ax, cy - ay, cz - az
            nx = uy * vz - uz * vy
            ny = uz * vx - ux * vz
            nz = ux * vy - uy * vx
            norm = (nx * nx + ny * ny + nz * nz) ** 0.5 or 1.0
            nx, ny, nz = nx / norm, ny / norm, nz / norm
        ax, ay, az = verts[a]
        bx, by, bz = verts[b]
        cx, cy, cz = verts[c]
        body.append(
            struct.pack(
                "<12fH",
                nx,
                ny,
                nz,
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                cx,
                cy,
                cz,
                0,
            )
        )
    path.write_bytes(b"".join(body))


def aabb(verts: list[tuple[float, float, float]]) -> dict[str, list[float]]:
    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    mn = [min(xs), min(ys), min(zs)]
    mx = [max(xs), max(ys), max(zs)]
    size = [max(mx[i] - mn[i], 1e-4) for i in range(3)]
    center = [(mn[i] + mx[i]) * 0.5 for i in range(3)]
    return {"origin_xyz": center, "size_xyz": size}


def voxel_decimate(
    verts: list[tuple[float, float, float]],
    faces: list[tuple[int, int, int]],
    normals: list[tuple[float, float, float]],
    target_faces: int,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]], list[tuple[float, float, float]]]:
    if len(faces) <= target_faces:
        return verts, faces, normals

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    mn = (min(xs), min(ys), min(zs))
    mx = (max(xs), max(ys), max(zs))
    diag = max(mx[i] - mn[i] for i in range(3)) or 1.0

    # Binary-search voxel size until face count is near the target.
    lo, hi = diag / 5000.0, diag / 2.0
    best = None
    for _ in range(18):
        mid = (lo + hi) * 0.5
        nv, nf, nn = _cluster(verts, faces, normals, mn, mid)
        best = (nv, nf, nn)
        if len(nf) > target_faces * 1.15:
            lo = mid
        else:
            hi = mid
    assert best is not None
    return best


def _cluster(
    verts: list[tuple[float, float, float]],
    faces: list[tuple[int, int, int]],
    normals: list[tuple[float, float, float]],
    mn: tuple[float, float, float],
    cell: float,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]], list[tuple[float, float, float]]]:
    cell = max(cell, 1e-9)
    buckets: dict[tuple[int, int, int], list[int]] = {}
    for i, (x, y, z) in enumerate(verts):
        key = (
            int((x - mn[0]) / cell),
            int((y - mn[1]) / cell),
            int((z - mn[2]) / cell),
        )
        buckets.setdefault(key, []).append(i)

    remap = [-1] * len(verts)
    new_verts: list[tuple[float, float, float]] = []
    for idxs in buckets.values():
        sx = sum(verts[i][0] for i in idxs) / len(idxs)
        sy = sum(verts[i][1] for i in idxs) / len(idxs)
        sz = sum(verts[i][2] for i in idxs) / len(idxs)
        nid = len(new_verts)
        new_verts.append((sx, sy, sz))
        for i in idxs:
            remap[i] = nid

    new_faces: list[tuple[int, int, int]] = []
    new_normals: list[tuple[float, float, float]] = []
    seen: set[tuple[int, int, int]] = set()
    for fi, (a, b, c) in enumerate(faces):
        na, nb, nc = remap[a], remap[b], remap[c]
        if na == nb or nb == nc or na == nc:
            continue
        face = tuple(sorted((na, nb, nc)))
        if face in seen:
            continue
        seen.add(face)
        new_faces.append((na, nb, nc))
        new_normals.append(normals[fi] if fi < len(normals) else (0.0, 0.0, 1.0))
    return new_verts, new_faces, new_normals


def main() -> int:
    HIRES.mkdir(parents=True, exist_ok=True)
    boxes: dict[str, dict] = {}
    for stem, faces_target in LINK_TARGETS.items():
        hires = HIRES / f"{stem}.STL"
        visual = STL_DIR / f"{stem}.STL"
        if not hires.exists():
            if not visual.exists():
                print(f"skip missing {stem}", file=sys.stderr)
                continue
            hires.write_bytes(visual.read_bytes())
        verts, faces, normals = read_binary_stl(hires)
        scale = 0.001 if stem.startswith("niryo_scoop") else 1.0
        scaled = [(v[0] * scale, v[1] * scale, v[2] * scale) for v in verts]
        boxes[stem] = aabb(scaled)

        nv, nf, nn = voxel_decimate(verts, faces, normals, faces_target)
        write_binary_stl(visual, nv, nf, nn)
        print(
            f"  decimate {stem}: {len(faces)} -> {len(nf)} faces "
            f"(target {faces_target})"
        )
    OUT_YAML.write_text(
        yaml.safe_dump(
            {
                "comment": (
                    "AABB collision boxes derived from hires STLs. "
                    "Used by niryo_ned3pro.urdf.xacro. Regenerated by "
                    "scripts/decimate_ned3pro_meshes.py."
                ),
                "links": boxes,
            },
            sort_keys=False,
        )
    )
    print(f"wrote {OUT_YAML}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
