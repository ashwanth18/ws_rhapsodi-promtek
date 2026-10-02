"""STL loading, top-down rasterisation and surface sampling (numpy only)."""

from __future__ import annotations

import os
import struct

import numpy as np


def resolve_resource(uri: str) -> str:
    """Turn ``package://pkg/path`` or ``file://path`` into a filesystem path."""
    if uri.startswith("package://"):
        rest = uri[len("package://"):]
        pkg, _, rel = rest.partition("/")
        from ament_index_python.packages import get_package_share_directory

        return os.path.join(get_package_share_directory(pkg), rel)
    if uri.startswith("file://"):
        return uri[len("file://"):]
    return uri


def load_stl(path: str) -> np.ndarray:
    """Return triangles as an ``(n, 3, 3)`` float64 array in file units."""
    with open(path, "rb") as fh:
        data = fh.read()
    if len(data) >= 84:
        n = struct.unpack("<I", data[80:84])[0]
        if 84 + n * 50 == len(data):
            rec = np.frombuffer(
                data[84:84 + n * 50],
                dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]),
            )
            return rec["v"].reshape(-1, 3, 3).astype(np.float64)
    verts = [
        [float(t) for t in line.split()[1:4]]
        for line in data.decode("ascii", errors="ignore").splitlines()
        if line.strip().startswith("vertex")
    ]
    if not verts or len(verts) % 3:
        raise ValueError(f"Could not parse STL: {path}")
    return np.asarray(verts, dtype=np.float64).reshape(-1, 3, 3)


def transform_triangles(tris: np.ndarray, rot: np.ndarray, trans) -> np.ndarray:
    return tris @ rot.T + np.asarray(trans, dtype=np.float64)


def rasterize_top(
    tris: np.ndarray,
    x0: float,
    y0: float,
    cell: float,
    nx: int,
    ny: int,
) -> np.ndarray:
    """Highest mesh surface z per grid cell centre (NaN where no triangle).

    Cell ``(i, j)`` has centre ``(x0 + (i + 0.5) * cell, y0 + (j + 0.5) * cell)``.
    Vertical faces have no top-down area and are skipped; closed meshes
    always carry a top face above them.
    """
    top = np.full((nx, ny), -np.inf)
    for tri in tris:
        a, b, c = tri
        det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(det) < 1e-12:
            continue
        lo = np.minimum(np.minimum(a, b), c)
        hi = np.maximum(np.maximum(a, b), c)
        i0 = max(int(np.floor((lo[0] - x0) / cell - 0.5)), 0)
        i1 = min(int(np.ceil((hi[0] - x0) / cell - 0.5)), nx - 1)
        j0 = max(int(np.floor((lo[1] - y0) / cell - 0.5)), 0)
        j1 = min(int(np.ceil((hi[1] - y0) / cell - 0.5)), ny - 1)
        if i1 < i0 or j1 < j0:
            continue
        xs = x0 + (np.arange(i0, i1 + 1) + 0.5) * cell
        ys = y0 + (np.arange(j0, j1 + 1) + 0.5) * cell
        px, py = np.meshgrid(xs, ys, indexing="ij")
        l1 = ((b[1] - c[1]) * (px - c[0]) + (c[0] - b[0]) * (py - c[1])) / det
        l2 = ((c[1] - a[1]) * (px - c[0]) + (a[0] - c[0]) * (py - c[1])) / det
        l3 = 1.0 - l1 - l2
        eps = -1e-9
        inside = (l1 >= eps) & (l2 >= eps) & (l3 >= eps)
        if not inside.any():
            continue
        z = l1 * a[2] + l2 * b[2] + l3 * c[2]
        sub = top[i0:i1 + 1, j0:j1 + 1]
        np.maximum(sub, np.where(inside, z, -np.inf), out=sub)
    top[np.isneginf(top)] = np.nan
    return top


def sample_surface(tris: np.ndarray, spacing: float, seed: int = 0) -> np.ndarray:
    """Points on the mesh surface, roughly ``spacing`` apart, plus all vertices."""
    rng = np.random.default_rng(seed)
    a, b, c = tris[:, 0], tris[:, 1], tris[:, 2]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    counts = np.ceil(area / (spacing * spacing)).astype(int)
    idx = np.repeat(np.arange(len(tris)), counts)
    r1 = np.sqrt(rng.random(len(idx)))
    r2 = rng.random(len(idx))
    pts = (
        (1.0 - r1)[:, None] * a[idx]
        + (r1 * (1.0 - r2))[:, None] * b[idx]
        + (r1 * r2)[:, None] * c[idx]
    )
    return np.vstack([pts, tris.reshape(-1, 3)])


def voxel_dedupe(points: np.ndarray, voxel: float) -> np.ndarray:
    """Keep one point per ``voxel`` cube (first occurrence)."""
    if len(points) == 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(keep)]
