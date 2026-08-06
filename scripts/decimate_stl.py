#!/usr/bin/env python3
"""Generic binary-STL decimator (voxel vertex clustering).

Reuses the same algorithm as scripts/decimate_ned3pro_meshes.py so tool
meshes outside niryo_robot_description stay reproducible without meshlab.

Example:
  python3 scripts/decimate_stl.py \\
    --input src/scoop_description/meshes/hires/schneider_scoop-ros.STL \\
    --output src/scoop_description/meshes/schneider_scoop-ros.STL \\
    --faces 3000
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path


def _load_ned3pro_helpers():
    """Import helpers from decimate_ned3pro_meshes without packaging them."""
    helper_path = Path(__file__).resolve().parent / "decimate_ned3pro_meshes.py"
    spec = importlib.util.spec_from_file_location(
        "decimate_ned3pro_meshes", helper_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"unable to load helpers from {helper_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="hires binary STL")
    parser.add_argument(
        "--output", required=True, type=Path, help="decimated binary STL destination"
    )
    parser.add_argument(
        "--faces",
        type=int,
        default=3000,
        help="target triangle count (default: 3000)",
    )
    args = parser.parse_args()

    if not args.input.is_file():
        print(f"input not found: {args.input}", file=sys.stderr)
        return 1
    if args.faces < 1:
        print("--faces must be >= 1", file=sys.stderr)
        return 1

    helpers = _load_ned3pro_helpers()
    verts, faces, normals = helpers.read_binary_stl(args.input)
    nv, nf, nn = helpers.voxel_decimate(verts, faces, normals, args.faces)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    helpers.write_binary_stl(args.output, nv, nf, nn)
    print(
        f"decimate {args.input.name}: {len(faces)} -> {len(nf)} faces "
        f"(target {args.faces}) -> {args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
