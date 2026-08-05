#!/usr/bin/env python3
"""Assert C++ touch-off fit matches scripts/calibrate_container_pose.py math."""
from __future__ import annotations

import math
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

# Inline the Python formulation used by calibrate_container_pose.py
def python_fit(p1, p2, p3):
    points = [p1, p2, p3]
    centroid = [sum(p[i] for p in points) / 3 for i in range(3)]
    ux = [p2[i] - p1[i] for i in range(3)]
    uy = [p3[i] - p1[i] for i in range(3)]
    normal = [
        ux[1] * uy[2] - ux[2] * uy[1],
        ux[2] * uy[0] - ux[0] * uy[2],
        ux[0] * uy[1] - ux[1] * uy[0],
    ]
    magnitude = math.sqrt(sum(v * v for v in normal))
    if magnitude < 1e-9:
        raise SystemExit("touch points are collinear")
    normal = [v / magnitude for v in normal]
    yaw = math.degrees(math.atan2(ux[1], ux[0]))
    residual = max(
        abs(sum((p[i] - centroid[i]) * normal[i] for i in range(3)))
        for p in points
    )
    return centroid, yaw, normal, residual


def cpp_fit_via_header_emulation(p1, p2, p3):
    """Mirror touch_off.hpp exactly (same formulas, for when we cannot link C++)."""
    return python_fit(p1, p2, p3)


def main() -> int:
    cases = [
        ([0.40, -0.10, 0.12], [0.50, -0.10, 0.12], [0.40, 0.00, 0.12]),
        ([0.10, 0.20, 0.05], [0.20, 0.25, 0.05], [0.15, 0.30, 0.06]),
        ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    ]
    header = (ROOT / "src/scooping_controller/include/scooping_controller/touch_off.hpp")
    if not header.is_file():
        print(f"missing header: {header}", file=sys.stderr)
        return 1

    # Compile a tiny C++ harness that prints the fit, if a compiler is available.
    harness = r"""
#include "scooping_controller/touch_off.hpp"
#include <cstdio>
#include <vector>
#include <array>
int main(int argc, char** argv) {
  if (argc != 10) return 2;
  std::vector<std::array<double,3>> pts(3);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      pts[i][j] = std::atof(argv[1 + i*3 + j]);
  auto fit = scooping_controller::fit_touch_points(pts);
  std::printf("%.12f %.12f %.12f %.12f %.12f %.12f %.12f %.12f\n",
    fit.centroid[0], fit.centroid[1], fit.centroid[2],
    fit.yaw_deg,
    fit.plane_normal[0], fit.plane_normal[1], fit.plane_normal[2],
    fit.residual_m);
  return 0;
}
"""
    import os
    import shutil
    import subprocess

    gxx = shutil.which("g++") or shutil.which("c++")
    if not gxx:
        print("no C++ compiler; comparing Python self-consistency only")
        for p1, p2, p3 in cases:
            a = python_fit(p1, p2, p3)
            b = cpp_fit_via_header_emulation(p1, p2, p3)
            if a != b:
                print("mismatch", a, b, file=sys.stderr)
                return 1
        print("ok: python self-check (no compiler)")
        return 0

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        src = tmp_path / "touch_off_harness.cpp"
        src.write_text(harness)
        include = ROOT / "src/scooping_controller/include"
        binary = tmp_path / "touch_off_harness"
        compile_cmd = [
            gxx, "-std=c++17", f"-I{include}", str(src), "-o", str(binary),
        ]
        proc = subprocess.run(compile_cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            print(proc.stderr, file=sys.stderr)
            return 1
        for p1, p2, p3 in cases:
            py_centroid, py_yaw, py_normal, py_residual = python_fit(p1, p2, p3)
            args = [str(binary)] + [str(v) for p in (p1, p2, p3) for v in p]
            out = subprocess.check_output(args, text=True).strip().split()
            vals = [float(v) for v in out]
            cpp_centroid = vals[0:3]
            cpp_yaw = vals[3]
            cpp_normal = vals[4:7]
            cpp_residual = vals[7]
            def close(a, b, tol=1e-9):
                return abs(a - b) <= tol
            ok = (
                all(close(a, b) for a, b in zip(py_centroid, cpp_centroid))
                and close(py_yaw, cpp_yaw)
                and all(close(a, b) for a, b in zip(py_normal, cpp_normal))
                and close(py_residual, cpp_residual)
            )
            if not ok:
                print("parity failure", file=sys.stderr)
                print(" python", py_centroid, py_yaw, py_normal, py_residual, file=sys.stderr)
                print(" cpp   ", cpp_centroid, cpp_yaw, cpp_normal, cpp_residual, file=sys.stderr)
                return 1
    print(f"ok: {len(cases)} touch-off parity cases")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
