#!/usr/bin/env python3
"""Generate a printable ChArUco board image from the package board YAML."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
from ament_index_python.packages import get_package_share_directory

from camera_robot_calibration.charuco_board import (
    generate_board_image,
    load_board_config,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default="",
        help="Path to charuco_board.yaml (default: package share config)",
    )
    parser.add_argument(
        "--output",
        default="",
        help="Output PNG path (default: boards/charuco_generated.png under cwd)",
    )
    args = parser.parse_args(argv)

    config_path = args.config
    if not config_path:
        share = Path(get_package_share_directory("camera_robot_calibration"))
        config_path = str(share / "config" / "charuco_board.yaml")

    cfg = load_board_config(config_path)
    img = generate_board_image(cfg)
    output = Path(args.output) if args.output else Path("charuco_generated.png")
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), img):
        raise RuntimeError(f"Failed to write {output}")
    w_m = cfg["squares_x"] * cfg["square_length_m"]
    h_m = cfg["squares_y"] * cfg["square_length_m"]
    print(f"Wrote {output.resolve()}")
    print(
        f"Print at 100% scale. Physical size: "
        f"{w_m * 1000:.1f} mm x {h_m * 1000:.1f} mm "
        f"({cfg['dictionary']}, square={cfg['square_length_m'] * 1000:.0f} mm, "
        f"marker={cfg['marker_length_m'] * 1000:.0f} mm)."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
