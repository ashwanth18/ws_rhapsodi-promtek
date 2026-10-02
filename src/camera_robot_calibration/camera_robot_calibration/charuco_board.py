"""Shared ChArUco board config loading and OpenCV board construction."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import yaml


DEFAULT_BOARD = {
    "squares_x": 5,
    "squares_y": 7,
    "square_length_m": 0.030,
    "marker_length_m": 0.022,
    "dictionary": "DICT_5X5_250",
    "legacy_pattern": False,
    "print_dpi": 300,
}


def load_board_config(path: str | Path | None = None) -> dict[str, Any]:
    cfg = dict(DEFAULT_BOARD)
    if path:
        with open(path, "r", encoding="utf-8") as f:
            loaded = yaml.safe_load(f) or {}
        if not isinstance(loaded, dict):
            raise ValueError(f"Board config must be a mapping: {path}")
        cfg.update(loaded)
    validate_board_config(cfg)
    return cfg


def validate_board_config(cfg: dict[str, Any]) -> None:
    required = (
        "squares_x",
        "squares_y",
        "square_length_m",
        "marker_length_m",
        "dictionary",
    )
    for key in required:
        if key not in cfg:
            raise ValueError(f"Missing board config key: {key}")
    if int(cfg["squares_x"]) < 2 or int(cfg["squares_y"]) < 2:
        raise ValueError("squares_x and squares_y must be >= 2")
    if float(cfg["square_length_m"]) <= 0 or float(cfg["marker_length_m"]) <= 0:
        raise ValueError("square_length_m and marker_length_m must be > 0")
    if float(cfg["marker_length_m"]) >= float(cfg["square_length_m"]):
        raise ValueError("marker_length_m must be smaller than square_length_m")
    dict_name = str(cfg["dictionary"])
    if not hasattr(cv2.aruco, dict_name):
        raise ValueError(f"Unknown OpenCV aruco dictionary: {dict_name}")


def make_dictionary(cfg: dict[str, Any]):
    dict_id = getattr(cv2.aruco, str(cfg["dictionary"]))
    return cv2.aruco.getPredefinedDictionary(dict_id)


def make_charuco_board(cfg: dict[str, Any]):
    dictionary = make_dictionary(cfg)
    board = cv2.aruco.CharucoBoard(
        (int(cfg["squares_x"]), int(cfg["squares_y"])),
        float(cfg["square_length_m"]),
        float(cfg["marker_length_m"]),
        dictionary,
    )
    if bool(cfg.get("legacy_pattern", False)) and hasattr(board, "setLegacyPattern"):
        board.setLegacyPattern(True)
    return board, dictionary


def board_image_size_px(cfg: dict[str, Any]) -> tuple[int, int]:
    dpi = int(cfg.get("print_dpi", 300))
    w_m = float(cfg["squares_x"]) * float(cfg["square_length_m"])
    h_m = float(cfg["squares_y"]) * float(cfg["square_length_m"])
    px_w = int(round(w_m / 0.0254 * dpi))
    px_h = int(round(h_m / 0.0254 * dpi))
    return px_w, px_h


def generate_board_image(cfg: dict[str, Any]):
    board, _ = make_charuco_board(cfg)
    px_w, px_h = board_image_size_px(cfg)
    return board.generateImage((px_w, px_h), marginSize=0, borderBits=1)
