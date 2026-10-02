"""Unit tests for ChArUco board config validation."""

from __future__ import annotations

import copy

import pytest

from camera_robot_calibration.charuco_board import (
    DEFAULT_BOARD,
    load_board_config,
    validate_board_config,
)


def test_default_board_valid():
    validate_board_config(DEFAULT_BOARD)


def test_marker_must_be_smaller_than_square():
    cfg = copy.deepcopy(DEFAULT_BOARD)
    cfg["marker_length_m"] = cfg["square_length_m"]
    with pytest.raises(ValueError, match="marker_length_m"):
        validate_board_config(cfg)


def test_unknown_dictionary():
    cfg = copy.deepcopy(DEFAULT_BOARD)
    cfg["dictionary"] = "DICT_NOT_REAL"
    with pytest.raises(ValueError, match="dictionary"):
        validate_board_config(cfg)


def test_load_none_returns_defaults():
    cfg = load_board_config(None)
    assert cfg["squares_x"] == 5
    assert cfg["squares_y"] == 7
