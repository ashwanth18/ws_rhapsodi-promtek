"""Shared fixtures: real RS6 / scoop meshes and the dual-container poses."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest
import yaml

PKG = Path(__file__).resolve().parents[1]
SRC = PKG.parent
REPO = SRC.parent
sys.path.insert(0, str(PKG))

from scoop_vision.container import ContainerModel  # noqa: E402
from scoop_vision.heightmap import SurfaceMap  # noqa: E402
from scoop_vision.mesh import load_stl  # noqa: E402
from scoop_vision.scoop_tool import ScoopTool  # noqa: E402
from scoop_vision.transforms import Pose  # noqa: E402

RS6 = SRC / "scooping_controller/models/scooping_container/meshes/rs6_container.STL"
SCOOP = SRC / "niryo_robot_description/meshes/ned3pro/stl/niryo_scoop_v4-ros.STL"
POSES = REPO / "config/layouts/dual-container/poses.yaml"
NIRYO_TCP = [0.15825, 0.0, -0.09356]


@pytest.fixture(scope="session")
def rs6() -> ContainerModel:
    return ContainerModel(load_stl(str(RS6)) * 0.001)


@pytest.fixture(scope="session")
def scoop() -> ScoopTool:
    return ScoopTool(load_stl(str(SCOOP)) * 0.001, NIRYO_TCP)


@pytest.fixture(scope="session")
def authored_poses() -> list[Pose]:
    doc = yaml.safe_load(POSES.read_text())
    return [
        Pose(
            tuple(m["pose"]["position"][k] for k in "xyz"),
            tuple(m["pose"]["orientation"][k] for k in "xyzw"),
        )
        for m in doc["markers"]
    ]


def make_surface(container: ContainerModel, height: np.ndarray) -> SurfaceMap:
    h = np.where(container.interior, np.maximum(height, container.floor_map), np.nan)
    return SurfaceMap(h, container.interior.copy(), 1.0, 1)
