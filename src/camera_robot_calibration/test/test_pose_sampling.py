"""Unit tests for MoveIt hand-eye pose sampling."""

from __future__ import annotations

import pytest

from camera_robot_calibration.pose_sampling import SamplePose, generate_sample_poses


def _seed() -> SamplePose:
    return SamplePose(0.3, 0.0, 0.25, 0.0, 0.0, 0.0, 1.0)


def test_generate_exact_count():
    poses = generate_sample_poses(_seed(), num_samples=10, include_seed=True)
    assert len(poses) == 10


def test_generate_includes_seed_first():
    seed = _seed()
    poses = generate_sample_poses(seed, num_samples=5, include_seed=True)
    assert poses[0].x == pytest.approx(seed.x)
    assert poses[0].qw == pytest.approx(1.0)


def test_num_samples_validation():
    with pytest.raises(ValueError):
        generate_sample_poses(_seed(), num_samples=0)


def test_rotations_change_orientation():
    poses = generate_sample_poses(
        _seed(), num_samples=8, translation_m=0.0, include_seed=True
    )
    assert any(p.qx != 0.0 or p.qy != 0.0 or p.qz != 0.0 for p in poses[1:])
