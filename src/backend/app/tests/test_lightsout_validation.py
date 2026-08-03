"""Lights-out target/stop validation helpers."""

import pytest

from app.modes.lightsout_validate import resolve_lightsout_targets


def test_stop_value_required_for_duration():
    with pytest.raises(ValueError, match='stop_value'):
        resolve_lightsout_targets(
            target_mode='stratified',
            target_weight_g=None,
            stop_on='duration_min',
            stop_value=0,
            episodes=10,
        )


def test_stop_value_required_for_total_weight():
    with pytest.raises(ValueError, match='stop_value'):
        resolve_lightsout_targets(
            target_mode='fixed',
            target_weight_g=100,
            stop_on='total_weight_g',
            stop_value=None,
            episodes=5,
        )


def test_episodes_stop_ignores_stop_value():
    out = resolve_lightsout_targets(
        target_mode='fixed',
        target_weight_g=100,
        stop_on='episodes',
        stop_value=99,
        episodes=5,
    )
    assert out['stop_value'] == 0.0
    assert out['target_weight_g'] == 100.0


def test_stratified_falls_back_to_powder_default():
    out = resolve_lightsout_targets(
        target_mode='stratified',
        target_weight_g=None,
        stop_on='episodes',
        stop_value=0,
        episodes=5,
        powder_default_target_g=250.0,
    )
    assert out['target_weight_g'] == 250.0


def test_fixed_requires_target_weight():
    with pytest.raises(ValueError, match='target_weight_g'):
        resolve_lightsout_targets(
            target_mode='fixed',
            target_weight_g=0,
            stop_on='episodes',
            stop_value=0,
            episodes=5,
        )


def test_target_min_max_order():
    with pytest.raises(ValueError, match='target_min_g'):
        resolve_lightsout_targets(
            target_mode='stratified',
            target_weight_g=None,
            stop_on='episodes',
            stop_value=0,
            episodes=5,
            target_min_g=100,
            target_max_g=50,
        )
