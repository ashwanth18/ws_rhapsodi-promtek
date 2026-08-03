"""Tests for powder catalog and target sampler."""
from pathlib import Path

import pytest

from app.modes.powders import get_powder, load_powders, validate_powder_id
from app.modes.target_sampler import generate_target_schedule
from app.run_spec import Environment, OperatingMode, RunLabel, RunSpec


def test_validate_rejects_numeric_powder_id():
    with pytest.raises(ValueError, match='numeric'):
        validate_powder_id('1327')


def test_load_powders_from_repo_catalog():
    catalog = Path(__file__).resolve().parents[4] / 'config' / 'powders.yaml'
    powders = load_powders(catalog)
    assert powders
    ids = {p.id for p in powders}
    assert 'alumina-5um' in ids
    powder = get_powder('alumina-5um', catalog)
    assert powder.container_target
    assert powder.pour_target


def test_stratified_schedule_covers_bins_and_is_reproducible():
    a = generate_target_schedule(
        cycles=5, target_mode='stratified', frac_min=0.4, frac_max=0.9, rng_seed=42
    )
    b = generate_target_schedule(
        cycles=5, target_mode='stratified', frac_min=0.4, frac_max=0.9, rng_seed=42
    )
    assert a.fractions == b.fractions
    assert len(a.fractions) == 5
    assert all(0.4 <= f <= 0.9 for f in a.fractions)


def test_fixed_schedule_requires_target():
    with pytest.raises(ValueError):
        generate_target_schedule(cycles=3, target_mode='fixed')
    sched = generate_target_schedule(
        cycles=3, target_mode='fixed', fixed_target_g=100.0, rng_seed=1
    )
    assert sched.fixed_target_g == 100.0
    assert sched.fractions == [1.0, 1.0, 1.0]


def test_run_label_in_metadata_dict():
    spec = RunSpec(
        mode=OperatingMode.LIGHTSOUT,
        environment=Environment.REAL,
        run_key='r1',
        target_weight_g=100.0,
        tolerance_g=2.0,
        label=RunLabel(
            powder_id='alumina-5um',
            powder_name='Alumina 5 um',
            cycles=10,
            target_mode='stratified',
            rng_seed=7,
        ),
    )
    meta = spec.to_metadata_dict()
    assert meta['powder_id'] == 'alumina-5um'
    assert meta['label']['cycles'] == 10
    assert meta['rng_seed'] == 7
