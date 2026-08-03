"""Phase 3: mode registry, arbitration, NullMesClient."""

from __future__ import annotations

import logging
from types import SimpleNamespace

import pytest

from app.mes_client import NullMesClient, get_mes_client
from app.modes.manager import (
    ModeManager,
    ModeSwitchConflict,
    ModeValidationError,
)
from app.modes.registry import build_default_registry
from app.modes.state import RuntimeModeState, sim_allowed
from app.run_spec import OperatingMode, RunSpec


def test_registry_contains_all_modes():
    registry = build_default_registry()
    ids = set(registry.ids())
    assert ids == {
        OperatingMode.MES_CONDOR.value,
        OperatingMode.MES_GENERIC.value,
        OperatingMode.MOCK_LOCAL.value,
        OperatingMode.LIGHTSOUT.value,
    }
    capabilities = registry.capability_modes()
    assert [c['mode'] for c in capabilities] == [
        'mes-condor',
        'mes-generic',
        'mock-local',
        'lightsout',
    ]
    for adapter in registry.modes():
        plan = adapter.build_plan(
            RunSpec(
                mode=adapter.mode,
                run_key='test-run',
                target_weight_g=100.0,
                tolerance_g=2.0,
            )
        )
        assert plan.tree_id
        assert adapter.job_source().poll() is None
        assert adapter.result_sink() is not None


def test_mode_switch_refused_when_active_run(tmp_path, monkeypatch):
    monkeypatch.setenv('SIM_ALLOWED', '0')
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)

    assert manager.current()['mode'] == 'mes-condor'

    active = SimpleNamespace(id=42, status='running', weightment_id=7)
    with pytest.raises(ModeSwitchConflict) as exc_info:
        manager.set_mode('mock-local', 'real', active_run=active)
    assert exc_info.value.active_run is active
    # Persisted mode must not change on conflict.
    assert manager.current()['mode'] == 'mes-condor'


def test_mode_switch_refused_when_awaiting_processing(tmp_path, monkeypatch):
    """Bugbot: awaiting_processing must block switches (MES still in flight)."""
    monkeypatch.setenv('SIM_ALLOWED', '0')
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    pending = SimpleNamespace(
        id=43, status='awaiting_processing', weightment_id=8
    )
    with pytest.raises(ModeSwitchConflict):
        manager.set_mode('mock-local', 'real', active_run=pending)
    assert manager.current()['mode'] == 'mes-condor'


def test_mode_switch_succeeds_when_idle(tmp_path, monkeypatch):
    monkeypatch.setenv('SIM_ALLOWED', '0')
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)

    updated = manager.set_mode('mock-local', 'real', active_run=None)
    assert updated == {'mode': 'mock-local', 'environment': 'real'}
    assert manager.current() == updated
    # Durable on disk.
    reloaded = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    assert reloaded.load() == updated


def test_sim_rejected_when_not_allowed(tmp_path, monkeypatch):
    monkeypatch.setenv('SIM_ALLOWED', '0')
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    with pytest.raises(ModeValidationError):
        manager.set_mode('mock-local', 'sim', active_run=None)
    assert not sim_allowed()


def test_capabilities_shape(tmp_path, monkeypatch):
    monkeypatch.setenv('SIM_ALLOWED', '0')
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    caps = manager.capabilities()
    assert caps['default_mode'] == 'mes-condor'
    assert caps['sim_allowed'] is False
    assert len(caps['modes']) == 4


def test_null_mes_client_noop(caplog):
    client = NullMesClient()
    with caplog.at_level(logging.INFO):
        weighment = client.post_weighment({'batchId': 1})
        batch_end = client.post_batch_end({'batchId': 1, 'endUtc': 't'})
        timeseries = client.post_timeseries({'batchId': '1', 'items': []})
    assert weighment['null'] is True and weighment['skipped'] is True
    assert batch_end['null'] is True
    assert timeseries['null'] is True
    assert get_mes_client('mock-local').__class__ is NullMesClient
    assert get_mes_client('lightsout').__class__ is NullMesClient
    assert get_mes_client(OperatingMode.MES_CONDOR).__class__.__name__ == (
        'CondorMesClient'
    )
