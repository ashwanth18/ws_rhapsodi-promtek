"""Phase 3–6: mode registry, arbitration, NullMesClient, mock/lightsout/mes-generic."""

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
from app.modes.lightsout_session import reset_lightsout_session_for_tests
from app.modes.mock_local import (
    MOCK_DEFAULT_PICKUP_TARGET,
    MOCK_DEFAULT_RETURN_TARGET,
    MOCK_DEFAULT_WEIGH_TARGET,
    MOCK_EVENT_ID_PREFIX,
    allocate_mock_batch_id,
    is_mock_event_id,
)
from app.modes.registry import build_default_registry
from app.modes.state import RuntimeModeState, sim_allowed
from app.run_spec import OperatingMode, RunSpec
from app.schemas import LightsoutRunRequest, MockLocalRunRequest


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
    # mes-generic defaults to CondorMesClient (MES_GENERIC_SINK=condor).
    assert get_mes_client(OperatingMode.MES_GENERIC).__class__.__name__ == (
        'CondorMesClient'
    )


def test_is_mock_event_id():
    assert is_mock_event_id(f'{MOCK_EVENT_ID_PREFIX}abc')
    assert is_mock_event_id('mock-1234-5678')
    assert not is_mock_event_id('webhook-abc')
    assert not is_mock_event_id(None)
    assert not is_mock_event_id('')


def test_allocate_mock_batch_id_is_negative():
    """Mock batch ids must be negative ints so they cannot collide with Condor."""
    batch_id = allocate_mock_batch_id('12345678-1234-5678-1234-567812345678')
    assert int(batch_id) < 0
    other = allocate_mock_batch_id('aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee')
    assert int(other) < 0
    assert batch_id != other


def test_mock_local_run_request_schema():
    req = MockLocalRunRequest(target_weight_g=100.0)
    assert req.target_weight_g == 100.0
    assert req.tolerance_g is None
    assert req.location_code is None
    full = MockLocalRunRequest(
        target_weight_g=250.5,
        tolerance_g=2.0,
        location_code='LOC-A',
        pickup_target_name=MOCK_DEFAULT_PICKUP_TARGET,
        weigh_target_name=MOCK_DEFAULT_WEIGH_TARGET,
        return_target_name=MOCK_DEFAULT_RETURN_TARGET,
    )
    assert full.location_code == 'LOC-A'
    assert full.pickup_target_name == 'MoveToScoopingContainer'


def test_mock_local_mode_defaults_match_webhook_tree():
    """Mock-local shares WebhookWeightment tree identity with MES family."""
    adapter = build_default_registry().get(OperatingMode.MOCK_LOCAL)
    plan = adapter.build_plan(
        RunSpec(
            mode=OperatingMode.MOCK_LOCAL,
            run_key='mock-test',
            target_weight_g=100.0,
            tolerance_g=2.0,
            location_code='MOCK',
        )
    )
    assert plan.tree_id == 'WebhookWeightment'
    assert plan.blackboard['target_weight_g'] == 100.0


def test_lightsout_run_request_schema():
    req = LightsoutRunRequest(
        powder_name='boxA',
        target_weight_g=250.0,
        episodes=10,
    )
    assert req.enable_scoop is False
    assert req.batch_id == ''
    full = LightsoutRunRequest(
        powder_name='alumina',
        target_weight_g=125.0,
        episodes=5,
        batch_id='batch-1',
        cycle_end_limit='10 episodes',
        enable_scoop=True,
    )
    assert full.enable_scoop is True
    assert full.powder_name == 'alumina'


def test_lightsout_mode_defaults_match_lightsout_tree():
    adapter = build_default_registry().get(OperatingMode.LIGHTSOUT)
    plan = adapter.build_plan(
        RunSpec(
            mode=OperatingMode.LIGHTSOUT,
            run_key='lightsout-test',
            target_weight_g=250.0,
            tolerance_g=5.0,
        )
    )
    assert plan.tree_id == 'LightsOut'
    assert plan.blackboard['enable_scoop'] is False
    assert plan.blackboard['target_weight_g'] == 250.0


def test_lightsout_session_blocks_mode_switch(tmp_path, monkeypatch):
    monkeypatch.setenv('SIM_ALLOWED', '0')
    session = reset_lightsout_session_for_tests(path=tmp_path / 'lo.json')
    session.mark_started({'powder_name': 'boxA', 'episodes': 3})
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    manager.set_mode('lightsout', 'real', active_run=None)
    with pytest.raises(ModeSwitchConflict):
        manager.set_mode(
            'mock-local', 'real', active_run=session.as_blocker()
        )
    assert manager.current()['mode'] == 'lightsout'
    session.clear()
    assert session.get_active() is None
    updated = manager.set_mode('mock-local', 'real', active_run=None)
    assert updated['mode'] == 'mock-local'
