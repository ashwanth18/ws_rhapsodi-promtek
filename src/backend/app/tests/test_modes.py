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
from app.modes.state import (
    RuntimeModeState,
    is_pi5_device,
    reset_runtime_mode_state_for_tests,
    resolve_data_output_root,
    sim_allowed,
)
from app.run_spec import OperatingMode, RunSpec
from app.schemas import LightsoutRunRequest, MockLocalRunRequest


def _laptop_sim_env(monkeypatch) -> None:
    """SIM_ALLOWED=1 on a non-pi5 host (clears DEVICE_CLASS / ROBOT_TYPE)."""
    monkeypatch.setenv('SIM_ALLOWED', '1')
    monkeypatch.delenv('DEVICE_CLASS', raising=False)
    monkeypatch.delenv('ROBOT_TYPE', raising=False)
    # Avoid any stray device.yaml with device_class: pi5.
    monkeypatch.setenv('RHAPSODI_DEVICE_CONFIG', '/nonexistent/device.yaml')


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
    monkeypatch.delenv('DEVICE_CLASS', raising=False)
    monkeypatch.delenv('ROBOT_TYPE', raising=False)
    monkeypatch.setenv('RHAPSODI_DEVICE_CONFIG', '/nonexistent/device.yaml')
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    with pytest.raises(ModeValidationError) as exc_info:
        manager.set_mode('mock-local', 'sim', active_run=None)
    assert 'SIM_ALLOWED=0' in exc_info.value.message
    assert not sim_allowed()


def test_sim_rejected_on_pi5_even_if_sim_allowed(tmp_path, monkeypatch):
    monkeypatch.setenv('SIM_ALLOWED', '1')
    monkeypatch.setenv('DEVICE_CLASS', 'pi5')
    monkeypatch.delenv('ROBOT_TYPE', raising=False)
    monkeypatch.setenv('RHAPSODI_DEVICE_CONFIG', '/nonexistent/device.yaml')
    assert is_pi5_device()
    assert not sim_allowed()
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    with pytest.raises(ModeValidationError) as exc_info:
        manager.set_mode('mock-local', 'sim', active_run=None)
    assert 'pi5' in exc_info.value.message.lower()
    assert manager.current()['environment'] == 'real'


def test_sim_accepted_on_laptop(tmp_path, monkeypatch):
    _laptop_sim_env(monkeypatch)
    assert sim_allowed()
    assert not is_pi5_device()
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    updated = manager.set_mode('mock-local', 'sim', active_run=None)
    assert updated == {'mode': 'mock-local', 'environment': 'sim'}
    lightsout = manager.set_mode('lightsout', 'sim', active_run=None)
    assert lightsout == {'mode': 'lightsout', 'environment': 'sim'}
    # MES family modes do not list SIM in allowed_environments.
    with pytest.raises(ModeValidationError):
        manager.set_mode('mes-condor', 'sim', active_run=None)


def test_get_mes_client_null_when_environment_sim(tmp_path, monkeypatch):
    _laptop_sim_env(monkeypatch)
    monkeypatch.delenv('MES_GENERIC_SINK', raising=False)
    state = reset_runtime_mode_state_for_tests(path=tmp_path / 'runtime_mode.json')
    state.set('mock-local', 'sim')
    assert get_mes_client('mock-local').__class__ is NullMesClient
    assert get_mes_client(OperatingMode.MES_CONDOR).__class__ is NullMesClient
    assert get_mes_client(OperatingMode.MES_GENERIC).__class__ is NullMesClient
    # Back to real: Condor modes bind Condor again.
    state.set('mes-condor', 'real')
    assert get_mes_client(OperatingMode.MES_CONDOR).__class__.__name__ == (
        'CondorMesClient'
    )


def test_resolve_data_output_root_sim(monkeypatch):
    monkeypatch.delenv('DATA_OUTPUT_ROOT', raising=False)
    monkeypatch.delenv('SIM_DATA_OUTPUT_ROOT', raising=False)
    assert resolve_data_output_root('sim') == '/tmp/rhapsodi-sim/runs'
    monkeypatch.setenv('SIM_DATA_OUTPUT_ROOT', '/custom/sim/runs')
    assert resolve_data_output_root('sim') == '/custom/sim/runs'
    monkeypatch.delenv('SIM_DATA_OUTPUT_ROOT', raising=False)
    monkeypatch.setenv('DATA_OUTPUT_ROOT', '/data/runs')
    assert resolve_data_output_root('sim') == '/data/runs'
    assert resolve_data_output_root('real') == '/data/runs'


def test_sim_default_mode_is_mock_local(tmp_path, monkeypatch):
    """ENVIRONMENT=sim must not seed mes-condor+sim (invalid pair)."""
    _laptop_sim_env(monkeypatch)
    monkeypatch.setenv('ENVIRONMENT', 'sim')
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    assert state.load() == {'mode': 'mock-local', 'environment': 'sim'}


def test_sim_rejected_when_robot_type_pi5(tmp_path, monkeypatch):
    monkeypatch.setenv('SIM_ALLOWED', '1')
    monkeypatch.delenv('DEVICE_CLASS', raising=False)
    monkeypatch.setenv('ROBOT_TYPE', 'pi5')
    monkeypatch.setenv('RHAPSODI_DEVICE_CONFIG', '/nonexistent/device.yaml')
    assert is_pi5_device()
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    with pytest.raises(ModeValidationError):
        manager.set_mode('mock-local', 'sim', active_run=None)


def test_persisted_sim_cleared_on_pi5(tmp_path, monkeypatch):
    """Loading runtime_mode.json with sim must not activate sim on pi5."""
    monkeypatch.setenv('SIM_ALLOWED', '1')
    monkeypatch.setenv('DEVICE_CLASS', 'pi5')
    monkeypatch.setenv('RHAPSODI_DEVICE_CONFIG', '/nonexistent/device.yaml')
    path = tmp_path / 'runtime_mode.json'
    path.write_text(
        '{"mode":"mock-local","environment":"sim"}\n', encoding='utf-8'
    )
    state = RuntimeModeState(path=path)
    assert state.load() == {'mode': 'mock-local', 'environment': 'real'}


def test_capabilities_shape(tmp_path, monkeypatch):
    monkeypatch.setenv('SIM_ALLOWED', '0')
    monkeypatch.delenv('DEVICE_CLASS', raising=False)
    state = RuntimeModeState(path=tmp_path / 'runtime_mode.json')
    manager = ModeManager(registry=build_default_registry(), state=state)
    caps = manager.capabilities()
    assert caps['default_mode'] == 'mes-condor'
    assert caps['sim_allowed'] is False
    assert caps['data_output_root']
    assert len(caps['modes']) == 4


def test_null_mes_client_noop(tmp_path, monkeypatch, caplog):
    # Ensure environment=real so Condor bindings are exercised.
    monkeypatch.setenv('SIM_ALLOWED', '0')
    reset_runtime_mode_state_for_tests(path=tmp_path / 'runtime_mode.json')
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
        powder_id='alumina-5um',
        target_weight_g=250.0,
        episodes=10,
    )
    assert req.enable_scoop is True
    assert req.batch_id == ''
    assert req.target_mode == 'stratified'
    full = LightsoutRunRequest(
        powder_id='alumina-5um',
        target_weight_g=125.0,
        episodes=5,
        batch_id='batch-1',
        enable_scoop=True,
        stop_on='duration_min',
        stop_value=30.0,
        target_mode='fixed',
    )
    assert full.enable_scoop is True
    assert full.powder_id == 'alumina-5um'
    assert full.stop_on == 'duration_min'


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
    assert plan.blackboard['enable_scoop'] is True
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
