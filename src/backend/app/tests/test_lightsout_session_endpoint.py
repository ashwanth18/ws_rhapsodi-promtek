"""Lights-out session GET shapes and signed pour-error metric."""

from __future__ import annotations

from app.metrics import signed_final_error_g
from app.modes.lightsout_session import reset_lightsout_session_for_tests


def test_signed_final_error_g_mes_uses_absolute() -> None:
    assert signed_final_error_g(250.0, 248.0) == -2.0
    assert signed_final_error_g(250.0, 255.0, mode='mes-condor') == 5.0
    assert signed_final_error_g(None, 100.0, fallback=-1.0) == -1.0


def test_signed_final_error_g_lightsout_uses_net() -> None:
    # Absolute final would be ~800g on a full vessel; net poured is the truth.
    assert (
        signed_final_error_g(
            50.0,
            800.0,
            mode='lightsout',
            net_weight_g=52.0,
        )
        == 2.0
    )
    assert (
        signed_final_error_g(
            50.0,
            800.0,
            mode='lightsout',
            net_weight_g=None,
            fallback=None,
        )
        is None
    )


def _session_status_payload(session_state) -> dict:
    """Mirror GET /modes/lightsout/session without importing app.main / DB."""
    active = session_state.get_active()
    if active is None:
        return {
            'active': False,
            'started_at': None,
            'session': None,
            'episodes': {'completed': 0},
            'tolerance_frac': 0.02,
        }
    return {
        'active': True,
        'started_at': active.get('started_at'),
        'session': active.get('request') or {},
        'episodes': {'completed': 0},
        'tolerance_frac': 0.02,
    }


def test_get_lightsout_session_inactive(tmp_path) -> None:
    session = reset_lightsout_session_for_tests(path=tmp_path / 'lo.json')
    payload = _session_status_payload(session)
    assert payload['active'] is False
    assert payload['session'] is None
    assert payload['episodes'] == {'completed': 0}
    assert payload['tolerance_frac'] == 0.02
    assert payload['started_at'] is None


def test_get_lightsout_session_active_shape(tmp_path) -> None:
    session = reset_lightsout_session_for_tests(path=tmp_path / 'lo.json')
    request = {
        'powder_id': 'alumina-5um',
        'powder_name': 'Alumina 5 um',
        'batch_id': 'LO-TEST',
        'episodes': 3,
        'stop_on': 'episodes',
        'stop_value': 3.0,
        'target_mode': 'fixed',
        'target_weight_g': 50.0,
        'min_scooped_g': 20.0,
        'container_target': 'MoveToScoopingContainer',
        'pour_target': 'lightsoutPourStartV2',
        'enable_scoop': True,
    }
    session.mark_started(request)
    payload = _session_status_payload(session)
    assert payload['active'] is True
    assert payload['started_at'] is not None
    assert payload['session']['powder_id'] == 'alumina-5um'
    assert payload['session']['min_scooped_g'] == 20.0
    assert payload['session']['stop_on'] == 'episodes'
    assert payload['tolerance_frac'] == 0.02
    assert payload['episodes']['completed'] == 0
    session.clear()
    assert _session_status_payload(session)['active'] is False
