"""Phase 6: inbound adapter registry + Condor normalization contract tests."""

from __future__ import annotations

import pytest

from app.mes_client import CondorMesClient, NullMesClient, get_mes_client
from app.modes.inbound import (
    DEFAULT_INBOUND_ADAPTER,
    get_inbound_adapter,
    get_inbound_adapter_name,
    list_inbound_adapters,
    normalize_inbound_event,
)
from app.modes.inbound.condor import normalize_condor_batch_released
from app.modes.inbound.generic_json import (
    DEFAULT_FIELD_MAP,
    load_field_map,
    normalize_generic_json,
)
from app.modes.mes_generic import (
    strip_generic_event_id_prefix,
    GENERIC_EVENT_ID_PREFIX,
    ensure_generic_event_id,
    is_mes_generic_event_id,
    mes_generic_sink_name,
)
from app.modes.registry import build_default_registry
from app.run_spec import OperatingMode, RunSpec

# Frozen Condor/Promtek sample — keys/types must match webhook_service output.
SAMPLE_CONDOR_BATCH_RELEASED = {
    'eventId': '568146dd-d903-406b-83c9-201b583b39e7',
    'sentUtc': '2026-03-11T12:00:00Z',
    'userId': 'user-1',
    'siteId': 'site-9',
    'batch': {
        'id': 22461,
        'batchNumber': 'BN-100',
        'workOrderId': 'WO-55',
        'targetQuantity': '12.5',
        'lines': [
            {
                'parameters': [
                    {
                        'batchInstructionParameterCode': 'Ingredient',
                        'target': 1327,
                    },
                    {
                        'batchInstructionParameterCode': 'Quantity',
                        'target': '0.250',
                    },
                ],
            },
            {
                'parameters': [
                    {
                        'BatchInstructionParameterCode': 'Ingredient',
                        'Target': '99',
                    },
                    {
                        'BatchInstructionParameterCode': 'Quantity',
                        'Target': 1.5,
                    },
                ],
            },
        ],
    },
}

# Exact keys webhook_service / Condor path populate on webhook_weightments.
CONDOR_WEIGHTMENT_KEYS = {
    'event_id',
    'sent_utc',
    'user_id',
    'site_id',
    'batch_id',
    'batch_number',
    'work_order_id',
    'batch_target_quantity',
    'ingredient_id',
    'target_weight_kg',
}


def test_inbound_registry_lists_condor_and_generic_json():
    names = list_inbound_adapters()
    assert 'condor' in names
    assert 'generic_json' in names
    assert DEFAULT_INBOUND_ADAPTER == 'condor'
    assert get_inbound_adapter_name(None) == 'condor'
    assert get_inbound_adapter('default').name == 'condor'
    assert get_inbound_adapter('condor').name == 'condor'
    assert get_inbound_adapter('generic_json').name == 'generic_json'


def test_inbound_adapter_name_from_env(monkeypatch):
    monkeypatch.setenv('MES_GENERIC_INBOUND_ADAPTER', 'generic_json')
    assert get_inbound_adapter_name() == 'generic_json'
    monkeypatch.setenv('MES_GENERIC_INBOUND_ADAPTER', 'CONDOR')
    assert get_inbound_adapter_name() == 'condor'


def test_unknown_inbound_adapter_raises():
    with pytest.raises(ValueError, match='Unknown inbound adapter'):
        get_inbound_adapter('not-a-real-adapter')


def test_condor_normalization_contract_keys_and_types():
    """Condor adapter must keep the same keys/types as webhook_service."""
    rows = normalize_condor_batch_released(SAMPLE_CONDOR_BATCH_RELEASED)
    assert len(rows) == 2

    first = rows[0].as_row_dict()
    assert CONDOR_WEIGHTMENT_KEYS.issubset(first.keys())

    assert first['event_id'] == '568146dd-d903-406b-83c9-201b583b39e7'
    assert isinstance(first['event_id'], str)
    assert first['sent_utc'] == '2026-03-11T12:00:00Z'
    assert isinstance(first['sent_utc'], str)
    assert first['user_id'] == 'user-1'
    assert first['site_id'] == 'site-9'
    assert first['batch_id'] == '22461'
    assert isinstance(first['batch_id'], str)
    assert first['batch_number'] == 'BN-100'
    assert first['work_order_id'] == 'WO-55'
    assert first['batch_target_quantity'] == 12.5
    assert isinstance(first['batch_target_quantity'], float)
    assert first['ingredient_id'] == '1327'
    assert isinstance(first['ingredient_id'], str)
    assert first['target_weight_kg'] == 0.250
    assert isinstance(first['target_weight_kg'], float)

    second = rows[1].as_row_dict()
    assert second['ingredient_id'] == '99'
    assert second['target_weight_kg'] == 1.5
    assert second['event_id'] == first['event_id']
    assert second['batch_id'] == first['batch_id']


def test_condor_adapter_via_registry_matches_direct():
    via_registry = normalize_inbound_event(
        SAMPLE_CONDOR_BATCH_RELEASED, adapter_name='condor'
    )
    direct = normalize_condor_batch_released(SAMPLE_CONDOR_BATCH_RELEASED)
    assert [r.as_row_dict() for r in via_registry] == [
        r.as_row_dict() for r in direct
    ]


def test_condor_empty_lines_returns_empty():
    payload = {
        'eventId': 'e1',
        'batch': {'id': 1, 'lines': []},
    }
    assert normalize_condor_batch_released(payload) == []


def test_generic_json_default_identity_map():
    payload = {
        'event_id': 'evt-1',
        'batch_id': '100',
        'ingredient_id': '7',
        'target_weight_kg': 0.5,
        'site_id': 'S1',
    }
    rows = normalize_generic_json(payload, dict(DEFAULT_FIELD_MAP))
    assert len(rows) == 1
    row = rows[0].as_row_dict()
    assert row['event_id'] == 'evt-1'
    assert row['batch_id'] == '100'
    assert row['ingredient_id'] == '7'
    assert row['target_weight_kg'] == 0.5
    assert isinstance(row['target_weight_kg'], float)
    assert row['site_id'] == 'S1'


def test_generic_json_custom_field_map():
    field_map = load_field_map(
        '{"event_id":"id","batch_id":"batch","target_weight_kg":"target_kg",'
        '"ingredient_id":"ingredient","site_id":"site"}'
    )
    payload = {
        'id': 'ext-42',
        'batch': '9001',
        'target_kg': '1.25',
        'ingredient': 55,
        'site': 'plant-a',
    }
    rows = normalize_generic_json(payload, field_map)
    assert len(rows) == 1
    row = rows[0].as_row_dict()
    assert row['event_id'] == 'ext-42'
    assert row['batch_id'] == '9001'
    assert row['target_weight_kg'] == 1.25
    assert row['ingredient_id'] == '55'
    assert row['site_id'] == 'plant-a'


def test_generic_json_dotted_paths_and_lines():
    field_map = {
        **DEFAULT_FIELD_MAP,
        'event_id': 'header.id',
        'batch_id': 'header.batch_id',
        'ingredient_id': 'ingredient',
        'target_weight_kg': 'qty_kg',
    }
    payload = {
        'header': {'id': 'hdr-1', 'batch_id': 'B-9'},
        'lines': [
            {'ingredient': '1', 'qty_kg': 0.1},
            {'ingredient': '2', 'qty_kg': 0.2},
        ],
    }
    rows = normalize_generic_json(payload, field_map, lines_path='lines')
    assert len(rows) == 2
    assert rows[0].event_id == 'hdr-1'
    assert rows[0].batch_id == 'B-9'
    assert rows[0].ingredient_id == '1'
    assert rows[0].target_weight_kg == 0.1
    assert rows[1].ingredient_id == '2'
    assert rows[1].target_weight_kg == 0.2


def test_generic_json_field_map_from_env(monkeypatch):
    monkeypatch.setenv(
        'MES_GENERIC_FIELD_MAP_JSON',
        '{"event_id":"id","batch_id":"batch","target_weight_kg":"kg"}',
    )
    rows = normalize_inbound_event(
        {'id': 'e', 'batch': '1', 'kg': 3},
        adapter_name='generic_json',
    )
    assert rows[0].event_id == 'e'
    assert rows[0].batch_id == '1'
    assert rows[0].target_weight_kg == 3.0


def test_mes_generic_event_id_helpers():
    assert is_mes_generic_event_id(f'{GENERIC_EVENT_ID_PREFIX}abc')
    assert not is_mes_generic_event_id('webhook-abc')
    assert not is_mes_generic_event_id(None)
    assert ensure_generic_event_id('abc') == f'{GENERIC_EVENT_ID_PREFIX}abc'
    assert ensure_generic_event_id(f'{GENERIC_EVENT_ID_PREFIX}abc') == (
        f'{GENERIC_EVENT_ID_PREFIX}abc'
    )
    # Blank / bare prefix must mint unique ids (never collapse to "generic-").
    blank_a = ensure_generic_event_id('')
    blank_b = ensure_generic_event_id(None)
    assert blank_a.startswith(GENERIC_EVENT_ID_PREFIX)
    assert blank_b.startswith(GENERIC_EVENT_ID_PREFIX)
    assert blank_a != GENERIC_EVENT_ID_PREFIX
    assert blank_a != blank_b
    assert ensure_generic_event_id(GENERIC_EVENT_ID_PREFIX) != GENERIC_EVENT_ID_PREFIX
    assert strip_generic_event_id_prefix(f'{GENERIC_EVENT_ID_PREFIX}xyz') == 'xyz'


def test_mes_generic_sink_and_client(monkeypatch):
    monkeypatch.delenv('MES_GENERIC_SINK', raising=False)
    assert mes_generic_sink_name() == 'condor'
    assert get_mes_client(OperatingMode.MES_GENERIC).__class__ is CondorMesClient
    # Condor path unchanged.
    assert get_mes_client(OperatingMode.MES_CONDOR).__class__ is CondorMesClient

    monkeypatch.setenv('MES_GENERIC_SINK', 'null')
    assert mes_generic_sink_name() == 'null'
    assert get_mes_client(OperatingMode.MES_GENERIC).__class__ is NullMesClient
    # mes-condor must still bind Condor even when generic sink is null.
    assert get_mes_client(OperatingMode.MES_CONDOR).__class__ is CondorMesClient


def test_mes_generic_mode_plan_matches_webhook_tree():
    adapter = build_default_registry().get(OperatingMode.MES_GENERIC)
    assert adapter.mode == OperatingMode.MES_GENERIC
    plan = adapter.build_plan(
        RunSpec(
            mode=OperatingMode.MES_GENERIC,
            run_key='generic-test',
            target_weight_g=100.0,
            tolerance_g=2.0,
            batch_id='1',
            ingredient_id='2',
            location_code='LOC',
        )
    )
    assert plan.tree_id == 'WebhookWeightment'
    assert plan.blackboard['target_weight_g'] == 100.0
    assert plan.blackboard['batch_id'] == '1'
