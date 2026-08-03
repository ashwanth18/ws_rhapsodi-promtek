"""MES generic mode — config-driven inbound adapters + configurable MES sink."""

from __future__ import annotations

import logging
import os
import uuid
from typing import Any, Dict, Optional

from ..mes_client import CondorMesClient, NullMesClient, get_mes_client
from ..run_spec import Environment, OperatingMode, RunSpec
from .base import ModeAdapter, ResultSink, RunPlan

logger = logging.getLogger('uvicorn.error')

GENERIC_EVENT_ID_PREFIX = 'generic-'


def is_mes_generic_event_id(event_id: str | None) -> bool:
    """Rows ingested via ``POST /modes/mes-generic/events`` use this prefix."""
    return bool(event_id) and str(event_id).startswith(GENERIC_EVENT_ID_PREFIX)


def strip_generic_event_id_prefix(event_id: str | None) -> str:
    """Return upstream id without the ``generic-`` routing prefix."""
    raw = str(event_id or '').strip()
    if raw.startswith(GENERIC_EVENT_ID_PREFIX):
        return raw[len(GENERIC_EVENT_ID_PREFIX) :]
    return raw


def ensure_generic_event_id(event_id: str | None = None) -> str:
    """Prefix upstream ids so outbound routing can bind mes-generic sink.

    Blank upstream ids get a fresh UUID so unrelated events never collapse
    onto the bare ``generic-`` key.
    """
    raw = str(event_id or '').strip()
    if not raw:
        return f'{GENERIC_EVENT_ID_PREFIX}{uuid.uuid4()}'
    if raw.startswith(GENERIC_EVENT_ID_PREFIX):
        # Reject the bare prefix (legacy bug); mint a unique id instead.
        if raw == GENERIC_EVENT_ID_PREFIX:
            return f'{GENERIC_EVENT_ID_PREFIX}{uuid.uuid4()}'
        return raw
    return f'{GENERIC_EVENT_ID_PREFIX}{raw}'


def mes_generic_sink_name() -> str:
    """``condor`` (default) or ``null`` — see ``MES_GENERIC_SINK``."""
    raw = os.environ.get('MES_GENERIC_SINK', 'condor').strip().lower()
    if raw in ('null', 'none', 'noop', 'off', '0', 'false'):
        return 'null'
    return 'condor'


class _GenericJobSource:
    """Jobs arrive via ``POST /modes/mes-generic/events`` (not polled)."""

    def poll(self) -> Optional[RunSpec]:
        return None


class _GenericResultSink:
    def __init__(self, client: CondorMesClient | NullMesClient | None = None) -> None:
        self._client = client

    def publish(self, run_spec: RunSpec, result: Dict[str, Any]) -> None:
        client = self._client or get_mes_client(OperatingMode.MES_GENERIC)
        logger.info(
            'mes-generic ResultSink: sink=%s run_key=%s result_keys=%s',
            type(client).__name__,
            run_spec.run_key,
            sorted(result.keys()),
        )
        _ = (client, run_spec, result)


class MesGenericMode(ModeAdapter):
    mode = OperatingMode.MES_GENERIC
    label = 'MES generic'
    description = (
        'Config-driven non-Promtek MES adapter '
        '(inbound registry + MES_GENERIC_SINK outbound).'
    )
    allowed_environments = frozenset({Environment.REAL})

    def job_source(self) -> _GenericJobSource:
        return _GenericJobSource()

    def build_plan(self, run_spec: RunSpec) -> RunPlan:
        # Same WebhookWeightment blackboard seed as mes-condor.
        return RunPlan(
            tree_id=run_spec.tree_id,
            run_spec=run_spec,
            blackboard={
                'target_weight_g': run_spec.target_weight_g,
                'tolerance_g': run_spec.tolerance_g,
                'location_code': run_spec.location_code or '',
                'batch_id': run_spec.batch_id or '',
                'ingredient_id': run_spec.ingredient_id or '',
                'expected_lot': run_spec.expected_lot or '',
            },
        )

    def result_sink(self) -> ResultSink:
        return _GenericResultSink()
