"""Mock / local test mode — synthetic jobs, null MES sink."""

from __future__ import annotations

import logging
import uuid
from typing import Any, Dict, Optional

from ..mes_client import NullMesClient
from ..run_spec import Environment, OperatingMode, RunSpec
from .base import ModeAdapter, ResultSink, RunPlan

logger = logging.getLogger('uvicorn.error')

MOCK_EVENT_ID_PREFIX = 'mock-'
MOCK_DEFAULT_PICKUP_TARGET = 'MoveToScoopingContainer'
MOCK_DEFAULT_WEIGH_TARGET = 'MoveToWeighingContainer'
MOCK_DEFAULT_RETURN_TARGET = 'MoveToScoopingContainer'
MOCK_SITE_ID = 'mock-local'
MOCK_INGREDIENT_ID = '0'


def is_mock_event_id(event_id: str | None) -> bool:
    """Synthetic mock-local weightments use event_id prefix ``mock-``."""
    return bool(event_id) and str(event_id).startswith(MOCK_EVENT_ID_PREFIX)


def allocate_mock_batch_id(mock_uuid: str) -> str:
    """Return a negative numeric batch_id that cannot collide with Condor.

    Condor batch ids are positive integers. Mock uses ``-(1..1e9)`` derived
    from the event UUID so adapter ``batch_id_int`` stays valid.
    """
    n = (uuid.UUID(mock_uuid).int % 1_000_000_000) + 1
    return str(-n)


class _MockJobSource:
    """Jobs arrive via ``POST /modes/mock/runs`` (not polled)."""

    def poll(self) -> Optional[RunSpec]:
        return None


class _NullResultSink:
    def __init__(self) -> None:
        self._client = NullMesClient()

    def publish(self, run_spec: RunSpec, result: Dict[str, Any]) -> None:
        logger.info(
            'mock-local NullResultSink: run_key=%s result_keys=%s',
            run_spec.run_key,
            sorted(result.keys()),
        )
        # Bind null MES client so mock never hits Condor.
        _ = self._client


class MockLocalMode(ModeAdapter):
    mode = OperatingMode.MOCK_LOCAL
    label = 'Mock local'
    description = (
        'Operator-triggered single-location test run; null MES sink '
        '(no Condor traffic).'
    )
    allowed_environments = frozenset({Environment.REAL, Environment.SIM})

    def job_source(self) -> _MockJobSource:
        return _MockJobSource()

    def build_plan(self, run_spec: RunSpec) -> RunPlan:
        return RunPlan(
            tree_id=run_spec.tree_id,
            run_spec=run_spec,
            blackboard={
                'target_weight_g': run_spec.target_weight_g,
                'tolerance_g': run_spec.tolerance_g,
                'location_code': run_spec.location_code or '',
            },
        )

    def result_sink(self) -> ResultSink:
        return _NullResultSink()
