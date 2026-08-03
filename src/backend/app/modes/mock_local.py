"""Mock / local test mode — synthetic jobs, null MES sink."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from ..mes_client import NullMesClient
from ..run_spec import Environment, OperatingMode, RunSpec
from .base import ModeAdapter, ResultSink, RunPlan

logger = logging.getLogger('uvicorn.error')


class _MockJobSource:
    """Phase 4 will accept ``POST /modes/mock/runs``; idle until then."""

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
