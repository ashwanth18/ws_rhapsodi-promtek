"""Lights-out training mode — training episodes, null MES sink."""

from __future__ import annotations

import logging
from typing import Any, Dict, Optional

from ..mes_client import NullMesClient
from ..run_spec import Environment, OperatingMode, TREE_LIGHTSOUT, RunSpec
from .base import ModeAdapter, ResultSink, RunPlan

logger = logging.getLogger('uvicorn.error')


class _LightsOutJobSource:
    """Jobs arrive via ``POST /modes/lightsout/runs`` (not polled)."""

    def poll(self) -> Optional[RunSpec]:
        return None


class _LightsOutResultSink:
    def __init__(self) -> None:
        self._client = NullMesClient()

    def publish(self, run_spec: RunSpec, result: Dict[str, Any]) -> None:
        logger.info(
            'lightsout ResultSink: run_key=%s result_keys=%s',
            run_spec.run_key,
            sorted(result.keys()),
        )
        _ = self._client


class LightsOutMode(ModeAdapter):
    mode = OperatingMode.LIGHTSOUT
    label = 'Lights-out'
    description = 'Training episodes via LightsOut tree; null MES sink.'
    allowed_environments = frozenset({Environment.REAL, Environment.SIM})

    def job_source(self) -> _LightsOutJobSource:
        return _LightsOutJobSource()

    def build_plan(self, run_spec: RunSpec) -> RunPlan:
        tree_id = run_spec.tree_id or TREE_LIGHTSOUT
        return RunPlan(
            tree_id=tree_id,
            run_spec=run_spec,
            blackboard={
                'target_weight_g': run_spec.target_weight_g,
                'tolerance_g': run_spec.tolerance_g,
                # ExecuteScoop gated off unless the start request opts in.
                'enable_scoop': False,
            },
        )

    def result_sink(self) -> ResultSink:
        return _LightsOutResultSink()
