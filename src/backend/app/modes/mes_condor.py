"""MES Condor mode — Promtek webhook jobs + Condor outbound MES."""

from __future__ import annotations

from typing import Any, Dict, Optional

from ..mes_client import CondorMesClient, get_mes_client
from ..run_spec import Environment, OperatingMode, RunSpec
from .base import ModeAdapter, ResultSink, RunPlan


class _CondorJobSource:
    """Jobs arrive via webhook_service → backend weightment rows (Phase 4+)."""

    def poll(self) -> Optional[RunSpec]:
        return None


class _CondorResultSink:
    def __init__(self, client: CondorMesClient | None = None) -> None:
        self._client = client

    def publish(self, run_spec: RunSpec, result: Dict[str, Any]) -> None:
        client = self._client or get_mes_client(OperatingMode.MES_CONDOR)
        # Phase 3: sink is wired; weighment send path still uses get_mes_client
        # from main.send_weightment_to_mes. Stash for future direct use.
        _ = (client, run_spec, result)


class MesCondorMode(ModeAdapter):
    mode = OperatingMode.MES_CONDOR
    label = 'MES Condor'
    description = 'Promtek / Condor weighment jobs with live MES outbound.'
    allowed_environments = frozenset({Environment.REAL})

    def job_source(self) -> _CondorJobSource:
        return _CondorJobSource()

    def build_plan(self, run_spec: RunSpec) -> RunPlan:
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
        return _CondorResultSink()
