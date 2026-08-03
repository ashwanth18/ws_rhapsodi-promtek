"""Generic MES mode stub — config-driven adapter lands in Phase 6."""

from __future__ import annotations

from typing import Any, Dict, Optional

from ..run_spec import Environment, OperatingMode, RunSpec
from .base import ModeAdapter, ResultSink, RunPlan


class _GenericJobSource:
    def poll(self) -> Optional[RunSpec]:
        return None


class _GenericResultSink:
    def publish(self, run_spec: RunSpec, result: Dict[str, Any]) -> None:
        _ = (run_spec, result)


class MesGenericMode(ModeAdapter):
    mode = OperatingMode.MES_GENERIC
    label = 'MES generic'
    description = (
        'Config-driven non-Promtek MES adapter (stub; Phase 6).'
    )
    allowed_environments = frozenset({Environment.REAL})

    def job_source(self) -> _GenericJobSource:
        return _GenericJobSource()

    def build_plan(self, run_spec: RunSpec) -> RunPlan:
        return RunPlan(tree_id=run_spec.tree_id, run_spec=run_spec)

    def result_sink(self) -> ResultSink:
        return _GenericResultSink()
