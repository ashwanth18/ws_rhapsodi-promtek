"""Protocols for pluggable operating-mode adapters.

Every mode supplies a ``JobSource`` (where work comes from), builds a
``RunPlan`` (tree + blackboard seed), and a ``ResultSink`` (where
completion goes). See ``docs/MODES.md``.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Protocol, runtime_checkable

from ..run_spec import Environment, OperatingMode, RunSpec


@dataclass
class RunPlan:
    """Resolved tree identity plus blackboard seed for a single run."""

    tree_id: str
    run_spec: RunSpec
    blackboard: Dict[str, Any] = field(default_factory=dict)


@runtime_checkable
class JobSource(Protocol):
    """Produces the next ``RunSpec`` for the active mode, or None if idle."""

    def poll(self) -> Optional[RunSpec]:
        ...


@runtime_checkable
class ResultSink(Protocol):
    """Publishes a completed run / weighment result downstream."""

    def publish(self, run_spec: RunSpec, result: Dict[str, Any]) -> None:
        ...


class ModeAdapter(ABC):
    """Registered operating mode: job source + plan builder + result sink."""

    mode: OperatingMode
    label: str
    description: str = ''
    # Environments this mode may run under. Sim is still gated by SIM_ALLOWED.
    allowed_environments: frozenset[Environment] = frozenset({Environment.REAL})

    @abstractmethod
    def job_source(self) -> JobSource:
        ...

    @abstractmethod
    def build_plan(self, run_spec: RunSpec) -> RunPlan:
        ...

    @abstractmethod
    def result_sink(self) -> ResultSink:
        ...

    def capability_dict(self) -> Dict[str, Any]:
        return {
            'mode': self.mode.value,
            'label': self.label,
            'description': self.description,
            'allowed_environments': sorted(e.value for e in self.allowed_environments),
        }
