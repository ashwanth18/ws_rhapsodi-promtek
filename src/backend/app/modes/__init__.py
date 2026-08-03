"""Operating-mode adapters, registry, and ModeManager."""

from .base import JobSource, ModeAdapter, ResultSink, RunPlan
from .manager import (
    ModeManager,
    ModeSwitchConflict,
    ModeValidationError,
    get_mode_manager,
)
from .registry import ModeRegistry, build_default_registry, get_mode_registry
from .state import DEFAULT_MODE, get_runtime_mode_state, sim_allowed

__all__ = [
    'DEFAULT_MODE',
    'JobSource',
    'ModeAdapter',
    'ModeManager',
    'ModeRegistry',
    'ModeSwitchConflict',
    'ModeValidationError',
    'ResultSink',
    'RunPlan',
    'build_default_registry',
    'get_mode_manager',
    'get_mode_registry',
    'get_runtime_mode_state',
    'sim_allowed',
]
