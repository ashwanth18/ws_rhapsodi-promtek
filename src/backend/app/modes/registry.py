"""Mode registry — maps mode id → ModeAdapter."""

from __future__ import annotations

from typing import Dict, Iterable, List

from ..run_spec import OperatingMode
from .base import ModeAdapter
from .lightsout import LightsOutMode
from .mes_condor import MesCondorMode
from .mes_generic import MesGenericMode
from .mock_local import MockLocalMode
from .state import DEFAULT_MODE


class ModeRegistry:
    def __init__(self) -> None:
        self._modes: Dict[str, ModeAdapter] = {}

    def register(self, adapter: ModeAdapter) -> None:
        self._modes[adapter.mode.value] = adapter

    def get(self, mode_id: str | OperatingMode) -> ModeAdapter:
        key = mode_id.value if isinstance(mode_id, OperatingMode) else str(mode_id)
        try:
            return self._modes[key]
        except KeyError as exc:
            raise KeyError(f'Unknown operating mode: {key}') from exc

    def has(self, mode_id: str | OperatingMode) -> bool:
        key = mode_id.value if isinstance(mode_id, OperatingMode) else str(mode_id)
        return key in self._modes

    def modes(self) -> List[ModeAdapter]:
        order = [
            OperatingMode.MES_CONDOR.value,
            OperatingMode.MES_GENERIC.value,
            OperatingMode.MOCK_LOCAL.value,
            OperatingMode.LIGHTSOUT.value,
        ]
        ordered = [self._modes[m] for m in order if m in self._modes]
        extras = [a for k, a in self._modes.items() if k not in order]
        return ordered + extras

    def capability_modes(self) -> List[dict]:
        return [adapter.capability_dict() for adapter in self.modes()]

    def ids(self) -> Iterable[str]:
        return self._modes.keys()


def build_default_registry() -> ModeRegistry:
    registry = ModeRegistry()
    for adapter in (
        MesCondorMode(),
        MesGenericMode(),
        MockLocalMode(),
        LightsOutMode(),
    ):
        registry.register(adapter)
    assert registry.has(DEFAULT_MODE)
    return registry


_REGISTRY: ModeRegistry | None = None


def get_mode_registry() -> ModeRegistry:
    global _REGISTRY
    if _REGISTRY is None:
        _REGISTRY = build_default_registry()
    return _REGISTRY


def reset_mode_registry_for_tests() -> ModeRegistry:
    global _REGISTRY
    _REGISTRY = build_default_registry()
    return _REGISTRY
