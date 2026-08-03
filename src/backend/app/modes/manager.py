"""ModeManager — active mode + single-active-run arbitration."""

from __future__ import annotations

from typing import Any

from ..run_spec import Environment, OperatingMode
from .registry import ModeRegistry, get_mode_registry
from .state import (
    DEFAULT_MODE,
    RuntimeModeState,
    get_runtime_mode_state,
    resolve_data_output_root,
    sim_allowed,
    sim_block_reason,
)


class ModeSwitchConflict(Exception):
    """Raised when a mode change is refused because a robot run is active."""

    def __init__(self, active_run: Any) -> None:
        self.active_run = active_run
        super().__init__('Cannot change mode while a robot run is active')


class ModeValidationError(Exception):
    """Invalid mode / environment for a switch request."""

    def __init__(self, message: str) -> None:
        self.message = message
        super().__init__(message)


class ModeManager:
    def __init__(
        self,
        registry: ModeRegistry | None = None,
        state: RuntimeModeState | None = None,
    ) -> None:
        self.registry = registry or get_mode_registry()
        self.state = state or get_runtime_mode_state()

    def current(self) -> dict[str, str]:
        return self.state.get()

    def capabilities(self) -> dict[str, Any]:
        current = self.current()
        return {
            'modes': self.registry.capability_modes(),
            'sim_allowed': sim_allowed(),
            'default_mode': DEFAULT_MODE.value,
            'data_output_root': resolve_data_output_root(
                current.get('environment')
            ),
        }

    def set_mode(
        self,
        mode: str,
        environment: str = Environment.REAL.value,
        *,
        active_run: Any = None,
    ) -> dict[str, str]:
        if active_run is not None:
            raise ModeSwitchConflict(active_run)

        try:
            mode_enum = OperatingMode(mode)
        except ValueError as exc:
            raise ModeValidationError(f'Unknown mode: {mode}') from exc

        try:
            env_enum = Environment(environment)
        except ValueError as exc:
            raise ModeValidationError(
                f'Unknown environment: {environment}'
            ) from exc

        if not self.registry.has(mode_enum):
            raise ModeValidationError(f'Unknown mode: {mode}')

        adapter = self.registry.get(mode_enum)
        if env_enum not in adapter.allowed_environments:
            raise ModeValidationError(
                f'Environment {env_enum.value} is not allowed for mode '
                f'{mode_enum.value}'
            )
        if env_enum == Environment.SIM:
            reason = sim_block_reason()
            if reason is not None:
                raise ModeValidationError(reason)

        return self.state.set(mode_enum.value, env_enum.value)


_MANAGER: ModeManager | None = None


def get_mode_manager() -> ModeManager:
    global _MANAGER
    if _MANAGER is None:
        _MANAGER = ModeManager()
    return _MANAGER


def reset_mode_manager_for_tests(
    registry: ModeRegistry | None = None,
    state: RuntimeModeState | None = None,
) -> ModeManager:
    global _MANAGER
    _MANAGER = ModeManager(registry=registry, state=state)
    return _MANAGER
