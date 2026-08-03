"""Durable active-mode storage for the Pi (JSON file + in-memory fallback)."""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Any

from ..run_spec import Environment, OperatingMode

logger = logging.getLogger('uvicorn.error')

DEFAULT_MODE = OperatingMode.MES_CONDOR
DEFAULT_SIM_MODE = OperatingMode.MOCK_LOCAL
DEFAULT_ENVIRONMENT = Environment.REAL
DEFAULT_RUNTIME_MODE_PATH = '/data/runtime_mode.json'
DEFAULT_REAL_DATA_OUTPUT_ROOT = '/data/runs'
DEFAULT_SIM_DATA_OUTPUT_ROOT = '/tmp/rhapsodi-sim/runs'
_PI5_MARKERS = frozenset({'pi5'})
# Modes known to list SIM in allowed_environments (avoid circular import).
_SIM_COMPATIBLE_MODES = frozenset(
    {
        OperatingMode.MOCK_LOCAL.value,
        OperatingMode.LIGHTSOUT.value,
    }
)


def _env_flag_true(name: str, default: str = '0') -> bool:
    raw = os.environ.get(name, default).strip().lower()
    return raw in {'1', 'true', 'yes', 'on'}


def _sim_env_allowed() -> bool:
    """True when ``SIM_ALLOWED`` env permits sim (ignores device class)."""
    return _env_flag_true('SIM_ALLOWED', default='0')


def _default_environment() -> Environment:
    """Compose/env seed for first boot — never sim when the device forbids it."""
    raw = (os.environ.get('ENVIRONMENT') or DEFAULT_ENVIRONMENT.value).strip().lower()
    try:
        env = Environment(raw)
    except ValueError:
        return DEFAULT_ENVIRONMENT
    if env == Environment.SIM and sim_block_reason() is not None:
        logger.warning(
            'ENVIRONMENT=sim ignored (%s); seeding environment=real',
            sim_block_reason(),
        )
        return DEFAULT_ENVIRONMENT
    return env


def _device_yaml_identity_hints() -> dict[str, str]:
    """Read ``device_class`` / ``robot_type`` from device.yaml (no PyYAML)."""
    candidates: list[str] = []
    env_path = os.environ.get('RHAPSODI_DEVICE_CONFIG')
    if env_path:
        candidates.append(env_path)
    candidates.extend(
        [
            '/ws/config/device.yaml',
            '/config/device.yaml',
            'config/device.yaml',
        ]
    )
    for path in candidates:
        try:
            if not os.path.isfile(path):
                continue
            hints: dict[str, str] = {}
            in_device = False
            with open(path, encoding='utf-8') as fh:
                for raw in fh:
                    line = raw.rstrip('\n')
                    if not line.strip() or line.lstrip().startswith('#'):
                        continue
                    if line.startswith('device:'):
                        in_device = True
                        continue
                    if in_device and line and not line.startswith((' ', '\t')):
                        break
                    if not in_device:
                        continue
                    stripped = line.strip()
                    if ':' not in stripped or stripped.endswith(':'):
                        continue
                    key, _, value = stripped.partition(':')
                    key = key.strip()
                    value = value.strip().strip("'\"")
                    if key in ('device_class', 'robot_type') and value:
                        hints[key] = value
            return hints
        except OSError:
            continue
    return {}


def is_pi5_device() -> bool:
    """True when env or device.yaml marks this host as a Pi5 cell.

    Belt-and-suspenders with ``SIM_ALLOWED``: sim must never be selectable
    on production Pi hardware even if an operator mistakenly sets
    ``SIM_ALLOWED=1``.
    """
    for env_name in ('DEVICE_CLASS', 'ROBOT_TYPE'):
        value = (os.environ.get(env_name) or '').strip().lower()
        if value in _PI5_MARKERS:
            return True
    hints = _device_yaml_identity_hints()
    for key in ('device_class', 'robot_type'):
        value = (hints.get(key) or '').strip().lower()
        if value in _PI5_MARKERS:
            return True
    return False


def sim_block_reason() -> str | None:
    """Human-readable reason sim is blocked, or ``None`` if allowed."""
    if is_pi5_device():
        return (
            'Sim environment is not allowed on pi5 device class '
            '(DEVICE_CLASS/ROBOT_TYPE/device.yaml)'
        )
    if not _sim_env_allowed():
        return 'Sim environment is not allowed on this device (SIM_ALLOWED=0)'
    return None


def sim_allowed() -> bool:
    """True when this process may select ``environment=sim``."""
    return sim_block_reason() is None


def resolve_data_output_root(environment: str | None = None) -> str:
    """Recorder root for the given (or active) environment axis.

    Sim prefers ``SIM_DATA_OUTPUT_ROOT``, then ``DATA_OUTPUT_ROOT``, then
    ``/tmp/rhapsodi-sim/runs``. Real uses ``DATA_OUTPUT_ROOT`` or
    ``/data/runs``.
    """
    if environment is None:
        environment = get_runtime_mode_state().environment
    env_norm = str(environment or DEFAULT_ENVIRONMENT.value).strip().lower()
    if env_norm == Environment.SIM.value:
        return (
            (os.environ.get('SIM_DATA_OUTPUT_ROOT') or '').strip()
            or (os.environ.get('DATA_OUTPUT_ROOT') or '').strip()
            or DEFAULT_SIM_DATA_OUTPUT_ROOT
        )
    return (
        (os.environ.get('DATA_OUTPUT_ROOT') or '').strip()
        or DEFAULT_REAL_DATA_OUTPUT_ROOT
    )


class RuntimeModeState:
    """Persist ``{mode, environment}`` under RUNTIME_MODE_PATH.

    Falls back to an in-memory dict when the path is missing/unwritable
    (unit tests, fresh containers without a ``/data`` mount).
    """

    def __init__(self, path: str | Path | None = None) -> None:
        env_path = os.environ.get('RUNTIME_MODE_PATH', DEFAULT_RUNTIME_MODE_PATH)
        self._path = Path(path if path is not None else env_path)
        self._lock = threading.Lock()
        default_env = _default_environment().value
        self._memory: dict[str, str] = {
            'mode': DEFAULT_MODE.value,
            'environment': default_env,
        }
        self._use_memory_only = False
        self._loaded = False

    @property
    def path(self) -> Path:
        return self._path

    def _default_payload(self) -> dict[str, str]:
        environment = _default_environment()
        mode = (
            DEFAULT_SIM_MODE
            if environment == Environment.SIM
            else DEFAULT_MODE
        )
        return {'mode': mode.value, 'environment': environment.value}

    def _normalize(self, payload: dict[str, Any]) -> dict[str, str]:
        mode_raw = str(payload.get('mode') or DEFAULT_MODE.value)
        # Missing environment key: honor compose ENVIRONMENT=sim on laptop
        # stacks rather than hard-coding real (avoids Condor MES in sim).
        if 'environment' in payload and payload.get('environment') is not None:
            env_raw = str(payload.get('environment') or DEFAULT_ENVIRONMENT.value)
        else:
            env_raw = _default_environment().value
        try:
            mode = OperatingMode(mode_raw).value
        except ValueError:
            logger.warning(
                'Unknown persisted mode %r; falling back to %s',
                mode_raw,
                DEFAULT_MODE.value,
            )
            mode = DEFAULT_MODE.value
        try:
            environment = Environment(env_raw).value
        except ValueError:
            logger.warning(
                'Unknown persisted environment %r; falling back to %s',
                env_raw,
                DEFAULT_ENVIRONMENT.value,
            )
            environment = DEFAULT_ENVIRONMENT.value
        # Never keep sim active when the device forbids it (pi5 / SIM_ALLOWED=0).
        if environment == Environment.SIM.value:
            block = sim_block_reason()
            if block is not None:
                logger.warning(
                    'Clearing persisted environment=sim (%s); using real',
                    block,
                )
                environment = DEFAULT_ENVIRONMENT.value
        # Persist may predate sim gating; never keep MES+sim as active pair.
        if (
            environment == Environment.SIM.value
            and mode not in _SIM_COMPATIBLE_MODES
        ):
            logger.warning(
                'Mode %s is not sim-compatible; falling back to %s',
                mode,
                DEFAULT_SIM_MODE.value,
            )
            mode = DEFAULT_SIM_MODE.value
        return {'mode': mode, 'environment': environment}

    def _read_file(self) -> dict[str, str] | None:
        try:
            if not self._path.is_file():
                return None
            with self._path.open('r', encoding='utf-8') as handle:
                data = json.load(handle)
            if not isinstance(data, dict):
                return None
            return self._normalize(data)
        except OSError as exc:
            logger.warning(
                'Unable to read runtime mode file %s: %s', self._path, exc
            )
            return None
        except (json.JSONDecodeError, TypeError) as exc:
            logger.warning(
                'Corrupt runtime mode file %s: %s', self._path, exc
            )
            return None

    def _write_file(self, payload: dict[str, str]) -> bool:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_suffix(self._path.suffix + '.tmp')
            with tmp.open('w', encoding='utf-8') as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
                handle.write('\n')
            tmp.replace(self._path)
            return True
        except OSError as exc:
            logger.warning(
                'Unable to write runtime mode file %s: %s; using in-memory state',
                self._path,
                exc,
            )
            return False

    def load(self) -> dict[str, str]:
        with self._lock:
            if self._use_memory_only:
                return dict(self._memory)
            file_payload = self._read_file()
            if file_payload is not None:
                self._memory = file_payload
                self._loaded = True
                return dict(self._memory)
            # Seed defaults (try persist once).
            payload = self._default_payload()
            if self._write_file(payload):
                self._memory = payload
            else:
                self._use_memory_only = True
                self._memory = payload
            self._loaded = True
            return dict(self._memory)

    def get(self) -> dict[str, str]:
        if not self._loaded and not self._use_memory_only:
            return self.load()
        with self._lock:
            return dict(self._memory)

    @property
    def mode(self) -> str:
        return self.get()['mode']

    @property
    def environment(self) -> str:
        return self.get()['environment']

    def set(self, mode: str, environment: str) -> dict[str, str]:
        payload = self._normalize({'mode': mode, 'environment': environment})
        with self._lock:
            if self._use_memory_only or not self._write_file(payload):
                self._use_memory_only = True
                self._memory = payload
            else:
                self._memory = payload
            self._loaded = True
            return dict(self._memory)


_STATE: RuntimeModeState | None = None


def get_runtime_mode_state() -> RuntimeModeState:
    global _STATE
    if _STATE is None:
        _STATE = RuntimeModeState()
    return _STATE


def reset_runtime_mode_state_for_tests(
    path: str | Path | None = None,
) -> RuntimeModeState:
    """Replace the process-wide singleton (tests only)."""
    global _STATE
    _STATE = RuntimeModeState(path=path)
    return _STATE
