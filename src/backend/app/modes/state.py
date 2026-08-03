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
DEFAULT_ENVIRONMENT = Environment.REAL
DEFAULT_RUNTIME_MODE_PATH = '/data/runtime_mode.json'


def _sim_allowed() -> bool:
    raw = os.environ.get('SIM_ALLOWED', '0').strip().lower()
    return raw in {'1', 'true', 'yes', 'on'}


class RuntimeModeState:
    """Persist ``{mode, environment}`` under RUNTIME_MODE_PATH.

    Falls back to an in-memory dict when the path is missing/unwritable
    (unit tests, fresh containers without a ``/data`` mount).
    """

    def __init__(self, path: str | Path | None = None) -> None:
        env_path = os.environ.get('RUNTIME_MODE_PATH', DEFAULT_RUNTIME_MODE_PATH)
        self._path = Path(path if path is not None else env_path)
        self._lock = threading.Lock()
        self._memory: dict[str, str] = {
            'mode': DEFAULT_MODE.value,
            'environment': DEFAULT_ENVIRONMENT.value,
        }
        self._use_memory_only = False
        self._loaded = False

    @property
    def path(self) -> Path:
        return self._path

    def _default_payload(self) -> dict[str, str]:
        return {
            'mode': DEFAULT_MODE.value,
            'environment': DEFAULT_ENVIRONMENT.value,
        }

    def _normalize(self, payload: dict[str, Any]) -> dict[str, str]:
        mode_raw = str(payload.get('mode') or DEFAULT_MODE.value)
        env_raw = str(payload.get('environment') or DEFAULT_ENVIRONMENT.value)
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


def sim_allowed() -> bool:
    return _sim_allowed()
