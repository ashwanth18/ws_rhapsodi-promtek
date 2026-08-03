"""Track an in-flight lights-out training session for mode arbitration.

Lights-out runs are not ``RobotWeightmentRun`` rows, so ModeManager would
otherwise allow a mode switch while the orchestrator is still ticking.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from pathlib import Path
from typing import Any

logger = logging.getLogger('uvicorn.error')

DEFAULT_SESSION_PATH = '/data/lightsout_session.json'
# Auto-clear stuck sessions (robot power-cycle mid-run, lost rosbridge).
DEFAULT_TTL_SECONDS = 6 * 60 * 60
LIGHTSOUT_ROSBRIDGE_RUN_ID = -1


def _session_path() -> Path:
    return Path(
        os.environ.get('LIGHTSOUT_SESSION_PATH', DEFAULT_SESSION_PATH)
    )


def _ttl_seconds() -> float:
    raw = os.environ.get('LIGHTSOUT_SESSION_TTL_SECONDS', str(DEFAULT_TTL_SECONDS))
    try:
        return max(60.0, float(raw))
    except ValueError:
        return float(DEFAULT_TTL_SECONDS)


class LightsoutSessionState:
    def __init__(self, path: str | Path | None = None) -> None:
        self._path = Path(path) if path is not None else _session_path()
        self._lock = threading.Lock()
        self._memory: dict[str, Any] | None = None

    @property
    def path(self) -> Path:
        return self._path

    def _read(self) -> dict[str, Any] | None:
        try:
            if not self._path.is_file():
                return None
            with self._path.open('r', encoding='utf-8') as handle:
                data = json.load(handle)
            if not isinstance(data, dict) or not data.get('active'):
                return None
            return data
        except (OSError, json.JSONDecodeError, TypeError) as exc:
            logger.warning('Unable to read lightsout session %s: %s', self._path, exc)
            return None

    def _write(self, payload: dict[str, Any] | None) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            if payload is None:
                if self._path.is_file():
                    self._path.unlink()
                return
            tmp = self._path.with_suffix(self._path.suffix + '.tmp')
            with tmp.open('w', encoding='utf-8') as handle:
                json.dump(payload, handle, indent=2, sort_keys=True)
                handle.write('\n')
            tmp.replace(self._path)
        except OSError as exc:
            logger.warning(
                'Unable to persist lightsout session %s: %s (memory only)',
                self._path,
                exc,
            )

    def mark_started(self, request: dict[str, Any]) -> dict[str, Any]:
        payload = {
            'active': True,
            'started_at': time.time(),
            'request': request,
        }
        with self._lock:
            self._memory = payload
            self._write(payload)
        return dict(payload)

    def clear(self) -> None:
        with self._lock:
            self._memory = None
            self._write(None)

    def get_active(self) -> dict[str, Any] | None:
        with self._lock:
            payload = self._memory if self._memory is not None else self._read()
            if payload is None:
                return None
            started = float(payload.get('started_at') or 0.0)
            if started and (time.time() - started) > _ttl_seconds():
                logger.warning(
                    'Clearing stale lightsout session started_at=%.0f', started
                )
                self._memory = None
                self._write(None)
                return None
            self._memory = payload
            return dict(payload)

    def as_blocker(self) -> Any | None:
        """Synthetic active_run object for ModeSwitchConflict."""
        active = self.get_active()
        if active is None:
            return None
        return type(
            'LightsoutSessionBlocker',
            (),
            {
                'id': LIGHTSOUT_ROSBRIDGE_RUN_ID,
                'status': 'running',
                'weightment_id': None,
                'event_id': 'lightsout-session',
                'batch_id': (active.get('request') or {}).get('batch_id'),
                'kind': 'lightsout',
            },
        )()


_SESSION: LightsoutSessionState | None = None


def get_lightsout_session() -> LightsoutSessionState:
    global _SESSION
    if _SESSION is None:
        _SESSION = LightsoutSessionState()
    return _SESSION


def reset_lightsout_session_for_tests(
    path: str | Path | None = None,
) -> LightsoutSessionState:
    global _SESSION
    _SESSION = LightsoutSessionState(path=path)
    return _SESSION
