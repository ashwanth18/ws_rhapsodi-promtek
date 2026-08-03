"""Outbound MES / Condor client abstraction.

``CondorMesClient`` posts weighment, batch-end, and timeseries payloads to
the Condor agent URLs (same env vars the backend historically used).
``NullMesClient`` logs and no-ops for mock-local / lights-out.
"""

from __future__ import annotations

import json
import logging
import os
import time
from typing import Any, Protocol, runtime_checkable
from urllib import error, request

from fastapi import HTTPException

from .run_spec import OperatingMode

logger = logging.getLogger('uvicorn.error')

WEIGHMENT_URL = os.environ.get(
    'WEIGHMENT_URL', 'http://localhost:5002/batch/weighment'
)
BATCH_END_URL = os.environ.get('BATCH_END_URL', 'http://localhost:5002/batch/end')
TIMESERIES_URL = os.environ.get(
    'TIMESERIES_URL', 'http://localhost:5002/timeseries'
)
TIMESERIES_TIMEOUT_SECONDS = float(
    os.environ.get('TIMESERIES_TIMEOUT_SECONDS', '60')
)


def post_json(url: str, payload: dict, timeout_seconds: float = 10) -> dict:
    """POST JSON to a downstream URL (shared Condor / adapter helper)."""
    body = json.dumps(payload).encode('utf-8')
    req = request.Request(
        url,
        data=body,
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    started_at = time.monotonic()
    logger.info(
        'Posting downstream JSON: url=%s timeout=%.2fs payload=%s',
        url,
        timeout_seconds,
        json.dumps(payload, sort_keys=True)[:1000],
    )
    try:
        with request.urlopen(req, timeout=timeout_seconds) as response:
            raw = response.read().decode('utf-8')
            logger.info(
                'Downstream JSON succeeded: url=%s status=%s elapsed=%.2fs body=%s',
                url,
                getattr(response, 'status', 'unknown'),
                time.monotonic() - started_at,
                raw[:1000] if raw else '<empty>',
            )
            return json.loads(raw) if raw else {}
    except error.HTTPError as exc:
        detail = exc.read().decode('utf-8')
        logger.error(
            'Downstream JSON failed: url=%s status=%s elapsed=%.2fs body=%s',
            url,
            exc.code,
            time.monotonic() - started_at,
            detail[:1000],
        )
        raise HTTPException(
            status_code=502,
            detail=f'Downstream request failed ({url}): {exc.code} {detail}',
        ) from exc
    except error.URLError as exc:
        logger.error(
            'Downstream JSON failed: url=%s elapsed=%.2fs error=%r',
            url,
            time.monotonic() - started_at,
            exc,
        )
        raise HTTPException(
            status_code=502, detail=f'Downstream request failed ({url}): {exc}'
        ) from exc


@runtime_checkable
class MesClient(Protocol):
    def post_weighment(self, payload: dict[str, Any]) -> dict[str, Any]:
        ...

    def post_batch_end(self, payload: dict[str, Any]) -> dict[str, Any]:
        ...

    def post_timeseries(
        self,
        payload: dict[str, Any],
        *,
        timeout_seconds: float | None = None,
    ) -> dict[str, Any]:
        ...


class CondorMesClient:
    """HTTP client for the Promtek / Condor agent endpoints."""

    def __init__(
        self,
        weighment_url: str = WEIGHMENT_URL,
        batch_end_url: str = BATCH_END_URL,
        timeseries_url: str = TIMESERIES_URL,
        timeseries_timeout_seconds: float = TIMESERIES_TIMEOUT_SECONDS,
    ) -> None:
        self.weighment_url = weighment_url
        self.batch_end_url = batch_end_url
        self.timeseries_url = timeseries_url
        self.timeseries_timeout_seconds = timeseries_timeout_seconds

    def post_weighment(self, payload: dict[str, Any]) -> dict[str, Any]:
        return post_json(self.weighment_url, payload)

    def post_batch_end(self, payload: dict[str, Any]) -> dict[str, Any]:
        return post_json(self.batch_end_url, payload)

    def post_timeseries(
        self,
        payload: dict[str, Any],
        *,
        timeout_seconds: float | None = None,
    ) -> dict[str, Any]:
        return post_json(
            self.timeseries_url,
            payload,
            timeout_seconds=(
                self.timeseries_timeout_seconds
                if timeout_seconds is None
                else timeout_seconds
            ),
        )


class NullMesClient:
    """No-op MES client for mock-local / lights-out (logs only)."""

    def post_weighment(self, payload: dict[str, Any]) -> dict[str, Any]:
        logger.info(
            'NullMesClient: skipping weighment post payload=%s',
            json.dumps(payload, sort_keys=True)[:1000],
        )
        return {'ok': True, 'null': True, 'skipped': True}

    def post_batch_end(self, payload: dict[str, Any]) -> dict[str, Any]:
        logger.info(
            'NullMesClient: skipping batch_end post payload=%s',
            json.dumps(payload, sort_keys=True)[:1000],
        )
        return {'ok': True, 'null': True, 'skipped': True}

    def post_timeseries(
        self,
        payload: dict[str, Any],
        *,
        timeout_seconds: float | None = None,
    ) -> dict[str, Any]:
        logger.info(
            'NullMesClient: skipping timeseries post timeout=%s payload=%s',
            timeout_seconds,
            json.dumps(payload, sort_keys=True)[:1000],
        )
        return {'ok': True, 'null': True, 'skipped': True}


_NULL_MODES = frozenset(
    {
        OperatingMode.MOCK_LOCAL.value,
        OperatingMode.LIGHTSOUT.value,
    }
)


def _normalize_mode(mode: str | OperatingMode | None) -> str:
    if mode is None:
        from .modes.state import get_runtime_mode_state

        return get_runtime_mode_state().mode
    if isinstance(mode, OperatingMode):
        return mode.value
    return str(mode)


def get_mes_client(mode: str | OperatingMode | None = None) -> MesClient:
    """Return the MES client bound to ``mode`` (default: active runtime mode)."""
    mode_id = _normalize_mode(mode)
    if mode_id in _NULL_MODES:
        return NullMesClient()
    # mes-condor and mes-generic (Phase 6 will specialize generic).
    return CondorMesClient()
