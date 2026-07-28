"""Auth helpers for Fleet Console: browser bearer, CI report token, agent token."""
from __future__ import annotations

import secrets

from fastapi import Depends, HTTPException, Request, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from .db import DeviceTarget, SessionLocal
from .settings_store import get as settings_get

_bearer = HTTPBearer(auto_error=False)


def _fleet_api_token() -> str:
    return settings_get('fleet_api_token')


def _ci_report_token() -> str:
    return settings_get('ci_report_token')


def _extract_bearer(
    request: Request,
    creds: HTTPAuthorizationCredentials | None,
) -> str | None:
    if creds is not None and creds.scheme.lower() == 'bearer':
        return creds.credentials
    # EventSource cannot set Authorization headers.
    return request.query_params.get('access_token')


def require_token(
    request: Request,
    creds: HTTPAuthorizationCredentials | None = Depends(_bearer),
) -> None:
    """Browser / operator shared bearer. If unset, allow (local Tailnet trust)."""
    expected = _fleet_api_token()
    if not expected:
        return
    token = _extract_bearer(request, creds)
    if not token or not secrets.compare_digest(token, expected):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail='Missing or invalid bearer token',
        )


def require_ci_token(
    request: Request,
    creds: HTTPAuthorizationCredentials | None = Depends(_bearer),
) -> None:
    """CI-only token for POST /api/releases/report."""
    expected = _ci_report_token() or _fleet_api_token()
    if not expected:
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail='CI_REPORT_TOKEN (or FLEET_API_TOKEN) not configured',
        )
    token = _extract_bearer(request, creds)
    if not token or not secrets.compare_digest(token, expected):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail='Missing or invalid CI report token',
        )


def require_agent(
    request: Request,
    creds: HTTPAuthorizationCredentials | None = Depends(_bearer),
) -> DeviceTarget:
    """Per-device agent token. Returns the matching DeviceTarget row (detached)."""
    token = _extract_bearer(request, creds)
    if not token:
        # Also accept X-Device-Token header.
        token = request.headers.get('X-Device-Token')
    if not token:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail='Missing agent token',
        )
    db = SessionLocal()
    try:
        row = (
            db.query(DeviceTarget)
            .filter(DeviceTarget.agent_token == token)
            .first()
        )
        if row is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail='Invalid agent token',
            )
        db.expunge(row)
        return row
    finally:
        db.close()
