"""Auth helpers for Fleet Console: browser bearer, CI report token, agent token."""
from __future__ import annotations

import os
import secrets

from fastapi import Depends, HTTPException, Request, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

from .db import DeviceTarget, SessionLocal

_bearer = HTTPBearer(auto_error=False)

FLEET_API_TOKEN = os.environ.get('FLEET_API_TOKEN', '').strip()
CI_REPORT_TOKEN = os.environ.get('CI_REPORT_TOKEN', '').strip()


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
    if not FLEET_API_TOKEN:
        return
    token = _extract_bearer(request, creds)
    if not token or not secrets.compare_digest(token, FLEET_API_TOKEN):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail='Missing or invalid bearer token',
        )


def require_ci_token(
    request: Request,
    creds: HTTPAuthorizationCredentials | None = Depends(_bearer),
) -> None:
    """CI-only token for POST /api/releases/report."""
    if not CI_REPORT_TOKEN:
        # Fall back to FLEET_API_TOKEN so a single-token setup still works.
        expected = FLEET_API_TOKEN
    else:
        expected = CI_REPORT_TOKEN
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
