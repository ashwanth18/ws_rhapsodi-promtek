"""Simple shared-bearer auth for Tailnet-only Fleet Console."""
from __future__ import annotations

import os
import secrets

from fastapi import Depends, HTTPException, Request, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer

_bearer = HTTPBearer(auto_error=False)

FLEET_API_TOKEN = os.environ.get('FLEET_API_TOKEN', '').strip()


def require_token(
    request: Request,
    creds: HTTPAuthorizationCredentials | None = Depends(_bearer),
) -> None:
    # If no token is configured, allow (local/dev Tailnet trust).
    if not FLEET_API_TOKEN:
        return
    token = None
    if creds is not None and creds.scheme.lower() == 'bearer':
        token = creds.credentials
    if not token:
        # EventSource cannot set Authorization headers.
        token = request.query_params.get('access_token')
    if not token or not secrets.compare_digest(token, FLEET_API_TOKEN):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail='Missing or invalid bearer token',
        )
