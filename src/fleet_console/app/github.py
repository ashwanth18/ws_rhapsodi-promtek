"""GitHub commit polling for branch tracking / update-available."""
from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.request
from typing import Any

GITHUB_TOKEN = os.environ.get('GITHUB_TOKEN', '').strip()
GITHUB_REPO = os.environ.get(
    'GITHUB_REPO',
    'ashwanth18/ws_rhapsodi-promtek',
).strip()
CACHE_TTL_SECONDS = int(os.environ.get('GITHUB_CACHE_TTL_SECONDS', '120'))

_cache: dict[str, tuple[float, dict[str, Any]]] = {}


def _headers() -> dict[str, str]:
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'rhapsodi-fleet-console',
    }
    if GITHUB_TOKEN:
        headers['Authorization'] = f'Bearer {GITHUB_TOKEN}'
    return headers


def latest_commit(branch: str) -> dict[str, Any]:
    """Return {sha, short_sha, html_url, message, date} for branch HEAD."""
    branch = (branch or 'main').strip()
    now = time.time()
    cached = _cache.get(branch)
    if cached and now - cached[0] < CACHE_TTL_SECONDS:
        return cached[1]

    url = f'https://api.github.com/repos/{GITHUB_REPO}/commits/{branch}'
    req = urllib.request.Request(url, headers=_headers())
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            payload = json.loads(resp.read().decode())
    except urllib.error.HTTPError as exc:
        body = exc.read().decode(errors='replace')
        raise RuntimeError(f'GitHub API {exc.code} for {branch}: {body[:200]}') from exc
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f'GitHub API error for {branch}: {exc}') from exc

    sha = str(payload.get('sha') or '')
    commit = payload.get('commit') or {}
    result = {
        'sha': sha,
        'short_sha': sha[:7] if sha else '',
        'html_url': payload.get('html_url'),
        'message': (commit.get('message') or '').splitlines()[0] if commit else '',
        'date': ((commit.get('author') or {}).get('date')),
        'branch': branch,
    }
    _cache[branch] = (now, result)
    return result


def version_check(tracked_branch: str, deployed_sha: str | None) -> dict[str, Any]:
    latest = latest_commit(tracked_branch)
    deployed = (deployed_sha or '').strip()
    latest_short = latest.get('short_sha') or ''
    update_available = bool(
        latest_short and deployed and not (
            deployed == latest_short
            or deployed.startswith(latest_short)
            or (latest.get('sha') or '').startswith(deployed)
        )
    )
    # If nothing deployed yet, treat latest as available.
    if latest_short and not deployed:
        update_available = True
    return {
        'tracked_branch': tracked_branch,
        'latest_sha': latest_short,
        'latest_full_sha': latest.get('sha'),
        'latest_message': latest.get('message'),
        'latest_date': latest.get('date'),
        'latest_url': latest.get('html_url'),
        'deployed_sha': deployed or None,
        'update_available': update_available,
    }
