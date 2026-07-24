"""GitHub API helpers: commits, branches, workflow_dispatch."""
from __future__ import annotations

import json
import os
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any

GITHUB_TOKEN = os.environ.get('GITHUB_TOKEN', '').strip()
GITHUB_REPO = os.environ.get(
    'GITHUB_REPO',
    'ashwanth18/ws_rhapsodi-promtek',
).strip()
CACHE_TTL_SECONDS = int(os.environ.get('GITHUB_CACHE_TTL_SECONDS', '120'))
WORKFLOW_FILE = os.environ.get(
    'GITHUB_BUILD_WORKFLOW',
    'build-and-release.yml',
)

_cache: dict[str, tuple[float, Any]] = {}


def _headers() -> dict[str, str]:
    headers = {
        'Accept': 'application/vnd.github+json',
        'User-Agent': 'rhapsodi-fleet-console',
        'X-GitHub-Api-Version': '2022-11-28',
    }
    if GITHUB_TOKEN:
        headers['Authorization'] = f'Bearer {GITHUB_TOKEN}'
    return headers


def _get_json(url: str) -> Any:
    req = urllib.request.Request(url, headers=_headers())
    try:
        with urllib.request.urlopen(req, timeout=20) as resp:
            return json.loads(resp.read().decode())
    except urllib.error.HTTPError as exc:
        body = exc.read().decode(errors='replace')
        raise RuntimeError(f'GitHub API {exc.code}: {body[:300]}') from exc
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError(f'GitHub API error: {exc}') from exc


def _post_json(url: str, payload: dict[str, Any]) -> Any:
    data = json.dumps(payload).encode()
    headers = _headers()
    headers['Content-Type'] = 'application/json'
    req = urllib.request.Request(url, data=data, headers=headers, method='POST')
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            raw = resp.read().decode()
            return json.loads(raw) if raw.strip() else {}
    except urllib.error.HTTPError as exc:
        body = exc.read().decode(errors='replace')
        raise RuntimeError(f'GitHub API {exc.code}: {body[:300]}') from exc


def latest_commit(branch: str) -> dict[str, Any]:
    """Return {sha, short_sha, html_url, message, date} for branch HEAD."""
    branch = (branch or 'main').strip()
    now = time.time()
    cached = _cache.get(f'commit:{branch}')
    if cached and now - cached[0] < CACHE_TTL_SECONDS:
        return cached[1]

    url = f'https://api.github.com/repos/{GITHUB_REPO}/commits/{branch}'
    payload = _get_json(url)
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
    _cache[f'commit:{branch}'] = (now, result)
    return result


def list_branches(limit: int = 50) -> list[dict[str, Any]]:
    """Return branch names from GitHub (cached)."""
    now = time.time()
    cached = _cache.get('branches')
    if cached and now - cached[0] < CACHE_TTL_SECONDS:
        return cached[1]

    url = (
        f'https://api.github.com/repos/{GITHUB_REPO}/branches'
        f'?per_page={max(1, min(limit, 100))}'
    )
    payload = _get_json(url)
    if not isinstance(payload, list):
        raise RuntimeError('Unexpected branches response')
    out: list[dict[str, Any]] = []
    for item in payload:
        name = item.get('name')
        if not name:
            continue
        commit = item.get('commit') or {}
        sha = str(commit.get('sha') or '')
        out.append(
            {
                'name': name,
                'sha': sha[:7] if sha else '',
                'protected': bool(item.get('protected')),
            }
        )
    out.sort(key=lambda b: (0 if b['name'] == 'main' else 1, b['name']))
    _cache['branches'] = (now, out)
    return out


def compare_commits(base: str, head: str) -> dict[str, Any]:
    """Compare base...head. status: ahead | behind | identical | diverged."""
    base = (base or '').strip()
    head = (head or '').strip()
    if not base or not head:
        return {'status': 'unknown', 'ahead_by': 0, 'behind_by': 0}
    if base == head or base.startswith(head) or head.startswith(base):
        return {'status': 'identical', 'ahead_by': 0, 'behind_by': 0}
    now = time.time()
    key = f'compare:{base}:{head}'
    cached = _cache.get(key)
    if cached and now - cached[0] < CACHE_TTL_SECONDS:
        return cached[1]
    url = (
        f'https://api.github.com/repos/{GITHUB_REPO}/compare/'
        f'{urllib.parse.quote(base)}...{urllib.parse.quote(head)}'
    )
    try:
        payload = _get_json(url)
    except RuntimeError:
        result = {'status': 'unknown', 'ahead_by': 0, 'behind_by': 0}
        _cache[key] = (now, result)
        return result
    result = {
        'status': str(payload.get('status') or 'unknown'),
        'ahead_by': int(payload.get('ahead_by') or 0),
        'behind_by': int(payload.get('behind_by') or 0),
    }
    _cache[key] = (now, result)
    return result


def version_check(tracked_branch: str, deployed_sha: str | None) -> dict[str, Any]:
    """Describe branch tip vs deployed SHA.

    ``branch_ahead`` means the tip has commits not in deployed.
    This is NOT the same as a deployable fleet Release existing.
    """
    latest = latest_commit(tracked_branch)
    deployed = (deployed_sha or '').strip()
    latest_short = latest.get('short_sha') or ''
    cmp = (
        compare_commits(deployed, latest_short)
        if latest_short and deployed
        else {'status': 'unknown', 'ahead_by': 0, 'behind_by': 0}
    )
    branch_ahead = bool(
        latest_short
        and (
            not deployed
            or (
                cmp.get('status') in ('ahead', 'diverged')
                and int(cmp.get('ahead_by') or 0) > 0
            )
        )
    )
    return {
        'tracked_branch': tracked_branch,
        'latest_sha': latest_short,
        'latest_full_sha': latest.get('sha'),
        'latest_message': latest.get('message'),
        'latest_date': latest.get('date'),
        'latest_url': latest.get('html_url'),
        'deployed_sha': deployed or None,
        'compare_status': cmp.get('status'),
        'ahead_by': cmp.get('ahead_by'),
        'behind_by': cmp.get('behind_by'),
        'branch_ahead': branch_ahead,
        # Legacy: do not treat unequal SHAs as an update (tip may be behind).
        'update_available': branch_ahead,
    }


def trigger_workflow(branch: str) -> dict[str, Any]:
    """Dispatch build-and-release.yml for the given branch via workflow_dispatch."""
    if not GITHUB_TOKEN:
        raise RuntimeError('GITHUB_TOKEN is required to trigger CI builds')
    branch = (branch or 'main').strip()
    url = (
        f'https://api.github.com/repos/{GITHUB_REPO}/actions/workflows/'
        f'{WORKFLOW_FILE}/dispatches'
    )
    _post_json(
        url,
        {
            'ref': branch,
            'inputs': {'branch': branch},
        },
    )
    # workflow_dispatch returns 204 with empty body; return a summary.
    return {
        'ok': True,
        'branch': branch,
        'workflow': WORKFLOW_FILE,
        'message': f'Dispatched {WORKFLOW_FILE} for ref={branch}',
        'actions_url': (
            f'https://github.com/{GITHUB_REPO}/actions/workflows/{WORKFLOW_FILE}'
        ),
    }


def latest_workflow_runs(branch: str | None = None, limit: int = 5) -> list[dict[str, Any]]:
    """Recent runs of the build-and-release workflow."""
    url = (
        f'https://api.github.com/repos/{GITHUB_REPO}/actions/workflows/'
        f'{WORKFLOW_FILE}/runs?per_page={max(1, min(limit, 20))}'
    )
    if branch:
        url += f'&branch={urllib.parse.quote(branch)}'
    payload = _get_json(url)
    runs = payload.get('workflow_runs') or []
    out = []
    for run in runs:
        out.append(
            {
                'id': run.get('id'),
                'status': run.get('status'),
                'conclusion': run.get('conclusion'),
                'html_url': run.get('html_url'),
                'head_branch': run.get('head_branch'),
                'head_sha': (run.get('head_sha') or '')[:7],
                'created_at': run.get('created_at'),
                'updated_at': run.get('updated_at'),
            }
        )
    return out
