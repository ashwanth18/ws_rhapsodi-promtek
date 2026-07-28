"""Live connectivity checks for console settings / tokens."""
from __future__ import annotations

import json
import urllib.error
import urllib.request
from typing import Any

from .settings_store import SECRET_KEYS, get_all


def _http_json(
    url: str,
    *,
    headers: dict[str, str] | None = None,
    timeout: float = 8.0,
) -> tuple[int, Any]:
    req = urllib.request.Request(url, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode(errors='replace')
            code = int(resp.status)
            if not raw.strip():
                return code, None
            try:
                return code, json.loads(raw)
            except json.JSONDecodeError:
                return code, raw[:200]
    except urllib.error.HTTPError as exc:
        body = exc.read().decode(errors='replace')[:300]
        try:
            payload = json.loads(body) if body.strip().startswith('{') else body
        except json.JSONDecodeError:
            payload = body
        return int(exc.code), payload
    except Exception as exc:  # noqa: BLE001
        return 0, str(exc)


def _http_probe(url: str, *, timeout: float = 8.0) -> tuple[int, str]:
    req = urllib.request.Request(url, method='GET')
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return int(resp.status), ''
    except urllib.error.HTTPError as exc:
        return int(exc.code), (exc.reason or '')[:120]
    except Exception as exc:  # noqa: BLE001
        return 0, str(exc)


def _ok(message: str, **extra: Any) -> dict[str, Any]:
    return {'ok': True, 'message': message, **extra}


def _fail(message: str, **extra: Any) -> dict[str, Any]:
    return {'ok': False, 'message': message, **extra}


def _merged(overrides: dict[str, str] | None) -> dict[str, str]:
    data = get_all()
    if not overrides:
        return data
    for key, value in overrides.items():
        if value is None:
            continue
        text = str(value).strip()
        if not text:
            continue
        # Ignore masked placeholders from the UI.
        if '…' in text or text.startswith('••••'):
            continue
        data[key] = text
    return data


def check_github(settings: dict[str, str]) -> dict[str, Any]:
    token = (settings.get('github_token') or '').strip()
    repo = (settings.get('github_repo') or '').strip()
    if not token:
        return _fail('GitHub token not set')
    if not repo or '/' not in repo:
        return _fail('GitHub repo must look like owner/name')

    headers = {
        'Accept': 'application/vnd.github+json',
        'Authorization': f'Bearer {token}',
        'User-Agent': 'rhapsodi-fleet-console',
        'X-GitHub-Api-Version': '2022-11-28',
    }
    code, user = _http_json('https://api.github.com/user', headers=headers)
    if code != 200 or not isinstance(user, dict):
        detail = user.get('message') if isinstance(user, dict) else user
        return _fail(f'GitHub auth failed (HTTP {code}): {detail}')

    login = user.get('login') or '?'
    wf_url = (
        f'https://api.github.com/repos/{repo}/actions/workflows/'
        'build-and-release.yml'
    )
    wcode, wf = _http_json(wf_url, headers=headers)
    if wcode != 200 or not isinstance(wf, dict):
        detail = wf.get('message') if isinstance(wf, dict) else wf
        return _fail(
            f'Authenticated as {login}, but cannot read workflow on {repo} '
            f'(HTTP {wcode}): {detail}'
        )
    return _ok(
        f'Connected as {login}; workflow "{wf.get("name")}" is {wf.get("state")}',
        login=login,
        repo=repo,
        workflow=wf.get('name'),
        workflow_state=wf.get('state'),
    )


def check_prometheus(settings: dict[str, str]) -> dict[str, Any]:
    base = (settings.get('prometheus_url') or '').strip().rstrip('/')
    if not base:
        return _fail('Prometheus URL not set')
    code, body = _http_json(f'{base}/api/v1/query?query=up', timeout=5.0)
    if code == 0:
        return _fail(f'Unreachable: {body}')
    if code != 200:
        return _fail(f'HTTP {code}: {body}')
    if isinstance(body, dict) and body.get('status') == 'success':
        n = len((body.get('data') or {}).get('result') or [])
        return _ok(f'Reachable; up query returned {n} series')
    return _fail(f'Unexpected response: {str(body)[:160]}')


def check_grafana(settings: dict[str, str]) -> dict[str, Any]:
    url = (settings.get('grafana_pi_overview_url') or '').strip()
    if not url:
        return _fail('Grafana URL not set')
    code, reason = _http_probe(url, timeout=5.0)
    if code == 0:
        return _fail(f'Unreachable: {reason}')
    # 200/302/401 all mean the host answered.
    if 200 <= code < 500:
        return _ok(f'Reachable (HTTP {code})')
    return _fail(f'HTTP {code}: {reason}')


def check_loki(settings: dict[str, str]) -> dict[str, Any]:
    base = (settings.get('loki_url') or '').strip().rstrip('/')
    if not base:
        return _fail('Loki URL not set')
    code, reason = _http_probe(f'{base}/ready', timeout=5.0)
    if code == 0:
        return _fail(f'Unreachable: {reason}')
    if code == 200:
        return _ok('Ready')
    if 200 <= code < 500:
        return _ok(f'Reachable (HTTP {code})')
    return _fail(f'HTTP {code}: {reason}')


def check_alertmanager(settings: dict[str, str]) -> dict[str, Any]:
    base = (settings.get('alertmanager_url') or '').strip().rstrip('/')
    if not base:
        return _fail('Alertmanager URL not set')
    code, body = _http_json(f'{base}/api/v2/status', timeout=5.0)
    if code == 0:
        return _fail(f'Unreachable: {body}')
    if code == 200:
        cluster = ''
        if isinstance(body, dict):
            cluster = (body.get('cluster') or {}).get('status') or ''
        msg = 'Reachable'
        if cluster:
            msg = f'Reachable; cluster {cluster}'
        return _ok(msg)
    return _fail(f'HTTP {code}: {body}')


def check_console_url(settings: dict[str, str]) -> dict[str, Any]:
    base = (settings.get('fleet_console_url') or '').strip().rstrip('/')
    if not base:
        return _fail('Public console URL not set')
    code, body = _http_json(f'{base}/health', timeout=5.0)
    if code == 0:
        return _fail(f'Unreachable from this host: {body}')
    if code == 200 and isinstance(body, dict) and body.get('status') == 'ok':
        return _ok(f'Health OK at {base}/health')
    return _fail(f'HTTP {code}: {body}')


def check_image_registry(settings: dict[str, str]) -> dict[str, Any]:
    registry = (settings.get('image_registry') or '').strip()
    if not registry:
        return _fail('Image registry not set')
    # Hub library-style namespace/repo — public tag list is enough as a smoke test.
    # Private repos may 401; treat that as "reachable + auth needed", still useful.
    if '/' not in registry:
        return _fail('Expected namespace/repo (e.g. iserenity/rhapsodi-promtek)')
    ns, name = registry.split('/', 1)
    url = f'https://hub.docker.com/v2/repositories/{ns}/{name}/'
    code, body = _http_json(url, timeout=8.0)
    if code == 0:
        return _fail(f'Unreachable: {body}')
    if code == 200 and isinstance(body, dict):
        return _ok(
            f'Docker Hub repo found ({body.get("pull_count", "?")} pulls)',
            name=body.get('name'),
        )
    if code in (401, 404):
        # 404 can mean private / missing; still proves DNS + Hub API path.
        return _ok(
            f'Docker Hub responded HTTP {code} for {registry} '
            '(ok if private; confirm name if unexpected)',
        )
    return _fail(f'HTTP {code}: {body}')


def check_operator_token(
    settings: dict[str, str],
    *,
    request_token: str | None,
) -> dict[str, Any]:
    expected = (settings.get('fleet_api_token') or '').strip()
    if not expected:
        return _ok('Not set — console allows unauthenticated access')
    if not request_token:
        return _fail('Token is set, but this request had no bearer')
    if request_token == expected:
        return _ok('Session bearer matches configured operator token')
    return _fail('Session bearer does not match configured operator token')


def check_ci_report_token(settings: dict[str, str]) -> dict[str, Any]:
    token = (settings.get('ci_report_token') or '').strip()
    api = (settings.get('fleet_api_token') or '').strip()
    if token:
        return _ok(
            'Configured — CI must send this as Bearer on /api/releases/report '
            '(cannot probe outbound; GitHub Actions calls us)'
        )
    if api:
        return _ok(
            'Not set — falls back to operator API token for CI report auth'
        )
    return _fail('Neither CI report token nor operator API token is set')


def run_all(
    overrides: dict[str, str] | None = None,
    *,
    request_token: str | None = None,
) -> dict[str, Any]:
    settings = _merged(overrides)
    checks = {
        'github': check_github(settings),
        'prometheus': check_prometheus(settings),
        'loki': check_loki(settings),
        'alertmanager': check_alertmanager(settings),
        'grafana': check_grafana(settings),
        'fleet_console_url': check_console_url(settings),
        'image_registry': check_image_registry(settings),
        'fleet_api_token': check_operator_token(
            settings, request_token=request_token
        ),
        'ci_report_token': check_ci_report_token(settings),
    }
    return {
        'ok': all(item.get('ok') for item in checks.values()),
        'checks': checks,
        'used_draft_overrides': bool(
            overrides
            and any(
                str(overrides.get(k) or '').strip()
                and '…' not in str(overrides.get(k))
                and not str(overrides.get(k)).startswith('••••')
                for k in list(settings) + list(SECRET_KEYS)
            )
        ),
    }
