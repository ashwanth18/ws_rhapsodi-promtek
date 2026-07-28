"""Thin Loki query helpers for fleet container logs."""
from __future__ import annotations

import json
import re
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import quote, urlencode
from urllib.request import Request, urlopen

from .settings_store import get as settings_get

_SINCE_RE = re.compile(r'^(\d+)([smhd])$')


def _loki_url() -> str:
    # Accept either base (http://host:3100) or a Promtail push URL mistakenly
    # pasted into settings — normalize to the HTTP API root.
    base = settings_get('loki_url', 'http://127.0.0.1:3100').rstrip('/')
    for suffix in (
        '/loki/api/v1/push',
        '/loki/api/v1',
        '/loki/api',
        '/loki',
    ):
        if base.endswith(suffix):
            base = base[: -len(suffix)]
            break
    return base.rstrip('/')


def _get(path: str, params: dict[str, Any] | None = None, timeout: float = 15.0) -> Any:
    qs = urlencode({k: v for k, v in (params or {}).items() if v is not None})
    url = f'{_loki_url()}{path}'
    if qs:
        url = f'{url}?{qs}'
    req = Request(url, headers={'Accept': 'application/json'})
    try:
        with urlopen(req, timeout=timeout) as resp:
            payload = json.loads(resp.read().decode())
    except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, OSError):
        return None
    return payload


def parse_since(since: str | None, default_seconds: int = 900) -> int:
    """Return unix start time for a since=15m / 1h / 6h / 24h style string."""
    now = int(time.time())
    if not since:
        return now - default_seconds
    m = _SINCE_RE.match(since.strip())
    if not m:
        return now - default_seconds
    n = int(m.group(1))
    unit = m.group(2)
    mult = {'s': 1, 'm': 60, 'h': 3600, 'd': 86400}[unit]
    return now - n * mult


def escape_label(value: str) -> str:
    return value.replace('\\', '\\\\').replace('"', '\\"')


def build_logql(
    host: str,
    *,
    container: str | None = None,
    q: str | None = None,
) -> str:
    h = escape_label(host)
    text = (q or '').strip()
    line_filter = ''
    if text:
        safe = text.replace('\\', '\\\\').replace('"', '\\"')
        line_filter = f' |= "{safe}"'

    if container:
        c = escape_label(container.lstrip('/'))
        # Prefer compose_service (friendly name from Promtail). Fall back to a
        # single-selector regex on the docker container name — avoid LogQL `or`
        # of two selectors (Loki rejects some forms as parse errors).
        if re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_.-]*', container.lstrip('/')):
            # Exact compose service, or container name containing it.
            selector = (
                '{'
                + f'host="{h}",compose_service="{c}"'
                + '}'
            )
            # Also match when the UI picked a full docker name.
            if '-' in c and c.count('-') >= 2:
                selector = '{' + f'host="{h}",container="{c}"' + '}'
            return selector + line_filter
        return '{' + f'host="{h}",container=~".*{c}.*"' + '}' + line_filter

    return '{' + f'host="{h}"' + '}' + line_filter


def query_range(
    logql: str,
    *,
    start: int | float | None = None,
    end: int | float | None = None,
    limit: int = 500,
    direction: str = 'backward',
) -> dict[str, Any]:
    """Proxy Loki query_range. Returns {status, resultType, result} or empty."""
    now = time.time()
    end_ts = float(end if end is not None else now)
    start_ts = float(start if start is not None else end_ts - 900)
    # Loki wants nanoseconds as strings for query_range.
    params = {
        'query': logql,
        'start': str(int(start_ts * 1e9)),
        'end': str(int(end_ts * 1e9)),
        'limit': str(max(1, min(int(limit), 5000))),
        'direction': direction if direction in ('forward', 'backward') else 'backward',
    }
    payload = _get('/loki/api/v1/query_range', params)
    if not payload or payload.get('status') != 'success':
        return {'status': 'error', 'resultType': 'streams', 'result': []}
    data = payload.get('data') or {}
    return {
        'status': 'success',
        'resultType': data.get('resultType') or 'streams',
        'result': data.get('result') or [],
    }


def labels() -> list[str]:
    """All Loki label names."""
    payload = _get('/loki/api/v1/labels')
    if not payload or payload.get('status') != 'success':
        return []
    values = payload.get('data') or []
    return sorted({str(v) for v in values if v})


def label_values(label: str, match: str | None = None) -> list[str]:
    params: dict[str, Any] = {}
    if match:
        # Loki accepts repeated `match[]` — urllib encodes as match%5B%5D=
        params['match[]'] = match
    path = f'/loki/api/v1/label/{quote(label, safe="")}/values'
    payload = _get(path, params)
    if not payload or payload.get('status') != 'success':
        return []
    values = payload.get('data') or []
    return sorted({str(v) for v in values if v})


def containers_for_host(host: str) -> list[str]:
    """List compose services / containers that actually have streams for host.

    Loki's label-values `match[]` is not always host-scoped, so we use the
    series API and extract labels from matching streams.
    """
    match = '{' + f'host="{escape_label(host)}"' + '}'
    # /loki/api/v1/series?match[]={host="..."}
    end = int(time.time())
    start = end - 86400
    payload = _get(
        '/loki/api/v1/series',
        {
            'match[]': match,
            'start': str(start * 1_000_000_000),
            'end': str(end * 1_000_000_000),
        },
    )
    services: list[str] = []
    names: list[str] = []
    seen_svc: set[str] = set()
    seen_name: set[str] = set()
    if payload and payload.get('status') == 'success':
        for item in payload.get('data') or []:
            if not isinstance(item, dict):
                continue
            svc = (item.get('compose_service') or '').strip()
            cont = (item.get('container') or '').lstrip('/').strip()
            if svc and svc not in seen_svc:
                seen_svc.add(svc)
                services.append(svc)
            if cont and cont not in seen_name and cont not in seen_svc:
                seen_name.add(cont)
                names.append(cont)
    if not services and not names:
        # Fallback to label API (may be broader than host).
        names = label_values('container', match=match)
        services = label_values('compose_service', match=match)
    out: list[str] = []
    seen: set[str] = set()
    for name in services + names:
        key = name.lstrip('/')
        if key and key not in seen:
            seen.add(key)
            out.append(key)
    return out


def flatten_lines(result: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Flatten Loki streams into [{ts, container, stream, text}] newest-last."""
    lines: list[dict[str, Any]] = []
    for stream in result:
        labels = stream.get('stream') or {}
        container = (
            labels.get('compose_service')
            or (labels.get('container') or '').lstrip('/')
            or ''
        )
        stream_name = labels.get('stream') or ''
        for entry in stream.get('values') or []:
            if not entry or len(entry) < 2:
                continue
            try:
                # ns → seconds float
                ts = int(entry[0]) / 1e9
            except (TypeError, ValueError):
                continue
            lines.append(
                {
                    'ts': ts,
                    'container': container,
                    'stream': stream_name,
                    'text': entry[1],
                }
            )
    lines.sort(key=lambda row: row['ts'])
    return lines


def device_logs(
    device_id: str,
    *,
    container: str | None = None,
    q: str | None = None,
    since: str | None = '15m',
    start: float | None = None,
    end: float | None = None,
    limit: int = 500,
    direction: str = 'backward',
) -> dict[str, Any]:
    logql = build_logql(device_id, container=container, q=q)
    start_ts = start if start is not None else float(parse_since(since))
    end_ts = end if end is not None else time.time()
    payload = query_range(
        logql,
        start=start_ts,
        end=end_ts,
        limit=limit,
        direction=direction,
    )
    lines = flatten_lines(payload.get('result') or [])
    # When querying backward Loki returns newest first per stream; we sorted
    # ascending. For live-tail UX keep chronological (oldest → newest).
    return {
        'device_id': device_id,
        'query': logql,
        'start': start_ts,
        'end': end_ts,
        'lines': lines,
    }
