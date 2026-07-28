"""Thin Prometheus query helpers for fleet alive / host metrics."""
from __future__ import annotations

import json
import time
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

from .settings_store import get as settings_get


def _prometheus_url() -> str:
    return settings_get('prometheus_url', 'http://127.0.0.1:9091').rstrip('/')


def _http_json(url: str, timeout: float = 8.0) -> Any:
    req = Request(url, headers={'Accept': 'application/json'})
    try:
        with urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, OSError):
        return None


def _query(expr: str) -> list[dict[str, Any]]:
    qs = urlencode({'query': expr})
    url = f'{_prometheus_url()}/api/v1/query?{qs}'
    payload = _http_json(url)
    if not payload or payload.get('status') != 'success':
        return []
    return payload.get('data', {}).get('result') or []


def query_range(
    expr: str,
    *,
    start: float | None = None,
    end: float | None = None,
    step: str | float = '30s',
) -> dict[str, Any]:
    """Proxy Prometheus query_range. Returns {status, resultType, result}."""
    now = time.time()
    end_ts = float(end if end is not None else now)
    start_ts = float(start if start is not None else end_ts - 3600)
    params = {
        'query': expr,
        'start': str(start_ts),
        'end': str(end_ts),
        'step': str(step),
    }
    url = f'{_prometheus_url()}/api/v1/query_range?{urlencode(params)}'
    payload = _http_json(url, timeout=20.0)
    if not payload or payload.get('status') != 'success':
        return {'status': 'error', 'resultType': 'matrix', 'result': []}
    data = payload.get('data') or {}
    return {
        'status': 'success',
        'resultType': data.get('resultType') or 'matrix',
        'result': data.get('result') or [],
    }


def query_instant(expr: str) -> dict[str, Any]:
    qs = urlencode({'query': expr})
    payload = _http_json(f'{_prometheus_url()}/api/v1/query?{qs}')
    if not payload or payload.get('status') != 'success':
        return {'status': 'error', 'resultType': 'vector', 'result': []}
    data = payload.get('data') or {}
    return {
        'status': 'success',
        'resultType': data.get('resultType') or 'vector',
        'result': data.get('result') or [],
    }


def _scalar_by_instance(expr: str) -> dict[str, float]:
    out: dict[str, float] = {}
    for item in _query(expr):
        labels = item.get('metric') or {}
        instance = labels.get('instance') or ''
        value = item.get('value')
        if not instance or not value or len(value) < 2:
            continue
        try:
            out[instance] = float(value[1])
        except (TypeError, ValueError):
            continue
    return out


def device_metrics() -> dict[str, dict[str, Any]]:
    """Return per-instance metrics keyed by Prometheus `instance` label."""
    up = _scalar_by_instance('up{job="node"}')
    cpu = _scalar_by_instance(
        '100 - (avg by (instance) (rate(node_cpu_seconds_total{mode="idle"}[2m])) * 100)'
    )
    mem = _scalar_by_instance(
        '(1 - (node_memory_MemAvailable_bytes / node_memory_MemTotal_bytes)) * 100'
    )
    disk = _scalar_by_instance(
        '(1 - (node_filesystem_avail_bytes{mountpoint="/",fstype!="rootfs"} '
        '/ node_filesystem_size_bytes{mountpoint="/",fstype!="rootfs"})) * 100'
    )
    restarts = _scalar_by_instance(
        'sum by (instance) (increase(container_last_seen{name!=""}[1h]) * 0)'
    )
    cadvisor_restarts = _scalar_by_instance(
        'sum by (instance) (container_start_time_seconds{name!=""})'
    )

    keys = set(up) | set(cpu) | set(mem) | set(disk)
    out: dict[str, dict[str, Any]] = {}
    for key in keys:
        out[key] = {
            'alive': up.get(key, 0.0) >= 1.0 if key in up else None,
            'cpu_pct': round(cpu[key], 1) if key in cpu else None,
            'mem_pct': round(mem[key], 1) if key in mem else None,
            'disk_pct': round(disk[key], 1) if key in disk else None,
            'container_starts': cadvisor_restarts.get(key) or restarts.get(key),
        }
    return out


def _series_points(expr: str, start: float, end: float, step: str) -> list[dict[str, Any]]:
    """Collapse a single-series matrix into [{t, v}, ...]."""
    payload = query_range(expr, start=start, end=end, step=step)
    result = payload.get('result') or []
    if not result:
        return []
    # Prefer the first series (queries are usually aggregated by instance).
    values = result[0].get('values') or []
    points: list[dict[str, Any]] = []
    for pair in values:
        if not pair or len(pair) < 2:
            continue
        try:
            points.append({'t': float(pair[0]), 'v': float(pair[1])})
        except (TypeError, ValueError):
            continue
    return points


def device_series(
    instance: str,
    *,
    since_seconds: int = 3600,
    step: str = '30s',
) -> dict[str, Any]:
    """CPU / mem / disk time series for one Prometheus instance (= device_id)."""
    end = time.time()
    start = end - max(60, since_seconds)
    # Escape instance for PromQL label match.
    inst = instance.replace('\\', '\\\\').replace('"', '\\"')
    cpu_expr = (
        f'100 - (avg by (instance) '
        f'(rate(node_cpu_seconds_total{{instance="{inst}",mode="idle"}}[2m])) * 100)'
    )
    mem_expr = (
        f'(1 - (node_memory_MemAvailable_bytes{{instance="{inst}"}} '
        f'/ node_memory_MemTotal_bytes{{instance="{inst}"}})) * 100'
    )
    disk_expr = (
        f'(1 - (node_filesystem_avail_bytes{{instance="{inst}",mountpoint="/",fstype!="rootfs"}} '
        f'/ node_filesystem_size_bytes{{instance="{inst}",mountpoint="/",fstype!="rootfs"}})) * 100'
    )
    return {
        'instance': instance,
        'start': start,
        'end': end,
        'step': step,
        'cpu_pct': _series_points(cpu_expr, start, end, step),
        'mem_pct': _series_points(mem_expr, start, end, step),
        'disk_pct': _series_points(disk_expr, start, end, step),
    }


def fleet_summary() -> dict[str, Any]:
    """Aggregate fleet health for the Monitoring overview page."""
    metrics = device_metrics()
    devices_total = len(metrics)
    devices_up = sum(1 for m in metrics.values() if m.get('alive') is True)
    devices_down = sum(1 for m in metrics.values() if m.get('alive') is False)
    high_mem = [
        inst
        for inst, m in metrics.items()
        if m.get('mem_pct') is not None and float(m['mem_pct']) >= 90
    ]
    high_disk = [
        inst
        for inst, m in metrics.items()
        if m.get('disk_pct') is not None and float(m['disk_pct']) >= 85
    ]
    return {
        'devices_total': devices_total,
        'devices_up': devices_up,
        'devices_down': devices_down,
        'high_mem': high_mem,
        'high_disk': high_disk,
        'devices': metrics,
    }


def _label_api(path: str, params: dict[str, Any] | None = None) -> list[str]:
    qs = urlencode({k: v for k, v in (params or {}).items() if v is not None})
    url = f'{_prometheus_url()}{path}'
    if qs:
        url = f'{url}?{qs}'
    payload = _http_json(url, timeout=12.0)
    if not payload or payload.get('status') != 'success':
        return []
    data = payload.get('data') or []
    if not isinstance(data, list):
        return []
    return sorted({str(v) for v in data if v is not None and str(v)})


def metric_names(prefix: str | None = None, limit: int = 2000) -> list[str]:
    """List metric names from Prometheus label/__name__/values."""
    names = _label_api('/api/v1/label/__name__/values')
    if prefix:
        p = prefix.strip().lower()
        names = [n for n in names if p in n.lower()]
    return names[: max(1, min(int(limit), 5000))]


def label_names(metric: str | None = None) -> list[str]:
    """Label names, optionally scoped to a metric via series match."""
    params: dict[str, Any] = {}
    if metric:
        # Prometheus /api/v1/labels accepts match[]
        params['match[]'] = '{' + f'__name__="{metric.replace(chr(34), "")}"' + '}'
    return _label_api('/api/v1/labels', params)


def label_values(
    label: str,
    *,
    metric: str | None = None,
    match: str | None = None,
) -> list[str]:
    """Values for a label, optionally scoped by metric or match selector."""
    if not label:
        return []
    params: dict[str, Any] = {}
    if match:
        params['match[]'] = match
    elif metric:
        safe = metric.replace('"', '')
        params['match[]'] = '{' + f'__name__="{safe}"' + '}'
    path = f'/api/v1/label/{label}/values'
    return _label_api(path, params)
