"""Thin Prometheus query helpers for fleet alive / host metrics."""
from __future__ import annotations

import os
from typing import Any
from urllib.parse import urlencode
from urllib.request import urlopen

PROMETHEUS_URL = os.environ.get('PROMETHEUS_URL', 'http://127.0.0.1:9091').rstrip(
    '/'
)


def _query(expr: str) -> list[dict[str, Any]]:
    qs = urlencode({'query': expr})
    url = f'{PROMETHEUS_URL}/api/v1/query?{qs}'
    try:
        with urlopen(url, timeout=5) as resp:
            payload = __import__('json').loads(resp.read().decode())
    except Exception:
        return []
    if payload.get('status') != 'success':
        return []
    return payload.get('data', {}).get('result') or []


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
        '(1 - (node_filesystem_avail_bytes{mountpoint="/"} / node_filesystem_size_bytes{mountpoint="/"})) * 100'
    )
    restarts = _scalar_by_instance(
        'sum by (instance) (increase(container_last_seen{name!=""}[1h]) * 0)'
        # Placeholder; prefer cadvisor restart count when available:
    )
    # Prefer a more useful restart proxy if cadvisor is scraped:
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
