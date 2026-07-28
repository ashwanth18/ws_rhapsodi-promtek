"""Merge Prometheus firing alerts with Alertmanager state."""
from __future__ import annotations

import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

from .prometheus import query_instant
from .settings_store import get as settings_get


def _alertmanager_url() -> str:
    return settings_get('alertmanager_url', 'http://127.0.0.1:9093').rstrip('/')


def _http_json(url: str, timeout: float = 8.0) -> Any:
    req = Request(url, headers={'Accept': 'application/json'})
    try:
        with urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode())
    except (HTTPError, URLError, TimeoutError, json.JSONDecodeError, OSError):
        return None


def _from_prometheus() -> list[dict[str, Any]]:
    """ALERTS metric — state=firing|pending."""
    payload = query_instant('ALERTS')
    out: list[dict[str, Any]] = []
    for item in payload.get('result') or []:
        labels = dict(item.get('metric') or {})
        labels.pop('__name__', None)
        alertname = labels.pop('alertname', None) or 'unknown'
        state = labels.pop('alertstate', None) or 'firing'
        value = item.get('value')
        active_at = None
        if value and len(value) >= 1:
            try:
                active_at = float(value[0])
            except (TypeError, ValueError):
                active_at = None
        instance = labels.get('instance') or labels.get('device') or ''
        out.append(
            {
                'source': 'prometheus',
                'alertname': alertname,
                'state': state,
                'severity': labels.get('severity') or 'none',
                'instance': instance,
                'labels': labels,
                'annotations': {},
                'active_at': active_at,
                'silenced': False,
                'inhibited': False,
            }
        )
    return out


def _from_alertmanager() -> list[dict[str, Any]]:
    payload = _http_json(f'{_alertmanager_url()}/api/v2/alerts')
    if not isinstance(payload, list):
        return []
    out: list[dict[str, Any]] = []
    for item in payload:
        labels = dict(item.get('labels') or {})
        annotations = dict(item.get('annotations') or {})
        status = item.get('status') or {}
        state = status.get('state') or 'active'
        # Alertmanager: active | suppressed | unprocessed
        silenced = bool(status.get('silencedBy'))
        inhibited = bool(status.get('inhibitedBy'))
        out.append(
            {
                'source': 'alertmanager',
                'alertname': labels.get('alertname') or 'unknown',
                'state': 'silenced' if silenced else state,
                'severity': labels.get('severity') or 'none',
                'instance': labels.get('instance') or labels.get('device') or '',
                'labels': labels,
                'annotations': annotations,
                'active_at': item.get('startsAt'),
                'silenced': silenced,
                'inhibited': inhibited,
                'fingerprint': item.get('fingerprint'),
                'receivers': item.get('receivers') or [],
            }
        )
    return out


def _key(alert: dict[str, Any]) -> tuple[str, str]:
    return (str(alert.get('alertname') or ''), str(alert.get('instance') or ''))


def list_alerts(*, instance: str | None = None) -> list[dict[str, Any]]:
    """Prefer Alertmanager (silence-aware); fill gaps from Prometheus ALERTS."""
    am = _from_alertmanager()
    prom = _from_prometheus()
    by_key: dict[tuple[str, str], dict[str, Any]] = {}
    for alert in prom:
        by_key[_key(alert)] = alert
    for alert in am:
        # Alertmanager wins (has silence / annotation detail).
        by_key[_key(alert)] = alert
    alerts = list(by_key.values())
    if instance:
        want = instance.strip().lower()
        alerts = [
            a
            for a in alerts
            if (a.get('instance') or '').lower() == want
            or want in (a.get('instance') or '').lower()
        ]
    # Firing / active first, then by name.
    def sort_key(a: dict[str, Any]) -> tuple[int, str]:
        state = (a.get('state') or '').lower()
        rank = 0 if state in ('firing', 'active') else 1 if state == 'pending' else 2
        return (rank, str(a.get('alertname') or ''))

    alerts.sort(key=sort_key)
    return alerts


def alert_counts_by_instance() -> dict[str, int]:
    counts: dict[str, int] = {}
    for alert in list_alerts():
        state = (alert.get('state') or '').lower()
        if state in ('silenced', 'suppressed'):
            continue
        if state not in ('firing', 'active', 'pending'):
            continue
        inst = alert.get('instance') or ''
        if not inst:
            continue
        counts[inst] = counts.get(inst, 0) + 1
    return counts
