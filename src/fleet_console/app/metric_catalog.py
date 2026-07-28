"""Load curated metric catalog from config/metric_catalog.yaml."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

from .profiles import REPO_ROOT

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore


def catalog_path() -> Path:
    override = os.environ.get('METRIC_CATALOG_YAML')
    if override:
        return Path(override)
    return REPO_ROOT / 'config' / 'metric_catalog.yaml'


def _normalize_entry(raw: dict[str, Any]) -> dict[str, Any] | None:
    mid = str(raw.get('id') or '').strip()
    label = str(raw.get('label') or '').strip()
    query = str(raw.get('query') or '').strip()
    if not mid or not label or not query:
        return None
    ds = str(raw.get('datasource') or 'prometheus').strip().lower()
    if ds not in ('prometheus', 'loki'):
        ds = 'prometheus'
    viz = str(raw.get('viz') or ('logs' if ds == 'loki' else 'timeseries')).strip()
    category = str(raw.get('category') or 'other').strip() or 'other'
    out: dict[str, Any] = {
        'id': mid,
        'label': label,
        'hint': str(raw.get('hint') or '').strip(),
        'category': category,
        'datasource': ds,
        'query': query,
        'viz': viz,
    }
    if raw.get('unit') is not None:
        out['unit'] = str(raw['unit'])
    if raw.get('min') is not None:
        try:
            out['min'] = float(raw['min'])
        except (TypeError, ValueError):
            pass
    if raw.get('max') is not None:
        try:
            out['max'] = float(raw['max'])
        except (TypeError, ValueError):
            pass
    opts = raw.get('options')
    if isinstance(opts, dict):
        out['options'] = opts
    # Allow top-level thresholds shorthand
    if 'thresholds' in raw and isinstance(raw['thresholds'], list):
        options = dict(out.get('options') or {})
        options['thresholds'] = raw['thresholds']
        out['options'] = options
    return out


def load_metric_catalog() -> list[dict[str, Any]]:
    path = catalog_path()
    if not path.is_file():
        return []
    if yaml is None:
        raise RuntimeError(f'PyYAML is required to load {path}')
    data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    rows = data.get('metrics') if isinstance(data, dict) else None
    if not isinstance(rows, list):
        return []
    out: list[dict[str, Any]] = []
    seen: set[str] = set()
    for row in rows:
        if not isinstance(row, dict):
            continue
        entry = _normalize_entry(row)
        if not entry or entry['id'] in seen:
            continue
        seen.add(entry['id'])
        out.append(entry)
    return out


def get_catalog_entry(metric_id: str) -> dict[str, Any] | None:
    want = (metric_id or '').strip()
    if not want:
        return None
    for entry in load_metric_catalog():
        if entry['id'] == want:
            return entry
    return None
