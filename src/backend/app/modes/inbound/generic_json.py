"""Config-mapped flat/simple JSON inbound adapter for non-Promtek MES events."""

from __future__ import annotations

import json
import os
from typing import Any

from .base import NormalizedWeightment
from .condor import to_float

# Internal field → default JSON path (dot-separated) in the request body.
DEFAULT_FIELD_MAP: dict[str, str] = {
    'event_id': 'event_id',
    'sent_utc': 'sent_utc',
    'user_id': 'user_id',
    'site_id': 'site_id',
    'batch_id': 'batch_id',
    'batch_number': 'batch_number',
    'work_order_id': 'work_order_id',
    'batch_target_quantity': 'batch_target_quantity',
    'ingredient_id': 'ingredient_id',
    'target_weight_kg': 'target_weight_kg',
    'lot_code': 'lot_code',
}

_FLOAT_FIELDS = frozenset({'batch_target_quantity', 'target_weight_kg'})


def load_field_map(
    raw: str | None = None,
    *,
    env_var: str = 'MES_GENERIC_FIELD_MAP_JSON',
) -> dict[str, str]:
    """Merge env/JSON field map over defaults. Values are dotted JSON paths."""
    mapping = dict(DEFAULT_FIELD_MAP)
    text = raw if raw is not None else os.environ.get(env_var, '')
    text = (text or '').strip()
    if not text:
        return mapping
    try:
        parsed = json.loads(text)
    except json.JSONDecodeError as exc:
        raise ValueError(
            f'{env_var} must be valid JSON object of internal_field→path: {exc}'
        ) from exc
    if not isinstance(parsed, dict):
        raise ValueError(f'{env_var} must be a JSON object')
    for key, path in parsed.items():
        if not isinstance(key, str) or not isinstance(path, str):
            raise ValueError(
                f'{env_var} entries must be string→string (got {key!r}: {path!r})'
            )
        mapping[key] = path
    return mapping


def get_by_path(payload: dict[str, Any], path: str) -> Any:
    """Resolve a dotted path (e.g. ``batch.id``) against a nested dict."""
    if not path:
        return None
    current: Any = payload
    for part in path.split('.'):
        if not isinstance(current, dict):
            return None
        if part in current:
            current = current[part]
            continue
        # Case-insensitive fallback for Promtek-style PascalCase keys.
        lowered = {str(k).lower(): v for k, v in current.items()}
        if part.lower() in lowered:
            current = lowered[part.lower()]
            continue
        return None
    return current


def _coerce_field(name: str, value: Any) -> Any:
    if value is None or value == '':
        return None
    if name in _FLOAT_FIELDS:
        return to_float(value)
    return str(value)


def map_record(
    record: dict[str, Any], field_map: dict[str, str]
) -> NormalizedWeightment:
    values: dict[str, Any] = {}
    for internal, path in field_map.items():
        if internal not in DEFAULT_FIELD_MAP and internal not in _FLOAT_FIELDS:
            # Allow only known weightment fields.
            if internal not in NormalizedWeightment.__dataclass_fields__:
                continue
        values[internal] = _coerce_field(internal, get_by_path(record, path))
    event_id = values.get('event_id')
    if event_id is None:
        event_id = ''
    return NormalizedWeightment(
        event_id=str(event_id),
        sent_utc=values.get('sent_utc'),
        user_id=values.get('user_id'),
        site_id=values.get('site_id'),
        batch_id=values.get('batch_id'),
        batch_number=values.get('batch_number'),
        work_order_id=values.get('work_order_id'),
        batch_target_quantity=values.get('batch_target_quantity'),
        ingredient_id=values.get('ingredient_id'),
        target_weight_kg=values.get('target_weight_kg'),
        lot_code=values.get('lot_code'),
    )


def normalize_generic_json(
    payload: dict[str, Any],
    field_map: dict[str, str] | None = None,
    *,
    lines_path: str | None = None,
) -> list[NormalizedWeightment]:
    """Normalize a flat JSON body (or list-of-lines) via field map.

    If ``lines_path`` (or mapped path ``lines`` / env
    ``MES_GENERIC_LINES_PATH``) points at a list, each element is mapped as a
    weightment (event-level fields merge under the line). Otherwise the whole
    body maps to a single weightment.
    """
    mapping = field_map if field_map is not None else load_field_map()
    path = lines_path
    if path is None:
        path = os.environ.get('MES_GENERIC_LINES_PATH', '').strip() or None
    if path is None and 'lines' in mapping:
        path = mapping.get('lines')

    if path:
        lines = get_by_path(payload, path)
        if isinstance(lines, list):
            # Event-level fields from root; line fields override.
            results: list[NormalizedWeightment] = []
            for line in lines:
                if not isinstance(line, dict):
                    continue
                merged = dict(payload)
                merged.update(line)
                results.append(map_record(merged, mapping))
            return results

    return [map_record(payload, mapping)]


class GenericJsonInboundAdapter:
    """Adapter name ``generic_json`` — config-mapped simple JSON bodies."""

    name = 'generic_json'

    def __init__(self, field_map: dict[str, str] | None = None) -> None:
        self._field_map = field_map

    def normalize(self, payload: dict[str, Any]) -> list[NormalizedWeightment]:
        mapping = (
            self._field_map if self._field_map is not None else load_field_map()
        )
        return normalize_generic_json(payload, mapping)
