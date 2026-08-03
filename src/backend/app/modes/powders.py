"""Powder catalog loader for lightsout / mock-local labeling."""
from __future__ import annotations

import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_NUMERIC_ID = re.compile(r'^\d+$')


@dataclass(frozen=True)
class Powder:
    id: str
    name: str
    container_target: str
    pour_target: str
    default_target_weight_g: float = 250.0
    default_tolerance_g: float = 5.0
    min_scooped_g: float = 20.0
    density_g_per_ml: float | None = None
    notes: str = ''

    def as_dict(self) -> dict[str, Any]:
        return {
            'id': self.id,
            'name': self.name,
            'container_target': self.container_target,
            'pour_target': self.pour_target,
            'default_target_weight_g': self.default_target_weight_g,
            'default_tolerance_g': self.default_tolerance_g,
            'min_scooped_g': self.min_scooped_g,
            'density_g_per_ml': self.density_g_per_ml,
            'notes': self.notes,
        }


def _default_catalog_path() -> Path:
    env = os.environ.get('POWDERS_CATALOG', '').strip()
    if env:
        return Path(env)
    # Prefer /ws/config (compose mount), then repo-relative config/.
    for candidate in (
        Path('/ws/config/powders.yaml'),
        Path(__file__).resolve().parents[4] / 'config' / 'powders.yaml',
        Path('config/powders.yaml'),
    ):
        if candidate.is_file():
            return candidate
    return Path('/ws/config/powders.yaml')


def validate_powder_id(powder_id: str) -> str:
    pid = (powder_id or '').strip()
    if not pid:
        raise ValueError('powder_id is required')
    if _NUMERIC_ID.match(pid):
        raise ValueError(
            f'powder_id {pid!r} must not be purely numeric '
            '(collides with Condor stock-item ids)'
        )
    return pid


def _parse_powder(raw: dict[str, Any]) -> Powder:
    if not isinstance(raw, dict):
        raise ValueError('powder entry must be a mapping')
    pid = validate_powder_id(str(raw.get('id') or ''))
    name = str(raw.get('name') or '').strip()
    if not name:
        raise ValueError(f'powder {pid!r} missing name')
    container = str(raw.get('container_target') or '').strip()
    if not container:
        raise ValueError(f'powder {pid!r} missing container_target')
    pour = str(raw.get('pour_target') or container).strip()
    if not pour:
        raise ValueError(f'powder {pid!r} missing pour_target')
    density = raw.get('density_g_per_ml')
    return Powder(
        id=pid,
        name=name,
        container_target=container,
        pour_target=pour,
        default_target_weight_g=float(
            raw.get('default_target_weight_g') or 250.0
        ),
        default_tolerance_g=float(raw.get('default_tolerance_g') or 5.0),
        min_scooped_g=float(raw.get('min_scooped_g') or 20.0),
        density_g_per_ml=float(density) if density is not None else None,
        notes=str(raw.get('notes') or ''),
    )


def load_powders(path: Path | None = None) -> list[Powder]:
    catalog = path or _default_catalog_path()
    if not catalog.is_file():
        return []
    data = yaml.safe_load(catalog.read_text(encoding='utf-8')) or {}
    entries = data.get('powders') or []
    if not isinstance(entries, list):
        raise ValueError('powders.yaml: top-level "powders" must be a list')
    powders = [_parse_powder(entry) for entry in entries]
    ids = [p.id for p in powders]
    if len(ids) != len(set(ids)):
        raise ValueError('powders.yaml: duplicate powder id')
    return powders


def get_powder(powder_id: str, path: Path | None = None) -> Powder:
    pid = validate_powder_id(powder_id)
    for powder in load_powders(path):
        if powder.id == pid:
            return powder
    raise KeyError(f'Unknown powder_id: {pid}')
