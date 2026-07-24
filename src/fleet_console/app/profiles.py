"""Load runtime profile catalog from config/profiles.yaml."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore

REPO_ROOT = Path(os.environ.get('REPO_ROOT', str(Path(__file__).resolve().parents[3])))


def profiles_path() -> Path:
    override = os.environ.get('PROFILES_YAML')
    if override:
        return Path(override)
    return REPO_ROOT / 'config' / 'profiles.yaml'


def load_profiles() -> dict[str, Any]:
    path = profiles_path()
    if not path.is_file():
        return {}
    raw = path.read_text(encoding='utf-8')
    if yaml is None:
        raise RuntimeError('PyYAML is required to load config/profiles.yaml')
    data = yaml.safe_load(raw) or {}
    profiles = data.get('profiles') if isinstance(data, dict) else None
    return profiles if isinstance(profiles, dict) else {}


def list_profiles(robot_type: str | None = None) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for profile_id, definition in load_profiles().items():
        if not isinstance(definition, dict):
            continue
        rtype = definition.get('robot_type')
        if robot_type and rtype and rtype != robot_type:
            continue
        out.append(
            {
                'id': profile_id,
                'description': definition.get('description') or '',
                'robot_type': rtype,
                'compose_file': definition.get('compose_file'),
                'env': definition.get('env') or {},
            }
        )
    out.sort(key=lambda p: p['id'])
    return out


def get_profile(profile_id: str) -> dict[str, Any] | None:
    definition = load_profiles().get(profile_id)
    if not isinstance(definition, dict):
        return None
    return {
        'id': profile_id,
        'description': definition.get('description') or '',
        'robot_type': definition.get('robot_type'),
        'compose_file': definition.get('compose_file'),
        'env': definition.get('env') or {},
    }


def list_robot_types() -> list[str]:
    """Distinct robot_type values from the profiles catalog."""
    types: set[str] = set()
    for definition in load_profiles().values():
        if not isinstance(definition, dict):
            continue
        rtype = definition.get('robot_type')
        if rtype:
            types.add(str(rtype))
    return sorted(types)
