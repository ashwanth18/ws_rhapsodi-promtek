"""Load runtime profile + device_class catalogs from config/."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore


def _default_repo_root() -> Path:
    here = Path(__file__).resolve()
    try:
        return here.parents[3]
    except IndexError:
        return Path('/repo')


REPO_ROOT = Path(os.environ['REPO_ROOT'] if os.environ.get('REPO_ROOT') else _default_repo_root())

DEFAULT_COMPOSE = 'compose/devices/pi5.yml'
DEFAULT_DEVICE_CLASS = 'pi5'


def profiles_path() -> Path:
    override = os.environ.get('PROFILES_YAML')
    if override:
        return Path(override)
    return REPO_ROOT / 'config' / 'profiles.yaml'


def device_classes_path() -> Path:
    override = os.environ.get('DEVICE_CLASSES_YAML')
    if override:
        return Path(override)
    return REPO_ROOT / 'config' / 'device_classes.yaml'


def _load_yaml_map(path: Path, root_key: str) -> dict[str, Any]:
    if not path.is_file():
        return {}
    raw = path.read_text(encoding='utf-8')
    if yaml is None:
        raise RuntimeError(f'PyYAML is required to load {path}')
    data = yaml.safe_load(raw) or {}
    section = data.get(root_key) if isinstance(data, dict) else None
    return section if isinstance(section, dict) else {}


def load_profiles() -> dict[str, Any]:
    return _load_yaml_map(profiles_path(), 'profiles')


def load_device_classes() -> dict[str, Any]:
    return _load_yaml_map(device_classes_path(), 'device_classes')


def get_device_class(class_id: str | None) -> dict[str, Any] | None:
    if not class_id:
        return None
    definition = load_device_classes().get(class_id)
    if not isinstance(definition, dict):
        return None
    return {
        'id': class_id,
        'description': definition.get('description') or '',
        'platform': definition.get('platform'),
        'compose_file': definition.get('compose_file') or DEFAULT_COMPOSE,
        'production': bool(definition.get('production', True)),
    }


def compose_for_device_class(class_id: str | None) -> str:
    """Resolve compose path for a device_class (defaults to pi5)."""
    info = get_device_class(class_id) or get_device_class(DEFAULT_DEVICE_CLASS)
    if info and info.get('compose_file'):
        return str(info['compose_file'])
    return DEFAULT_COMPOSE


def list_device_classes(*, production_only: bool = False) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for class_id, definition in load_device_classes().items():
        if not isinstance(definition, dict):
            continue
        item = get_device_class(class_id)
        if item is None:
            continue
        if production_only and not item.get('production'):
            continue
        out.append(item)
    out.sort(key=lambda p: p['id'])
    return out


def list_profiles(
    robot_type: str | None = None,
    device_class: str | None = None,
) -> list[dict[str, Any]]:
    out: list[dict[str, Any]] = []
    for profile_id, definition in load_profiles().items():
        if not isinstance(definition, dict):
            continue
        rtype = definition.get('robot_type')
        if robot_type and rtype and rtype != robot_type:
            continue
        allowed = definition.get('device_classes') or []
        if isinstance(allowed, str):
            allowed = [allowed]
        if device_class and allowed and device_class not in allowed:
            continue
        out.append(_serialize_profile(profile_id, definition))
    out.sort(key=lambda p: p['id'])
    return out


def get_profile(profile_id: str) -> dict[str, Any] | None:
    definition = load_profiles().get(profile_id)
    if not isinstance(definition, dict):
        return None
    return _serialize_profile(profile_id, definition)


def _serialize_profile(profile_id: str, definition: dict[str, Any]) -> dict[str, Any]:
    allowed = definition.get('device_classes') or []
    if isinstance(allowed, str):
        allowed = [allowed]
    # Legacy profiles may still list compose_file; prefer device_class resolution.
    return {
        'id': profile_id,
        'description': definition.get('description') or '',
        'robot_type': definition.get('robot_type'),
        'device_classes': [str(x) for x in allowed] if allowed else [],
        'compose_file': definition.get('compose_file'),  # legacy / unused for hardware
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


def profile_allows_device_class(profile_id: str | None, device_class: str | None) -> bool:
    if not profile_id or not device_class:
        return True
    profile = get_profile(profile_id)
    if not profile:
        return True
    allowed = profile.get('device_classes') or []
    if not allowed:
        return True
    return device_class in allowed
