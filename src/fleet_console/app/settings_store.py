"""Persistent console settings (operator-editable) stored under FLEET_DATA_DIR.

Env vars remain the bootstrap defaults. Values saved in the UI override env
for the running process. CI-only secrets (Docker Hub, Tailscale, Jetson SSH)
stay in GitHub Actions — they are never stored here.
"""
from __future__ import annotations

import json
import os
import threading
from pathlib import Path
from typing import Any

_LOCK = threading.Lock()

# Non-secret keys shown in full; secret keys are masked on read.
PUBLIC_KEYS = (
    'fleet_console_url',
    'image_registry',
    'github_repo',
    'prometheus_url',
    'loki_url',
    'alertmanager_url',
    'grafana_pi_overview_url',
)
SECRET_KEYS = (
    'fleet_api_token',
    'ci_report_token',
    'github_token',
)

ALL_KEYS = PUBLIC_KEYS + SECRET_KEYS


def _settings_path() -> Path:
    data = Path(os.environ.get('FLEET_DATA_DIR', '/data'))
    data.mkdir(parents=True, exist_ok=True)
    return data / 'console_settings.json'


def _env_defaults() -> dict[str, str]:
    return {
        'fleet_api_token': os.environ.get('FLEET_API_TOKEN', '').strip(),
        'ci_report_token': os.environ.get('CI_REPORT_TOKEN', '').strip(),
        'fleet_console_url': os.environ.get(
            'FLEET_CONSOLE_URL', 'http://127.0.0.1:8090'
        ).strip(),
        'image_registry': os.environ.get(
            'IMAGE_REGISTRY', 'iserenity/rhapsodi-promtek'
        ).strip(),
        'github_token': os.environ.get('GITHUB_TOKEN', '').strip(),
        'github_repo': os.environ.get(
            'GITHUB_REPO', 'ashwanth18/ws_rhapsodi-promtek'
        ).strip(),
        'prometheus_url': os.environ.get(
            'PROMETHEUS_URL', 'http://127.0.0.1:9091'
        ).strip(),
        'loki_url': os.environ.get('LOKI_URL', 'http://127.0.0.1:3100').strip(),
        'alertmanager_url': os.environ.get(
            'ALERTMANAGER_URL', 'http://127.0.0.1:9093'
        ).strip(),
        'grafana_pi_overview_url': os.environ.get(
            'GRAFANA_PI_OVERVIEW_URL',
            'http://127.0.0.1:3001/d/pi-overview/rhapsodi-pi-overview',
        ).strip(),
    }


def _read_file() -> dict[str, str]:
    path = _settings_path()
    if not path.is_file():
        return {}
    try:
        raw = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(raw, dict):
        return {}
    out: dict[str, str] = {}
    for key, value in raw.items():
        if key in ALL_KEYS and value is not None:
            out[key] = str(value)
    return out


def _write_file(data: dict[str, str]) -> None:
    path = _settings_path()
    path.write_text(json.dumps(data, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass


def get_all() -> dict[str, str]:
    """Merged settings: file overrides env defaults."""
    with _LOCK:
        merged = _env_defaults()
        merged.update(_read_file())
        return merged


def get(key: str, default: str = '') -> str:
    return get_all().get(key, default) or default


def update(patch: dict[str, Any]) -> dict[str, str]:
    """Apply patch. Empty string clears a key back to env default on next merge
    by storing empty; omit secret keys or send null to leave unchanged.
    Secret sentinel '__UNCHANGED__' means keep existing file/env value.
    """
    with _LOCK:
        current = _env_defaults()
        current.update(_read_file())
        stored = _read_file()
        for key, value in patch.items():
            if key not in ALL_KEYS:
                continue
            if value is None or value == '__UNCHANGED__':
                continue
            text = str(value).strip()
            stored[key] = text
            current[key] = text
        _write_file(stored)
        return current


def mask_secret(value: str) -> str | None:
    if not value:
        return None
    if len(value) <= 8:
        return '••••••••'
    return value[:4] + '…' + value[-4:]


def public_view() -> dict[str, Any]:
    """Safe payload for the Settings UI."""
    data = get_all()
    out: dict[str, Any] = {
        'auth_required': bool(data.get('fleet_api_token')),
        'values': {},
        'secrets_set': {},
        'github_actions_secrets': [
            'DOCKERHUB_USERNAME',
            'DOCKERHUB_TOKEN',
            'TAILSCALE_AUTHKEY',
            'JETSON_SSH_HOST',
            'JETSON_SSH_USER',
            'JETSON_SSH_KEY',
            'FLEET_CONSOLE_URL',
            'CI_REPORT_TOKEN',
        ],
        'note': (
            'Console settings below are stored on this host under FLEET_DATA_DIR. '
            'GitHub Actions build secrets are managed in the repo Settings → Secrets '
            'and are not edited here.'
        ),
    }
    for key in PUBLIC_KEYS:
        out['values'][key] = data.get(key) or ''
    for key in SECRET_KEYS:
        val = data.get(key) or ''
        out['secrets_set'][key] = bool(val)
        out['values'][key] = mask_secret(val)
    return out
