"""Minimal .env file helpers for robot-prod.env rendering."""
from __future__ import annotations

from pathlib import Path


def read_env(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    if not path.is_file():
        return out
    for line in path.read_text(encoding='utf-8', errors='replace').splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith('#') or '=' not in stripped:
            continue
        key, _, value = stripped.partition('=')
        out[key.strip()] = value
    return out


def write_env(path: Path, updates: dict[str, str], *, create_missing: bool = True) -> None:
    """Upsert keys in an env file, preserving other lines/comments."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_file():
        lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
    else:
        if not create_missing:
            raise FileNotFoundError(path)
        lines = []

    seen: set[str] = set()
    out: list[str] = []
    for line in lines:
        stripped = line.strip()
        if stripped and not stripped.startswith('#') and '=' in stripped:
            key = stripped.split('=', 1)[0].strip()
            if key in updates:
                out.append(f'{key}={updates[key]}')
                seen.add(key)
                continue
        out.append(line)

    for key, value in updates.items():
        if key not in seen:
            out.append(f'{key}={value}')

    path.write_text('\n'.join(out) + '\n', encoding='utf-8')
