"""OCI platform helpers for device ↔ release matching.

robot_type (niryo/jaka) is OEM/stack. platform (linux/arm64) is what Docker
must pull. device_class (pi5/jetson/x86) is form-factor for operators / future
compose variants — both Pi and Jetson are usually arm64 today.
"""
from __future__ import annotations

import json
import re
import subprocess
from typing import Any

KNOWN_DEVICE_CLASSES = ('pi5', 'jetson', 'x86', 'unknown')


def normalize_platform(value: str | None) -> str | None:
    if not value:
        return None
    text = value.strip().lower().replace('_', '/')
    if not text:
        return None
    # Already OCI-ish.
    if text.startswith('linux/'):
        arch = text.split('/', 1)[1]
        if arch in ('arm64', 'aarch64'):
            return 'linux/arm64'
        if arch in ('amd64', 'x86_64', 'x64'):
            return 'linux/amd64'
        if arch in ('arm', 'arm/v7', 'armhf'):
            return 'linux/arm/v7'
        return f'linux/{arch}'
    # uname -m / dpkg style
    if text in ('aarch64', 'arm64'):
        return 'linux/arm64'
    if text in ('x86_64', 'amd64', 'x64'):
        return 'linux/amd64'
    if text in ('armv7l', 'armhf', 'arm'):
        return 'linux/arm/v7'
    return None


def parse_platforms(value: Any) -> list[str]:
    """Normalize a platforms field from CI (list, CSV string, or timings)."""
    if value is None:
        return []
    items: list[str] = []
    if isinstance(value, list):
        items = [str(x) for x in value]
    elif isinstance(value, str):
        items = re.split(r'[\s,]+', value.strip())
    else:
        return []
    out: list[str] = []
    seen: set[str] = set()
    for item in items:
        plat = normalize_platform(item)
        if plat and plat not in seen:
            seen.add(plat)
            out.append(plat)
    return out


def platforms_from_timings(timings: dict[str, Any] | None) -> list[str]:
    if not timings:
        return []
    return parse_platforms(timings.get('platforms'))


def infer_device_class(
    *,
    hostname: str | None = None,
    device_id: str | None = None,
    platform: str | None = None,
) -> str:
    blob = f'{hostname or ""} {device_id or ""}'.lower()
    if any(tok in blob for tok in ('pi5', 'raspberry', 'rpi', 'raspi')):
        return 'pi5'
    if any(tok in blob for tok in ('jetson', 'orin', 'tegra')):
        return 'jetson'
    plat = normalize_platform(platform)
    if plat == 'linux/amd64':
        return 'x86'
    if plat == 'linux/arm64':
        # Arm robot without a stronger hostname cue — treat as pi-class default.
        return 'pi5'
    return 'unknown'


def release_platforms(release_like: Any) -> list[str]:
    """Extract platforms from a Release ORM row or serialized dict."""
    if release_like is None:
        return []
    # Explicit column / field.
    raw = getattr(release_like, 'platforms', None)
    if raw is None and isinstance(release_like, dict):
        raw = release_like.get('platforms')
    if isinstance(raw, str) and raw.strip().startswith('['):
        try:
            raw = json.loads(raw)
        except json.JSONDecodeError:
            pass
    parsed = parse_platforms(raw)
    if parsed:
        return parsed
    # Fallback: build_timings JSON on the row or dict.
    timings_raw = getattr(release_like, 'timings', None)
    if timings_raw is None and isinstance(release_like, dict):
        timings_raw = release_like.get('build_timings') or release_like.get('timings')
    if isinstance(timings_raw, str):
        try:
            timings_raw = json.loads(timings_raw)
        except json.JSONDecodeError:
            timings_raw = None
    if isinstance(timings_raw, dict):
        return platforms_from_timings(timings_raw)
    return []


def release_supports_platform(release_like: Any, device_platform: str | None) -> bool:
    """True if release lists the device platform, or lists nothing (unknown)."""
    want = normalize_platform(device_platform)
    if not want:
        return True
    have = release_platforms(release_like)
    if not have:
        return True  # unknown → allow; registry probe can tighten
    return want in have


def parse_device_classes(value: Any) -> list[str]:
    if value is None:
        return []
    items: list[str] = []
    if isinstance(value, list):
        items = [str(x) for x in value]
    elif isinstance(value, str):
        text = value.strip()
        if text.startswith('['):
            try:
                parsed = json.loads(text)
                if isinstance(parsed, list):
                    items = [str(x) for x in parsed]
                else:
                    items = re.split(r'[\s,]+', text)
            except json.JSONDecodeError:
                items = re.split(r'[\s,]+', text)
        else:
            items = re.split(r'[\s,]+', text)
    out: list[str] = []
    seen: set[str] = set()
    for item in items:
        name = item.strip().lower()
        if not name or name in seen:
            continue
        seen.add(name)
        out.append(name)
    return out


def release_device_classes(release_like: Any) -> list[str]:
    if release_like is None:
        return []
    raw = getattr(release_like, 'device_classes', None)
    if raw is None and isinstance(release_like, dict):
        raw = release_like.get('device_classes')
    return parse_device_classes(raw)


def release_supports_device_class(
    release_like: Any,
    device_class: str | None,
) -> bool:
    """True if release allow-lists the class, or lists no classes (unknown)."""
    want = (device_class or '').strip().lower()
    if not want or want == 'unknown':
        return True
    have = release_device_classes(release_like)
    if not have:
        return True
    return want in have


def _manifest_platforms(repository: str, tag: str) -> list[str] | None:
    """Return platforms from `docker manifest inspect`, or None if unavailable."""
    ref = f'{repository}:{tag}'
    try:
        result = subprocess.run(
            ['docker', 'manifest', 'inspect', ref],
            check=False,
            capture_output=True,
            text=True,
            timeout=45,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None
    try:
        payload = json.loads(result.stdout or '{}')
    except json.JSONDecodeError:
        return None
    found: list[str] = []
    manifests = payload.get('manifests')
    if isinstance(manifests, list):
        for item in manifests:
            plat = (item or {}).get('platform') or {}
            os_name = plat.get('os') or 'linux'
            arch = plat.get('architecture')
            variant = plat.get('variant')
            if not arch:
                continue
            if variant and arch == 'arm':
                key = f'{os_name}/{arch}/{variant}'
            else:
                key = f'{os_name}/{arch}'
            norm = normalize_platform(key)
            if norm and norm not in found:
                found.append(norm)
        return found
    # Single-arch image: top-level architecture.
    arch = payload.get('architecture')
    os_name = payload.get('os') or 'linux'
    if arch:
        norm = normalize_platform(f'{os_name}/{arch}')
        return [norm] if norm else []
    return []


def manifest_supports_platform(
    repository: str,
    tag: str,
    device_platform: str | None,
) -> tuple[bool | None, str]:
    """Check registry manifest for device platform.

    Returns (True/False/None, reason). None = check unavailable.
    """
    want = normalize_platform(device_platform)
    if not want:
        return True, 'no device platform set'
    plats = _manifest_platforms(repository, tag)
    if plats is None:
        return None, 'manifest inspect unavailable'
    if not plats:
        return None, 'manifest had no platform metadata'
    if want in plats:
        return True, f'{want} present in {repository}:{tag} ({", ".join(plats)})'
    return False, (
        f'{repository}:{tag} has [{", ".join(plats)}] but device needs {want}'
    )
