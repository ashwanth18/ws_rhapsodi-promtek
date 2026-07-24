"""Single source of truth for edge-device identity.

Every Rhapsodi edge node (ROS 2 python nodes, the uplink daemon, the
recorder) should read robot/device/site identity through this module
instead of hardcoding defaults like ``"robot-1"`` or a ``localhost`` URL
directly in node source. Config file resolution order:

1. Explicit ``path`` argument.
2. ``RHAPSODI_DEVICE_CONFIG`` environment variable.
3. ``config/device.yaml`` discovered by walking up from this file's
   location, or under ``/ws`` (the workspace mount point used by
   docker-compose *.yml), or under the current working directory.
4. Built-in fallback values, identical to the historical hardcoded
   defaults, so a Pi with no device.yaml still behaves like the original
   single-robot deployment instead of failing to record.

`device_id` should match the Tailscale hostname used for fleet monitoring
(see scripts/generate_prom_targets.sh) so health/fault data and
infrastructure metrics correlate on the same identifier.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import yaml

DEVICE_CONFIG_ENV_VAR = 'RHAPSODI_DEVICE_CONFIG'
DEFAULT_RELATIVE_PATH = Path('config') / 'device.yaml'

# Historical hardcoded defaults from before this module existed. Kept as the
# last-resort fallback so nothing regresses when device.yaml is absent.
_FALLBACK_DEVICE_ID = 'robot-1'
_FALLBACK_ROBOT_ID = 'robot-1'
_FALLBACK_ROBOT_TYPE = 'niryo'
_FALLBACK_SITE_ID = 'site-1'
_FALLBACK_PROCESSING_URL = 'http://localhost:8002/process'
_FALLBACK_INGESTION_URL = 'http://localhost:8011/ingest'


@dataclass(frozen=True)
class DeviceConfig:
    device_id: str
    robot_id: str
    robot_type: str
    site_id: str
    processing_url: str
    ingestion_url: str
    source_path: Optional[Path] = field(default=None)

    @property
    def is_fallback(self) -> bool:
        return self.source_path is None


def _candidate_paths(explicit: Optional[Path]) -> List[Path]:
    candidates: List[Path] = []
    if explicit is not None:
        candidates.append(explicit)
    env_value = os.environ.get(DEVICE_CONFIG_ENV_VAR)
    if env_value:
        candidates.append(Path(env_value))
    here = Path(__file__).resolve()
    for ancestor in [here.parent, *here.parents]:
        candidates.append(ancestor / DEFAULT_RELATIVE_PATH)
    # Common in-container mount point used by docker-compose *.yml (bind
    # mounts ./config -> /ws/config alongside the rest of the workspace).
    candidates.append(Path('/ws') / DEFAULT_RELATIVE_PATH)
    candidates.append(Path.cwd() / DEFAULT_RELATIVE_PATH)
    return candidates


def _fallback_config() -> DeviceConfig:
    return DeviceConfig(
        device_id=_FALLBACK_DEVICE_ID,
        robot_id=_FALLBACK_ROBOT_ID,
        robot_type=_FALLBACK_ROBOT_TYPE,
        site_id=_FALLBACK_SITE_ID,
        processing_url=_FALLBACK_PROCESSING_URL,
        ingestion_url=_FALLBACK_INGESTION_URL,
        source_path=None,
    )


def load_device_config(path: Optional[str] = None) -> DeviceConfig:
    """Load this device's identity + fleet endpoints.

    Never raises: any missing file, read error, or parse error is treated
    as "no config found" and falls back to the historical single-robot
    defaults. Device-identity loading must never be the reason data
    collection goes down.
    """
    explicit = Path(path) if path else None
    seen: set = set()
    for candidate in _candidate_paths(explicit):
        resolved = str(candidate)
        if resolved in seen:
            continue
        seen.add(resolved)
        try:
            if not candidate.is_file():
                continue
            raw = yaml.safe_load(candidate.read_text()) or {}
        except (OSError, yaml.YAMLError):
            continue
        device = raw.get('device') if isinstance(raw, dict) else None
        if not isinstance(device, dict):
            continue
        fleet = device.get('fleet')
        fleet = fleet if isinstance(fleet, dict) else {}
        return DeviceConfig(
            device_id=str(device.get('device_id', _FALLBACK_DEVICE_ID)),
            robot_id=str(device.get('robot_id', _FALLBACK_ROBOT_ID)),
            robot_type=str(device.get('robot_type', _FALLBACK_ROBOT_TYPE)),
            site_id=str(device.get('site_id', _FALLBACK_SITE_ID)),
            processing_url=str(
                fleet.get('processing_url', _FALLBACK_PROCESSING_URL)
            ),
            ingestion_url=str(
                fleet.get('ingestion_url', _FALLBACK_INGESTION_URL)
            ),
            source_path=candidate,
        )
    return _fallback_config()
