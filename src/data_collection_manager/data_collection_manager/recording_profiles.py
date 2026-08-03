"""Loads ``config/recording_profiles.yaml`` (recording-profiles).

Declares which topics belong to which run phase (``pour`` / ``scoop`` /
``transport``) plus an ``always_on`` set — see the YAML file itself for
the full rationale (phase-slicing via existing PhaseMarker events, why
RecorderV2 doesn't do live per-phase topic gating, and the contract with
``src/scooping_controller/config/robots.yaml``).

Resolution order mirrors ``rhapsodi_common.device_config`` for
consistency: explicit path, then well-known on-disk locations, then a
built-in fallback so a missing/broken config file degrades to "record the
historical topic set" instead of taking recording down entirely.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

import yaml

RECORDING_PROFILES_ENV_VAR = 'RHAPSODI_RECORDING_PROFILES'
DEFAULT_RELATIVE_PATH = Path('config') / 'recording_profiles.yaml'

# Historical hardcoded topic list from before recording_profiles.yaml
# existed — used verbatim as the always_on fallback so a missing/broken
# config file degrades to "record what we always recorded", never to
# "record nothing".
_FALLBACK_TOPICS = [
    '/weight',
    '/system_status',
    '/lightsout_training/phase',
    '/lightsout_training/run_id',
    '/lightsout_training/batch_id',
    '/lightsout_training/ingredient_id',
    '/lightsout_training/target_weight_g',
    '/lightsout_training/episode',
    '/lightsout_training/episodes_total',
    '/lightsout_training/mode',
    '/lightsout_training/robot_id',
    '/webhook_run/active',
    '/webhook_run/metadata',
    '/webhook_run/phase',
    '/tf_static',
    '/joint_states',
]


@dataclass(frozen=True)
class RecordingProfiles:
    profiles: Dict[str, List[str]]
    source_path: Optional[Path] = field(default=None)

    @property
    def is_fallback(self) -> bool:
        return self.source_path is None

    def topics_for(self, profile: str) -> List[str]:
        return list(self.profiles.get(profile, []))

    def all_topics(self) -> List[str]:
        """Union of every profile's topics, order-preserving and
        deduplicated — this is what RecorderV2 records for the whole run,
        since it does not do live per-phase topic gating (see module
        docstring)."""
        seen: List[str] = []
        for topics in self.profiles.values():
            for topic in topics:
                if topic not in seen:
                    seen.append(topic)
        return seen


def _candidate_paths(explicit: Optional[Path]) -> List[Path]:
    candidates: List[Path] = []
    if explicit is not None:
        candidates.append(explicit)
    env_value = os.environ.get(RECORDING_PROFILES_ENV_VAR)
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


def _fallback_profiles() -> RecordingProfiles:
    return RecordingProfiles(
        profiles={'always_on': list(_FALLBACK_TOPICS)}, source_path=None
    )


def load_recording_profiles(path: Optional[str] = None) -> RecordingProfiles:
    """Load the phase -> topics contract for this Pi.

    Never raises: any missing file, read error, or parse error is treated
    as "no config found" and falls back to the historical topic set —
    a malformed recording_profiles.yaml must never be the reason
    recording stops, only the reason pour/scoop topics are missing.
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
        if not isinstance(raw, dict):
            continue
        profiles: Dict[str, List[str]] = {}
        for profile_name, profile_body in raw.items():
            if not isinstance(profile_body, dict):
                continue
            topics = profile_body.get('topics')
            if isinstance(topics, list):
                profiles[str(profile_name)] = [str(t) for t in topics]
        if not profiles:
            continue
        return RecordingProfiles(profiles=profiles, source_path=candidate)
    return _fallback_profiles()
