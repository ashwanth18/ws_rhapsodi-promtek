"""Lifecycle helpers for staged export bundles under /data/exports."""
from __future__ import annotations

import logging
import os
import time
from pathlib import Path

LOG = logging.getLogger('exports_dir')

DEFAULT_EXPORTS_SUBDIR = 'exports'
DEFAULT_MAX_AGE_SECONDS = 6 * 60 * 60  # 6 hours


def resolve_data_output_root() -> Path:
    """Resolve DATA_OUTPUT_ROOT, shared with mode/export code paths."""
    try:
        from app.modes.state import resolve_data_output_root as _resolve

        return Path(_resolve())
    except Exception:  # noqa: BLE001
        raw = os.environ.get('DATA_OUTPUT_ROOT', '/data/runs').strip()
        return Path(raw or '/data/runs')


def exports_root() -> Path:
    """Return ``{DATA_OUTPUT_ROOT.parent}/exports`` (sibling of runs/)."""
    data_root = resolve_data_output_root()
    # /data/runs -> /data/exports; fall back to sibling under same parent.
    parent = data_root.parent if data_root.name == 'runs' else data_root
    return parent / DEFAULT_EXPORTS_SUBDIR


def ensure_exports_dir() -> Path:
    root = exports_root()
    root.mkdir(parents=True, exist_ok=True)
    return root


def sweep_stale_exports(
    max_age_seconds: int = DEFAULT_MAX_AGE_SECONDS,
) -> int:
    """Delete staged export files older than ``max_age_seconds``.

    Returns the number of files removed.
    """
    root = ensure_exports_dir()
    cutoff = time.time() - max_age_seconds
    removed = 0
    for path in root.iterdir():
        if not path.is_file():
            continue
        try:
            if path.stat().st_mtime < cutoff:
                path.unlink(missing_ok=True)
                removed += 1
        except OSError as exc:
            LOG.warning('Failed to sweep export %s: %s', path, exc)
    if removed:
        LOG.info('Swept %s stale export file(s) from %s', removed, root)
    return removed
