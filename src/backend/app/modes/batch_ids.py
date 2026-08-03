"""Incremental session-label helpers for operator batch ids."""

from __future__ import annotations

import re

from ..run_spec import OperatingMode

BATCH_ID_PREFIXES = {
    OperatingMode.LIGHTSOUT.value: 'LO',
    OperatingMode.MOCK_LOCAL.value: 'MOCK',
    OperatingMode.MES_GENERIC.value: 'GEN',
    OperatingMode.MES_CONDOR.value: 'CND',
}


def prefix_for_mode(mode: str) -> str | None:
    return BATCH_ID_PREFIXES.get((mode or '').strip())


def suggest_next_batch_id(
    prefix: str, existing_ids: list[str]
) -> tuple[str, str | None]:
    """Return (next_id, previous_id) from existing labels.

    Ignores free-text ids that do not match ``{PREFIX}-{N}``.
    """
    pattern = re.compile(rf'^{re.escape(prefix)}-(\d+)$', re.IGNORECASE)
    seen_nums: set[int] = set()
    previous_n = 0
    for raw in existing_ids:
        if not raw:
            continue
        match = pattern.match(str(raw).strip())
        if not match:
            continue
        n = int(match.group(1))
        seen_nums.add(n)
        if n > previous_n:
            previous_n = n

    next_n = previous_n + 1 if previous_n > 0 else 1
    while next_n in seen_nums:
        next_n += 1

    batch_id = f'{prefix}-{next_n:04d}'
    previous = f'{prefix}-{previous_n:04d}' if previous_n > 0 else None
    return batch_id, previous
