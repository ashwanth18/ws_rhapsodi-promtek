"""Pure validation helpers for lights-out run requests."""

from __future__ import annotations

from typing import Any


def resolve_lightsout_targets(
    *,
    target_mode: str,
    target_weight_g: float | None,
    stop_on: str,
    stop_value: float | None,
    episodes: int,
    target_min_g: float = 0.0,
    target_max_g: float = 0.0,
    powder_default_target_g: float = 250.0,
) -> dict[str, Any]:
    """Validate and resolve target/stop fields.

    Returns ``{'target_weight_g', 'stop_value'}`` or raises ``ValueError``.
    """
    if episodes <= 0:
        raise ValueError('episodes must be > 0')

    if stop_on in ('total_weight_g', 'duration_min') and (
        stop_value is None or float(stop_value) <= 0
    ):
        raise ValueError(f'stop_value must be > 0 when stop_on is {stop_on}')

    resolved_stop = (
        0.0 if stop_on == 'episodes' else float(stop_value or 0.0)
    )

    if target_min_g > 0 and target_max_g > 0 and target_min_g > target_max_g:
        raise ValueError('target_min_g must be <= target_max_g')

    resolved_target = float(target_weight_g or 0.0)
    if target_mode == 'fixed':
        if resolved_target <= 0:
            raise ValueError('target_weight_g must be > 0')
    elif resolved_target <= 0:
        resolved_target = float(powder_default_target_g)

    return {
        'target_weight_g': resolved_target,
        'stop_value': resolved_stop,
    }
