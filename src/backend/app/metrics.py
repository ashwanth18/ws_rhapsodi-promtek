"""Shared metric helpers for processed-run APIs."""

from __future__ import annotations


def signed_final_error_g(
    target_weight_g: float | None,
    final_weight_g: float | None,
    fallback: float | None = None,
    *,
    mode: str | None = None,
    net_weight_g: float | None = None,
) -> float | None:
    """Signed pour error (g): poured − target.

    Lights-out is a closed scoop-and-return loop on one vessel, so the
    comparable quantity is net poured mass (final − baseline), not the
    absolute scale reading. MES/mock rows keep the absolute formula.
    """
    if target_weight_g is None:
        return fallback
    if mode == 'lightsout':
        if net_weight_g is None:
            return fallback
        return net_weight_g - target_weight_g
    if final_weight_g is None:
        return fallback
    return final_weight_g - target_weight_g
