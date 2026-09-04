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

    Prefer net poured mass (final − baseline) whenever processing computed
    it — residual powder in the destination vessel must not inflate
    overshoot. Lights-out requires net (absolute scale readings are not
    comparable). When net is unavailable, fall back to absolute final for
    MES/mock rows that never recorded a baseline.
    """
    if target_weight_g is None:
        return fallback
    if net_weight_g is not None:
        return net_weight_g - target_weight_g
    if mode == 'lightsout':
        return fallback
    if final_weight_g is None:
        return fallback
    return final_weight_g - target_weight_g
