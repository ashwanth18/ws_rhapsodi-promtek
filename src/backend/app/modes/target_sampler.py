"""Session-level target fraction schedules for closed-loop lights-out."""
from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Literal

TargetMode = Literal['fixed', 'random_fraction', 'stratified']


@dataclass(frozen=True)
class TargetSchedule:
    target_mode: TargetMode
    fractions: list[float]
    rng_seed: int
    fixed_target_g: float | None = None


def generate_target_schedule(
    *,
    cycles: int,
    target_mode: str = 'stratified',
    frac_min: float = 0.4,
    frac_max: float = 0.9,
    fixed_target_g: float | None = None,
    rng_seed: int | None = None,
) -> TargetSchedule:
    """Build a per-episode fraction (or fixed grams) schedule.

    For ``fixed``, fractions are all 1.0 and ``fixed_target_g`` is used
    as the absolute target by the BT. For ``random_fraction`` /
    ``stratified``, the BT multiplies each fraction by measured
    ``scooped_mass_g``.
    """
    if cycles <= 0:
        raise ValueError('cycles must be > 0')
    mode = (target_mode or 'stratified').strip().lower()
    if mode not in ('fixed', 'random_fraction', 'stratified'):
        raise ValueError(f'Unknown target_mode: {target_mode}')
    if frac_min < 0 or frac_max <= 0 or frac_min > frac_max:
        raise ValueError('frac_min/frac_max invalid')
    if frac_max > 1.0:
        raise ValueError('frac_max must be <= 1.0')

    seed = int(rng_seed) if rng_seed is not None else random.SystemRandom().randint(
        0, 2**31 - 1
    )
    rng = random.Random(seed)

    if mode == 'fixed':
        if fixed_target_g is None or fixed_target_g <= 0:
            raise ValueError('fixed_target_g required for target_mode=fixed')
        return TargetSchedule(
            target_mode='fixed',
            fractions=[1.0] * cycles,
            rng_seed=seed,
            fixed_target_g=float(fixed_target_g),
        )

    if mode == 'random_fraction':
        fractions = [
            rng.uniform(frac_min, frac_max) for _ in range(cycles)
        ]
        return TargetSchedule(
            target_mode='random_fraction',
            fractions=fractions,
            rng_seed=seed,
        )

    # stratified: one draw per equal-width bin, then shuffle
    width = (frac_max - frac_min) / cycles
    fractions = []
    for i in range(cycles):
        lo = frac_min + i * width
        hi = lo + width
        fractions.append(rng.uniform(lo, hi))
    rng.shuffle(fractions)
    return TargetSchedule(
        target_mode='stratified',
        fractions=fractions,
        rng_seed=seed,
    )
