"""Inbound adapter registry for mes-generic event ingestion."""

from __future__ import annotations

import os
from typing import Any

from .base import InboundAdapter, NormalizedWeightment
from .condor import CondorInboundAdapter
from .generic_json import GenericJsonInboundAdapter

DEFAULT_INBOUND_ADAPTER = 'condor'

_ADAPTERS: dict[str, InboundAdapter] = {
    CondorInboundAdapter.name: CondorInboundAdapter(),
    GenericJsonInboundAdapter.name: GenericJsonInboundAdapter(),
    # Alias: "default" → Condor/Promtek shape.
    'default': CondorInboundAdapter(),
}


def list_inbound_adapters() -> list[str]:
    return sorted({a.name for a in _ADAPTERS.values()})


def get_inbound_adapter_name(
    name: str | None = None,
    *,
    env_var: str = 'MES_GENERIC_INBOUND_ADAPTER',
) -> str:
    raw = name if name is not None else os.environ.get(env_var, '')
    key = (raw or DEFAULT_INBOUND_ADAPTER).strip().lower()
    return key or DEFAULT_INBOUND_ADAPTER


def get_inbound_adapter(
    name: str | None = None,
) -> InboundAdapter:
    key = get_inbound_adapter_name(name)
    adapter = _ADAPTERS.get(key)
    if adapter is None:
        known = ', '.join(list_inbound_adapters())
        raise ValueError(
            f'Unknown inbound adapter {key!r}; known: {known}'
        )
    return adapter


def normalize_inbound_event(
    payload: dict[str, Any],
    *,
    adapter_name: str | None = None,
) -> list[NormalizedWeightment]:
    """Run the configured (or named) adapter against ``payload``."""
    return get_inbound_adapter(adapter_name).normalize(payload)
