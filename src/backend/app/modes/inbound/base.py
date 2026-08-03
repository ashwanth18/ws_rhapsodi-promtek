"""Shared inbound adapter contract → internal weightment shape."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Protocol, runtime_checkable


@dataclass(frozen=True)
class NormalizedWeightment:
    """Internal weightment fields used by the webhook start / DB path.

    Keys match ``webhook_weightments`` columns populated by Condor's
    ``BatchReleasedEvent`` parser (see ``webhook_service.build_weightments``).
    """

    event_id: str
    sent_utc: str | None = None
    user_id: str | None = None
    site_id: str | None = None
    batch_id: str | None = None
    batch_number: str | None = None
    work_order_id: str | None = None
    batch_target_quantity: float | None = None
    ingredient_id: str | None = None
    target_weight_kg: float | None = None
    lot_code: str | None = None

    def as_row_dict(self) -> dict[str, Any]:
        """Dict suitable for ``WebhookWeightment(**…)`` construction."""
        return asdict(self)


@runtime_checkable
class InboundAdapter(Protocol):
    """Parse an external event body into zero or more weightments."""

    name: str

    def normalize(self, payload: dict[str, Any]) -> list[NormalizedWeightment]:
        ...
