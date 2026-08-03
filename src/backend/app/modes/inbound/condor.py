"""Condor / Promtek ``BatchReleasedEvent`` inbound adapter.

Faithful extraction of ``webhook_service.main.build_weightments`` normalization
into pure dicts so mes-generic can reuse the same shape and contract tests can
lock Condor keys/types without touching the live webhook_service path.
"""

from __future__ import annotations

from typing import Any

from .base import NormalizedWeightment


def to_float(value: Any) -> float | None:
    if value is None or value == '':
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def extract_parameter_target(
    parameters: list[dict[str, Any]], parameter_code: str
) -> Any | None:
    for parameter in parameters:
        code = parameter.get('batchInstructionParameterCode') or parameter.get(
            'BatchInstructionParameterCode'
        )
        if code == parameter_code:
            return parameter.get('target') or parameter.get('Target')
    return None


def normalize_condor_batch_released(
    payload: dict[str, Any],
) -> list[NormalizedWeightment]:
    """Normalize a Promtek/Condor BatchReleasedEvent into weightment rows."""
    batch = payload.get('batch') or {}
    lines = batch.get('lines') or []

    event_id = payload.get('eventId') or payload.get('EventId')
    sent_utc = payload.get('sentUtc') or payload.get('SentUtc')
    user_id = payload.get('userId') or payload.get('UserId')
    site_id = payload.get('siteId') or payload.get('SiteId')

    batch_id = batch.get('id')
    batch_number = batch.get('batchNumber')
    work_order_id = batch.get('workOrderId')
    batch_target_quantity = to_float(batch.get('targetQuantity'))

    weightments: list[NormalizedWeightment] = []
    for line in lines:
        parameters = line.get('parameters') or []
        ingredient_id = extract_parameter_target(parameters, 'Ingredient')
        target_weight_kg = to_float(
            extract_parameter_target(parameters, 'Quantity')
        )
        weightments.append(
            NormalizedWeightment(
                event_id=str(event_id) if event_id is not None else '',
                sent_utc=str(sent_utc) if sent_utc is not None else None,
                user_id=str(user_id) if user_id is not None else None,
                site_id=str(site_id) if site_id is not None else None,
                batch_id=str(batch_id) if batch_id is not None else None,
                batch_number=(
                    str(batch_number) if batch_number is not None else None
                ),
                work_order_id=(
                    str(work_order_id) if work_order_id is not None else None
                ),
                batch_target_quantity=batch_target_quantity,
                ingredient_id=(
                    str(ingredient_id) if ingredient_id is not None else None
                ),
                target_weight_kg=target_weight_kg,
            )
        )
    return weightments


class CondorInboundAdapter:
    """Adapter name ``condor`` — default Promtek/Condor webhook payload."""

    name = 'condor'

    def normalize(self, payload: dict[str, Any]) -> list[NormalizedWeightment]:
        return normalize_condor_batch_released(payload)
