import json
import os
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotTargetMapping:
    pickup_target_name: str
    weigh_target_name: str = 'MoveToWeighingContainer'
    return_target_name: str = 'ReturnHome'
    weight_tolerance_g: float = 0.5


DEFAULT_TARGET_MAPPING = RobotTargetMapping(
    pickup_target_name='MoveToScoopingContainer'
)


def _normalize_key(value: str | int | None) -> str | None:
    if value is None:
        return None
    key = str(value).strip()
    return key or None


def _coerce_mapping(value: object) -> RobotTargetMapping | None:
    if not isinstance(value, dict):
        return None
    pickup_target_name = str(
        value.get('pickup_target_name')
        or value.get('container_name')
        or value.get('target_name')
        or DEFAULT_TARGET_MAPPING.pickup_target_name
    ).strip()
    weigh_target_name = str(
        value.get('weigh_target_name') or DEFAULT_TARGET_MAPPING.weigh_target_name
    ).strip()
    return_target_name = str(
        value.get('return_target_name') or DEFAULT_TARGET_MAPPING.return_target_name
    ).strip()
    try:
        weight_tolerance_g = float(
            value.get('weight_tolerance_g')
            or DEFAULT_TARGET_MAPPING.weight_tolerance_g
        )
    except (TypeError, ValueError):
        weight_tolerance_g = DEFAULT_TARGET_MAPPING.weight_tolerance_g
    return RobotTargetMapping(
        pickup_target_name=pickup_target_name,
        weigh_target_name=weigh_target_name,
        return_target_name=return_target_name,
        weight_tolerance_g=weight_tolerance_g,
    )


def _load_location_target_mappings() -> dict[str, RobotTargetMapping]:
    raw = os.environ.get('ROBOT_LOCATION_TARGETS_JSON', '').strip()
    if not raw:
        return {}
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return {}
    if not isinstance(payload, dict):
        return {}
    mappings: dict[str, RobotTargetMapping] = {}
    for key, value in payload.items():
        normalized = _normalize_key(key)
        mapping = _coerce_mapping(value)
        if normalized and mapping is not None:
            mappings[normalized] = mapping
    return mappings


LOCATION_TARGET_MAPPINGS = _load_location_target_mappings()


def resolve_robot_targets(
    location_id: int | None, location_code: str | None
) -> RobotTargetMapping:
    keys = []
    if location_code is not None:
        keys.append(location_code)
    if location_id is not None:
        keys.append(location_id)
    for key in keys:
        normalized = _normalize_key(key)
        if normalized and normalized in LOCATION_TARGET_MAPPINGS:
            return LOCATION_TARGET_MAPPINGS[normalized]
    wildcard = LOCATION_TARGET_MAPPINGS.get('*')
    if wildcard is not None:
        return wildcard
    return DEFAULT_TARGET_MAPPING
