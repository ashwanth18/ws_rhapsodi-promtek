"""Incident-detection rules engine: known failure signatures -> structured
`incidents` rows (incident-detection).

Runs over the same record shape in both places incidents get detected
from - a fleet-wide `/v1/fleet/{device_id}/health` line, and a per-run
`events.jsonl` line synced via Tier 0 - because both are literally the
same JSONL `HealthEvent` schema produced by
`rhapsodi_common.health_log.health_event_to_dict` on the edge:
``{"device_id", "component", "severity", "code", "message", "context",
"stamp_sec", "stamp_nanosec"}``. One `evaluate_health_event` function
backs both ingestion points (see `main.py`).

Each detector below documents which signature it encodes - either a
structured `HealthEvent.code` this repo already publishes (see
`pour_server.cpp`, `weighing_scale_node.cpp`,
`rhapsodi_common/microros_watchdog.py`, `robot_adapter/main.py`), or a
free-text symptom hand-diagnosed before the HealthEvent bus existed
(`src/backend/webhook-run-observability.md`'s DetachedInstanceError and
`idle in transaction` postmortems) that a detector matches on message
substrings instead, since nothing yet publishes those as a structured
code.

Deliberately dependency-free (no DB, no FastAPI) so the detection logic
itself - the part with actual failure-diagnosis knowledge worth
getting right - is unit-testable without spinning up Postgres/FastAPI.
`main.py` owns turning a `List[IncidentCandidate]` into `Incident` rows
(and dedup against recent, still-open incidents).
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

# A sustained pour overshoot beyond this many grams over target is treated
# as a more severe incident than routine overshoot noise - matches the
# plan's explicit call for "pour overshoot thresholds", not just a bare
# code match. Context fields come straight from pour_server.cpp's
# `pour_overshoot` HealthEvent (`target_weight`, `final_net_g`).
POUR_OVERSHOOT_SEVERE_G = 5.0

# Signature substrings for symptoms documented before the HealthEvent bus
# existed (src/backend/webhook-run-observability.md) - matched against
# free-text `message` rather than a structured `code`, since nothing
# currently publishes these onto /system/health_events.
_DETACHED_INSTANCE_ERROR_SUBSTRING = 'DetachedInstanceError'
_IDLE_IN_TRANSACTION_SUBSTRING = 'idle in transaction'
_DDS_TIMEOUT_CODE = 'robot_start_service_call_timeout'
_POUR_STALL_ABORT_CODES = frozenset(
    {
        'pour_no_progress_timeout',
        'pour_stale_weight_abort',
        'pour_stale_weight_during_settle',
    }
)


@dataclass
class IncidentCandidate:
    signature_id: str
    title: str
    severity: str
    evidence: Dict[str, Any] = field(default_factory=dict)


Detector = Callable[[Dict[str, Any]], Optional['IncidentCandidate']]


def _context(event: Dict[str, Any]) -> Dict[str, Any]:
    context = event.get('context')
    return context if isinstance(context, dict) else {}


def detect_dds_transport_timeout(
    event: Dict[str, Any]
) -> Optional[IncidentCandidate]:
    """Matches `robot_start_service_call_timeout` - the structured code
    for the "Fast DDS transport/runtime problem affecting request/response
    traffic from the container" postmortem (fixed by
    `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`)."""
    if event.get('code') != _DDS_TIMEOUT_CODE:
        return None
    return IncidentCandidate(
        signature_id='dds_transport_timeout',
        title='DDS/Fast-DDS service call timed out from a container '
        '(check FASTDDS_BUILTIN_TRANSPORTS=UDPv4)',
        severity='ERROR',
        evidence={'code': event.get('code'), 'message': event.get('message')},
    )


def detect_pour_overshoot(
    event: Dict[str, Any]
) -> Optional[IncidentCandidate]:
    """Matches `pour_overshoot`, escalating to `pour_overshoot_severe` once
    the actual overshoot amount (derived from the event's own
    `target_weight`/`final_net_g` context, not re-parsed from a bag) beats
    `POUR_OVERSHOOT_SEVERE_G`."""
    if event.get('code') != 'pour_overshoot':
        return None
    context = _context(event)
    target = context.get('target_weight')
    final_net = context.get('final_net_g')
    overshoot_g = None
    if isinstance(target, (int, float)) and isinstance(final_net, (int, float)):
        overshoot_g = final_net - target
    severe = overshoot_g is not None and overshoot_g >= POUR_OVERSHOOT_SEVERE_G
    title = (
        f'Severe pour overshoot: {overshoot_g:.1f}g over target'
        if severe
        else (
            f'Pour overshoot: {overshoot_g:.1f}g over target'
            if overshoot_g is not None
            else 'Pour overshoot'
        )
    )
    return IncidentCandidate(
        signature_id='pour_overshoot_severe' if severe else 'pour_overshoot',
        title=title,
        severity='ERROR' if severe else 'WARN',
        evidence={
            'code': event.get('code'),
            'overshoot_g': overshoot_g,
            'context': context,
        },
    )


def detect_pour_stalled_or_aborted(
    event: Dict[str, Any]
) -> Optional[IncidentCandidate]:
    """Matches the pour-controller's stall/abort family: no-progress
    watchdog timeout, or an abort triggered by the weight reading going
    stale mid-pour or mid-settle."""
    code = event.get('code')
    if code not in _POUR_STALL_ABORT_CODES:
        return None
    return IncidentCandidate(
        signature_id='pour_stalled_abort',
        title=f'Pour stalled or aborted ({code})',
        severity='ERROR',
        evidence={'code': code, 'message': event.get('message')},
    )


def detect_micro_ros_heartbeat_stale(
    event: Dict[str, Any]
) -> Optional[IncidentCandidate]:
    """Matches `microros_heartbeat_stale` from
    `rhapsodi_common.microros_watchdog` - the microcontroller side of the
    robot (scale/vibration/valve actuation) has stopped responding."""
    if event.get('code') != 'microros_heartbeat_stale':
        return None
    return IncidentCandidate(
        signature_id='microros_heartbeat_stale',
        title='micro-ROS heartbeat went stale',
        severity='WARN',
        evidence={'code': event.get('code'), 'message': event.get('message')},
    )


def detect_scale_readings_stale(
    event: Dict[str, Any]
) -> Optional[IncidentCandidate]:
    """Matches `scale_readings_stale` from `weighing_scale_node.cpp` - no
    weighing-scale reading for longer than its configured threshold."""
    if event.get('code') != 'scale_readings_stale':
        return None
    return IncidentCandidate(
        signature_id='scale_readings_stale',
        title='Weighing scale readings went stale',
        severity='WARN',
        evidence={'code': event.get('code'), 'message': event.get('message')},
    )


def detect_detached_instance_error(
    event: Dict[str, Any]
) -> Optional[IncidentCandidate]:
    """Free-text match for SQLAlchemy `DetachedInstanceError` - the
    backend `/processed` postmortem in webhook-run-observability.md
    (reading ORM attributes after `db.commit()` re-opened a transaction
    and then failed). Nothing publishes this as a structured HealthEvent
    code today; this is exactly the kind of legacy symptom the plan
    calls out as a signature to encode alongside the structured ones."""
    message = event.get('message') or ''
    if _DETACHED_INSTANCE_ERROR_SUBSTRING not in message:
        return None
    return IncidentCandidate(
        signature_id='sqlalchemy_detached_instance_error',
        title='SQLAlchemy DetachedInstanceError (ORM attribute read after commit)',
        severity='ERROR',
        evidence={'message': message},
    )


def detect_idle_in_transaction(
    event: Dict[str, Any]
) -> Optional[IncidentCandidate]:
    """Free-text match for Postgres sessions stuck `idle in transaction`
    - the webhook-detail-page-stuck-on-Refreshing postmortem (a DB
    transaction held open across a blocking external call)."""
    message = event.get('message') or ''
    if _IDLE_IN_TRANSACTION_SUBSTRING not in message.lower():
        return None
    return IncidentCandidate(
        signature_id='postgres_idle_in_transaction',
        title='Postgres session stuck idle in transaction',
        severity='ERROR',
        evidence={'message': message},
    )


DETECTORS: List[Detector] = [
    detect_dds_transport_timeout,
    detect_pour_overshoot,
    detect_pour_stalled_or_aborted,
    detect_micro_ros_heartbeat_stale,
    detect_scale_readings_stale,
    detect_detached_instance_error,
    detect_idle_in_transaction,
]


def evaluate_health_event(event: Dict[str, Any]) -> List[IncidentCandidate]:
    """Runs every detector over one health-event-shaped record. An event
    can legitimately match more than one detector (e.g. a message that
    happens to carry both a known `code` and incidentally mentions a
    substring) - callers get every match, not just the first."""
    candidates = []
    for detector in DETECTORS:
        candidate = detector(event)
        if candidate is not None:
            candidates.append(candidate)
    return candidates
