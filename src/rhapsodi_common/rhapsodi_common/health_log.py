"""Subscriber-side counterpart to ``health.py``: turns the
``/system/health_events`` stream into durable JSONL logs.

Per the plan's data-format decisions: one rolling fleet-wide
``health.jsonl`` (device-scoped — "has this Pi been flaky") plus, while a
run is active, a per-run ``events.jsonl`` fed the same events ("why did
this run fail"). Append-only JSON Lines so a bare Pi can inspect either
file with nothing but ``cat``/``jq``.
"""
from __future__ import annotations

import json
import threading
from pathlib import Path
from typing import Any, Dict, Optional

from rclpy.node import Node
from robot_common_msgs.msg import HealthEvent

from rhapsodi_common.health import HEALTH_EVENTS_TOPIC

_SEVERITY_NAMES = {
    HealthEvent.DEBUG: 'DEBUG',
    HealthEvent.INFO: 'INFO',
    HealthEvent.WARN: 'WARN',
    HealthEvent.ERROR: 'ERROR',
    HealthEvent.CRITICAL: 'CRITICAL',
}


def health_event_to_dict(msg: HealthEvent) -> Dict[str, Any]:
    try:
        context = json.loads(msg.context_json) if msg.context_json else {}
    except (json.JSONDecodeError, TypeError):
        context = {'_context_json_raw': msg.context_json}
    return {
        'stamp_sec': msg.stamp.sec,
        'stamp_nanosec': msg.stamp.nanosec,
        'device_id': msg.device_id,
        'component': msg.component,
        'severity': _SEVERITY_NAMES.get(msg.severity, msg.severity),
        'code': msg.code,
        'message': msg.message,
        'context': context,
    }


class HealthEventLogger:
    """Subscribes to ``/system/health_events`` and appends every event to a
    rolling fleet-wide JSONL log, plus a per-run JSONL log while one is
    active (see ``set_run_log``/``clear_run_log``).

    A logging failure (disk full, permissions, etc.) must never take down
    the node hosting this subscription — file errors are logged, not
    raised, from the subscription callback.
    """

    def __init__(
        self,
        node: Node,
        fleet_log_path: Path,
        topic: str = HEALTH_EVENTS_TOPIC,
        queue_size: int = 50,
    ) -> None:
        self._node = node
        self._fleet_log_path = Path(fleet_log_path)
        self._run_log_path: Optional[Path] = None
        self._lock = threading.Lock()
        self._fleet_log_path.parent.mkdir(parents=True, exist_ok=True)
        self._sub = node.create_subscription(
            HealthEvent, topic, self._on_event, queue_size
        )

    def set_run_log(self, path: Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            self._run_log_path = path

    def clear_run_log(self) -> None:
        with self._lock:
            self._run_log_path = None

    def _on_event(self, msg: HealthEvent) -> None:
        line = json.dumps(health_event_to_dict(msg), sort_keys=True) + '\n'
        with self._lock:
            run_log_path = self._run_log_path
        self._append(self._fleet_log_path, line)
        if run_log_path is not None:
            self._append(run_log_path, line)

    def _append(self, path: Path, line: str) -> None:
        try:
            with open(path, 'a') as f:
                f.write(line)
        except OSError as exc:
            self._node.get_logger().error(
                f'Failed to append health event to {path}: {exc!r}'
            )
