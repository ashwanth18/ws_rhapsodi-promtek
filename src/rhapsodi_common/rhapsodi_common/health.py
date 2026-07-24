"""Shared helper for publishing onto ``/system/health_events``.

Every Python ROS 2 node — and any non-ROS Python service that keeps a live
``rclpy`` node around, like ``robot_start_adapter`` — should use this
instead of only logging, so faults are queryable fleet-wide instead of
stuck in ``docker logs``. See ``robot_common_msgs/msg/HealthEvent.msg`` for
the wire schema and the (future) RecorderV2 for how this stream ends up in
each run's ``events.jsonl``.
"""
from __future__ import annotations

import json
from typing import Any, Dict, Optional

from rclpy.node import Node
from robot_common_msgs.msg import HealthEvent

from rhapsodi_common.device_config import load_device_config

HEALTH_EVENTS_TOPIC = '/system/health_events'

DEBUG = HealthEvent.DEBUG
INFO = HealthEvent.INFO
WARN = HealthEvent.WARN
ERROR = HealthEvent.ERROR
CRITICAL = HealthEvent.CRITICAL


class HealthEventPublisher:
    """Publishes ``HealthEvent`` messages for one component on one node."""

    def __init__(
        self,
        node: Node,
        component: str,
        device_id: Optional[str] = None,
        queue_size: int = 20,
    ) -> None:
        self._node = node
        self._component = component
        self._device_id = device_id or load_device_config().device_id
        self._publisher = node.create_publisher(
            HealthEvent, HEALTH_EVENTS_TOPIC, queue_size
        )

    def publish(
        self,
        severity: int,
        code: str,
        message: str,
        context: Optional[Dict[str, Any]] = None,
    ) -> None:
        msg = HealthEvent()
        msg.stamp = self._node.get_clock().now().to_msg()
        msg.device_id = self._device_id
        msg.component = self._component
        msg.severity = int(severity)
        msg.code = code
        msg.message = message
        try:
            msg.context_json = json.dumps(context or {}, default=str)
        except TypeError:
            msg.context_json = '{}'
        self._publisher.publish(msg)

    def debug(
        self, code: str, message: str, context: Optional[Dict] = None
    ) -> None:
        self.publish(DEBUG, code, message, context)

    def info(
        self, code: str, message: str, context: Optional[Dict] = None
    ) -> None:
        self.publish(INFO, code, message, context)

    def warn(
        self, code: str, message: str, context: Optional[Dict] = None
    ) -> None:
        self.publish(WARN, code, message, context)

    def error(
        self, code: str, message: str, context: Optional[Dict] = None
    ) -> None:
        self.publish(ERROR, code, message, context)

    def critical(
        self, code: str, message: str, context: Optional[Dict] = None
    ) -> None:
        self.publish(CRITICAL, code, message, context)
