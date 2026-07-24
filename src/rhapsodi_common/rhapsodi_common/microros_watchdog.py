"""Publishes HealthEvents for micro-ROS connectivity loss.

Complements (does not replace) the dashboard's own staleness check
described in control_heartbeat_method.md — that check is JS-side, in-memory,
and disappears when nobody has the dashboard open. This node applies the
same "no heartbeat for N seconds" rule but onto /system/health_events, so
connectivity flaps end up in events.jsonl / health.jsonl for fault-finding
even when nobody was watching the dashboard at the time.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

from rhapsodi_common.health import HealthEventPublisher

COMPONENT = 'microros_health_watchdog'


class MicroRosHealthWatchdog(Node):
    def __init__(self) -> None:
        super().__init__('microros_health_watchdog')
        self.declare_parameter('heartbeat_topic', '/microros/heartbeat')
        self.declare_parameter('stale_after_seconds', 3.0)
        self.declare_parameter('check_interval_seconds', 1.0)

        self._stale_after_s = float(
            self.get_parameter('stale_after_seconds').value
        )
        heartbeat_topic = str(
            self.get_parameter('heartbeat_topic').value
        )

        self._health = HealthEventPublisher(self, COMPONENT)
        self._last_heartbeat: float = self._now()
        # None = unknown yet (no verdict published); True/False = last
        # published state, so we only publish on transitions, not every tick.
        self._is_stale: bool = False
        self._has_seen_heartbeat = False

        self.create_subscription(
            Empty, heartbeat_topic, self._on_heartbeat, 10
        )
        self.create_timer(
            max(0.1, float(self.get_parameter('check_interval_seconds').value)),
            self._check_stale,
        )
        self.get_logger().info(
            f'Watching {heartbeat_topic} for staleness '
            f'(threshold={self._stale_after_s}s)'
        )

    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _on_heartbeat(self, _msg: Empty) -> None:
        self._last_heartbeat = self._now()
        self._has_seen_heartbeat = True
        if self._is_stale:
            self._is_stale = False
            self._health.info(
                'microros_heartbeat_recovered',
                'micro-ROS heartbeat resumed',
            )

    def _check_stale(self) -> None:
        if not self._has_seen_heartbeat:
            return
        age_s = self._now() - self._last_heartbeat
        if age_s > self._stale_after_s and not self._is_stale:
            self._is_stale = True
            self._health.warn(
                'microros_heartbeat_stale',
                f'No micro-ROS heartbeat for {age_s:.1f}s '
                f'(threshold={self._stale_after_s}s)',
                {'age_seconds': age_s},
            )


def main() -> None:
    rclpy.init()
    node = MicroRosHealthWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
