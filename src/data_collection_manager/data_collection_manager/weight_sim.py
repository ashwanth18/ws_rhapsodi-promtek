import json
import random
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32, String


class WeightSimNode(Node):
    def __init__(self) -> None:
        super().__init__('weight_sim')
        self.declare_parameter('topic', '/weight')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('baseline', 0.0)
        self.declare_parameter('target', 260.0)
        self.declare_parameter('ramp_seconds', 3.0)
        self.declare_parameter('settle_seconds', 2.0)
        self.declare_parameter('noise_std', 0.2)
        self.declare_parameter('publish_when_active', True)
        self.declare_parameter('reset_on_episode_end', True)
        self.declare_parameter('use_metadata_target', True)
        self.declare_parameter('clamp_to_target', True)
        self.declare_parameter('gate_on_phase', False)
        self.declare_parameter('phase_topic', '/lightsout_training/phase')
        self.declare_parameter('phase_start', 'pour_start')
        self.declare_parameter('phase_end', 'pour_end')
        self.declare_parameter('seed', 0)
        self.declare_parameter(
            'lightsout_active_topic',
            '/lightsout_training/active',
        )
        self.declare_parameter(
            'lightsout_metadata_topic',
            '/lightsout_training/metadata',
        )
        self.declare_parameter(
            'lightsout_episode_topic',
            '/lightsout_training/episode',
        )
        self.declare_parameter(
            'lightsout_episode_end_topic',
            '/lightsout_training/episode_end',
        )

        self._topic = str(self.get_parameter('topic').value)
        self._rate_hz = float(self.get_parameter('rate_hz').value)
        self._baseline = float(self.get_parameter('baseline').value)
        self._default_target = float(self.get_parameter('target').value)
        self._ramp_seconds = max(
            0.01, float(self.get_parameter('ramp_seconds').value)
        )
        self._settle_seconds = max(
            0.0, float(self.get_parameter('settle_seconds').value)
        )
        self._noise_std = max(
            0.0, float(self.get_parameter('noise_std').value)
        )
        self._publish_when_active = bool(
            self.get_parameter('publish_when_active').value
        )
        self._reset_on_episode_end = bool(
            self.get_parameter('reset_on_episode_end').value
        )
        self._use_metadata_target = bool(
            self.get_parameter('use_metadata_target').value
        )
        self._clamp_to_target = bool(
            self.get_parameter('clamp_to_target').value
        )
        self._gate_on_phase = bool(
            self.get_parameter('gate_on_phase').value
        )
        self._phase_topic = str(self.get_parameter('phase_topic').value)
        self._phase_start = str(self.get_parameter('phase_start').value)
        self._phase_end = str(self.get_parameter('phase_end').value)
        seed = int(self.get_parameter('seed').value)
        if seed:
            random.seed(seed)

        self._active_topic = self.get_parameter(
            'lightsout_active_topic'
        ).value
        self._metadata_topic = self.get_parameter(
            'lightsout_metadata_topic'
        ).value
        self._episode_topic = self.get_parameter(
            'lightsout_episode_topic'
        ).value
        self._episode_end_topic = self.get_parameter(
            'lightsout_episode_end_topic'
        ).value

        self._active = False
        self._episode_index = 0
        self._episode_start_time: Optional[float] = None
        self._current_target = self._default_target
        self._metadata: Dict[str, Any] = {}
        self._phase_active = False

        self._pub = self.create_publisher(Float64, self._topic, 10)
        self.create_subscription(Bool, self._active_topic, self._on_active, 10)
        self.create_subscription(
            String, self._metadata_topic, self._on_metadata, 10
        )
        self.create_subscription(
            Int32, self._episode_topic, self._on_episode, 10
        )
        self.create_subscription(
            Int32, self._episode_end_topic, self._on_episode_end, 10
        )
        if self._gate_on_phase:
            self.create_subscription(
                String, self._phase_topic, self._on_phase, 10
            )

        period = 1.0 / max(0.1, self._rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'WeightSim started on {self._topic} at {self._rate_hz} Hz'
        )

    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _pick_target(self) -> float:
        if self._use_metadata_target:
            target = self._metadata.get('target_weight')
            if target is None:
                target = self._metadata.get('target_weight_g')
            if isinstance(target, (int, float)):
                return float(target)
            if isinstance(target, str):
                try:
                    return float(target)
                except ValueError:
                    pass
        return self._default_target

    def _clamp_weight(self, weight: float) -> float:
        if not self._clamp_to_target:
            return weight
        target = self._current_target
        if target >= self._baseline:
            return min(weight, target)
        return max(weight, target)

    def _on_active(self, msg: Bool) -> None:
        self._active = bool(msg.data)
        if not self._active and self._reset_on_episode_end:
            self._episode_start_time = None

    def _on_metadata(self, msg: String) -> None:
        if not msg.data:
            self._metadata = {}
            return
        try:
            self._metadata = json.loads(msg.data)
        except json.JSONDecodeError:
            self._metadata = {}

    def _on_episode(self, msg: Int32) -> None:
        self._episode_index = int(msg.data)
        self._current_target = self._pick_target()
        if not self._gate_on_phase:
            self._episode_start_time = self._now()

    def _on_episode_end(self, msg: Int32) -> None:
        if int(msg.data) != self._episode_index:
            return
        if self._reset_on_episode_end:
            self._episode_start_time = None
            self._phase_active = False

    def _on_phase(self, msg: String) -> None:
        phase = msg.data or ''
        if phase == self._phase_start:
            self._phase_active = True
            self._episode_start_time = self._now()
            self._current_target = self._pick_target()
        elif phase == self._phase_end:
            self._phase_active = False
            if self._reset_on_episode_end:
                self._episode_start_time = None

    def _tick(self) -> None:
        if self._publish_when_active and not self._active:
            return
        now = self._now()
        if self._gate_on_phase and not self._phase_active:
            weight = self._baseline
        elif self._episode_start_time is None:
            weight = self._baseline
        else:
            t = max(0.0, now - self._episode_start_time)
            if t <= self._ramp_seconds:
                ratio = t / self._ramp_seconds
                weight = self._baseline + (
                    self._current_target - self._baseline
                ) * ratio
            elif t <= self._ramp_seconds + self._settle_seconds:
                weight = self._current_target
            else:
                weight = self._current_target
        if self._noise_std > 0.0:
            weight += random.gauss(0.0, self._noise_std)
        weight = self._clamp_weight(weight)

        msg = Float64()
        msg.data = float(weight)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = WeightSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
