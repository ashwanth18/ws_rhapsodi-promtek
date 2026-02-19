import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


class LightsOutSimNode(Node):
    def __init__(self) -> None:
        super().__init__('lightsout_sim')
        self.declare_parameter('episodes', 3)
        self.declare_parameter('episode_duration', 8.0)
        self.declare_parameter('gap_seconds', 1.0)
        self.declare_parameter('start_delay', 1.0)
        self.declare_parameter('exit_on_done', False)
        self.declare_parameter('metadata_json', '')
        self.declare_parameter('batch_id', 'sim-batch')
        self.declare_parameter('powder_name', 'sim-powder')
        self.declare_parameter('cycle_end_limit', 0.0)
        self.declare_parameter('target_weight', 50.0)
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

        self._episodes = int(self.get_parameter('episodes').value)
        self._episode_duration = float(
            self.get_parameter('episode_duration').value
        )
        self._gap_seconds = float(self.get_parameter('gap_seconds').value)
        self._start_delay = float(self.get_parameter('start_delay').value)
        self._exit_on_done = bool(self.get_parameter('exit_on_done').value)
        self._metadata_json = str(self.get_parameter('metadata_json').value)
        self._batch_id = str(self.get_parameter('batch_id').value)
        self._powder_name = str(self.get_parameter('powder_name').value)
        self._cycle_end_limit = float(
            self.get_parameter('cycle_end_limit').value
        )
        self._target_weight = float(
            self.get_parameter('target_weight').value
        )

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

        self._active_pub = self.create_publisher(Bool, self._active_topic, 10)
        self._metadata_pub = self.create_publisher(
            String, self._metadata_topic, 10
        )
        self._episode_pub = self.create_publisher(
            Int32, self._episode_topic, 10
        )
        self._episode_end_pub = self.create_publisher(
            Int32, self._episode_end_topic, 10
        )

        self._state = 'waiting_start'
        self._current_episode = 0
        self._next_event_time = self._now() + self._start_delay
        self._done = False

        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'LightsOutSim ready: {self._episodes} episodes'
        )

    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _build_metadata(self) -> Dict[str, Any]:
        if self._metadata_json:
            try:
                return json.loads(self._metadata_json)
            except json.JSONDecodeError:
                return {}
        return {
            'batch_id': self._batch_id,
            'powder_name': self._powder_name,
            'cycle_end_limit': self._cycle_end_limit,
            'target_weight': self._target_weight,
            'episodes': self._episodes,
        }

    def _publish_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = bool(active)
        self._active_pub.publish(msg)

    def _publish_metadata(self) -> None:
        payload = json.dumps(self._build_metadata())
        msg = String()
        msg.data = payload
        self._metadata_pub.publish(msg)

    def _publish_episode(self, idx: int) -> None:
        msg = Int32()
        msg.data = int(idx)
        self._episode_pub.publish(msg)

    def _publish_episode_end(self, idx: int) -> None:
        msg = Int32()
        msg.data = int(idx)
        self._episode_end_pub.publish(msg)

    def _finish(self) -> None:
        if self._done:
            return
        self._done = True
        self._publish_active(False)
        self.get_logger().info('LightsOutSim finished')
        if self._exit_on_done:
            rclpy.shutdown()

    def _tick(self) -> None:
        now = self._now()
        if self._state == 'waiting_start' and now >= self._next_event_time:
            self._publish_metadata()
            self._publish_active(True)
            self._state = 'episode_start'
            self._next_event_time = now
            return

        if self._state == 'episode_start' and now >= self._next_event_time:
            if self._current_episode >= self._episodes:
                self._finish()
                self._state = 'done'
                return
            self._current_episode += 1
            self._publish_episode(self._current_episode)
            self._state = 'episode_running'
            self._next_event_time = now + self._episode_duration
            return

        if self._state == 'episode_running' and now >= self._next_event_time:
            self._publish_episode_end(self._current_episode)
            self._state = 'episode_gap'
            self._next_event_time = now + self._gap_seconds
            return

        if self._state == 'episode_gap' and now >= self._next_event_time:
            self._state = 'episode_start'


def main() -> None:
    rclpy.init()
    node = LightsOutSimNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()






