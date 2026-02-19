import subprocess
import time
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32, String


@dataclass
class EpisodeContext:
    episode_index: int
    folder: Path
    bag_path: Path


class DataCollectionManager(Node):
    def __init__(self) -> None:
        super().__init__('data_collection_manager')
        self.declare_parameter('output_root', 'data/lightsout')
        self.declare_parameter(
            'topics',
            [
                '/weight',
                '/system_status',
                '/lightsout_training/phase',
                '/lightsout_training/run_id',
                '/lightsout_training/batch_id',
                '/lightsout_training/ingredient_id',
                '/lightsout_training/target_weight_g',
                '/lightsout_training/episode',
                '/lightsout_training/episodes_total',
                '/lightsout_training/mode',
                '/lightsout_training/robot_id',
                '/tf_static',
                '/joint_states',
            ],
        )
        self.declare_parameter(
            'lightsout_active_topic',
            '/lightsout_training/active',
        )
        self.declare_parameter(
            'lightsout_episode_topic',
            '/lightsout_training/episode',
        )
        self.declare_parameter(
            'lightsout_episode_end_topic',
            '/lightsout_training/episode_end',
        )
        self.declare_parameter(
            'run_id_topic', '/lightsout_training/run_id'
        )
        self.declare_parameter(
            'batch_id_topic', '/lightsout_training/batch_id'
        )
        self.declare_parameter(
            'ingredient_id_topic', '/lightsout_training/ingredient_id'
        )
        self.declare_parameter(
            'target_weight_topic', '/lightsout_training/target_weight_g'
        )
        self.declare_parameter('mode_topic', '/lightsout_training/mode')
        self.declare_parameter('robot_id', 'robot-1')
        self.declare_parameter('mode', 'lightsout')
        self.declare_parameter(
            'processing_url', 'http://localhost:8002/process'
        )

        self._output_root = Path(self.get_parameter('output_root').value)
        self._topics = list(self.get_parameter('topics').value)
        self._active_topic = self.get_parameter(
            'lightsout_active_topic',
        ).value
        self._episode_topic = self.get_parameter(
            'lightsout_episode_topic',
        ).value
        self._episode_end_topic = self.get_parameter(
            'lightsout_episode_end_topic',
        ).value
        self._run_id_topic = self.get_parameter('run_id_topic').value
        self._batch_id_topic = self.get_parameter('batch_id_topic').value
        self._ingredient_id_topic = self.get_parameter(
            'ingredient_id_topic'
        ).value
        self._target_weight_topic = self.get_parameter(
            'target_weight_topic'
        ).value
        self._mode_topic = self.get_parameter('mode_topic').value
        self._robot_id = str(self.get_parameter('robot_id').value)
        self._mode = str(self.get_parameter('mode').value)
        self._processing_url = str(
            self.get_parameter('processing_url').value or ''
        ).strip()
        self._lightsout_active = False
        self._current_episode: Optional[EpisodeContext] = None
        self._bag_proc: Optional[subprocess.Popen] = None
        self._run_folder: Optional[Path] = None
        self._run_id: Optional[str] = None
        self._batch_id: Optional[str] = None
        self._ingredient_id: Optional[str] = None
        self._target_weight_g: Optional[float] = None
        self._run_mode: Optional[str] = None

        self.create_subscription(
            Bool, self._active_topic, self._on_active, 10
        )
        self.create_subscription(
            Int32, self._episode_topic, self._on_episode, 10
        )
        self.create_subscription(
            Int32, self._episode_end_topic, self._on_episode_end, 10
        )
        self.create_subscription(
            String, self._run_id_topic, self._on_run_id, 10
        )
        self.create_subscription(
            String, self._batch_id_topic, self._on_batch_id, 10
        )
        self.create_subscription(
            String, self._ingredient_id_topic, self._on_ingredient_id, 10
        )
        self.create_subscription(
            Float64, self._target_weight_topic, self._on_target_weight, 10
        )
        self.create_subscription(
            String, self._mode_topic, self._on_mode, 10
        )

        self.get_logger().info(
            f'Data collection manager started. Output: {self._output_root}'
        )

    def _now_times(self) -> Dict[str, int]:
        ros_ns = int(self.get_clock().now().nanoseconds)
        wall_ns = int(time.time() * 1_000_000_000)
        return {'ros_ns': ros_ns, 'wall_ns': wall_ns}

    def _format_timestamp(self) -> str:
        return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')

    def _safe_run_folder(self) -> str:
        ts = self._format_timestamp()
        run_id = self._run_id or ts
        batch_id = self._batch_id or 'batch'
        mode = self._run_mode or self._mode
        return f'{self._robot_id}_{run_id}_{batch_id}_{ts}_{mode}'

    def _start_bag(self, ctx: EpisodeContext) -> None:
        cmd = [
            'ros2', 'bag', 'record', '--storage', 'mcap',
            '-o', str(ctx.bag_path),
        ]
        cmd.extend(self._topics)
        self.get_logger().info(f'Starting MCAP recording: {" ".join(cmd)}')
        self._bag_proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _stop_bag(self) -> None:
        if not self._bag_proc:
            return
        self.get_logger().info('Stopping MCAP recording')
        self._bag_proc.terminate()
        try:
            self._bag_proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self._bag_proc.kill()
        self._bag_proc = None

    def _post_process(self, ctx: EpisodeContext) -> None:
        if not self._processing_url:
            self.get_logger().warn('processing_url is empty')
            return
        payload = (
            f'{{"run_folder":"{ctx.folder}","bag_path":"{ctx.bag_path}"}}'
        ).encode('utf-8')
        req = urllib.request.Request(
            self._processing_url,
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                self.get_logger().info(f'Process response: {resp.status}')
        except Exception as exc:
            self.get_logger().warn(f'Failed to process run: {exc}')

    def _start_episode(self, episode_index: int) -> None:
        if self._current_episode is not None:
            self._finalize_episode('episode_rollover')
        if self._run_folder is None:
            self._run_folder = self._output_root / self._safe_run_folder()
            self._run_folder.mkdir(parents=True, exist_ok=True)
        folder = self._run_folder
        bag_path = folder / f'episode_{episode_index}'
        ctx = EpisodeContext(
            episode_index=episode_index,
            folder=folder,
            bag_path=bag_path,
        )
        self._current_episode = ctx
        self._start_bag(ctx)

    def _finalize_episode(self, reason: str) -> None:
        if self._current_episode is None:
            return
        ctx = self._current_episode
        self._stop_bag()
        self._post_process(ctx)
        self._current_episode = None

    def _finish_active(self) -> None:
        self._finalize_episode('run_end')

    def _on_active(self, msg: Bool) -> None:
        self._lightsout_active = bool(msg.data)
        if not self._lightsout_active:
            self._finish_active()
            self._run_folder = None

    def _on_run_id(self, msg: String) -> None:
        self._run_id = msg.data or None

    def _on_batch_id(self, msg: String) -> None:
        self._batch_id = msg.data or None

    def _on_ingredient_id(self, msg: String) -> None:
        self._ingredient_id = msg.data or None

    def _on_target_weight(self, msg: Float64) -> None:
        self._target_weight_g = float(msg.data)

    def _on_mode(self, msg: String) -> None:
        self._run_mode = msg.data or None

    def _on_episode(self, msg: Int32) -> None:
        if not self._lightsout_active:
            return
        episode_index = int(msg.data)
        if (
            self._current_episode
            and self._current_episode.episode_index == episode_index
        ):
            return
        self._start_episode(episode_index)

    def _on_episode_end(self, msg: Int32) -> None:
        if not self._current_episode:
            return
        if int(msg.data) != self._current_episode.episode_index:
            return
        self._finalize_episode('episode_end')


def main() -> None:
    rclpy.init()
    node = DataCollectionManager()
    try:
        rclpy.spin(node)
    finally:
        node._finish_active()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
