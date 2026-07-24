import json
import signal
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32, String

from rhapsodi_common.device_config import load_device_config
from rhapsodi_common.health import HealthEventPublisher
from rhapsodi_common.health_log import HealthEventLogger

from data_collection_manager.manifest import TIER_0, TIER_1, RunManifest
from data_collection_manager.recorder_v2 import RecorderStartError, RecorderV2


@dataclass
class EpisodeContext:
    episode_index: int
    folder: Path
    bag_path: Path
    source: str


class DataCollectionManager(Node):
    def __init__(self) -> None:
        super().__init__('data_collection_manager')
        # Single source of truth for robot_id / processing_url defaults —
        # see src/rhapsodi_common/rhapsodi_common/device_config.py and
        # config/device.yaml. Still overridable per-node via ROS params for
        # tests/sims.
        device = load_device_config()
        if device.is_fallback:
            self.get_logger().warn(
                'No config/device.yaml found; using built-in single-robot '
                f'defaults (robot_id={device.robot_id}). See '
                'config/device.yaml.example to configure this Pi.'
            )
        else:
            self.get_logger().info(
                f'Loaded device identity from {device.source_path}: '
                f'device_id={device.device_id} robot_id={device.robot_id} '
                f'robot_type={device.robot_type} site_id={device.site_id}'
            )
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
                '/webhook_run/active',
                '/webhook_run/metadata',
                '/webhook_run/phase',
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
        self.declare_parameter('webhook_active_topic', '/webhook_run/active')
        self.declare_parameter(
            'webhook_metadata_topic', '/webhook_run/metadata'
        )
        self.declare_parameter('robot_id', device.robot_id)
        self.declare_parameter('mode', 'lightsout')
        self.declare_parameter('webhook_metadata_wait_seconds', 1.0)
        self.declare_parameter('processing_url', device.processing_url)

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
        self._webhook_active_topic = self.get_parameter(
            'webhook_active_topic'
        ).value
        self._webhook_metadata_topic = self.get_parameter(
            'webhook_metadata_topic'
        ).value
        self._robot_id = str(self.get_parameter('robot_id').value)
        self._mode = str(self.get_parameter('mode').value)
        self._webhook_metadata_wait_seconds = max(
            0.0,
            float(
                self.get_parameter('webhook_metadata_wait_seconds').value
            ),
        )
        self._processing_url = str(
            self.get_parameter('processing_url').value or ''
        ).strip()
        self.declare_parameter('manifest_path', '')
        manifest_path_param = str(
            self.get_parameter('manifest_path').value or ''
        ).strip()
        self._manifest_path = (
            Path(manifest_path_param)
            if manifest_path_param
            else self._output_root / 'manifest.sqlite'
        )
        self._device_id = device.device_id
        self._health = HealthEventPublisher(
            self, 'data_collection_manager', device_id=self._device_id
        )
        self._health_log = HealthEventLogger(
            self, fleet_log_path=self._output_root / 'health.jsonl'
        )
        self._recorder = RecorderV2(health=self._health)
        self._manifest = RunManifest(self._manifest_path)
        self._run_key: Optional[str] = None
        self._lightsout_active = False
        self._current_episode: Optional[EpisodeContext] = None
        self._run_folder: Optional[Path] = None
        self._run_id: Optional[str] = None
        self._batch_id: Optional[str] = None
        self._ingredient_id: Optional[str] = None
        self._target_weight_g: Optional[float] = None
        self._run_mode: Optional[str] = None
        self._weightment_id: Optional[str] = None
        self._location_id: Optional[str] = None
        self._location_code: Optional[str] = None
        self._recording_source: Optional[str] = None
        self._webhook_metadata: Dict[str, str] = {}
        self._webhook_active = False
        self._webhook_start_pending = False
        self._webhook_active_started_at: Optional[float] = None

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
        self.create_subscription(
            Bool, self._webhook_active_topic, self._on_webhook_active, 10
        )
        self.create_subscription(
            String,
            self._webhook_metadata_topic,
            self._on_webhook_metadata,
            10,
        )
        self.create_timer(0.1, self._check_pending_webhook_start)

        self.get_logger().info(
            f'Data collection manager started. Output: {self._output_root}'
        )

    def _now_times(self) -> Dict[str, int]:
        ros_ns = int(self.get_clock().now().nanoseconds)
        wall_ns = int(time.time() * 1_000_000_000)
        return {'ros_ns': ros_ns, 'wall_ns': wall_ns}

    def _now(self) -> float:
        return float(self.get_clock().now().nanoseconds) / 1e9

    def _format_timestamp(self) -> str:
        return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')

    def _safe_run_folder(self) -> str:
        ts = self._format_timestamp()
        run_id = self._run_id or ts
        batch_id = self._batch_id or 'batch'
        mode = self._run_mode or self._mode
        suffix = ''
        if self._recording_source == 'webhook' and self._weightment_id:
            suffix = f'_weightment_{self._weightment_id}'
        return f'{self._robot_id}_{run_id}_{batch_id}_{ts}_{mode}{suffix}'

    def _metadata_payload(self, ctx: EpisodeContext) -> Dict[str, object]:
        payload: Dict[str, object] = {
            'source': ctx.source,
            'robot_id': self._robot_id,
            'run_id': self._run_id,
            'batch_id': self._batch_id,
            'ingredient_id': self._ingredient_id,
            'target_weight_g': self._target_weight_g,
            'mode': self._run_mode or self._mode,
            'bag_path': str(ctx.bag_path),
            'run_folder': str(ctx.folder),
            'episode_index': ctx.episode_index,
        }
        if ctx.source == 'webhook':
            payload.update(
                {
                    'weightment_id': self._weightment_id,
                    'location_id': self._location_id,
                    'location_code': self._location_code,
                    'webhook_metadata': self._webhook_metadata,
                }
            )
        return payload

    def _register_run(self, ctx: EpisodeContext) -> None:
        """Ensure `manifest.sqlite` and the per-run events.jsonl are wired
        up for a freshly-created run folder. Idempotent — safe to call once
        per run (upsert_run no-ops if the run_key already exists).
        """
        self._run_key = ctx.folder.name
        self._manifest.upsert_run(
            run_key=self._run_key,
            robot_id=self._robot_id,
            run_folder=ctx.folder,
            source=ctx.source,
            mode=self._run_mode or self._mode,
            device_id=self._device_id,
        )
        self._health_log.set_run_log(ctx.folder / 'events.jsonl')
        self._manifest.record_artifact(
            self._run_key, TIER_0, ctx.folder / 'events.jsonl'
        )

    def _close_run_registration(self) -> None:
        if self._run_key is not None:
            self._manifest.mark_run_complete(self._run_key)
        self._run_key = None
        self._health_log.clear_run_log()

    def _write_metadata(self, ctx: EpisodeContext) -> None:
        metadata_path = ctx.folder / 'metadata.json'
        metadata_path.write_text(
            json.dumps(self._metadata_payload(ctx), indent=2, sort_keys=True)
        )
        if self._run_key is not None:
            self._manifest.record_artifact(
                self._run_key, TIER_0, metadata_path
            )

    def _start_bag(self, ctx: EpisodeContext) -> None:
        """Start native MCAP recording for `ctx`. Raises RecorderStartError
        on failure (a CRITICAL health event is published by RecorderV2
        before raising) — callers must not treat a failed start as if
        recording is happening.
        """
        self.get_logger().info(
            f'Starting MCAP recording: bag_path={ctx.bag_path} '
            f'topics={self._topics}'
        )
        self._recorder.start(ctx.bag_path, self._topics)
        # bag_path is a directory (rosbag2_py writes N .mcap chunks +
        # metadata.yaml under it) — tracked as one Tier-1 artifact so
        # retention prunes/measures the whole thing as a unit.
        if self._run_key is not None:
            self._manifest.record_artifact(
                self._run_key, TIER_1, ctx.bag_path
            )

    def _stop_bag(self) -> None:
        if not self._recorder.is_recording:
            return
        self.get_logger().info('Stopping MCAP recording')
        self._recorder.stop()

    def _register_parquet_if_present(self, ctx: EpisodeContext) -> None:
        """Best-effort: the `processing` service writes
        `<mcap_stem>.parquet` next to the MCAP it read (see
        `src/backend/processing/main.py::_write_parquet`) once it responds
        successfully. Pick it up as a Tier-0 artifact if present — a miss
        here is not an error, just nothing to register yet.
        """
        if self._run_key is None:
            return
        for candidate_dir in {ctx.bag_path, ctx.folder}:
            if not candidate_dir.is_dir():
                continue
            for parquet_path in candidate_dir.glob('*.parquet'):
                self._manifest.record_artifact(
                    self._run_key, TIER_0, parquet_path
                )

    def _post_process(self, ctx: EpisodeContext) -> None:
        if not self._processing_url:
            self.get_logger().warn('processing_url is empty')
            return
        payload_dict = {
            'run_folder': str(ctx.folder),
            'bag_path': str(ctx.bag_path),
        }
        payload = json.dumps(payload_dict).encode('utf-8')
        req = urllib.request.Request(
            self._processing_url,
            data=payload,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        started_at = time.monotonic()
        self.get_logger().info(
            'Posting run for processing: '
            f'source={ctx.source} '
            f'episode={ctx.episode_index} '
            f'folder={ctx.folder} '
            f'bag_path={ctx.bag_path} '
            f'url={self._processing_url}'
        )
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                body = resp.read().decode('utf-8', errors='replace')
                elapsed_s = time.monotonic() - started_at
                self.get_logger().info(
                    'Process response: '
                    f'status={resp.status} '
                    f'elapsed={elapsed_s:.2f}s '
                    f'body={body[:500] or "<empty>"}'
                )
        except urllib.error.HTTPError as exc:
            elapsed_s = time.monotonic() - started_at
            try:
                error_body = exc.read().decode('utf-8', errors='replace')
            except Exception:
                error_body = '<unreadable>'
            self.get_logger().warn(
                'Failed to process run: '
                f'http_error status={exc.code} '
                f'elapsed={elapsed_s:.2f}s '
                f'url={self._processing_url} '
                f'source={ctx.source} '
                f'episode={ctx.episode_index} '
                f'folder={ctx.folder} '
                f'bag_path={ctx.bag_path} '
                f'body={error_body[:500]}'
            )
            self._health.error(
                'processing_post_http_error',
                f'Processing service returned HTTP {exc.code} for '
                f'{ctx.folder}',
                {
                    'status': exc.code,
                    'url': self._processing_url,
                    'folder': str(ctx.folder),
                    'bag_path': str(ctx.bag_path),
                },
            )
        except Exception as exc:
            elapsed_s = time.monotonic() - started_at
            self.get_logger().warn(
                'Failed to process run: '
                f'error={exc!r} '
                f'elapsed={elapsed_s:.2f}s '
                f'url={self._processing_url} '
                f'source={ctx.source} '
                f'episode={ctx.episode_index} '
                f'folder={ctx.folder} '
                f'bag_path={ctx.bag_path}'
            )
            # This is exactly the "network blip silently drops the
            # processing step" gap called out in the plan: the raw MCAP
            # survives on disk, but nothing revisited it. Surfacing it as a
            # health event at least makes the drop visible fleet-wide until
            # the uplink daemon (uplink-daemon todo) can retry processing
            # from the local manifest.
            self._health.error(
                'processing_post_failed',
                f'Failed to POST run for processing: {exc!r}',
                {
                    'url': self._processing_url,
                    'folder': str(ctx.folder),
                    'bag_path': str(ctx.bag_path),
                },
            )

    def _start_episode(self, episode_index: int) -> None:
        if self._current_episode is not None:
            self._finalize_episode('episode_rollover')
        self._recording_source = 'lightsout'
        is_new_run = self._run_folder is None
        if self._run_folder is None:
            self._run_folder = self._output_root / self._safe_run_folder()
            self._run_folder.mkdir(parents=True, exist_ok=True)
        folder = self._run_folder
        bag_path = folder / f'episode_{episode_index}'
        ctx = EpisodeContext(
            episode_index=episode_index,
            folder=folder,
            bag_path=bag_path,
            source='lightsout',
        )
        if is_new_run:
            self._register_run(ctx)
        self._write_metadata(ctx)
        try:
            self._start_bag(ctx)
        except RecorderStartError as exc:
            # Health event already published by RecorderV2.start(). Do not
            # set _current_episode: a failed start must not be mistaken for
            # an active recording by episode_end/webhook_active handlers.
            self.get_logger().error(
                f'Failed to start episode {episode_index} recording: {exc}'
            )
            return
        self._current_episode = ctx

    def _start_webhook_run(self) -> None:
        if self._current_episode is not None:
            return
        self._recording_source = 'webhook'
        is_new_run = self._run_folder is None
        if self._run_folder is None:
            self._run_folder = self._output_root / self._safe_run_folder()
            self._run_folder.mkdir(parents=True, exist_ok=True)
        folder = self._run_folder
        bag_path = folder / 'webhook_run'
        ctx = EpisodeContext(
            episode_index=1,
            folder=folder,
            bag_path=bag_path,
            source='webhook',
        )
        if is_new_run:
            self._register_run(ctx)
        self._write_metadata(ctx)
        try:
            self._start_bag(ctx)
        except RecorderStartError as exc:
            self.get_logger().error(
                f'Failed to start webhook run recording: {exc}'
            )
            return
        self._current_episode = ctx

    def _has_webhook_metadata(self) -> bool:
        return bool(self._webhook_metadata.get('run_id'))

    def _begin_pending_webhook_start(self) -> None:
        self._webhook_start_pending = True
        self._webhook_active_started_at = self._now()

    def _check_pending_webhook_start(self) -> None:
        if not self._webhook_start_pending or not self._webhook_active:
            return
        if self._current_episode is not None:
            self._webhook_start_pending = False
            return
        if self._has_webhook_metadata():
            self._webhook_start_pending = False
            self._start_webhook_run()
            return
        started_at = self._webhook_active_started_at
        if started_at is None:
            self._webhook_active_started_at = self._now()
            return
        if (self._now() - started_at) < self._webhook_metadata_wait_seconds:
            return
        self.get_logger().warn(
            'Starting webhook recording without metadata after timeout'
        )
        self._webhook_start_pending = False
        self._start_webhook_run()

    def _finalize_episode(self, reason: str) -> None:
        if self._current_episode is None:
            return
        ctx = self._current_episode
        self._stop_bag()
        self._post_process(ctx)
        self._register_parquet_if_present(ctx)
        self._current_episode = None

    def _finish_active(self) -> None:
        self._finalize_episode('run_end')

    def _on_active(self, msg: Bool) -> None:
        self._lightsout_active = bool(msg.data)
        if not self._lightsout_active:
            self._finish_active()
            self._run_folder = None
            self._close_run_registration()
            if self._recording_source == 'lightsout':
                self._recording_source = None

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

    def _on_webhook_metadata(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data or '{}')
        except json.JSONDecodeError:
            self.get_logger().warn('Failed to decode webhook metadata JSON')
            return
        if not isinstance(payload, dict):
            return
        self._webhook_metadata = {str(k): str(v) for k, v in payload.items()}
        self._run_id = self._webhook_metadata.get('run_id') or self._run_id
        self._batch_id = (
            self._webhook_metadata.get('batch_id') or self._batch_id
        )
        self._ingredient_id = (
            self._webhook_metadata.get('ingredient_id') or self._ingredient_id
        )
        self._run_mode = self._webhook_metadata.get('mode') or self._run_mode
        self._weightment_id = (
            self._webhook_metadata.get('weightment_id') or self._weightment_id
        )
        self._location_id = (
            self._webhook_metadata.get('location_id') or self._location_id
        )
        self._location_code = (
            self._webhook_metadata.get('location_code') or self._location_code
        )
        target_weight = self._webhook_metadata.get('target_weight_g')
        if target_weight is not None:
            try:
                self._target_weight_g = float(target_weight)
            except ValueError:
                pass
        if (
            self._webhook_active
            and self._webhook_start_pending
            and self._current_episode is None
        ):
            self._webhook_start_pending = False
            self._start_webhook_run()
        if self._current_episode and self._current_episode.source == 'webhook':
            self._write_metadata(self._current_episode)

    def _on_webhook_active(self, msg: Bool) -> None:
        self._webhook_active = bool(msg.data)
        if self._webhook_active:
            if self._current_episode is None:
                if self._has_webhook_metadata():
                    self._start_webhook_run()
                else:
                    self._begin_pending_webhook_start()
            return
        if self._current_episode and self._current_episode.source == 'webhook':
            self._finalize_episode('webhook_run_end')
            self._run_folder = None
            self._close_run_registration()
            self._recording_source = None
        self._webhook_start_pending = False
        self._webhook_active_started_at = None
        self._webhook_metadata = {}

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

    # rclpy installs a SIGINT handler by default, but not SIGTERM — the
    # signal `docker stop` / orchestrated shutdowns actually send. Without
    # this, a container stop would jump straight to the SIGKILL grace-period
    # timeout without ever closing the MCAP writer, risking a truncated bag.
    def _handle_sigterm(signum, frame) -> None:  # noqa: ANN001
        node.get_logger().info(
            'Received SIGTERM: finishing active recording before shutdown'
        )
        node._finish_active()
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        # Can race with the custom SIGTERM handler above if rclpy's own
        # default signal handling wins the race — still a normal shutdown,
        # not a crash.
        pass
    finally:
        node._finish_active()
        node._manifest.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
