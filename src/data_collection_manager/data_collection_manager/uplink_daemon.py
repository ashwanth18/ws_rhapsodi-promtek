"""Store-and-forward uplink daemon (uplink-daemon).

Watches `manifest.sqlite` and pushes data to `central-ingestion-service`
(see `src/backend/ingestion/ingestion/main.py`), edge -> cloud, in
priority order: fleet health log first, then Tier-0
(features/metadata/events), then Tier-1 (raw MCAP/vision) — matching
the plan's "small local storage + intermittent connectivity" deployment
target. Never deletes anything locally itself; it only sets
`tier0_synced_at`/`tier1_acked_at` in the manifest once the server has
actually acknowledged the data, which is what unblocks
`retention_watchdog`'s Tier-1 pruning.

Runs as its own supervised unit (independent restart from the recorder
and from retention_watchdog), same rationale as recorder-v2: a stuck
upload must not be able to block or crash recording, and vice versa.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node

from rhapsodi_common.device_config import load_device_config
from rhapsodi_common.health import HealthEventPublisher

from data_collection_manager.manifest import TIER_0, TIER_1, RunManifest
from data_collection_manager.uplink_client import UplinkClient, UplinkError

_TIER0_FIELD_NAMES = {
    'metadata.json': 'metadata_json',
    'events.jsonl': 'events_jsonl',
    'features.parquet': 'features_parquet',
}


@dataclass
class UplinkSummary:
    fleet_log_synced_bytes: int = 0
    tier0_synced_run_keys: List[str] = field(default_factory=list)
    tier1_acked_run_keys: List[str] = field(default_factory=list)
    failed_run_keys: List[str] = field(default_factory=list)


def _tier0_field_name(path: Path) -> str:
    return _TIER0_FIELD_NAMES.get(path.name, path.suffix.lstrip('.') or path.name)


def _iter_blob_files(
    artifact_index: int, artifact_path: Path
) -> List[Tuple[str, Path]]:
    """Every regular file that makes up a Tier-1 artifact, paired with a
    stable blob key. Artifacts are usually directories (a bag_path holds
    N .mcap chunks + metadata.yaml); each file inside becomes its own
    resumable blob rather than trying to stream a whole directory as one
    unit."""
    if artifact_path.is_file():
        return [(f'{artifact_index}/{artifact_path.name}', artifact_path)]
    if not artifact_path.is_dir():
        return []
    blobs = []
    for file_path in sorted(artifact_path.rglob('*')):
        if not file_path.is_file():
            continue
        rel = file_path.relative_to(artifact_path).as_posix()
        blobs.append((f'{artifact_index}/{rel}', file_path))
    return blobs


def _sync_fleet_health(
    manifest: RunManifest,
    client: UplinkClient,
    fleet_log_path: Path,
    device_id: str,
) -> int:
    if not fleet_log_path.is_file():
        return 0
    size = fleet_log_path.stat().st_size
    offset = manifest.get_fleet_sync_offset(fleet_log_path)
    if offset >= size:
        return 0
    with open(fleet_log_path, 'rb') as f:
        f.seek(offset)
        new_bytes = f.read()
    if not new_bytes:
        return 0
    client.append_fleet_log(
        device_id, 'health', new_bytes.decode('utf-8', errors='replace')
    )
    manifest.set_fleet_sync_offset(fleet_log_path, offset + len(new_bytes))
    return len(new_bytes)


def run_uplink_pass(
    manifest: RunManifest,
    client: UplinkClient,
    device_id: str,
    fleet_log_path: Optional[Path] = None,
    health: Optional[HealthEventPublisher] = None,
) -> UplinkSummary:
    """One uplink pass: fleet log, then Tier 0, then Tier 1, in that
    priority order. Never raises — a single run's or the fleet log's
    failure is reported and skipped so it doesn't block the rest of the
    pass; everything here is safe to retry next tick since the server
    (not local state) is the source of truth for how much of a Tier-1
    blob has already arrived.
    """
    summary = UplinkSummary()

    if fleet_log_path is not None:
        try:
            summary.fleet_log_synced_bytes = _sync_fleet_health(
                manifest, client, fleet_log_path, device_id
            )
        except UplinkError as exc:
            if health is not None:
                health.warn(
                    'uplink_fleet_log_sync_failed',
                    f'Failed to sync fleet health log: {exc!r}',
                    {'fleet_log_path': str(fleet_log_path)},
                )

    for run in manifest.list_runs_needing_tier0_sync():
        tier0_artifacts = manifest.list_artifacts(run.run_key, tier=TIER_0)
        files: Dict[str, Path] = {}
        for artifact in tier0_artifacts:
            files[_tier0_field_name(Path(artifact.path))] = Path(
                artifact.path
            )
        try:
            client.sync_tier0(
                run.run_key,
                robot_id=run.robot_id,
                device_id=run.device_id or device_id,
                files=files,
            )
            manifest.mark_tier0_synced(run.run_key)
            summary.tier0_synced_run_keys.append(run.run_key)
        except UplinkError as exc:
            summary.failed_run_keys.append(run.run_key)
            if health is not None:
                health.error(
                    'uplink_tier0_sync_failed',
                    f'Failed to sync Tier-0 for run {run.run_key}: {exc!r}',
                    {'run_key': run.run_key},
                )

    for run in manifest.list_runs_needing_tier1_upload():
        tier1_artifacts = manifest.list_artifacts(run.run_key, tier=TIER_1)
        try:
            for artifact_index, artifact in enumerate(tier1_artifacts):
                for blob_key, file_path in _iter_blob_files(
                    artifact_index, Path(artifact.path)
                ):
                    client.upload_blob(run.run_key, blob_key, file_path)
            client.complete_tier1(run.run_key)
            manifest.mark_tier1_acked(run.run_key)
            summary.tier1_acked_run_keys.append(run.run_key)
        except UplinkError as exc:
            summary.failed_run_keys.append(run.run_key)
            if health is not None:
                health.error(
                    'uplink_tier1_upload_failed',
                    f'Failed to upload Tier-1 for run {run.run_key}: '
                    f'{exc!r}',
                    {'run_key': run.run_key},
                )

    return summary


class UplinkDaemonNode(Node):
    def __init__(self) -> None:
        super().__init__('uplink_daemon')
        device = load_device_config()
        self.declare_parameter('output_root', 'data/runs')
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('ingestion_url', device.ingestion_url)
        self.declare_parameter('check_interval_seconds', 30.0)

        output_root = Path(self.get_parameter('output_root').value)
        manifest_path_param = str(
            self.get_parameter('manifest_path').value or ''
        ).strip()
        manifest_path = (
            Path(manifest_path_param)
            if manifest_path_param
            else output_root / 'manifest.sqlite'
        )
        self._device_id = device.device_id
        self._fleet_log_path = output_root / 'health.jsonl'
        ingestion_url = str(
            self.get_parameter('ingestion_url').value or ''
        ).strip()
        check_interval = max(
            5.0, float(self.get_parameter('check_interval_seconds').value)
        )

        self._health = HealthEventPublisher(
            self, 'uplink_daemon', device_id=self._device_id
        )
        self._manifest = RunManifest(manifest_path)
        self._client = UplinkClient(base_url=ingestion_url)
        self.get_logger().info(
            f'Uplink daemon started. manifest={manifest_path} '
            f'ingestion_url={ingestion_url} '
            f'check_interval_seconds={check_interval}'
        )
        self.create_timer(check_interval, self._tick)

    def _tick(self) -> None:
        summary = run_uplink_pass(
            self._manifest,
            self._client,
            device_id=self._device_id,
            fleet_log_path=self._fleet_log_path,
            health=self._health,
        )
        if (
            summary.tier0_synced_run_keys
            or summary.tier1_acked_run_keys
            or summary.failed_run_keys
        ):
            self.get_logger().info(
                f'Uplink pass: fleet_bytes={summary.fleet_log_synced_bytes} '
                f'tier0_synced={len(summary.tier0_synced_run_keys)} '
                f'tier1_acked={len(summary.tier1_acked_run_keys)} '
                f'failed={len(summary.failed_run_keys)}'
            )


def main() -> None:
    rclpy.init()
    node = UplinkDaemonNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
