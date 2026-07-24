"""Tiered local-storage retention watchdog (`local-manifest-retention`).

Reads `manifest.sqlite` (see `manifest.py`) and enforces the plan's
retention policy against the filesystem:

- Tier 0 (`metadata.json`, `features.parquet`, `events.jsonl`): never
  touched here — small, always kept, synced first by the future uplink
  daemon.
- Tier 1 (`run.mcap` and its directory, `vision/`): deleted only once a
  run has been marked `tier1_acked_at` (by the future uplink daemon, once
  it exists), oldest-first, with anomaly-flagged runs exempt.

Until the uplink daemon exists, no run will ever have `tier1_acked_at`
set, so this watchdog is a safe no-op for deletion by construction —
its job today is (a) to be the enforcement point ready for when acks
start arriving, and (b) to surface local storage backlog as a health
event so an unattended Pi filling up its SD card is visible fleet-wide
instead of silently failing the next recording.

The actual policy logic (`run_retention_pass`) is a plain function over a
`RunManifest`, independent of rclpy, so it can be unit-tested without
spinning a node.
"""
from __future__ import annotations

import shutil
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node

from rhapsodi_common.device_config import load_device_config
from rhapsodi_common.health import HealthEventPublisher

from data_collection_manager.manifest import RunManifest, TIER_1


@dataclass
class PruneSummary:
    pruned_run_keys: List[str] = field(default_factory=list)
    bytes_freed: int = 0
    failed_run_keys: List[str] = field(default_factory=list)
    unacked_run_count: int = 0
    unacked_bytes: int = 0


def _path_size_bytes(path: Path) -> int:
    if not path.exists():
        return 0
    if path.is_file():
        return path.stat().st_size
    total = 0
    for child in path.rglob('*'):
        if child.is_file():
            try:
                total += child.stat().st_size
            except OSError:
                continue
    return total


def _delete_path(path: Path) -> None:
    if path.is_dir():
        shutil.rmtree(path, ignore_errors=False)
    elif path.exists():
        path.unlink()


def run_retention_pass(
    manifest: RunManifest,
    health: Optional[HealthEventPublisher] = None,
    unacked_warn_bytes: int = 20 * 1024 * 1024 * 1024,
) -> PruneSummary:
    """One retention-policy pass: prune everything eligible, then report
    backlog. Never raises — a single run's deletion failure is reported and
    skipped so it doesn't block pruning the rest.
    """
    summary = PruneSummary()

    for run in manifest.list_prunable_tier1_runs():
        artifacts = manifest.list_artifacts(run.run_key, tier=TIER_1)
        run_bytes = 0
        failed = False
        for artifact in artifacts:
            artifact_path = Path(artifact.path)
            try:
                run_bytes += _path_size_bytes(artifact_path)
                _delete_path(artifact_path)
            except OSError as exc:
                failed = True
                if health is not None:
                    health.error(
                        'retention_tier1_prune_failed',
                        f'Failed to prune Tier-1 artifact {artifact_path} '
                        f'for run {run.run_key}: {exc!r}',
                        {'run_key': run.run_key, 'path': str(artifact_path)},
                    )
        if failed:
            summary.failed_run_keys.append(run.run_key)
            continue
        manifest.mark_tier1_pruned(run.run_key, bytes_freed=run_bytes)
        summary.pruned_run_keys.append(run.run_key)
        summary.bytes_freed += run_bytes
        if health is not None:
            health.info(
                'retention_tier1_pruned',
                f'Pruned {run_bytes} bytes of acknowledged Tier-1 data '
                f'for run {run.run_key}',
                {'run_key': run.run_key, 'bytes_freed': run_bytes},
            )

    unacked_runs = manifest.list_unacked_tier1_runs()
    summary.unacked_run_count = len(unacked_runs)
    unacked_bytes = 0
    for run in unacked_runs:
        for artifact in manifest.list_artifacts(run.run_key, tier=TIER_1):
            unacked_bytes += _path_size_bytes(Path(artifact.path))
    summary.unacked_bytes = unacked_bytes

    if unacked_bytes >= unacked_warn_bytes and health is not None:
        health.warn(
            'retention_tier1_backlog_high',
            f'{unacked_bytes} bytes of Tier-1 data across '
            f'{summary.unacked_run_count} runs are still unacknowledged '
            'and cannot be pruned yet',
            {
                'unacked_bytes': unacked_bytes,
                'unacked_run_count': summary.unacked_run_count,
                'warn_threshold_bytes': unacked_warn_bytes,
            },
        )

    return summary


class RetentionWatchdogNode(Node):
    """Supervised unit that periodically runs `run_retention_pass` against
    the same `manifest.sqlite` `data_collection_manager` writes to. Runs
    independently of the recorder so a stuck/crashed recorder doesn't stop
    retention enforcement, and vice versa.
    """

    def __init__(self) -> None:
        super().__init__('retention_watchdog')
        device = load_device_config()
        self.declare_parameter('output_root', 'data/lightsout')
        self.declare_parameter('manifest_path', '')
        self.declare_parameter('check_interval_seconds', 300.0)
        self.declare_parameter(
            'unacked_warn_bytes', 20 * 1024 * 1024 * 1024
        )

        output_root = Path(self.get_parameter('output_root').value)
        manifest_path_param = str(
            self.get_parameter('manifest_path').value or ''
        ).strip()
        manifest_path = (
            Path(manifest_path_param)
            if manifest_path_param
            else output_root / 'manifest.sqlite'
        )
        check_interval = max(
            5.0, float(self.get_parameter('check_interval_seconds').value)
        )
        self._unacked_warn_bytes = int(
            self.get_parameter('unacked_warn_bytes').value
        )

        self._health = HealthEventPublisher(
            self, 'retention_watchdog', device_id=device.device_id
        )
        self._manifest = RunManifest(manifest_path)
        self.get_logger().info(
            f'Retention watchdog started. manifest={manifest_path} '
            f'check_interval_seconds={check_interval}'
        )
        self.create_timer(check_interval, self._tick)

    def _tick(self) -> None:
        summary = run_retention_pass(
            self._manifest,
            health=self._health,
            unacked_warn_bytes=self._unacked_warn_bytes,
        )
        if summary.pruned_run_keys or summary.failed_run_keys:
            self.get_logger().info(
                f'Retention pass: pruned={len(summary.pruned_run_keys)} '
                f'bytes_freed={summary.bytes_freed} '
                f'failed={len(summary.failed_run_keys)} '
                f'unacked_runs={summary.unacked_run_count} '
                f'unacked_bytes={summary.unacked_bytes}'
            )


def main() -> None:
    rclpy.init()
    node = RetentionWatchdogNode()
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        # rclpy's own default SIGINT/SIGTERM handling already called
        # rclpy.shutdown() — nothing to clean up here beyond the node
        # itself, and this is a normal shutdown path, not a crash.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
