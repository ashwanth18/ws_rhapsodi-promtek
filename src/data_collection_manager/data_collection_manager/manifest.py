"""Local run index / outbox state (``manifest.sqlite``).

Durable, transactional record of every run this Pi has produced and where
it stands in the sync/retention lifecycle — replacing the current
"fire, forget, hope" behaviour where the only trace of a run's fate is a
log line that may have already scrolled off `docker logs`.

Schema follows the plan's tiering directly:

- Tier 0 (``metadata.json``, ``features.parquet``, ``events.jsonl``): small,
  always kept, synced first. Never pruned by this module.
- Tier 1 (``run.mcap`` and friends, ``vision/``): bulky, pruned only once
  the (future) uplink daemon has recorded a `tier1_acked_at` for the run,
  oldest-first, with anomaly-flagged runs exempt.

This module owns only the SQLite index. It does not talk to the network
(that's `uplink-daemon`) and does not decide *when* to prune beyond "is it
eligible" (that's `retention_watchdog.py`, which also does the actual
filesystem deletion).
"""
from __future__ import annotations

import sqlite3
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

TIER_0 = 0
TIER_1 = 1

_SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_key TEXT PRIMARY KEY,
    robot_id TEXT NOT NULL,
    device_id TEXT,
    run_folder TEXT NOT NULL,
    source TEXT,
    mode TEXT,
    created_at TEXT NOT NULL,
    completed_at TEXT,
    tier0_synced_at TEXT,
    tier1_acked_at TEXT,
    tier1_pruned_at TEXT,
    tier1_bytes_freed INTEGER,
    anomaly_flag INTEGER NOT NULL DEFAULT 0,
    anomaly_reason TEXT
);

CREATE TABLE IF NOT EXISTS artifacts (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_key TEXT NOT NULL REFERENCES runs(run_key),
    tier INTEGER NOT NULL CHECK (tier IN (0, 1)),
    path TEXT NOT NULL,
    recorded_at TEXT NOT NULL,
    UNIQUE(run_key, path)
);

CREATE INDEX IF NOT EXISTS idx_artifacts_run_key ON artifacts(run_key);
CREATE INDEX IF NOT EXISTS idx_runs_tier1_ack ON runs(tier1_acked_at, tier1_pruned_at, anomaly_flag);

-- Resume offset for fleet-wide, non-run-scoped append-only files (today:
-- health.jsonl) that the uplink daemon tails incrementally instead of
-- re-uploading in full each pass.
CREATE TABLE IF NOT EXISTS fleet_sync_state (
    file_path TEXT PRIMARY KEY,
    synced_bytes INTEGER NOT NULL DEFAULT 0,
    updated_at TEXT NOT NULL
);
"""


def _utcnow() -> str:
    return datetime.now(timezone.utc).isoformat()


@dataclass
class RunRecord:
    run_key: str
    robot_id: str
    device_id: Optional[str]
    run_folder: str
    source: Optional[str]
    mode: Optional[str]
    created_at: str
    completed_at: Optional[str]
    tier0_synced_at: Optional[str]
    tier1_acked_at: Optional[str]
    tier1_pruned_at: Optional[str]
    tier1_bytes_freed: Optional[int]
    anomaly_flag: bool
    anomaly_reason: Optional[str]

    @classmethod
    def from_row(cls, row: sqlite3.Row) -> 'RunRecord':
        return cls(
            run_key=row['run_key'],
            robot_id=row['robot_id'],
            device_id=row['device_id'],
            run_folder=row['run_folder'],
            source=row['source'],
            mode=row['mode'],
            created_at=row['created_at'],
            completed_at=row['completed_at'],
            tier0_synced_at=row['tier0_synced_at'],
            tier1_acked_at=row['tier1_acked_at'],
            tier1_pruned_at=row['tier1_pruned_at'],
            tier1_bytes_freed=row['tier1_bytes_freed'],
            anomaly_flag=bool(row['anomaly_flag']),
            anomaly_reason=row['anomaly_reason'],
        )


@dataclass
class ArtifactRecord:
    run_key: str
    tier: int
    path: str
    recorded_at: str

    @classmethod
    def from_row(cls, row: sqlite3.Row) -> 'ArtifactRecord':
        return cls(
            run_key=row['run_key'],
            tier=row['tier'],
            path=row['path'],
            recorded_at=row['recorded_at'],
        )


class RunManifest:
    """Thread-safe handle onto one Pi's `manifest.sqlite`.

    One instance is shared by `data_collection_manager` (the writer, as
    runs/artifacts are produced) and `retention_watchdog` (the reader/pruner)
    — they may be separate processes, so WAL mode is used for safe
    concurrent access from multiple connections to the same file.
    """

    def __init__(self, db_path: Path) -> None:
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._conn = sqlite3.connect(
            str(self.db_path), check_same_thread=False
        )
        self._conn.row_factory = sqlite3.Row
        with self._lock:
            self._conn.execute('PRAGMA journal_mode=WAL')
            self._conn.executescript(_SCHEMA)
            self._conn.commit()

    def close(self) -> None:
        with self._lock:
            self._conn.close()

    def upsert_run(
        self,
        run_key: str,
        robot_id: str,
        run_folder: Path,
        source: Optional[str] = None,
        mode: Optional[str] = None,
        device_id: Optional[str] = None,
        created_at: Optional[str] = None,
    ) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                """
                INSERT INTO runs (
                    run_key, robot_id, device_id, run_folder, source,
                    mode, created_at
                ) VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(run_key) DO NOTHING
                """,
                (
                    run_key,
                    robot_id,
                    device_id,
                    str(run_folder),
                    source,
                    mode,
                    created_at or _utcnow(),
                ),
            )

    def record_artifact(
        self,
        run_key: str,
        tier: int,
        path: Path,
        recorded_at: Optional[str] = None,
    ) -> None:
        if tier not in (TIER_0, TIER_1):
            raise ValueError(f'tier must be 0 or 1, got {tier}')
        with self._lock, self._conn:
            self._conn.execute(
                """
                INSERT INTO artifacts (run_key, tier, path, recorded_at)
                VALUES (?, ?, ?, ?)
                ON CONFLICT(run_key, path) DO UPDATE SET
                    tier = excluded.tier,
                    recorded_at = excluded.recorded_at
                """,
                (run_key, tier, str(path), recorded_at or _utcnow()),
            )

    def mark_run_complete(
        self, run_key: str, completed_at: Optional[str] = None
    ) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                'UPDATE runs SET completed_at = ? WHERE run_key = ?',
                (completed_at or _utcnow(), run_key),
            )

    def mark_tier0_synced(
        self, run_key: str, synced_at: Optional[str] = None
    ) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                'UPDATE runs SET tier0_synced_at = ? WHERE run_key = ?',
                (synced_at or _utcnow(), run_key),
            )

    def mark_tier1_acked(
        self, run_key: str, acked_at: Optional[str] = None
    ) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                'UPDATE runs SET tier1_acked_at = ? WHERE run_key = ?',
                (acked_at or _utcnow(), run_key),
            )

    def flag_anomaly(self, run_key: str, reason: str) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                'UPDATE runs SET anomaly_flag = 1, anomaly_reason = ? '
                'WHERE run_key = ?',
                (reason, run_key),
            )

    def mark_tier1_pruned(
        self,
        run_key: str,
        bytes_freed: int,
        pruned_at: Optional[str] = None,
    ) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                'UPDATE runs SET tier1_pruned_at = ?, tier1_bytes_freed = ? '
                'WHERE run_key = ?',
                (pruned_at or _utcnow(), int(bytes_freed), run_key),
            )

    def get_run(self, run_key: str) -> Optional[RunRecord]:
        with self._lock:
            cur = self._conn.execute(
                'SELECT * FROM runs WHERE run_key = ?', (run_key,)
            )
            row = cur.fetchone()
        return RunRecord.from_row(row) if row else None

    def list_runs(self) -> List[RunRecord]:
        with self._lock:
            cur = self._conn.execute(
                'SELECT * FROM runs ORDER BY created_at ASC'
            )
            rows = cur.fetchall()
        return [RunRecord.from_row(r) for r in rows]

    def list_artifacts(
        self, run_key: str, tier: Optional[int] = None
    ) -> List[ArtifactRecord]:
        with self._lock:
            if tier is None:
                cur = self._conn.execute(
                    'SELECT * FROM artifacts WHERE run_key = ? '
                    'ORDER BY path ASC',
                    (run_key,),
                )
            else:
                cur = self._conn.execute(
                    'SELECT * FROM artifacts WHERE run_key = ? AND tier = ? '
                    'ORDER BY path ASC',
                    (run_key, tier),
                )
            rows = cur.fetchall()
        return [ArtifactRecord.from_row(r) for r in rows]

    def list_prunable_tier1_runs(self) -> List[RunRecord]:
        """Runs whose Tier-1 artifacts are safe to delete right now:
        acknowledged by the ingestion service, not already pruned, and not
        exempted by an anomaly flag. Oldest-created first.
        """
        with self._lock:
            cur = self._conn.execute(
                """
                SELECT * FROM runs
                WHERE tier1_acked_at IS NOT NULL
                  AND tier1_pruned_at IS NULL
                  AND anomaly_flag = 0
                ORDER BY created_at ASC
                """
            )
            rows = cur.fetchall()
        return [RunRecord.from_row(r) for r in rows]

    def get_fleet_sync_offset(self, file_path: Path) -> int:
        with self._lock:
            cur = self._conn.execute(
                'SELECT synced_bytes FROM fleet_sync_state '
                'WHERE file_path = ?',
                (str(file_path),),
            )
            row = cur.fetchone()
        return int(row['synced_bytes']) if row else 0

    def set_fleet_sync_offset(self, file_path: Path, synced_bytes: int) -> None:
        with self._lock, self._conn:
            self._conn.execute(
                """
                INSERT INTO fleet_sync_state (file_path, synced_bytes, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(file_path) DO UPDATE SET
                    synced_bytes = excluded.synced_bytes,
                    updated_at = excluded.updated_at
                """,
                (str(file_path), int(synced_bytes), _utcnow()),
            )

    def list_runs_needing_tier0_sync(self) -> List[RunRecord]:
        with self._lock:
            cur = self._conn.execute(
                """
                SELECT * FROM runs
                WHERE tier0_synced_at IS NULL
                ORDER BY created_at ASC
                """
            )
            rows = cur.fetchall()
        return [RunRecord.from_row(r) for r in rows]

    def list_runs_needing_tier1_upload(self) -> List[RunRecord]:
        """Runs whose Tier 0 has already synced (so the server has a run
        record to attach Tier-1 blobs to) but whose Tier 1 hasn't been
        acked yet."""
        with self._lock:
            cur = self._conn.execute(
                """
                SELECT * FROM runs
                WHERE tier0_synced_at IS NOT NULL
                  AND tier1_acked_at IS NULL
                  AND tier1_pruned_at IS NULL
                ORDER BY created_at ASC
                """
            )
            rows = cur.fetchall()
        return [RunRecord.from_row(r) for r in rows]

    def list_unacked_tier1_runs(self) -> List[RunRecord]:
        """Runs still holding local Tier-1 data because nothing has acked
        it yet (either no uplink daemon exists yet, or it hasn't caught up)
        — used to size the storage-backlog warning."""
        with self._lock:
            cur = self._conn.execute(
                """
                SELECT * FROM runs
                WHERE tier1_acked_at IS NULL
                  AND tier1_pruned_at IS NULL
                ORDER BY created_at ASC
                """
            )
            rows = cur.fetchall()
        return [RunRecord.from_row(r) for r in rows]
