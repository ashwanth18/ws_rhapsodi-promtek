"""Export API: run listings, CSV downloads, artifact serving, bundle zips."""
from __future__ import annotations

import csv
import io
import json
import logging
import sqlite3
import uuid
import zipfile
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from fastapi import APIRouter, HTTPException, Query
from fastapi.responses import FileResponse, StreamingResponse

from .database import SessionLocal
from .exports_dir import ensure_exports_dir, resolve_data_output_root, sweep_stale_exports
from .models import Artifact, LightsOutProcessed, RobotWeightmentRun, Run

LOG = logging.getLogger('export')

router = APIRouter(prefix='/export', tags=['export'])

ARTIFACT_KINDS = frozenset({'metadata', 'events', 'parquet', 'mcap'})
BUNDLE_MAX_BYTES = 4 * 1024 * 1024 * 1024  # 4 GiB

_KIND_FILENAMES = {
    'metadata': 'metadata.json',
    'events': 'events.jsonl',
    'parquet': 'features.parquet',
    'mcap': 'run.mcap',
}


def parse_request_datetime(value: str | None) -> datetime | None:
    if not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=f'Invalid datetime: {value}') from exc
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def datetime_to_ns(value: datetime | None) -> int | None:
    if value is None:
        return None
    return int(value.timestamp() * 1_000_000_000)


def _safe_artifact_path(raw_path: str | Path, *, data_root: Path | None = None) -> Path:
    """Resolve ``raw_path`` and ensure it stays under ``DATA_OUTPUT_ROOT``."""
    root = (data_root or resolve_data_output_root()).resolve()
    candidate = Path(raw_path)
    if not candidate.is_absolute():
        candidate = root / candidate
    resolved = candidate.resolve()
    try:
        resolved.relative_to(root)
    except ValueError as exc:
        raise HTTPException(status_code=403, detail='Path outside DATA_OUTPUT_ROOT') from exc
    return resolved


def _artifact_info(path: Path | None) -> dict[str, Any]:
    if path is None:
        return {'present': False, 'size_bytes': None, 'path': None}
    if not path.is_file():
        return {'present': False, 'size_bytes': None, 'path': str(path)}
    try:
        size = path.stat().st_size
    except OSError:
        size = None
    return {'present': True, 'size_bytes': size, 'path': str(path)}


def _lookup_manifest_sync(run_key: str | None, data_root: Path) -> dict[str, Any] | None:
    if not run_key:
        return None
    manifest_path = data_root / 'manifest.sqlite'
    if not manifest_path.is_file():
        return None
    try:
        conn = sqlite3.connect(f'file:{manifest_path}?mode=ro', uri=True)
        conn.row_factory = sqlite3.Row
        row = conn.execute(
            'SELECT tier0_synced_at, tier1_acked_at, tier1_pruned_at, anomaly_flag '
            'FROM runs WHERE run_key = ?',
            (run_key,),
        ).fetchone()
        conn.close()
    except sqlite3.Error as exc:
        LOG.warning('manifest.sqlite read failed: %s', exc)
        return None
    if row is None:
        return None
    return {
        'tier0_synced_at': row['tier0_synced_at'],
        'tier1_acked_at': row['tier1_acked_at'],
        'tier1_pruned_at': row['tier1_pruned_at'],
        'anomaly_flag': bool(row['anomaly_flag']),
    }


def _resolve_kind_path(
    kind: str,
    *,
    run_key: str,
    data_root: Path,
    db_paths: dict[str, str | None],
) -> Path | None:
    db_key = kind if kind != 'events' else None
    for source in (db_paths.get(kind), db_paths.get(f'{kind}_path')):
        if source:
            try:
                return _safe_artifact_path(source, data_root=data_root)
            except HTTPException:
                pass
    run_dir = data_root / run_key
    filename = _KIND_FILENAMES.get(kind)
    if filename and run_dir.is_dir():
        candidate = run_dir / filename
        if candidate.is_file():
            return candidate
    return None


def _artifact_paths_for_run(
    run_row: Run,
    processed_row: LightsOutProcessed,
    robot_run: RobotWeightmentRun | None,
    db: Session,
) -> dict[str, str | None]:
    paths: dict[str, str | None] = {
        'parquet': processed_row.parquet_path,
        'mcap': robot_run.mcap_path if robot_run else None,
    }
    if robot_run and robot_run.parquet_path:
        paths['parquet'] = paths['parquet'] or robot_run.parquet_path
    for artifact in db.query(Artifact).filter(Artifact.run_db_id == run_row.id).all():
        atype = (artifact.artifact_type or '').lower()
        if atype in {'parquet', 'mcap', 'metadata', 'events'}:
            paths[atype] = artifact.path
    return paths


def _build_export_query(
    db: Session,
    *,
    mode: str | None,
    batch_id: str | None,
    episode_index: int | None,
    powder_id: str | None,
    run_key: str | None,
    time_from: str | None,
    time_to: str | None,
):
    start_from_ns = datetime_to_ns(parse_request_datetime(time_from))
    start_to_ns = datetime_to_ns(parse_request_datetime(time_to))

    query = db.query(LightsOutProcessed, Run).join(
        Run, LightsOutProcessed.run_db_id == Run.id
    )
    if mode:
        query = query.filter(LightsOutProcessed.mode == mode)
    if batch_id:
        query = query.filter(LightsOutProcessed.batch_id == batch_id)
    if episode_index is not None:
        query = query.filter(LightsOutProcessed.episode_index == episode_index)
    if powder_id:
        query = query.filter(LightsOutProcessed.powder_id == powder_id)
    if run_key:
        query = query.filter(Run.run_key == run_key)
    if start_from_ns is not None:
        query = query.filter(
            Run.start_time_ns.is_not(None), Run.start_time_ns >= start_from_ns
        )
    if start_to_ns is not None:
        query = query.filter(
            Run.start_time_ns.is_not(None), Run.start_time_ns <= start_to_ns
        )
    return query


def _serialize_export_row(
    processed_row: LightsOutProcessed,
    run_row: Run,
    robot_run: RobotWeightmentRun | None,
    db: Session,
    data_root: Path,
) -> dict[str, Any]:
    db_paths = _artifact_paths_for_run(run_row, processed_row, robot_run, db)
    rk = run_row.run_key or ''
    artifacts: dict[str, Any] = {}
    for kind in ARTIFACT_KINDS:
        apath = _resolve_kind_path(
            kind, run_key=rk, data_root=data_root, db_paths=db_paths
        )
        artifacts[kind] = _artifact_info(apath)

    return {
        'processed_id': processed_row.id,
        'run_db_id': run_row.id,
        'run_key': run_row.run_key,
        'run_id': processed_row.run_id or run_row.run_id,
        'robot_id': processed_row.robot_id,
        'batch_id': processed_row.batch_id,
        'episode_index': processed_row.episode_index,
        'mode': processed_row.mode,
        'environment': run_row.environment,
        'layout_id': processed_row.layout_id or run_row.layout_id,
        'layout_hash': processed_row.layout_hash or run_row.layout_hash,
        'poses_hash': processed_row.poses_hash or run_row.poses_hash,
        'tool_id': processed_row.tool_id or run_row.tool_id,
        'authored_in': processed_row.authored_in or run_row.authored_in,
        'robot_key': getattr(processed_row, 'robot_key', None)
        or getattr(run_row, 'robot_key', None),
        'powder_id': processed_row.powder_id,
        'powder_name': processed_row.powder_name,
        'lot_code': processed_row.lot_code,
        'operator': processed_row.operator,
        'notes': processed_row.notes,
        'target_weight_g': processed_row.target_weight_g,
        'final_weight_g': processed_row.final_weight_g,
        'net_weight_g': processed_row.net_weight_g,
        'stop_on': processed_row.stop_on,
        'stop_value': processed_row.stop_value,
        'stop_reason': processed_row.stop_reason,
        'start_time_ns': run_row.start_time_ns,
        'end_time_ns': run_row.end_time_ns,
        'parquet_path': processed_row.parquet_path,
        'artifacts': artifacts,
        'manifest_sync': _lookup_manifest_sync(run_row.run_key, data_root),
        'event_id': robot_run.event_id if robot_run else None,
        'weightment_id': robot_run.weightment_id if robot_run else None,
    }


def _find_robot_run(db: Session, processed_row: LightsOutProcessed, run_row: Run):
    robot_run = None
    if processed_row.id is not None:
        robot_run = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.processed_id == processed_row.id)
            .order_by(RobotWeightmentRun.id.desc())
            .first()
        )
    if robot_run is None and run_row.run_id:
        robot_run = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.trace_run_id == run_row.run_id)
            .order_by(RobotWeightmentRun.id.desc())
            .first()
        )
    return robot_run


def _fetch_export_rows(
    db: Session,
    *,
    mode: str | None = None,
    batch_id: str | None = None,
    episode_index: int | None = None,
    powder_id: str | None = None,
    run_key: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
    limit: int = 50,
    offset: int = 0,
) -> tuple[list[dict[str, Any]], int]:
    safe_limit = max(limit, 0)
    safe_offset = max(offset, 0)
    data_root = resolve_data_output_root()

    base_query = _build_export_query(
        db,
        mode=mode,
        batch_id=batch_id,
        episode_index=episode_index,
        powder_id=powder_id,
        run_key=run_key,
        time_from=time_from,
        time_to=time_to,
    )
    total = base_query.count()
    rows_query = base_query.order_by(
        Run.start_time_ns.is_(None),
        Run.start_time_ns.desc(),
        LightsOutProcessed.id.desc(),
    ).offset(safe_offset)
    if safe_limit > 0:
        rows_query = rows_query.limit(safe_limit)

    data = []
    for processed_row, run_row in rows_query.all():
        robot_run = _find_robot_run(db, processed_row, run_row)
        data.append(
            _serialize_export_row(
                processed_row, run_row, robot_run, db, data_root
            )
        )
    return data, total


RUNS_CSV_FIELDS = [
    'run_key',
    'run_id',
    'mode',
    'layout_id',
    'layout_hash',
    'poses_hash',
    'tool_id',
    'authored_in',
    'robot_key',
    'batch_id',
    'episode_index',
    'powder_id',
    'powder_name',
    'lot_code',
    'operator',
    'target_weight_g',
    'final_weight_g',
    'net_weight_g',
    'stop_on',
    'stop_reason',
    'start_time_ns',
    'end_time_ns',
    'metadata_present',
    'events_present',
    'parquet_present',
    'mcap_present',
]


def runs_to_csv(rows: list[dict[str, Any]]) -> str:
    buf = io.StringIO()
    writer = csv.DictWriter(buf, fieldnames=RUNS_CSV_FIELDS, extrasaction='ignore')
    writer.writeheader()
    for row in rows:
        artifacts = row.get('artifacts') or {}
        writer.writerow(
            {
                **row,
                'metadata_present': artifacts.get('metadata', {}).get('present', False),
                'events_present': artifacts.get('events', {}).get('present', False),
                'parquet_present': artifacts.get('parquet', {}).get('present', False),
                'mcap_present': artifacts.get('mcap', {}).get('present', False),
            }
        )
    return buf.getvalue()


def _parquet_to_csv_bytes(parquet_path: Path) -> bytes:
    try:
        import pyarrow.parquet as pq  # noqa: PLC0415
    except ImportError as exc:
        raise HTTPException(
            status_code=501, detail='pyarrow is required for timeseries export'
        ) from exc
    table = pq.read_table(parquet_path)
    sink = io.BytesIO()
    sink.write(table.to_pandas().to_csv(index=False).encode('utf-8'))
    return sink.getvalue()


@router.get('/runs')
def export_runs(
    limit: int = 50,
    offset: int = 0,
    mode: str | None = None,
    batch_id: str | None = None,
    episode_index: int | None = None,
    powder_id: str | None = None,
    run_key: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
) -> dict:
    db = SessionLocal()
    try:
        rows, total = _fetch_export_rows(
            db,
            mode=mode,
            batch_id=batch_id,
            episode_index=episode_index,
            powder_id=powder_id,
            run_key=run_key,
            time_from=time_from,
            time_to=time_to,
            limit=limit,
            offset=offset,
        )
    finally:
        db.close()
    return {
        'rows': rows,
        'total': total,
        'limit': max(limit, 0),
        'offset': max(offset, 0),
    }


@router.get('/runs.csv')
def export_runs_csv(
    limit: int = 500,
    offset: int = 0,
    mode: str | None = None,
    batch_id: str | None = None,
    episode_index: int | None = None,
    powder_id: str | None = None,
    run_key: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
) -> StreamingResponse:
    db = SessionLocal()
    try:
        rows, _ = _fetch_export_rows(
            db,
            mode=mode,
            batch_id=batch_id,
            episode_index=episode_index,
            powder_id=powder_id,
            run_key=run_key,
            time_from=time_from,
            time_to=time_to,
            limit=limit,
            offset=offset,
        )
    finally:
        db.close()
    csv_text = runs_to_csv(rows)
    return StreamingResponse(
        iter([csv_text]),
        media_type='text/csv',
        headers={'Content-Disposition': 'attachment; filename="runs.csv"'},
    )


@router.get('/timeseries.csv')
def export_timeseries_csv(
    limit: int = 100,
    mode: str | None = None,
    batch_id: str | None = None,
    powder_id: str | None = None,
    run_key: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
) -> StreamingResponse:
    db = SessionLocal()
    data_root = resolve_data_output_root()
    try:
        rows, _ = _fetch_export_rows(
            db,
            mode=mode,
            batch_id=batch_id,
            powder_id=powder_id,
            run_key=run_key,
            time_from=time_from,
            time_to=time_to,
            limit=limit,
            offset=0,
        )
    finally:
        db.close()

    combined = io.BytesIO()
    header_written = False
    for row in rows:
        rk = row.get('run_key') or ''
        apath = _resolve_kind_path(
            'parquet',
            run_key=rk,
            data_root=data_root,
            db_paths={'parquet': row.get('parquet_path')},
        )
        if apath is None or not apath.is_file():
            continue
        chunk = _parquet_to_csv_bytes(apath)
        text = chunk.decode('utf-8')
        lines = text.splitlines()
        if not lines:
            continue
        if not header_written:
            combined.write((text + '\n').encode('utf-8'))
            header_written = True
        elif len(lines) > 1:
            combined.write(('\n'.join(lines[1:]) + '\n').encode('utf-8'))

    if not header_written:
        combined.write(b'run_key\n')

    return StreamingResponse(
        iter([combined.getvalue()]),
        media_type='text/csv',
        headers={'Content-Disposition': 'attachment; filename="timeseries.csv"'},
    )


@router.get('/runs/{run_key}/artifacts/{kind}')
def get_run_artifact(run_key: str, kind: str) -> FileResponse:
    if kind not in ARTIFACT_KINDS:
        raise HTTPException(status_code=400, detail=f'Unknown artifact kind: {kind}')

    db = SessionLocal()
    data_root = resolve_data_output_root()
    try:
        pair = (
            db.query(LightsOutProcessed, Run)
            .join(Run, LightsOutProcessed.run_db_id == Run.id)
            .filter(Run.run_key == run_key)
            .order_by(LightsOutProcessed.id.desc())
            .first()
        )
        if pair is None:
            raise HTTPException(status_code=404, detail=f'Run not found: {run_key}')
        processed_row, run_row = pair
        robot_run = _find_robot_run(db, processed_row, run_row)
        db_paths = _artifact_paths_for_run(
            run_row, processed_row, robot_run, db
        )
        apath = _resolve_kind_path(
            kind, run_key=run_key, data_root=data_root, db_paths=db_paths
        )
    finally:
        db.close()

    if apath is None or not apath.is_file():
        raise HTTPException(status_code=404, detail=f'Artifact {kind} not found')

    media_types = {
        'metadata': 'application/json',
        'events': 'application/x-ndjson',
        'parquet': 'application/octet-stream',
        'mcap': 'application/octet-stream',
    }
    return FileResponse(
        apath,
        media_type=media_types.get(kind, 'application/octet-stream'),
        filename=apath.name,
    )


@router.get('/bundle.zip')
def export_bundle(
    run_key: list[str] = Query(default=[]),
    mode: str | None = None,
    batch_id: str | None = None,
    powder_id: str | None = None,
    limit: int = 50,
) -> FileResponse:
    sweep_stale_exports()
    exports_dir = ensure_exports_dir()
    data_root = resolve_data_output_root()

    db = SessionLocal()
    try:
        if run_key:
            rows: list[dict[str, Any]] = []
            for rk in run_key:
                batch_rows, _ = _fetch_export_rows(db, run_key=rk, limit=1, offset=0)
                rows.extend(batch_rows)
        else:
            rows, _ = _fetch_export_rows(
                db,
                mode=mode,
                batch_id=batch_id,
                powder_id=powder_id,
                limit=limit,
                offset=0,
            )
    finally:
        db.close()

    if not rows:
        raise HTTPException(status_code=404, detail='No runs matched export filters')

    bundle_name = f'export-{uuid.uuid4().hex[:12]}.zip'
    bundle_path = exports_dir / bundle_name
    total_bytes = 0

    with zipfile.ZipFile(bundle_path, 'w', compression=zipfile.ZIP_DEFLATED) as zf:
        zf.writestr(
            'manifest.json',
            json.dumps({'runs': [r.get('run_key') for r in rows]}, indent=2),
        )
        for row in rows:
            rk = row.get('run_key') or 'unknown'
            db_paths = {'parquet': row.get('parquet_path')}
            for kind in ARTIFACT_KINDS:
                apath = _resolve_kind_path(
                    kind, run_key=rk, data_root=data_root, db_paths=db_paths
                )
                if apath is None or not apath.is_file():
                    continue
                size = apath.stat().st_size
                if total_bytes + size > BUNDLE_MAX_BYTES:
                    bundle_path.unlink(missing_ok=True)
                    raise HTTPException(
                        status_code=413,
                        detail=f'Bundle exceeds {BUNDLE_MAX_BYTES} byte cap',
                    )
                arcname = f'{rk}/{apath.name}'
                zf.write(apath, arcname=arcname)
                total_bytes += size

    return FileResponse(
        bundle_path,
        media_type='application/zip',
        filename='rhapsodi-export.zip',
    )
