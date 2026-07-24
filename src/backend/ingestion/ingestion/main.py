"""Rhapsodi data-collection ingestion service (central-ingestion-service).

Receiving side of the store-and-forward uplink implemented by
`data_collection_manager`'s `uplink_client.py`/`uplink_daemon.py` on each
Pi - see that module's docstring for the wire protocol this mirrors.
Deliberately its own FastAPI app/router, fully separate from
`src/backend/app` (condor_agent / MES): a Pi's raw telemetry, health
events, and ML-training artifacts have nothing to do with batch/MES
business events, and keeping them in different deployables means a
change or outage on one side can't take the other down.

Endpoints (all under /v1, matching UplinkClient exactly):

- ``POST /v1/fleet/{device_id}/{log_name}``: append fleet-wide log lines
  (today: only ``log_name == "health"``), parsed into `FleetHealthEvent`
  rows for `incident-detection`'s rules engine to query later.
- ``POST /v1/runs/{run_key}/tier0``: one multipart upload per run for
  metadata.json/events.jsonl (stored inline - small, queried often) and
  features.parquet (stored in the object store - a real file).
- ``GET/PUT /v1/runs/{run_key}/tier1/{blob_key}[/status]``: resumable
  chunked upload of one Tier-1 file, offset-verified server-side.
- ``POST /v1/runs/{run_key}/tier1/{blob_key}/finalize``: sha256-checked
  completion of one blob; a mismatch deletes the partial upload so the
  client's next pass restarts clean instead of endlessly re-finalizing
  corrupt bytes.
- ``POST /v1/runs/{run_key}/tier1/complete``: marks the run fully
  ingested once every blob has finalized.
"""
from __future__ import annotations

import json
import logging
import os
from contextlib import asynccontextmanager
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

from fastapi import Depends, FastAPI, File, Form, HTTPException, Request, UploadFile
from sqlalchemy.orm import Session

from ingestion.database import SessionLocal, engine
from ingestion.models import Base, FleetHealthEvent, IngestedBlob, IngestedRun
from ingestion.object_store import LocalFilesystemObjectStore

logger = logging.getLogger('ingestion')

OBJECT_STORE_ROOT = Path(os.environ.get('OBJECT_STORE_ROOT', '/data/ingested'))

_store: Optional[LocalFilesystemObjectStore] = None


def get_object_store() -> LocalFilesystemObjectStore:
    global _store
    if _store is None:
        _store = LocalFilesystemObjectStore(OBJECT_STORE_ROOT)
    return _store


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()


@asynccontextmanager
async def lifespan(app: FastAPI):
    Base.metadata.create_all(bind=engine)
    yield


app = FastAPI(
    title='Rhapsodi Data-Collection Ingestion Service', lifespan=lifespan
)


@app.get('/health')
def health_check() -> dict:
    return {'status': 'ok'}


# --- Fleet-wide append-only logs ---------------------------------------


@app.post('/v1/fleet/{device_id}/{log_name}')
async def append_fleet_log(
    device_id: str,
    log_name: str,
    request: Request,
    db: Session = Depends(get_db),
) -> dict:
    if log_name != 'health':
        raise HTTPException(
            status_code=404, detail=f'unknown fleet log {log_name!r}'
        )
    raw = (await request.body()).decode('utf-8', errors='replace')
    received_lines = 0
    skipped_lines = 0
    for line in raw.splitlines():
        line = line.strip()
        if not line:
            continue
        received_lines += 1
        try:
            payload = json.loads(line)
        except json.JSONDecodeError:
            # A malformed line must never break the rest of the batch -
            # the sender already lost this event's structure, but the
            # remaining lines are still worth keeping.
            skipped_lines += 1
            continue
        db.add(
            FleetHealthEvent(
                device_id=device_id,
                stamp_sec=payload.get('stamp_sec'),
                stamp_nanosec=payload.get('stamp_nanosec'),
                component=payload.get('component'),
                severity=payload.get('severity'),
                code=payload.get('code'),
                message=payload.get('message'),
                context_json=json.dumps(payload.get('context', {})),
            )
        )
    db.commit()
    return {'received_lines': received_lines, 'skipped_lines': skipped_lines}


# --- Tier 0: small, whole-file --------------------------------------------


@app.post('/v1/runs/{run_key}/tier0')
async def sync_tier0(
    run_key: str,
    robot_id: str = Form(...),
    device_id: str = Form(...),
    metadata_json: Optional[UploadFile] = File(None),
    events_jsonl: Optional[UploadFile] = File(None),
    features_parquet: Optional[UploadFile] = File(None),
    db: Session = Depends(get_db),
    store: LocalFilesystemObjectStore = Depends(get_object_store),
) -> dict:
    run = db.query(IngestedRun).filter_by(run_key=run_key).one_or_none()
    if run is None:
        run = IngestedRun(run_key=run_key)
        db.add(run)
    run.robot_id = robot_id
    run.device_id = device_id
    if metadata_json is not None:
        run.metadata_json = (await metadata_json.read()).decode(
            'utf-8', errors='replace'
        )
    if events_jsonl is not None:
        run.events_jsonl = (await events_jsonl.read()).decode(
            'utf-8', errors='replace'
        )
    if features_parquet is not None:
        key = f'{run_key}/tier0/features.parquet'
        store.write(key, await features_parquet.read())
        run.features_parquet_key = key
    db.commit()
    return {'status': 'ok'}


# --- Tier 1: bulky, resumable, checksum-verified --------------------------


def _blob_object_key(run_key: str, blob_key: str) -> str:
    return f'{run_key}/tier1/{blob_key}'


@app.get('/v1/runs/{run_key}/tier1/{blob_key:path}/status')
def blob_status(
    run_key: str,
    blob_key: str,
    store: LocalFilesystemObjectStore = Depends(get_object_store),
) -> dict:
    key = _blob_object_key(run_key, blob_key)
    return {'received_bytes': store.part_size(key)}


@app.put('/v1/runs/{run_key}/tier1/{blob_key:path}')
async def upload_blob_chunk(
    run_key: str,
    blob_key: str,
    request: Request,
    store: LocalFilesystemObjectStore = Depends(get_object_store),
) -> dict:
    offset = int(request.headers.get('X-Upload-Offset', '0'))
    body = await request.body()
    key = _blob_object_key(run_key, blob_key)
    try:
        new_size = store.append_part(key, offset, body)
    except ValueError:
        # Tell the client what we actually have so its next `blob_status`
        # GET (triggered by the resulting UplinkError on the client side)
        # resumes from the correct point instead of retrying the same bad
        # offset forever.
        raise HTTPException(
            status_code=409,
            detail={'received_bytes': store.part_size(key)},
        )
    return {'received_bytes': new_size}


@app.post('/v1/runs/{run_key}/tier1/{blob_key:path}/finalize')
async def finalize_blob(
    run_key: str,
    blob_key: str,
    payload: dict,
    db: Session = Depends(get_db),
    store: LocalFilesystemObjectStore = Depends(get_object_store),
) -> dict:
    key = _blob_object_key(run_key, blob_key)
    expected_sha256 = payload.get('sha256')
    expected_size = payload.get('size')
    actual_size = store.part_size(key)
    actual_sha256 = store.part_sha256(key) if actual_size else None

    if actual_sha256 != expected_sha256 or actual_size != expected_size:
        # Corrupt or incomplete transfer: delete the partial upload so a
        # retry starts from byte 0 instead of endlessly re-finalizing the
        # same bad bytes (see `blob_status`/`part_size`'s "already
        # finalized" fallback, which would otherwise make this permanent).
        store.delete_part(key)
        raise HTTPException(
            status_code=409,
            detail={
                'error': 'checksum_mismatch',
                'expected_sha256': expected_sha256,
                'actual_sha256': actual_sha256,
            },
        )

    store.finalize_part(key)
    existing = (
        db.query(IngestedBlob)
        .filter_by(run_key=run_key, blob_key=blob_key)
        .one_or_none()
    )
    if existing is None:
        existing = IngestedBlob(run_key=run_key, blob_key=blob_key)
        db.add(existing)
    existing.object_store_key = key
    existing.size_bytes = actual_size
    existing.sha256 = actual_sha256
    db.commit()
    return {'acked': True}


@app.post('/v1/runs/{run_key}/tier1/complete')
def complete_tier1(run_key: str, db: Session = Depends(get_db)) -> dict:
    run = db.query(IngestedRun).filter_by(run_key=run_key).one_or_none()
    if run is None:
        # Tier 1 completion always follows a prior Tier-0 sync in
        # UplinkClient's protocol (see uplink_daemon.list_runs_needing_
        # tier1_upload's tier0_synced_at precondition) - a request for an
        # unknown run means the caller and server have desynced state,
        # which is worth surfacing rather than silently no-op-ing.
        raise HTTPException(
            status_code=404, detail=f'unknown run_key {run_key!r}'
        )
    run.tier1_completed_at = datetime.now(timezone.utc)
    db.commit()
    return {'acked': True}
