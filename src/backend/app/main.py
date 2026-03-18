import asyncio
import json
import logging
import os
import socket
from datetime import datetime, timedelta, timezone
from typing import Any
from urllib import error, request

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from sqlalchemy import text

from .database import SessionLocal, engine
from .models import (
    Base,
    LightsOutProcessed,
    RobotWeightmentRun,
    Run,
    StockLocationAllocation,
    WebhookWeightment,
)
from .pipeline import store_processed_run
from .rosbridge_client import rosbridge_robot_client
from .robot_mapping import resolve_robot_targets
from .schemas import (
    ProcessedRequest,
    ProcessedResponse,
    RobotRunCompletionRequest,
)

Base.metadata.create_all(bind=engine)

logger = logging.getLogger(__name__)

WEIGHMENT_URL = os.environ.get(
    'WEIGHMENT_URL', 'http://localhost:5002/batch/weighment'
)
BATCH_END_URL = os.environ.get('BATCH_END_URL', 'http://localhost:5002/batch/end')
ROBOT_START_ADAPTER_URL = os.environ.get(
    'ROBOT_START_ADAPTER_URL',
    'http://host.docker.internal:8010/start_webhook_weightment',
)
ROBOT_START_ADAPTER_TIMEOUT_SECONDS = float(
    os.environ.get('ROBOT_START_ADAPTER_TIMEOUT_SECONDS', '15')
)
ROBOT_START_TIMEOUT_SECONDS = int(
    os.environ.get('ROBOT_START_TIMEOUT_SECONDS', '30')
)
ROBOT_RUNNING_TIMEOUT_SECONDS = int(
    os.environ.get('ROBOT_RUNNING_TIMEOUT_SECONDS', '300')
)


def utc_now_dt() -> datetime:
    return datetime.now(timezone.utc)


def utc_now() -> str:
    return utc_now_dt().isoformat()


def ensure_webhook_weightment_columns() -> None:
    statements = [
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS actual_weight_kg DOUBLE PRECISION',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS batch_auto_run_enabled BOOLEAN DEFAULT FALSE',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS start_utc VARCHAR',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS end_utc VARCHAR',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS energy_kwh INTEGER',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS lot_code VARCHAR',
    ]
    with engine.begin() as conn:
        for statement in statements:
            conn.execute(text(statement))


ensure_webhook_weightment_columns()


def ensure_stock_location_allocation_columns() -> None:
    statements = [
        'ALTER TABLE stock_location_allocations ADD COLUMN IF NOT EXISTS context_id VARCHAR',
        'ALTER TABLE stock_location_allocations ADD COLUMN IF NOT EXISTS site_id VARCHAR',
        'ALTER TABLE stock_location_allocations ADD COLUMN IF NOT EXISTS created_utc VARCHAR',
        'ALTER TABLE stock_location_allocations ADD COLUMN IF NOT EXISTS stock_item_location_id INTEGER',
        'ALTER TABLE stock_location_allocations ADD COLUMN IF NOT EXISTS stock_item_location_code VARCHAR',
        'ALTER TABLE stock_location_allocations ADD COLUMN IF NOT EXISTS stock_item_id VARCHAR',
        'ALTER TABLE stock_location_allocations ADD COLUMN IF NOT EXISTS stock_item_code VARCHAR',
        'ALTER TABLE stock_location_allocations ADD COLUMN IF NOT EXISTS stock_item_name VARCHAR',
    ]
    with engine.begin() as conn:
        for statement in statements:
            conn.execute(text(statement))


ensure_stock_location_allocation_columns()


def ensure_run_columns() -> None:
    statements = [
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS metadata_json TEXT',
    ]
    with engine.begin() as conn:
        for statement in statements:
            conn.execute(text(statement))


ensure_run_columns()


def ensure_robot_weightment_run_columns() -> None:
    statements = [
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS trace_run_id VARCHAR',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS processed_run_db_id INTEGER',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS processed_id INTEGER',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS mcap_path VARCHAR',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS parquet_path VARCHAR',
    ]
    with engine.begin() as conn:
        for statement in statements:
            conn.execute(text(statement))


ensure_robot_weightment_run_columns()

app = FastAPI(title='Rhapsodi Backend')

cors_origins = [
    origin.strip()
    for origin in os.environ.get('CORS_ORIGINS', '').split(',')
    if origin.strip()
]
if not cors_origins:
    cors_origins = ['http://localhost:5173', 'http://localhost:3000']

app.add_middleware(
    CORSMiddleware,
    allow_origins=cors_origins,
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)


@app.on_event('startup')
def reconcile_stale_runs_on_startup() -> None:
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
    except Exception:
        logger.exception('Failed to reconcile stale robot runs during startup')
        db.rollback()
    finally:
        db.close()


@app.get('/health')
def health() -> dict:
    return {'status': 'ok'}


@app.get('/host_info')
def host_info() -> dict:
    return {'hostname': socket.gethostname()}


def parse_json_text(value: str | None) -> Any:
    if not value:
        return None
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        return value


def ns_to_iso_utc(value: int | None) -> str | None:
    if value is None:
        return None
    try:
        return datetime.fromtimestamp(value / 1e9, tz=timezone.utc).isoformat()
    except (OverflowError, OSError, ValueError, TypeError):
        return None


def serialize_webhook_weightment(row: WebhookWeightment) -> dict:
    return {
        'id': row.id,
        'event_id': row.event_id,
        'sent_utc': row.sent_utc,
        'user_id': row.user_id,
        'site_id': row.site_id,
        'batch_id': row.batch_id,
        'batch_number': row.batch_number,
        'work_order_id': row.work_order_id,
        'batch_target_quantity': row.batch_target_quantity,
        'ingredient_id': row.ingredient_id,
        'target_weight_kg': row.target_weight_kg,
        'actual_weight_kg': row.actual_weight_kg,
        'weightment_completed': row.weightment_completed,
        'batch_auto_run_enabled': row.batch_auto_run_enabled,
        'start_utc': row.start_utc,
        'end_utc': row.end_utc,
        'energy_kwh': row.energy_kwh,
        'lot_code': row.lot_code,
        'created_at': row.created_at.isoformat() if row.created_at else None,
    }


def serialize_stock_location_allocation(row: StockLocationAllocation) -> dict:
    return {
        'id': row.id,
        'event_id': row.event_id,
        'context_id': row.context_id,
        'site_id': row.site_id,
        'created_utc': row.created_utc,
        'stock_item_location_id': row.stock_item_location_id,
        'stock_item_location_code': row.stock_item_location_code,
        'stock_item_id': row.stock_item_id,
        'stock_item_code': row.stock_item_code,
        'stock_item_name': row.stock_item_name,
        'created_at': row.created_at.isoformat() if row.created_at else None,
    }


def serialize_robot_run(row: RobotWeightmentRun) -> dict:
    return {
        'id': row.id,
        'weightment_id': row.weightment_id,
        'event_id': row.event_id,
        'batch_id': row.batch_id,
        'ingredient_id': row.ingredient_id,
        'site_id': row.site_id,
        'stock_location_id': row.stock_location_id,
        'stock_location_code': row.stock_location_code,
        'ingredient_name': row.ingredient_name,
        'pickup_target_name': row.pickup_target_name,
        'weigh_target_name': row.weigh_target_name,
        'return_target_name': row.return_target_name,
        'target_weight_kg': row.target_weight_kg,
        'target_weight_g': row.target_weight_g,
        'weight_tolerance_g': row.weight_tolerance_g,
        'expected_lot': row.expected_lot,
        'trace_run_id': row.trace_run_id,
        'status': row.status,
        'error_message': row.error_message,
        'requested_at': row.requested_at.isoformat() if row.requested_at else None,
        'started_at': row.started_at.isoformat() if row.started_at else None,
        'finished_at': row.finished_at.isoformat() if row.finished_at else None,
        'start_utc': row.start_utc,
        'end_utc': row.end_utc,
        'actual_weight_kg': row.actual_weight_kg,
        'energy_kwh': row.energy_kwh,
        'processed_run_db_id': row.processed_run_db_id,
        'processed_id': row.processed_id,
        'mcap_path': row.mcap_path,
        'parquet_path': row.parquet_path,
        'mes_weighment_sent': row.mes_weighment_sent,
        'mes_batch_end_sent': row.mes_batch_end_sent,
        'contract': parse_json_text(row.request_payload_json),
        'result_payload': parse_json_text(row.result_payload_json),
    }


def find_latest_location_allocation(
    db, stock_item_id: str | None, site_id: str | None
) -> StockLocationAllocation | None:
    if stock_item_id is None:
        return None
    query = db.query(StockLocationAllocation).filter(
        StockLocationAllocation.stock_item_id == stock_item_id
    )
    if site_id is not None:
        query = query.filter(StockLocationAllocation.site_id == site_id)
    return query.order_by(StockLocationAllocation.id.desc()).first()


def latest_robot_run(db, weightment_id: int) -> RobotWeightmentRun | None:
    return (
        db.query(RobotWeightmentRun)
        .filter(RobotWeightmentRun.weightment_id == weightment_id)
        .order_by(RobotWeightmentRun.id.desc())
        .first()
    )


def serialize_active_robot_run(
    db, run_row: RobotWeightmentRun | None
) -> dict | None:
    if run_row is None:
        return None
    weightment_row = (
        db.query(WebhookWeightment)
        .filter(WebhookWeightment.id == run_row.weightment_id)
        .first()
    )
    return {
        'run_id': run_row.id,
        'event_id': run_row.event_id,
        'batch_id': run_row.batch_id,
        'weightment_id': run_row.weightment_id,
        'status': run_row.status,
        'ingredient_id': run_row.ingredient_id,
        'ingredient_name': run_row.ingredient_name,
        'target_weight_kg': run_row.target_weight_kg,
        'location_id': run_row.stock_location_id,
        'location_code': run_row.stock_location_code,
        'requested_at': (
            run_row.requested_at.isoformat() if run_row.requested_at else None
        ),
        'started_at': (
            run_row.started_at.isoformat() if run_row.started_at else None
        ),
        'finished_at': (
            run_row.finished_at.isoformat() if run_row.finished_at else None
        ),
        'start_time': (
            weightment_row.start_utc if weightment_row is not None else None
        ),
        'end_time': weightment_row.end_utc if weightment_row is not None else None,
        'actual_weight_kg': (
            weightment_row.actual_weight_kg if weightment_row is not None else None
        ),
        'energy_kwh': (
            weightment_row.energy_kwh if weightment_row is not None else None
        ),
    }


def get_latest_active_robot_run(db) -> RobotWeightmentRun | None:
    return (
        db.query(RobotWeightmentRun)
        .filter(
            RobotWeightmentRun.status.in_(
                ['starting', 'running', 'awaiting_processing']
            )
        )
        .order_by(RobotWeightmentRun.id.desc())
        .first()
    )


def build_webhook_event_summary(
    db,
    rows: list[WebhookWeightment],
    row_data: list[dict],
) -> dict:
    summary = {
        'event_id': rows[0].event_id,
        'sent_utc': rows[0].sent_utc,
        'batch_id': rows[0].batch_id,
        'completed': False,
        'batch_auto_run_enabled': any(row.batch_auto_run_enabled for row in rows),
        'batch_run_in_progress': False,
        'next_weightment_id': None,
        'active_weightment_id': None,
        'active_run_id': None,
        'last_completed_weightment_id': None,
    }
    if rows[0].batch_id is not None:
        batch_rows = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.batch_id == rows[0].batch_id)
            .all()
        )
        summary['completed'] = all(item.weightment_completed for item in batch_rows)
    else:
        summary['completed'] = all(item.weightment_completed for item in rows)
    summary['batch_run_in_progress'] = any(
        row.get('robot_status') in {'starting', 'running', 'awaiting_processing'}
        for row in row_data
    )
    next_row = next((item for item in rows if not item.weightment_completed), None)
    summary['next_weightment_id'] = next_row.id if next_row is not None else None
    active_row = next(
        (
            item
            for item in row_data
            if item.get('robot_status')
            in {'starting', 'running', 'awaiting_processing'}
        ),
        None,
    )
    if active_row is not None:
        summary['active_weightment_id'] = active_row.get('weightment_id')
        summary['active_run_id'] = active_row.get('robot_run_id')
    completed_ids = [row.id for row in rows if row.weightment_completed]
    if completed_ids:
        summary['last_completed_weightment_id'] = completed_ids[-1]
    return summary


def set_event_batch_auto_run(db, event_id: str, enabled: bool) -> None:
    rows = (
        db.query(WebhookWeightment)
        .filter(WebhookWeightment.event_id == event_id)
        .all()
    )
    for row in rows:
        row.batch_auto_run_enabled = enabled


def has_active_robot_run(db) -> RobotWeightmentRun | None:
    return (
        db.query(RobotWeightmentRun)
        .filter(RobotWeightmentRun.status.in_(['starting', 'running']))
        .order_by(RobotWeightmentRun.id.desc())
        .first()
    )


def has_inflight_event_run(db, event_id: str) -> RobotWeightmentRun | None:
    return (
        db.query(RobotWeightmentRun)
        .filter(
            RobotWeightmentRun.event_id == event_id,
            RobotWeightmentRun.status.in_(['starting', 'running', 'awaiting_processing']),
        )
        .order_by(RobotWeightmentRun.id.desc())
        .first()
    )


def next_incomplete_event_weightment(
    db, event_id: str
) -> WebhookWeightment | None:
    return (
        db.query(WebhookWeightment)
        .filter(
            WebhookWeightment.event_id == event_id,
            WebhookWeightment.weightment_completed.is_(False),
        )
        .order_by(WebhookWeightment.id.asc())
        .first()
    )


def event_weightments_missing_locations(
    db, event_id: str
) -> list[WebhookWeightment]:
    rows = (
        db.query(WebhookWeightment)
        .filter(
            WebhookWeightment.event_id == event_id,
            WebhookWeightment.weightment_completed.is_(False),
        )
        .order_by(WebhookWeightment.id.asc())
        .all()
    )
    missing_rows: list[WebhookWeightment] = []
    for row in rows:
        location_row = find_latest_location_allocation(db, row.ingredient_id, row.site_id)
        if location_row is None or location_row.stock_item_location_id is None:
            missing_rows.append(row)
    return missing_rows


def reconcile_stale_robot_runs(db) -> None:
    start_cutoff = utc_now_dt() - timedelta(seconds=ROBOT_START_TIMEOUT_SECONDS)
    running_cutoff = utc_now_dt() - timedelta(seconds=ROBOT_RUNNING_TIMEOUT_SECONDS)
    stale_start_runs = (
        db.query(RobotWeightmentRun)
        .filter(
            RobotWeightmentRun.status == 'starting',
            RobotWeightmentRun.started_at.is_(None),
            RobotWeightmentRun.requested_at.is_not(None),
            RobotWeightmentRun.requested_at < start_cutoff,
        )
        .all()
    )
    running_candidates = (
        db.query(RobotWeightmentRun)
        .filter(
            RobotWeightmentRun.status == 'running',
            RobotWeightmentRun.finished_at.is_(None),
            RobotWeightmentRun.processed_id.is_(None),
            RobotWeightmentRun.mes_weighment_sent.is_(False),
        )
        .all()
    )
    stale_running_runs = []
    for run in running_candidates:
        reference_time = run.started_at or run.requested_at
        if reference_time is not None and reference_time < running_cutoff:
            stale_running_runs.append(run)
    if not stale_start_runs and not stale_running_runs:
        return
    rosbridge_robot_client.reset(background=True)
    now_dt = utc_now_dt()
    for run in stale_start_runs:
        run.status = 'failed'
        run.error_message = (
            'Timed out waiting for robot start confirmation '
            f'after {ROBOT_START_TIMEOUT_SECONDS}s'
        )
        run.finished_at = run.finished_at or now_dt
        run.result_payload_json = json.dumps(
            {
                'timeout': 'start_confirmation',
                'timeout_seconds': ROBOT_START_TIMEOUT_SECONDS,
            }
        )
        if run.event_id:
            set_event_batch_auto_run(db, run.event_id, False)
    for run in stale_running_runs:
        run.status = 'failed'
        run.error_message = (
            'Timed out waiting for robot completion '
            f'after {ROBOT_RUNNING_TIMEOUT_SECONDS}s'
        )
        run.finished_at = run.finished_at or now_dt
        run.result_payload_json = json.dumps(
            {
                'timeout': 'robot_completion',
                'timeout_seconds': ROBOT_RUNNING_TIMEOUT_SECONDS,
            }
        )
        if run.event_id:
            set_event_batch_auto_run(db, run.event_id, False)
        logger.warning(
            'Recovered stale running robot run %s for weightment %s after %ss',
            run.id,
            run.weightment_id,
            ROBOT_RUNNING_TIMEOUT_SECONDS,
        )
    db.commit()


def post_json(url: str, payload: dict, timeout_seconds: float = 10) -> dict:
    body = json.dumps(payload).encode('utf-8')
    req = request.Request(
        url,
        data=body,
        headers={'Content-Type': 'application/json'},
        method='POST',
    )
    try:
        with request.urlopen(req, timeout=timeout_seconds) as response:
            raw = response.read().decode('utf-8')
            return json.loads(raw) if raw else {}
    except error.HTTPError as exc:
        detail = exc.read().decode('utf-8')
        raise HTTPException(
            status_code=502,
            detail=f'Downstream request failed ({url}): {exc.code} {detail}',
        ) from exc
    except error.URLError as exc:
        raise HTTPException(
            status_code=502, detail=f'Downstream request failed ({url}): {exc}'
        ) from exc


def start_robot_run_via_adapter(contract: dict[str, Any]) -> dict[str, Any]:
    adapter_payload = {
        'run_id': str(contract['trace_run_id']),
        'weightment_id': str(contract['weightment_id']),
        'batch_id': str(contract['batch_id']),
        'ingredient_id': str(contract['ingredient_id']),
        'location_id': str(contract['location_id']),
        'location_code': str(contract.get('location_code') or ''),
        'pickup_target_name': str(contract['pickup_target_name']),
        'weigh_target_name': str(contract['weigh_target_name']),
        'return_target_name': str(contract['return_target_name']),
        'target_weight_g': float(contract['target_weight_g']),
        'tolerance_g': float(contract['weight_tolerance_g']),
        'expected_lot': str(contract.get('expected_lot') or ''),
    }
    return post_json(
        ROBOT_START_ADAPTER_URL,
        adapter_payload,
        timeout_seconds=ROBOT_START_ADAPTER_TIMEOUT_SECONDS,
    )


def build_robot_run_contract(db, row: WebhookWeightment) -> dict:
    if row.batch_id is None or row.ingredient_id is None or row.target_weight_kg is None:
        raise HTTPException(
            status_code=400,
            detail='Weightment is missing batch_id, ingredient_id, or target_weight_kg',
        )
    location_row = find_latest_location_allocation(db, row.ingredient_id, row.site_id)
    if location_row is None or location_row.stock_item_location_id is None:
        raise HTTPException(
            status_code=400,
            detail=(
                'No stock location allocation found for '
                f'stock item {row.ingredient_id}'
            ),
        )
    try:
        batch_id_int = int(row.batch_id)
        ingredient_id_int = int(row.ingredient_id)
    except ValueError as exc:
        raise HTTPException(
            status_code=400,
            detail='batch_id or ingredient_id could not be converted to integers',
        ) from exc
    targets = resolve_robot_targets(
        location_row.stock_item_location_id, location_row.stock_item_location_code
    )
    target_weight_kg = float(row.target_weight_kg)
    target_weight_g = target_weight_kg * 1000.0
    return {
        'weightment_id': row.id,
        'event_id': row.event_id,
        'batch_id': row.batch_id,
        'batch_id_int': batch_id_int,
        'ingredient_id': row.ingredient_id,
        'ingredient_id_int': ingredient_id_int,
        'site_id': row.site_id,
        'location_id': location_row.stock_item_location_id,
        'location_code': location_row.stock_item_location_code,
        'ingredient_name': location_row.stock_item_name,
        'pickup_target_name': targets.pickup_target_name,
        'weigh_target_name': targets.weigh_target_name,
        'return_target_name': targets.return_target_name,
        'target_weight_kg': target_weight_kg,
        'target_weight_g': target_weight_g,
        'weight_tolerance_g': targets.weight_tolerance_g,
        'expected_lot': row.lot_code or '',
        'mode': 'webhook',
    }


def start_robot_run_for_weightment_row(db, row: WebhookWeightment) -> dict:
    if row.weightment_completed:
        return {
            'ok': True,
            'alreadyCompleted': True,
            'alreadyRunning': False,
            'weightmentId': row.id,
            'run': None,
        }
    active_run = has_active_robot_run(db)
    if active_run is not None:
        raise HTTPException(
            status_code=409,
            detail=(
                'Another robot run is already active '
                f'(run {active_run.id}, weightment {active_run.weightment_id})'
            ),
        )
    existing_run = (
        db.query(RobotWeightmentRun)
        .filter(
            RobotWeightmentRun.weightment_id == row.id,
            RobotWeightmentRun.status.in_(['starting', 'running', 'awaiting_processing']),
        )
        .order_by(RobotWeightmentRun.id.desc())
        .first()
    )
    if existing_run is not None:
        return {
            'ok': True,
            'alreadyCompleted': False,
            'alreadyRunning': True,
            'weightmentId': row.id,
            'run': serialize_robot_run(existing_run),
        }
    contract = build_robot_run_contract(db, row)
    run_row = RobotWeightmentRun(
        weightment_id=row.id,
        event_id=row.event_id,
        batch_id=row.batch_id,
        ingredient_id=row.ingredient_id,
        site_id=row.site_id,
        stock_location_id=contract['location_id'],
        stock_location_code=contract['location_code'],
        ingredient_name=contract['ingredient_name'],
        pickup_target_name=contract['pickup_target_name'],
        weigh_target_name=contract['weigh_target_name'],
        return_target_name=contract['return_target_name'],
        target_weight_kg=contract['target_weight_kg'],
        target_weight_g=contract['target_weight_g'],
        weight_tolerance_g=contract['weight_tolerance_g'],
        expected_lot=contract['expected_lot'],
        status='starting',
    )
    db.add(run_row)
    db.commit()
    db.refresh(run_row)
    run_row_id = run_row.id
    trace_run_id = f'webhook-run-{run_row.id}'
    contract['trace_run_id'] = trace_run_id
    run_row.trace_run_id = trace_run_id
    run_row.request_payload_json = json.dumps(contract)
    db.commit()
    try:
        service_response = start_robot_run_via_adapter(contract)
        if not bool(service_response.get('accepted')):
            raise RuntimeError(
                str(service_response.get('message') or 'Robot run was not accepted')
            )
        rosbridge_robot_client.register_active_run(run_row_id, contract, service_response)
        run_row = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.id == run_row_id)
            .first()
        )
        if run_row is None:
            raise RuntimeError('Robot run disappeared before start update')
        run_row.status = 'running'
        run_row.started_at = utc_now_dt()
        run_row.start_utc = utc_now()
        run_row.result_payload_json = json.dumps(
            {'start_service_response': service_response}
        )
        db.commit()
        db.refresh(run_row)
    except Exception as exc:
        db.rollback()
        rosbridge_robot_client.reset(background=True)
        run_row = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.id == run_row_id)
            .first()
        )
        if run_row is not None:
            run_row.status = 'failed'
            run_row.error_message = str(exc)
            run_row.finished_at = utc_now_dt()
            db.commit()
            db.refresh(run_row)
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    return {
        'ok': True,
        'alreadyCompleted': False,
        'alreadyRunning': False,
        'weightmentId': row.id,
        'run': serialize_robot_run(run_row),
    }


def apply_measurement_to_weightment(
    row: WebhookWeightment,
    *,
    actual_weight_kg: float,
    start_utc: str,
    end_utc: str,
    energy_kwh: int,
    lot_code: str,
) -> None:
    row.start_utc = start_utc
    row.end_utc = end_utc
    row.actual_weight_kg = actual_weight_kg
    row.energy_kwh = int(energy_kwh)
    row.lot_code = lot_code


def send_weightment_to_mes(db, row: WebhookWeightment) -> dict:
    if row.batch_id is None or row.ingredient_id is None or row.target_weight_kg is None:
        raise HTTPException(
            status_code=400,
            detail='Weightment is missing batch_id, ingredient_id, or target_weight_kg',
        )
    if row.actual_weight_kg is None or row.start_utc is None or row.end_utc is None:
        raise HTTPException(
            status_code=400,
            detail='Weightment is missing actual_weight_kg, start_utc, or end_utc',
        )
    location_row = find_latest_location_allocation(db, row.ingredient_id, row.site_id)
    if location_row is None or location_row.stock_item_location_id is None:
        raise HTTPException(
            status_code=400,
            detail=(
                'No stock location allocation found for '
                f'stock item {row.ingredient_id}'
            ),
        )
    try:
        batch_id_int = int(row.batch_id)
        ingredient_id_int = int(row.ingredient_id)
    except ValueError as exc:
        raise HTTPException(
            status_code=400,
            detail='batch_id or ingredient_id could not be converted to integers',
        ) from exc

    weighment_payload = {
        'batchId': batch_id_int,
        'stockItemId': ingredient_id_int,
        'locationId': int(location_row.stock_item_location_id),
        'lotCode': row.lot_code or '',
        'targetKg': float(row.target_weight_kg),
        'actualKg': float(row.actual_weight_kg),
        'startUtc': row.start_utc,
        'endUtc': row.end_utc,
        'energyKwh': int(row.energy_kwh or 0),
    }
    weighment_response = post_json(WEIGHMENT_URL, weighment_payload)

    row.weightment_completed = True
    db.commit()

    batch_end_sent = False
    batch_end_response = None
    if row.batch_id is not None:
        related_rows = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.batch_id == row.batch_id)
            .all()
        )
        if related_rows and all(item.weightment_completed for item in related_rows):
            batch_end_response = post_json(
                BATCH_END_URL,
                {
                    'batchId': batch_id_int,
                    'endUtc': row.end_utc,
                },
            )
            batch_end_sent = True

    return {
        'weightmentId': row.id,
        'locationId': int(location_row.stock_item_location_id),
        'weighmentResponse': weighment_response,
        'batchEndSent': batch_end_sent,
        'batchEndResponse': batch_end_response,
    }


def maybe_start_next_batch_weightment(event_id: str | None) -> dict | None:
    if not event_id:
        return None
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        event_rows = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.event_id == event_id)
            .order_by(WebhookWeightment.id.asc())
            .all()
        )
        if not event_rows or not any(row.batch_auto_run_enabled for row in event_rows):
            return None
        inflight_run = has_inflight_event_run(db, event_id)
        if inflight_run is not None:
            return {
                'started': False,
                'reason': 'run_inflight',
                'runId': inflight_run.id,
                'weightmentId': inflight_run.weightment_id,
            }
        next_row = next_incomplete_event_weightment(db, event_id)
        if next_row is None:
            set_event_batch_auto_run(db, event_id, False)
            db.commit()
            return {'started': False, 'reason': 'batch_complete'}
        result = start_robot_run_for_weightment_row(db, next_row)
        return {
            'started': True,
            'weightmentId': next_row.id,
            'run': result.get('run'),
        }
    except Exception:
        db.rollback()
        set_event_batch_auto_run(db, event_id, False)
        db.commit()
        logger.exception('Failed to auto-start next weightment for event %s', event_id)
        return {'started': False, 'reason': 'start_failed'}
    finally:
        db.close()


def send_processed_weightment_to_mes(
    db,
    run_row: RobotWeightmentRun,
    weightment_row: WebhookWeightment,
) -> dict:
    if weightment_row.weightment_completed or run_row.mes_weighment_sent:
        return {
            'ok': True,
            'status': run_row.status,
            'alreadyCompleted': True,
            'run': serialize_robot_run(run_row),
        }
    try:
        send_result = send_weightment_to_mes(db, weightment_row)
    except HTTPException as exc:
        if weightment_row.event_id:
            set_event_batch_auto_run(db, weightment_row.event_id, False)
        run_row.status = 'mes_send_failed'
        run_row.error_message = str(exc.detail)
        run_row.mes_weighment_sent = False
        run_row.mes_batch_end_sent = False
        db.commit()
        return {
            'ok': False,
            'status': run_row.status,
            'detail': exc.detail,
            'run': serialize_robot_run(run_row),
        }

    run_row.status = 'succeeded'
    run_row.error_message = None
    run_row.mes_weighment_sent = True
    run_row.mes_batch_end_sent = bool(send_result['batchEndSent'])
    db.commit()
    db.refresh(run_row)
    next_batch_run = maybe_start_next_batch_weightment(weightment_row.event_id)
    return {
        'ok': True,
        'status': run_row.status,
        'run': serialize_robot_run(run_row),
        'sendResult': send_result,
        'nextBatchRun': next_batch_run,
    }


def finalize_robot_weightment_run(
    db,
    run_row: RobotWeightmentRun,
    weightment_row: WebhookWeightment,
    req: RobotRunCompletionRequest,
) -> dict:
    actual_weight_kg = req.actual_weight_kg
    if actual_weight_kg is None and req.final_scale_weight_g is not None:
        actual_weight_kg = req.final_scale_weight_g / 1000.0
    if actual_weight_kg is None:
        actual_weight_kg = (
            weightment_row.actual_weight_kg
            if weightment_row.actual_weight_kg is not None
            else weightment_row.target_weight_kg
        )
    if actual_weight_kg is None:
        actual_weight_kg = 0.0

    start_utc = req.start_utc or run_row.start_utc or weightment_row.start_utc or utc_now()
    end_utc = req.end_utc or utc_now()
    energy_kwh = int(req.energy_kwh or 0)
    lot_code = weightment_row.lot_code or ''

    run_row.result_payload_json = json.dumps(req.result_payload or {})
    run_row.start_utc = start_utc
    run_row.end_utc = end_utc
    run_row.actual_weight_kg = actual_weight_kg
    run_row.energy_kwh = energy_kwh
    run_row.started_at = run_row.started_at or utc_now_dt()
    run_row.finished_at = utc_now_dt()

    if not req.success:
        if weightment_row.event_id:
            set_event_batch_auto_run(db, weightment_row.event_id, False)
        run_row.status = 'failed'
        run_row.error_message = req.error_message or 'Robot run failed'
        db.commit()
        return {
            'ok': False,
            'status': run_row.status,
            'run': serialize_robot_run(run_row),
        }

    apply_measurement_to_weightment(
        weightment_row,
        actual_weight_kg=actual_weight_kg,
        start_utc=start_utc,
        end_utc=end_utc,
        energy_kwh=energy_kwh,
        lot_code=lot_code,
    )
    run_row.status = 'awaiting_processing'
    run_row.error_message = None
    run_row.mes_weighment_sent = False
    run_row.mes_batch_end_sent = False
    db.commit()
    db.refresh(run_row)
    if run_row.processed_id is not None and not weightment_row.weightment_completed:
        return send_processed_weightment_to_mes(db, run_row, weightment_row)
    return {
        'ok': True,
        'status': run_row.status,
        'run': serialize_robot_run(run_row),
        'awaitingProcessing': True,
    }


def handle_rosbridge_completion(payload: dict[str, Any]) -> None:
    db = SessionLocal()
    try:
        run_id = int(payload['run_id'])
        run_row = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.id == run_id)
            .first()
        )
        if run_row is None:
            logger.error('Rosbridge completion for unknown run id %s', run_id)
            return
        weightment_row = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.id == run_row.weightment_id)
            .first()
        )
        if weightment_row is None:
            logger.error(
                'Rosbridge completion for run %s has missing weightment %s',
                run_id,
                run_row.weightment_id,
            )
            return
        finalize_robot_weightment_run(
            db,
            run_row,
            weightment_row,
            RobotRunCompletionRequest(
                success=bool(payload['success']),
                actual_weight_kg=payload.get('actual_weight_kg'),
                final_scale_weight_g=payload.get('final_scale_weight_g'),
                start_utc=payload.get('start_utc'),
                end_utc=payload.get('end_utc'),
                energy_kwh=payload.get('energy_kwh'),
                error_message=payload.get('error_message'),
                result_payload=payload.get('result_payload'),
            ),
        )
    except Exception:
        db.rollback()
        logger.exception('Failed to process rosbridge completion')
    finally:
        db.close()


rosbridge_robot_client.set_completion_handler(handle_rosbridge_completion)


@app.post('/processed', response_model=ProcessedResponse)
def processed(req: ProcessedRequest) -> ProcessedResponse:
    db = SessionLocal()
    try:
        processed_row = store_processed_run(db, req.model_dump())
        processed_id = processed_row.id
        run_db_id = processed_row.run_db_id
        run_row = None
        if req.run_id:
            run_row = (
                db.query(RobotWeightmentRun)
                .filter(RobotWeightmentRun.trace_run_id == req.run_id)
                .first()
            )
        if run_row is None and req.robot_weightment_run_id is not None:
            run_row = (
                db.query(RobotWeightmentRun)
                .filter(RobotWeightmentRun.id == req.robot_weightment_run_id)
                .first()
            )
        if (
            run_row is not None
            and run_row.status in {'starting', 'running'}
            and not run_row.mes_weighment_sent
        ):
            # Recover from missed runtime completion callbacks by treating the
            # processed MCAP result as authoritative completion evidence.
            run_row.status = 'awaiting_processing'
            run_row.error_message = None
            db.commit()
        if (
            run_row is not None
            and run_row.status == 'awaiting_processing'
            and not run_row.mes_weighment_sent
        ):
            weightment_row = (
                db.query(WebhookWeightment)
                .filter(WebhookWeightment.id == run_row.weightment_id)
                .first()
            )
            if weightment_row is not None:
                send_processed_weightment_to_mes(db, run_row, weightment_row)
    except Exception as exc:
        db.rollback()
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    finally:
        db.close()
    return ProcessedResponse(processed_id=processed_id, run_db_id=run_db_id)


@app.get('/lightsout_processed')
def list_lightsout_processed(
    limit: int = 50,
    offset: int = 0,
    mode: str | None = None,
    batch_id: str | None = None,
    episode_index: int | None = None,
) -> dict:
    db = SessionLocal()
    try:
        safe_limit = max(limit, 0)
        safe_offset = max(offset, 0)

        base_query = db.query(LightsOutProcessed, Run).join(
            Run, LightsOutProcessed.run_db_id == Run.id
        )
        if mode:
            base_query = base_query.filter(LightsOutProcessed.mode == mode)
        if batch_id:
            base_query = base_query.filter(LightsOutProcessed.batch_id == batch_id)
        if episode_index is not None:
            base_query = base_query.filter(
                LightsOutProcessed.episode_index == episode_index
            )

        total = base_query.count()

        rows_query = base_query.order_by(LightsOutProcessed.id.desc()).offset(safe_offset)
        if safe_limit > 0:
            rows_query = rows_query.limit(safe_limit)
        rows = rows_query.all()

        available_batches = [
            value
            for (value,) in db.query(LightsOutProcessed.batch_id)
            .filter(LightsOutProcessed.batch_id.isnot(None))
            .distinct()
            .order_by(LightsOutProcessed.batch_id.asc())
            .all()
            if value
        ]
        available_modes = [
            value
            for (value,) in db.query(LightsOutProcessed.mode)
            .filter(LightsOutProcessed.mode.isnot(None))
            .distinct()
            .order_by(LightsOutProcessed.mode.asc())
            .all()
            if value
        ]
        data = []
        for processed_row, run_row in rows:
            robot_run_row = None
            if processed_row.id is not None:
                robot_run_row = (
                    db.query(RobotWeightmentRun)
                    .filter(RobotWeightmentRun.processed_id == processed_row.id)
                    .order_by(RobotWeightmentRun.id.desc())
                    .first()
                )
            if robot_run_row is None and run_row.run_id:
                robot_run_row = (
                    db.query(RobotWeightmentRun)
                    .filter(RobotWeightmentRun.trace_run_id == run_row.run_id)
                    .order_by(RobotWeightmentRun.id.desc())
                    .first()
                )
            data.append(
                {
                    'id': processed_row.id,
                    'run_db_id': processed_row.run_db_id,
                    'event_id': robot_run_row.event_id if robot_run_row is not None else None,
                    'weightment_id': robot_run_row.weightment_id
                    if robot_run_row is not None
                    else None,
                    'robot_id': processed_row.robot_id,
                    'run_id': processed_row.run_id,
                    'batch_id': processed_row.batch_id,
                    'ingredient_id': processed_row.ingredient_id,
                    'episode_index': processed_row.episode_index,
                    'mode': processed_row.mode,
                    'start_time_ns': run_row.start_time_ns,
                    'end_time_ns': run_row.end_time_ns,
                    'target_weight_g': processed_row.target_weight_g,
                    'final_weight_g': processed_row.final_weight_g,
                    'net_weight_g': processed_row.net_weight_g,
                    'avg_flow_rate_g_s': processed_row.avg_flow_rate_g_s,
                    'total_episode_time_s': processed_row.total_episode_time_s,
                    'overshoot_g': processed_row.overshoot_g,
                    'scoop_duration_s': processed_row.scoop_duration_s,
                    'pour_duration_s': processed_row.pour_duration_s,
                    'parquet_path': processed_row.parquet_path,
                }
            )
    finally:
        db.close()
    return {
        'rows': data,
        'total': total,
        'limit': safe_limit,
        'offset': safe_offset,
        'available_batches': available_batches,
        'available_modes': available_modes,
    }


@app.get('/webhook_weightments')
def list_webhook_weightments(limit: int = 200) -> dict:
    db = SessionLocal()
    try:
        rows = (
            db.query(WebhookWeightment)
            .order_by(WebhookWeightment.id.desc())
            .limit(limit)
            .all()
        )
        data = [serialize_webhook_weightment(row) for row in rows]
    finally:
        db.close()
    return {'rows': data}


@app.get('/stock_location_allocations')
def list_stock_location_allocations(limit: int = 200) -> dict:
    db = SessionLocal()
    try:
        rows = (
            db.query(StockLocationAllocation)
            .order_by(StockLocationAllocation.id.desc())
            .limit(limit)
            .all()
        )
        data = [serialize_stock_location_allocation(row) for row in rows]
    finally:
        db.close()
    return {'rows': data}


@app.get('/robot_weightment_runs')
def list_robot_weightment_runs(limit: int = 200) -> dict:
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        rows = (
            db.query(RobotWeightmentRun)
            .order_by(RobotWeightmentRun.id.desc())
            .limit(limit)
            .all()
        )
        data = [serialize_robot_run(row) for row in rows]
    finally:
        db.close()
    return {'rows': data}


@app.get('/robot_weightment_runs/active')
def get_active_robot_weightment_run() -> dict:
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        run_row = get_latest_active_robot_run(db)
        return {'active': serialize_active_robot_run(db, run_row)}
    finally:
        db.close()


@app.get('/robot_weightment_runs/active/stream')
async def stream_active_robot_weightment_run(request: Request) -> StreamingResponse:
    async def event_stream():
        last_payload = None
        # Ask the browser to retry quickly after disconnects.
        yield 'retry: 2000\n\n'
        while True:
            if await request.is_disconnected():
                break
            db = SessionLocal()
            try:
                reconcile_stale_robot_runs(db)
                payload = {
                    'active': serialize_active_robot_run(
                        db, get_latest_active_robot_run(db)
                    )
                }
                payload_json = json.dumps(payload, sort_keys=True)
                if payload_json != last_payload:
                    yield f'data: {payload_json}\n\n'
                    last_payload = payload_json
                else:
                    yield ': keep-alive\n\n'
            except Exception:
                logger.exception('Failed while streaming active robot run')
                yield 'event: error\ndata: {"active": null}\n\n'
            finally:
                db.close()
            await asyncio.sleep(1)

    return StreamingResponse(
        event_stream(),
        media_type='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'Connection': 'keep-alive',
            'X-Accel-Buffering': 'no',
        },
    )


@app.get('/webhook_weightments/summary')
def list_webhook_weightment_summary(limit: int = 200) -> dict:
    db = SessionLocal()
    try:
        rows = (
            db.query(WebhookWeightment)
            .order_by(WebhookWeightment.id.desc())
            .limit(limit)
            .all()
        )
        batch_completion: dict[str, bool] = {}
        for row in rows:
            if row.batch_id is None:
                continue
            current = batch_completion.get(row.batch_id, True)
            batch_completion[row.batch_id] = bool(current and row.weightment_completed)
        grouped: dict[str, dict] = {}
        for row in rows:
            event_id = row.event_id or f'row-{row.id}'
            summary = grouped.get(event_id)
            if summary is None:
                summary = {
                    'event_id': row.event_id,
                    'sent_utc': row.sent_utc,
                    'batch_id': row.batch_id,
                    'completed': True,
                    'weightment_count': 0,
                }
                grouped[event_id] = summary
            summary['weightment_count'] += 1
            if row.batch_id is not None:
                summary['completed'] = batch_completion.get(row.batch_id, False)
            else:
                summary['completed'] = bool(
                    summary['completed'] and row.weightment_completed
                )
        data = list(grouped.values())
    finally:
        db.close()
    return {'rows': data}


@app.get('/webhook_weightments/{event_id}')
def get_webhook_weightment_details(event_id: str) -> dict:
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        rows = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.event_id == event_id)
            .order_by(WebhookWeightment.id.asc())
            .all()
        )
        if not rows:
            raise HTTPException(status_code=404, detail='Webhook event not found')
        data = []
        for row in rows:
            location_row = find_latest_location_allocation(
                db, row.ingredient_id, row.site_id
            )
            run_row = latest_robot_run(db, row.id)
            processed_row = None
            processed_run_row = None
            live_completion_weight_kg = None
            if run_row is not None:
                if run_row.processed_id is not None:
                    processed_row = (
                        db.query(LightsOutProcessed)
                        .filter(LightsOutProcessed.id == run_row.processed_id)
                        .first()
                    )
                if run_row.processed_run_db_id is not None:
                    processed_run_row = (
                        db.query(Run)
                        .filter(Run.id == run_row.processed_run_db_id)
                        .first()
                    )
                result_payload = parse_json_text(run_row.result_payload_json)
                latest_weight_g = None
                if isinstance(result_payload, dict):
                    latest_weight_g = result_payload.get('latest_weight_g')
                try:
                    if latest_weight_g is not None:
                        live_completion_weight_kg = float(latest_weight_g) / 1000.0
                except (TypeError, ValueError):
                    live_completion_weight_kg = None
            data.append(
                {
                    'weightment_id': row.id,
                    'target_weight_kg': row.target_weight_kg,
                    'actual_weight_kg': row.actual_weight_kg,
                    'completed': row.weightment_completed,
                    'stock_item_id': row.ingredient_id,
                    'ingredient_name': location_row.stock_item_name
                    if location_row is not None
                    else None,
                    'location_id': location_row.stock_item_location_id
                    if location_row is not None
                    else None,
                    'location_code': location_row.stock_item_location_code
                    if location_row is not None
                    else None,
                    'start_time': row.start_utc,
                    'end_time': row.end_utc,
                    'energy_kwh': row.energy_kwh,
                    'robot_run_id': run_row.id if run_row is not None else None,
                    'robot_status': run_row.status if run_row is not None else None,
                    'robot_error': run_row.error_message
                    if run_row is not None
                    else None,
                    'robot_requested_at': run_row.requested_at.isoformat()
                    if run_row is not None and run_row.requested_at
                    else None,
                    'robot_started_at': run_row.started_at.isoformat()
                    if run_row is not None and run_row.started_at
                    else None,
                    'robot_finished_at': run_row.finished_at.isoformat()
                    if run_row is not None and run_row.finished_at
                    else None,
                    'robot_trace_run_id': run_row.trace_run_id
                    if run_row is not None
                    else None,
                    'robot_processed_run_db_id': run_row.processed_run_db_id
                    if run_row is not None
                    else None,
                    'robot_processed_id': run_row.processed_id
                    if run_row is not None
                    else None,
                    'robot_live_completion_weight_kg': live_completion_weight_kg,
                    'robot_processed_final_weight_kg': (
                        processed_row.final_weight_g / 1000.0
                        if processed_row is not None
                        and processed_row.final_weight_g is not None
                        else None
                    ),
                    'robot_processed_start_time': (
                        ns_to_iso_utc(processed_run_row.start_time_ns)
                        if processed_run_row is not None
                        else None
                    ),
                    'robot_processed_end_time': (
                        ns_to_iso_utc(processed_run_row.end_time_ns)
                        if processed_run_row is not None
                        else None
                    ),
                    'robot_processed_overshoot_g': (
                        processed_row.overshoot_g
                        if processed_row is not None
                        else None
                    ),
                    'robot_processed_scoop_duration_s': (
                        processed_row.scoop_duration_s
                        if processed_row is not None
                        else None
                    ),
                    'robot_processed_pour_duration_s': (
                        processed_row.pour_duration_s
                        if processed_row is not None
                        else None
                    ),
                    'robot_processed_settle_time_s': (
                        processed_row.settle_time_s
                        if processed_row is not None
                        else None
                    ),
                    'robot_phase_events': (
                        parse_json_text(processed_row.phase_events_json)
                        if processed_row is not None
                        else None
                    ),
                    'robot_mcap_path': run_row.mcap_path
                    if run_row is not None
                    else None,
                    'robot_parquet_path': run_row.parquet_path
                    if run_row is not None
                    else None,
                    'robot_mes_weighment_sent': run_row.mes_weighment_sent
                    if run_row is not None
                    else False,
                    'robot_mes_batch_end_sent': run_row.mes_batch_end_sent
                    if run_row is not None
                    else False,
                }
            )
        summary = build_webhook_event_summary(db, rows, data)
    finally:
        db.close()
    return {'summary': summary, 'rows': data}


@app.get('/webhook_events/{event_id}/live_status')
def get_webhook_event_live_status(event_id: str) -> dict:
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        rows = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.event_id == event_id)
            .order_by(WebhookWeightment.id.asc())
            .all()
        )
        if not rows:
            raise HTTPException(status_code=404, detail='Webhook event not found')
        data = []
        for row in rows:
            run_row = latest_robot_run(db, row.id)
            data.append(
                {
                    'weightment_id': row.id,
                    'completed': row.weightment_completed,
                    'actual_weight_kg': row.actual_weight_kg,
                    'start_time': row.start_utc,
                    'end_time': row.end_utc,
                    'energy_kwh': row.energy_kwh,
                    'robot_run_id': run_row.id if run_row is not None else None,
                    'robot_status': run_row.status if run_row is not None else None,
                    'robot_error': run_row.error_message
                    if run_row is not None
                    else None,
                    'robot_requested_at': run_row.requested_at.isoformat()
                    if run_row is not None and run_row.requested_at
                    else None,
                    'robot_started_at': run_row.started_at.isoformat()
                    if run_row is not None and run_row.started_at
                    else None,
                    'robot_finished_at': run_row.finished_at.isoformat()
                    if run_row is not None and run_row.finished_at
                    else None,
                    'robot_trace_run_id': run_row.trace_run_id
                    if run_row is not None
                    else None,
                    'robot_processed_id': run_row.processed_id
                    if run_row is not None
                    else None,
                    'robot_mes_weighment_sent': run_row.mes_weighment_sent
                    if run_row is not None
                    else False,
                    'robot_mes_batch_end_sent': run_row.mes_batch_end_sent
                    if run_row is not None
                    else False,
                }
            )
        summary = build_webhook_event_summary(db, rows, data)
    finally:
        db.close()
    return {'summary': summary, 'rows': data}


@app.post('/webhook_weightments/{weightment_id}/run_robot')
def run_robot_for_webhook_weightment(weightment_id: int) -> dict:
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        row = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.id == weightment_id)
            .first()
        )
        if row is None:
            raise HTTPException(status_code=404, detail='Weightment not found')
        return start_robot_run_for_weightment_row(db, row)
    except HTTPException:
        db.rollback()
        raise
    except Exception as exc:
        db.rollback()
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    finally:
        db.close()


@app.post('/webhook_events/{event_id}/run_all')
def run_all_webhook_weightments(event_id: str) -> dict:
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        rows = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.event_id == event_id)
            .order_by(WebhookWeightment.id.asc())
            .all()
        )
        if not rows:
            raise HTTPException(status_code=404, detail='Webhook event not found')
        missing_location_rows = event_weightments_missing_locations(db, event_id)
        if missing_location_rows:
            missing_details = ', '.join(
                f'weightment {row.id} (ingredient {row.ingredient_id or "unknown"})'
                for row in missing_location_rows
            )
            raise HTTPException(
                status_code=400,
                detail=(
                    'Run full batch requires a location for every incomplete weightment. '
                    f'Missing: {missing_details}'
                ),
            )
        active_run = has_active_robot_run(db)
        if active_run is not None and active_run.event_id != event_id:
            raise HTTPException(
                status_code=409,
                detail=(
                    'Another robot run is already active '
                    f'(run {active_run.id}, weightment {active_run.weightment_id})'
                ),
            )
        set_event_batch_auto_run(db, event_id, True)
        db.commit()
        inflight_run = has_inflight_event_run(db, event_id)
        if inflight_run is not None:
            return {
                'ok': True,
                'batchAutoRunEnabled': True,
                'alreadyRunning': True,
                'completed': False,
                'nextWeightmentId': inflight_run.weightment_id,
                'run': serialize_robot_run(inflight_run),
            }
        next_row = next_incomplete_event_weightment(db, event_id)
        if next_row is None:
            set_event_batch_auto_run(db, event_id, False)
            db.commit()
            return {
                'ok': True,
                'batchAutoRunEnabled': False,
                'alreadyRunning': False,
                'completed': True,
                'nextWeightmentId': None,
                'run': None,
            }
        try:
            start_result = start_robot_run_for_weightment_row(db, next_row)
        except HTTPException:
            db.rollback()
            set_event_batch_auto_run(db, event_id, False)
            db.commit()
            raise
        return {
            'ok': True,
            'batchAutoRunEnabled': True,
            'alreadyRunning': bool(start_result.get('alreadyRunning')),
            'completed': False,
            'nextWeightmentId': next_row.id,
            'run': start_result.get('run'),
        }
    except HTTPException:
        db.rollback()
        raise
    except Exception as exc:
        db.rollback()
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    finally:
        db.close()


@app.post('/robot_weightment_runs/{run_id}/complete')
def complete_robot_weightment_run(
    run_id: int, req: RobotRunCompletionRequest
) -> dict:
    db = SessionLocal()
    try:
        run_row = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.id == run_id)
            .first()
        )
        if run_row is None:
            raise HTTPException(status_code=404, detail='Robot run not found')
        weightment_row = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.id == run_row.weightment_id)
            .first()
        )
        if weightment_row is None:
            raise HTTPException(status_code=404, detail='Weightment not found')
        return finalize_robot_weightment_run(db, run_row, weightment_row, req)
    except HTTPException:
        db.rollback()
        raise
    except Exception as exc:
        db.rollback()
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    finally:
        db.close()


@app.post('/webhook_weightments/{weightment_id}/send')
def send_webhook_weightment(weightment_id: int) -> dict:
    db = SessionLocal()
    try:
        row = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.id == weightment_id)
            .first()
        )
        if row is None:
            raise HTTPException(status_code=404, detail='Weightment not found')
        run_row = latest_robot_run(db, row.id)
        if row.weightment_completed:
            if (
                run_row is not None
                and run_row.processed_id is not None
                and run_row.status in {'running', 'starting', 'awaiting_processing', 'mes_send_failed'}
            ):
                run_row.status = 'succeeded'
                run_row.error_message = None
                run_row.start_utc = row.start_utc
                run_row.end_utc = row.end_utc
                run_row.actual_weight_kg = row.actual_weight_kg
                run_row.energy_kwh = row.energy_kwh
                run_row.mes_weighment_sent = True
                run_row.finished_at = run_row.finished_at or utc_now_dt()
                db.commit()
                next_batch_run = maybe_start_next_batch_weightment(row.event_id)
                return {
                    'ok': True,
                    'alreadyCompleted': True,
                    'weightmentId': row.id,
                    'batchEndSent': bool(run_row.mes_batch_end_sent),
                    'nextBatchRun': next_batch_run,
                }
            return {
                'ok': True,
                'alreadyCompleted': True,
                'weightmentId': row.id,
                'batchEndSent': False,
            }

        start_utc = row.start_utc or utc_now()
        start_dt = datetime.fromisoformat(start_utc.replace('Z', '+00:00'))
        end_utc = row.end_utc or (start_dt + timedelta(seconds=30)).isoformat()
        actual_weight_kg = (
            row.actual_weight_kg
            if row.actual_weight_kg is not None
            else row.target_weight_kg
        )
        if actual_weight_kg is None:
            actual_weight_kg = 0.0
        energy_kwh = row.energy_kwh if row.energy_kwh is not None else 0
        lot_code = row.lot_code or ''

        apply_measurement_to_weightment(
            row,
            actual_weight_kg=float(actual_weight_kg),
            start_utc=start_utc,
            end_utc=end_utc,
            energy_kwh=int(energy_kwh),
            lot_code=lot_code,
        )
        send_result = send_weightment_to_mes(db, row)

        if (
            run_row is not None
            and run_row.processed_id is not None
            and run_row.status in {'running', 'starting', 'mes_send_failed', 'awaiting_processing'}
        ):
            run_row.status = 'succeeded'
            run_row.error_message = None
            run_row.start_utc = row.start_utc
            run_row.end_utc = row.end_utc
            run_row.actual_weight_kg = row.actual_weight_kg
            run_row.energy_kwh = row.energy_kwh
            run_row.mes_weighment_sent = True
            run_row.mes_batch_end_sent = bool(send_result['batchEndSent'])
            run_row.finished_at = run_row.finished_at or utc_now_dt()
            db.commit()
        next_batch_run = maybe_start_next_batch_weightment(row.event_id)

        return {
            'ok': True,
            'alreadyCompleted': False,
            'weightmentId': row.id,
            **send_result,
            'nextBatchRun': next_batch_run,
        }
    except HTTPException:
        db.rollback()
        raise
    except Exception as exc:
        db.rollback()
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    finally:
        db.close()
