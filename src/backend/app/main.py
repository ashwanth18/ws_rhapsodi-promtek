import asyncio
import json
import logging
import os
import re
import socket
import threading
import time
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any
from urllib import error, parse, request

from fastapi import FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from sqlalchemy import func, text

from .database import SessionLocal, engine
from .mes_client import (
    TIMESERIES_TIMEOUT_SECONDS,
    WEIGHMENT_URL,
    get_mes_client,
    post_json,
)
from .modes import (
    ModeSwitchConflict,
    ModeValidationError,
    get_mode_manager,
)
from .modes.lightsout_session import (
    LIGHTSOUT_ROSBRIDGE_RUN_ID,
    get_lightsout_session,
)
from .modes.inbound import (
    get_inbound_adapter_name,
    list_inbound_adapters,
    normalize_inbound_event,
)
from .modes.mes_generic import (
    ensure_generic_event_id,
    is_mes_generic_event_id,
    mes_generic_sink_name,
    strip_generic_event_id_prefix,
)
from .modes.mock_local import (
    MOCK_DEFAULT_PICKUP_TARGET,
    MOCK_DEFAULT_RETURN_TARGET,
    MOCK_DEFAULT_WEIGH_TARGET,
    MOCK_EVENT_ID_PREFIX,
    MOCK_INGREDIENT_ID,
    MOCK_SITE_ID,
    allocate_mock_batch_id,
    is_mock_event_id,
)
from .modes.batch_ids import prefix_for_mode, suggest_next_batch_id
from .modes.cell_layout import (
    configured_layout_id,
    layout_path,
    layout_provenance,
    list_layouts,
    load_layout,
    provenance_is_safe,
)
from .modes.lightsout_validate import resolve_lightsout_targets
from .modes.powders import get_powder, load_powders
from .modes.target_sampler import generate_target_schedule
from .export import router as export_router
from .exports_dir import ensure_exports_dir, sweep_stale_exports
from .run_spec import OperatingMode, RunLabel
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
from .timeseries import (
    build_timeseries_items_from_runs,
    build_timeseries_payload,
)
from .schemas import (
    LightsoutRunRequest,
    MockLocalRunRequest,
    ProcessedRequest,
    ProcessedResponse,
    RobotRunCompletionRequest,
    RuntimeModeRequest,
)

Base.metadata.create_all(bind=engine)

logger = logging.getLogger('uvicorn.error')
ROBOT_START_ADAPTER_URL = os.environ.get(
    'ROBOT_START_ADAPTER_URL',
    'http://host.docker.internal:8010/start_webhook_weightment',
)
ROBOT_LIGHTSOUT_ADAPTER_URL = os.environ.get(
    'ROBOT_LIGHTSOUT_ADAPTER_URL',
    'http://host.docker.internal:8010/start_lightsout',
)
ROBOT_LAYOUT_ADAPTER_URL = os.environ.get(
    'ROBOT_LAYOUT_ADAPTER_URL',
    'http://host.docker.internal:8010/apply_cell_layout',
)
ACTIVE_CELL_LAYOUT_PATH = Path(
    os.environ.get('ACTIVE_CELL_LAYOUT_PATH', '/data/active_cell_layout.json')
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


def get_active_cell_layout() -> dict[str, Any] | None:
    try:
        value = json.loads(ACTIVE_CELL_LAYOUT_PATH.read_text())
    except (OSError, ValueError, json.JSONDecodeError):
        return None
    return value if isinstance(value, dict) else None


def save_active_cell_layout(state: dict[str, Any]) -> None:
    ACTIVE_CELL_LAYOUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    temp = ACTIVE_CELL_LAYOUT_PATH.with_suffix('.tmp')
    temp.write_text(json.dumps(state, sort_keys=True, indent=2))
    temp.replace(ACTIVE_CELL_LAYOUT_PATH)


def apply_mode_layout(mode: str) -> dict[str, Any]:
    """Apply the layout selected for mode and persist only ROS-confirmed state."""
    layout_id = configured_layout_id(mode)
    layout = load_layout(layout_path(layout_id))
    provenance = layout_provenance(layout)
    response = post_json(
        ROBOT_LAYOUT_ADAPTER_URL,
        {'layout_id': layout_id},
        timeout_seconds=ROBOT_START_ADAPTER_TIMEOUT_SECONDS,
    )
    applied = bool(response.get('success')) and bool(response.get('preflight_ok'))
    result = {
        'layout_id': layout_id,
        'layout_hash': response.get('layout_hash') or provenance['layout_hash'],
        'applied': applied,
        'message': str(response.get('message') or ''),
        'preflight_ok': bool(response.get('preflight_ok')),
    }
    if applied:
        save_active_cell_layout({
            **provenance,
            'layout_hash': result['layout_hash'],
            'applied_at': utc_now(),
        })
    return result


def require_layout_for_run(current: dict[str, str]) -> dict[str, Any]:
    state = get_active_cell_layout()
    if not state:
        raise HTTPException(
            status_code=409,
            detail='No cell layout has been successfully applied for this runtime mode',
        )
    try:
        expected_layout_id = configured_layout_id(current['mode'])
    except Exception as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    if state.get('layout_id') != expected_layout_id:
        raise HTTPException(
            status_code=409,
            detail=(
                f"Applied layout '{state.get('layout_id')}' does not match "
                f"mode layout '{expected_layout_id}'"
            ),
        )
    safe, reason = provenance_is_safe(
        state,
        environment=current['environment'],
        production_mode=current['mode'] in {
            OperatingMode.MES_CONDOR.value,
            OperatingMode.MES_GENERIC.value,
        },
    )
    if not safe:
        raise HTTPException(status_code=409, detail=reason)
    return state


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


def parse_stored_datetime(value: str | None) -> datetime | None:
    if not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except ValueError:
        return None
    if parsed.tzinfo is None:
        parsed = parsed.replace(tzinfo=timezone.utc)
    return parsed.astimezone(timezone.utc)


def datetime_to_ns(value: datetime | None) -> int | None:
    if value is None:
        return None
    return int(value.timestamp() * 1_000_000_000)


def signed_final_error_g(
    target_weight_g: float | None,
    final_weight_g: float | None,
    fallback: float | None = None,
) -> float | None:
    if target_weight_g is None or final_weight_g is None:
        return fallback
    return final_weight_g - target_weight_g


def ensure_webhook_weightment_columns() -> None:
    statements = [
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS actual_weight_kg DOUBLE PRECISION',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS batch_auto_run_enabled BOOLEAN DEFAULT FALSE',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS start_utc VARCHAR',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS end_utc VARCHAR',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS energy_kwh INTEGER',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS lot_code VARCHAR',
        'ALTER TABLE webhook_weightments ADD COLUMN IF NOT EXISTS mes_timeseries_sent BOOLEAN DEFAULT FALSE',
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
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS run_key VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS environment VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS powder_id VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS powder_name VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS lot_code VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS operator VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS notes TEXT',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS episodes_total INTEGER',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS scooped_mass_g DOUBLE PRECISION',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS target_mode VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS target_fraction DOUBLE PRECISION',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS pour_outcome VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS rng_seed INTEGER',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS stop_on VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS stop_value DOUBLE PRECISION',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS stop_reason VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS layout_id VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS layout_hash VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS poses_hash VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS tool_id VARCHAR',
        'ALTER TABLE runs ADD COLUMN IF NOT EXISTS authored_in VARCHAR',
    ]
    with engine.begin() as conn:
        for statement in statements:
            conn.execute(text(statement))


ensure_run_columns()


def ensure_lightsout_processed_label_columns() -> None:
    statements = [
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS powder_id VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS powder_name VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS lot_code VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS operator VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS notes TEXT',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS episodes_total INTEGER',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS scooped_mass_g DOUBLE PRECISION',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS target_mode VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS target_fraction DOUBLE PRECISION',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS pour_outcome VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS rng_seed INTEGER',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS stop_on VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS stop_value DOUBLE PRECISION',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS stop_reason VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS layout_id VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS layout_hash VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS poses_hash VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS tool_id VARCHAR',
        'ALTER TABLE lightsout_processed ADD COLUMN IF NOT EXISTS authored_in VARCHAR',
    ]
    with engine.begin() as conn:
        for statement in statements:
            conn.execute(text(statement))


ensure_lightsout_processed_label_columns()

try:
    ensure_exports_dir()
    sweep_stale_exports()
except Exception:  # noqa: BLE001
    logger = logging.getLogger('uvicorn.error')
    logger.warning('exports dir init/sweep failed', exc_info=True)


def ensure_robot_weightment_run_columns() -> None:
    statements = [
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS trace_run_id VARCHAR',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS processed_run_db_id INTEGER',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS processed_id INTEGER',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS mcap_path VARCHAR',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS parquet_path VARCHAR',
        'ALTER TABLE robot_weightment_runs ADD COLUMN IF NOT EXISTS mes_timeseries_sent BOOLEAN DEFAULT FALSE',
    ]
    with engine.begin() as conn:
        for statement in statements:
            conn.execute(text(statement))


ensure_robot_weightment_run_columns()

app = FastAPI(title='Rhapsodi Backend')
app.include_router(export_router)

# Robot dashboards are opened via Tailscale MagicDNS / LAN hostnames
# (e.g. http://rhapsodi-pi5:8080). Default to reflecting any http(s) Origin
# when CORS_ORIGINS is unset or '*'; otherwise use an explicit allow-list.
_cors_raw = os.environ.get('CORS_ORIGINS', '*').strip()
_cors_origins = [o.strip() for o in _cors_raw.split(',') if o.strip()]
if not _cors_origins or _cors_origins == ['*']:
    app.add_middleware(
        CORSMiddleware,
        allow_origins=[],
        allow_origin_regex=r'https?://.*',
        allow_credentials=True,
        allow_methods=['*'],
        allow_headers=['*'],
    )
else:
    app.add_middleware(
        CORSMiddleware,
        allow_origins=_cors_origins,
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


@app.get('/runtime/mode')
def get_runtime_mode() -> dict:
    manager = get_mode_manager()
    current = manager.current()
    active_layout = get_active_cell_layout()
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        active = has_active_robot_run(db)
        active_payload = None
        if active is not None:
            active_payload = {
                'id': active.id,
                'status': active.status,
                'weightment_id': active.weightment_id,
                'event_id': active.event_id,
                'batch_id': active.batch_id,
            }
        else:
            lo = get_lightsout_session().as_blocker()
            if lo is not None:
                active_payload = {
                    'id': lo.id,
                    'status': lo.status,
                    'weightment_id': lo.weightment_id,
                    'event_id': lo.event_id,
                    'batch_id': lo.batch_id,
                    'kind': 'lightsout',
                }
        return {
            'mode': current['mode'],
            'environment': current['environment'],
            'active_run': active_payload,
            'layout_id': (active_layout or {}).get('layout_id'),
            'layout_hash': (active_layout or {}).get('layout_hash'),
            'layout_applied': active_layout is not None,
        }
    finally:
        db.close()


@app.put('/runtime/mode')
def put_runtime_mode(payload: RuntimeModeRequest) -> dict:
    manager = get_mode_manager()
    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        # Broader than has_active_robot_run: also block during awaiting_processing /
        # mes_send_failed so Condor outbound cannot be null-routed mid-flight.
        # Lights-out training is tracked separately (no RobotWeightmentRun row).
        blocker = has_mode_switch_blocker(db)
        if blocker is None:
            blocker = get_lightsout_session().as_blocker()
        try:
            current = manager.set_mode(
                payload.mode, payload.environment, active_run=blocker
            )
        except ModeSwitchConflict as exc:
            run = exc.active_run
            raise HTTPException(
                status_code=409,
                detail={
                    'message': (
                        'Cannot change mode while a robot run is active '
                        'or still sending MES results'
                    ),
                    'active_run': {
                        'id': getattr(run, 'id', None),
                        'status': getattr(run, 'status', None),
                        'weightment_id': getattr(run, 'weightment_id', None),
                    },
                },
            ) from exc
        except ModeValidationError as exc:
            raise HTTPException(status_code=400, detail=exc.message) from exc
        response = {
            'mode': current['mode'],
            'environment': current['environment'],
            'active_run': None,
        }
        try:
            response['layout_apply'] = apply_mode_layout(current['mode'])
        except Exception as exc:  # Layout application must not undo a mode switch.
            logger.warning('Mode layout apply failed: %s', exc, exc_info=True)
            response['layout_apply'] = {
                'applied': False,
                'message': str(exc),
            }
        return response
    finally:
        db.close()


@app.get('/runtime/capabilities')
def get_runtime_capabilities() -> dict:
    return get_mode_manager().capabilities()


@app.get('/layouts')
def get_layouts() -> dict:
    """List cell layouts present on this device's deploy bundle."""
    active = get_active_cell_layout() or {}
    active_id = active.get('layout_id')
    layouts = []
    for item in list_layouts():
        layouts.append(
            {
                **item,
                'active': bool(active_id) and item.get('layout_id') == active_id,
            }
        )
    return {
        'layouts': layouts,
        'active_layout_id': active_id,
        'active_layout_hash': active.get('layout_hash'),
    }


@app.get('/powders')
def list_powders() -> dict:
    """Operator powder catalog for lightsout / mock-local labeling."""
    try:
        powders = [p.as_dict() for p in load_powders()]
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    return {'powders': powders}


@app.get('/batch_ids/next')
def next_batch_id(mode: str) -> dict:
    """Suggest the next free incremental batch id for a mode (advisory)."""
    mode_id = (mode or '').strip()
    prefix = prefix_for_mode(mode_id)
    if not prefix:
        raise HTTPException(status_code=400, detail=f'Unknown mode: {mode_id}')

    db = SessionLocal()
    try:
        lo_ids = [
            value
            for (value,) in db.query(LightsOutProcessed.batch_id)
            .filter(
                LightsOutProcessed.mode == mode_id,
                LightsOutProcessed.batch_id.isnot(None),
            )
            .distinct()
            .all()
            if value
        ]
        run_ids = [
            value
            for (value,) in db.query(Run.batch_id)
            .filter(Run.mode == mode_id, Run.batch_id.isnot(None))
            .distinct()
            .all()
            if value
        ]
    finally:
        db.close()

    batch_id, previous = suggest_next_batch_id(prefix, [*lo_ids, *run_ids])
    return {
        'mode': mode_id,
        'prefix': prefix,
        'batch_id': batch_id,
        'previous': previous,
    }


def build_mock_robot_run_contract(
    row: WebhookWeightment,
    *,
    location_code: str | None,
    pickup_target_name: str | None,
    weigh_target_name: str | None,
    return_target_name: str | None,
    tolerance_g: float | None,
) -> dict:
    """Build adapter contract for a mock-local synthetic weightment row."""
    if row.target_weight_kg is None:
        raise HTTPException(
            status_code=400, detail='Mock weightment is missing target_weight_kg'
        )
    mapped = resolve_robot_targets(None, location_code)
    pickup = (
        (pickup_target_name or '').strip()
        or mapped.pickup_target_name
        or MOCK_DEFAULT_PICKUP_TARGET
    )
    weigh = (
        (weigh_target_name or '').strip()
        or mapped.weigh_target_name
        or MOCK_DEFAULT_WEIGH_TARGET
    )
    ret = (
        (return_target_name or '').strip()
        or mapped.return_target_name
        or MOCK_DEFAULT_RETURN_TARGET
    )
    # Prefer explicit mock defaults when mapping falls back to ReturnHome
    # and the operator did not override return.
    if not (return_target_name or '').strip() and ret == 'ReturnHome':
        ret = MOCK_DEFAULT_RETURN_TARGET
    target_weight_kg = float(row.target_weight_kg)
    target_weight_g = target_weight_kg * 1000.0
    if tolerance_g is not None and float(tolerance_g) > 0:
        weight_tolerance_g = float(tolerance_g)
    else:
        weight_tolerance_g = (
            float(mapped.weight_tolerance_g)
            if mapped.weight_tolerance_g > 0
            else target_weight_g * 0.02
        )
    location_id = 0
    code = (location_code or '').strip() or 'MOCK'
    try:
        batch_id_int = int(row.batch_id) if row.batch_id is not None else 0
        ingredient_id_int = (
            int(row.ingredient_id) if row.ingredient_id is not None else 0
        )
    except ValueError:
        batch_id_int = 0
        ingredient_id_int = 0
    return {
        'weightment_id': row.id,
        'event_id': row.event_id,
        'batch_id': row.batch_id,
        'batch_id_int': batch_id_int,
        'ingredient_id': row.ingredient_id or MOCK_INGREDIENT_ID,
        'ingredient_id_int': ingredient_id_int,
        'site_id': row.site_id or MOCK_SITE_ID,
        'location_id': location_id,
        'location_code': code,
        'ingredient_name': 'Mock local',
        'pickup_target_name': pickup,
        'weigh_target_name': weigh,
        'return_target_name': ret,
        'target_weight_kg': target_weight_kg,
        'target_weight_g': target_weight_g,
        'weight_tolerance_g': weight_tolerance_g,
        'expected_lot': row.lot_code or '',
        'mode': 'mock-local',
    }


@app.post('/modes/mock/runs')
def create_mock_local_run(payload: MockLocalRunRequest) -> dict:
    """Start a synthetic single-location robot run in mock-local mode."""
    manager = get_mode_manager()
    current = manager.current()
    if current['mode'] != OperatingMode.MOCK_LOCAL.value:
        raise HTTPException(
            status_code=409,
            detail={
                'message': (
                    'Active mode must be mock-local to start a mock run '
                    f"(current={current['mode']})"
                ),
                'mode': current['mode'],
                'environment': current['environment'],
            },
        )
    require_layout_for_run(current)
    if payload.target_weight_g <= 0:
        raise HTTPException(
            status_code=400, detail='target_weight_g must be > 0'
        )
    if payload.tolerance_g is not None and payload.tolerance_g < 0:
        raise HTTPException(
            status_code=400, detail='tolerance_g must be >= 0'
        )
    cycles = int(payload.cycles or 1)
    if cycles <= 0:
        raise HTTPException(status_code=400, detail='cycles must be > 0')

    if get_lightsout_session().get_active() is not None:
        raise HTTPException(
            status_code=409,
            detail='Cannot start mock run while lights-out training is active',
        )

    powder = None
    powder_id = (payload.powder_id or '').strip()
    if powder_id:
        try:
            powder = get_powder(powder_id)
        except (KeyError, ValueError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
    ingredient_id = powder.id if powder else MOCK_INGREDIENT_ID
    lot_code = (payload.lot_code or '').strip()

    db = SessionLocal()
    row: WebhookWeightment | None = None
    created_rows: list[WebhookWeightment] = []
    try:
        reconcile_stale_robot_runs(db)
        mock_uuid = str(uuid.uuid4())
        event_id = f'{MOCK_EVENT_ID_PREFIX}{mock_uuid}'
        # Negative batch ids cannot collide with Condor (positive) ids.
        batch_id = allocate_mock_batch_id(mock_uuid)
        target_weight_kg = float(payload.target_weight_g) / 1000.0
        auto_run = cycles > 1
        for _ in range(cycles):
            created = WebhookWeightment(
                event_id=event_id,
                sent_utc=utc_now(),
                site_id=MOCK_SITE_ID,
                batch_id=batch_id,
                batch_number=f'mock{batch_id}',
                ingredient_id=ingredient_id,
                target_weight_kg=target_weight_kg,
                weightment_completed=False,
                mes_timeseries_sent=False,
                batch_auto_run_enabled=auto_run,
                lot_code=lot_code,
            )
            db.add(created)
            created_rows.append(created)
        db.commit()
        for created in created_rows:
            db.refresh(created)
        row = created_rows[0]

        contract = build_mock_robot_run_contract(
            row,
            location_code=payload.location_code,
            pickup_target_name=payload.pickup_target_name,
            weigh_target_name=payload.weigh_target_name,
            return_target_name=payload.return_target_name,
            tolerance_g=payload.tolerance_g,
        )
        contract.update(require_layout_for_run(current))
        if powder is not None:
            contract['ingredient_id'] = powder.id
            contract['ingredient_name'] = powder.name
            contract['powder_id'] = powder.id
            contract['powder_name'] = powder.name
            contract['operator'] = (payload.operator or '').strip()
            contract['notes'] = (payload.notes or '').strip()
        result = start_robot_run_for_weightment_row(db, row, contract=contract)
        return {
            'ok': True,
            'mode': current['mode'],
            'environment': current['environment'],
            'event_id': event_id,
            'weightment_id': row.id,
            'cycles': cycles,
            'weightment_ids': [r.id for r in created_rows],
            'powder_id': powder.id if powder else None,
            'powder_name': powder.name if powder else None,
            'run': result.get('run'),
            'alreadyRunning': bool(result.get('alreadyRunning')),
            'contract': {
                'location_code': contract['location_code'],
                'pickup_target_name': contract['pickup_target_name'],
                'weigh_target_name': contract['weigh_target_name'],
                'return_target_name': contract['return_target_name'],
                'target_weight_g': contract['target_weight_g'],
                'weight_tolerance_g': contract['weight_tolerance_g'],
            },
        }
    except HTTPException:
        # Roll back orphaned mock weightments if adapter start failed after insert.
        try:
            for orphan_row in created_rows:
                if orphan_row.id is None:
                    continue
                orphan = (
                    db.query(WebhookWeightment)
                    .filter(WebhookWeightment.id == orphan_row.id)
                    .first()
                )
                if orphan is None or not is_mock_event_id(orphan.event_id):
                    continue
                has_run = (
                    db.query(RobotWeightmentRun)
                    .filter(RobotWeightmentRun.weightment_id == orphan.id)
                    .first()
                )
                if has_run is None:
                    db.delete(orphan)
            db.commit()
        except Exception:
            logger.exception(
                'Failed to purge orphaned mock weightments for event'
            )
        raise
    except Exception as exc:
        db.rollback()
        logger.exception('Failed to start mock-local run')
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    finally:
        db.close()


def _resolve_lightsout_powder(payload: LightsoutRunRequest):
    """Resolve powder_id (preferred) against the catalog."""
    powder_id = (payload.powder_id or '').strip()
    if not powder_id and (payload.powder_name or '').strip():
        # One-release compat: map free-text name to catalog by name or id.
        wanted = payload.powder_name.strip().lower()
        for powder in load_powders():
            if powder.id.lower() == wanted or powder.name.lower() == wanted:
                return powder
        raise HTTPException(
            status_code=400,
            detail=(
                'powder_id is required; free-text powder_name is deprecated '
                'and did not match the catalog'
            ),
        )
    try:
        return get_powder(powder_id)
    except KeyError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


def start_lightsout_run_via_adapter(
    payload: LightsoutRunRequest,
    *,
    powder,
    schedule,
    target_weight_g: float,
    stop_value: float,
) -> dict[str, Any]:
    """POST training start to robot_start_adapter /start_lightsout (no Condor)."""
    min_scooped = (
        float(payload.min_scooped_g)
        if payload.min_scooped_g and payload.min_scooped_g > 0
        else float(powder.min_scooped_g)
    )
    adapter_payload = {
        'powder_id': powder.id,
        'powder_name': powder.name,
        'container_target': powder.container_target,
        'pour_target': powder.pour_target,
        'lot_code': (payload.lot_code or '').strip(),
        'operator': (payload.operator or '').strip(),
        'notes': (payload.notes or '').strip(),
        'target_weight_g': float(target_weight_g),
        'episodes': int(payload.episodes),
        'batch_id': (payload.batch_id or '').strip(),
        'enable_scoop': bool(payload.enable_scoop),
        'stop_on': payload.stop_on,
        'stop_value': float(stop_value),
        'target_mode': schedule.target_mode,
        'target_fractions': list(schedule.fractions),
        'min_scooped_g': min_scooped,
        'target_min_g': float(payload.target_min_g or 0.0),
        'target_max_g': float(payload.target_max_g or 0.0),
        'rng_seed': int(schedule.rng_seed),
    }
    return post_json(
        ROBOT_LIGHTSOUT_ADAPTER_URL,
        adapter_payload,
        timeout_seconds=ROBOT_START_ADAPTER_TIMEOUT_SECONDS,
    )


@app.post('/modes/lightsout/runs')
def create_lightsout_run(payload: LightsoutRunRequest) -> dict:
    """Start a lights-out training run (null MES; LightsOut tree)."""
    manager = get_mode_manager()
    current = manager.current()
    if current['mode'] != OperatingMode.LIGHTSOUT.value:
        raise HTTPException(
            status_code=409,
            detail={
                'message': (
                    'Active mode must be lightsout to start a training run '
                    f"(current={current['mode']})"
                ),
                'mode': current['mode'],
                'environment': current['environment'],
            },
        )
    require_layout_for_run(current)
    powder = _resolve_lightsout_powder(payload)
    try:
        resolved = resolve_lightsout_targets(
            target_mode=payload.target_mode,
            target_weight_g=payload.target_weight_g,
            stop_on=payload.stop_on,
            stop_value=payload.stop_value,
            episodes=int(payload.episodes),
            target_min_g=float(payload.target_min_g or 0.0),
            target_max_g=float(payload.target_max_g or 0.0),
            powder_default_target_g=float(powder.default_target_weight_g),
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    resolved_target = float(resolved['target_weight_g'])
    stop_value = float(resolved['stop_value'])

    try:
        schedule = generate_target_schedule(
            cycles=int(payload.episodes),
            target_mode=payload.target_mode,
            frac_min=float(payload.frac_min),
            frac_max=float(payload.frac_max),
            fixed_target_g=(
                resolved_target if payload.target_mode == 'fixed' else None
            ),
            rng_seed=payload.rng_seed,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    label = RunLabel(
        powder_id=powder.id,
        powder_name=powder.name,
        container_target=powder.container_target,
        pour_target=powder.pour_target,
        lot_code=(payload.lot_code or '').strip() or None,
        operator=(payload.operator or '').strip() or None,
        notes=(payload.notes or '').strip() or None,
        cycles=int(payload.episodes),
        stop_on=payload.stop_on,
        stop_value=stop_value or None,
        target_mode=schedule.target_mode,
        target_fractions=list(schedule.fractions),
        rng_seed=schedule.rng_seed,
    )
    request_meta = {
        'powder_id': powder.id,
        'powder_name': powder.name,
        'container_target': powder.container_target,
        'pour_target': powder.pour_target,
        'target_weight_g': resolved_target,
        'episodes': int(payload.episodes),
        'batch_id': (payload.batch_id or '').strip(),
        'enable_scoop': bool(payload.enable_scoop),
        'lot_code': label.lot_code or '',
        'operator': label.operator or '',
        'notes': label.notes or '',
        'stop_on': payload.stop_on,
        'stop_value': stop_value,
        'target_mode': schedule.target_mode,
        'target_fractions': list(schedule.fractions),
        'rng_seed': schedule.rng_seed,
        'label': label.model_dump(exclude_none=True),
    }
    request_meta.update(require_layout_for_run(current))

    db = SessionLocal()
    try:
        reconcile_stale_robot_runs(db)
        # Refuse while a webhook/mock robot run is mid-flight so mode sinks
        # stay coherent (lightsout itself is tracked via session + rosbridge).
        active = has_active_robot_run(db)
        if active is not None:
            raise HTTPException(
                status_code=409,
                detail={
                    'message': (
                        'Cannot start lights-out training while a robot '
                        f'weightment run is active (run_id={active.id})'
                    ),
                    'active_run': serialize_robot_run(active),
                },
            )
        existing_lo = get_lightsout_session().get_active()
        if existing_lo is not None:
            raise HTTPException(
                status_code=409,
                detail={
                    'message': 'Lights-out training session already active',
                    'session': existing_lo,
                },
            )
    finally:
        db.close()

    try:
        service_response = start_lightsout_run_via_adapter(
            payload,
            powder=powder,
            schedule=schedule,
            target_weight_g=resolved_target,
            stop_value=stop_value,
        )
    except HTTPException:
        raise
    except Exception as exc:
        logger.exception('Failed to start lights-out run')
        raise HTTPException(status_code=502, detail=str(exc)) from exc

    accepted = bool(service_response.get('accepted'))
    message = str(service_response.get('message') or '')
    if not accepted:
        raise HTTPException(
            status_code=409,
            detail={
                'message': message or 'Lights-out start rejected by orchestrator',
                'mode': current['mode'],
                'environment': current['environment'],
            },
        )

    session = get_lightsout_session().mark_started(request_meta)
    # Reuse rosbridge run_state completion to clear the session when the
    # orchestrator finishes (succeeded/failed/stopped).
    rosbridge_robot_client.register_active_run(
        LIGHTSOUT_ROSBRIDGE_RUN_ID,
        {'kind': 'lightsout', **request_meta},
        service_response,
    )
    return {
        'ok': True,
        'mode': current['mode'],
        'environment': current['environment'],
        'accepted': True,
        'message': message,
        'request': request_meta,
        'session': session,
        'schedule': {
            'target_mode': schedule.target_mode,
            'fractions': list(schedule.fractions),
            'rng_seed': schedule.rng_seed,
        },
    }


@app.post('/modes/mes-generic/events')
def ingest_mes_generic_event(payload: dict[str, Any]) -> dict:
    """Ingest a non-Promtek (or Condor-shaped) event via the inbound adapter.

    Active mode must be ``mes-generic``. Adapter selected by
    ``MES_GENERIC_INBOUND_ADAPTER`` (default ``condor``). Rows are stored in
    ``webhook_weightments`` with a ``generic-`` event_id prefix so outbound
    routing uses ``MES_GENERIC_SINK``.
    """
    manager = get_mode_manager()
    current = manager.current()
    if current['mode'] != OperatingMode.MES_GENERIC.value:
        raise HTTPException(
            status_code=409,
            detail={
                'message': (
                    'Active mode must be mes-generic to ingest events '
                    f"(current={current['mode']})"
                ),
                'mode': current['mode'],
                'environment': current['environment'],
            },
        )

    try:
        adapter_name = get_inbound_adapter_name()
        weightments = normalize_inbound_event(payload)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    if not weightments:
        return {
            'accepted': True,
            'duplicate': False,
            'mode': current['mode'],
            'adapter': adapter_name,
            'insertedRows': 0,
            'eventId': None,
        }

    # All rows from one POST share one prefixed event_id for routing / status.
    upstream_event_id = weightments[0].event_id
    event_id = ensure_generic_event_id(upstream_event_id)
    upstream_raw = strip_generic_event_id_prefix(event_id)

    db = SessionLocal()
    try:
        # Dedupe against both the prefixed id and any prior webhook_service
        # row that stored the raw Condor eventId.
        dedupe_ids = {event_id}
        if upstream_raw:
            dedupe_ids.add(upstream_raw)
        existing_count = (
            db.query(func.count(WebhookWeightment.id))
            .filter(WebhookWeightment.event_id.in_(list(dedupe_ids)))
            .scalar()
        )
        if existing_count:
            logger.info(
                'Skipping duplicate mes-generic event_id=%s existing_rows=%s',
                event_id,
                existing_count,
            )
            return {
                'accepted': True,
                'duplicate': True,
                'mode': current['mode'],
                'adapter': adapter_name,
                'eventId': event_id,
                'insertedRows': 0,
            }

        rows: list[WebhookWeightment] = []
        for item in weightments:
            rows.append(
                WebhookWeightment(
                    event_id=event_id,
                    sent_utc=item.sent_utc,
                    user_id=item.user_id,
                    site_id=item.site_id,
                    batch_id=item.batch_id,
                    batch_number=item.batch_number,
                    work_order_id=item.work_order_id,
                    batch_target_quantity=item.batch_target_quantity,
                    ingredient_id=item.ingredient_id,
                    target_weight_kg=item.target_weight_kg,
                    lot_code=item.lot_code or '',
                    weightment_completed=False,
                    mes_timeseries_sent=False,
                    batch_auto_run_enabled=False,
                )
            )
        db.add_all(rows)
        db.flush()
        weightment_ids = [int(row.id) for row in rows]
        db.commit()
        inserted = len(weightment_ids)
        logger.info(
            'Stored mes-generic event_id=%s adapter=%s inserted_rows=%s sink=%s',
            event_id,
            adapter_name,
            inserted,
            mes_generic_sink_name(),
        )
        return {
            'accepted': True,
            'duplicate': False,
            'mode': current['mode'],
            'adapter': adapter_name,
            'adapters': list_inbound_adapters(),
            'sink': mes_generic_sink_name(),
            'eventId': event_id,
            'insertedRows': inserted,
            'weightmentIds': weightment_ids,
        }
    except HTTPException:
        raise
    except Exception as exc:
        db.rollback()
        logger.exception('Failed to ingest mes-generic event')
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    finally:
        db.close()


def _load_device_identity() -> dict[str, str | None]:
    """Resolve device identity for fleet polling.

    Preference order for each field:
    1. Explicit env overrides (DEVICE_ID / ROBOT_TYPE / SITE_ID / ROBOT_ID)
    2. Mounted config/device.yaml (RHAPSODI_DEVICE_CONFIG or /ws/config/device.yaml)
    3. None (caller decides fallbacks)
    """
    identity: dict[str, str | None] = {
        'device_id': os.environ.get('DEVICE_ID') or None,
        'robot_id': os.environ.get('ROBOT_ID') or None,
        'robot_type': os.environ.get('ROBOT_TYPE') or None,
        'site_id': os.environ.get('SITE_ID') or None,
    }
    if all(identity.values()):
        return identity

    candidates = []
    env_path = os.environ.get('RHAPSODI_DEVICE_CONFIG')
    if env_path:
        candidates.append(env_path)
    candidates.extend(
        [
            '/ws/config/device.yaml',
            '/config/device.yaml',
            'config/device.yaml',
        ]
    )
    for path in candidates:
        try:
            if not os.path.isfile(path):
                continue
            # Minimal YAML parse for the known device.yaml shape (no PyYAML dep).
            device_block: dict[str, str] = {}
            in_device = False
            with open(path, encoding='utf-8') as fh:
                for raw in fh:
                    line = raw.rstrip('\n')
                    if not line.strip() or line.lstrip().startswith('#'):
                        continue
                    if line.startswith('device:'):
                        in_device = True
                        continue
                    if in_device and line and not line.startswith(' ') and not line.startswith('\t'):
                        break
                    if not in_device:
                        continue
                    stripped = line.strip()
                    if ':' not in stripped or stripped.endswith(':'):
                        continue
                    key, _, value = stripped.partition(':')
                    key = key.strip()
                    value = value.strip().strip("'\"")
                    if key in ('device_id', 'robot_id', 'robot_type', 'site_id') and value:
                        device_block[key] = value
            for key in identity:
                if not identity[key] and key in device_block:
                    identity[key] = device_block[key]
            break
        except OSError:
            continue
    return identity


def _load_image_tag() -> str | None:
    """Resolve the deployed image tag for fleet version reporting."""
    env_tag = (os.environ.get('IMAGE_TAG') or '').strip()
    if env_tag:
        return env_tag
    for path in (
        os.environ.get('RHAPSODI_VERSION_FILE') or '',
        '/etc/rhapsodi-version',
        '/ws/.rhapsodi-version',
    ):
        if not path:
            continue
        try:
            if not os.path.isfile(path):
                continue
            raw = open(path, encoding='utf-8').read().strip()
            if not raw:
                continue
            if raw.startswith('{'):
                try:
                    payload = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                tag = str(payload.get('image_tag') or '').strip()
                if tag:
                    return tag
            else:
                return raw
        except OSError:
            continue
    return None


def _load_profile_id() -> str | None:
    env_profile = (os.environ.get('PROFILE_ID') or '').strip()
    if env_profile:
        return env_profile
    for path in (
        os.environ.get('RHAPSODI_VERSION_FILE') or '',
        '/etc/rhapsodi-version',
        '/ws/.rhapsodi-version',
    ):
        if not path or not os.path.isfile(path):
            continue
        try:
            raw = open(path, encoding='utf-8').read().strip()
            if raw.startswith('{'):
                payload = json.loads(raw)
                profile = str(payload.get('profile_id') or '').strip()
                if profile:
                    return profile
        except (OSError, json.JSONDecodeError):
            continue
    return None


def _proc_root() -> Path:
    """Host /proc when bind-mounted; else container /proc."""
    for candidate in (
        os.environ.get('HOST_PROC') or '',
        '/host/proc',
        '/proc',
    ):
        if candidate and Path(candidate, 'meminfo').is_file():
            return Path(candidate)
    return Path('/proc')


def _sys_root() -> Path:
    for candidate in (
        os.environ.get('HOST_SYS') or '',
        '/host/sys',
        '/sys',
    ):
        if candidate and Path(candidate).is_dir():
            return Path(candidate)
    return Path('/sys')


_CPU_CACHE: dict[str, Any] = {'at': 0.0, 'pct': None}


def _nproc(proc: Path) -> int:
    try:
        n = 0
        for line in (proc / 'stat').read_text(encoding='utf-8').splitlines():
            if line.startswith('cpu') and len(line) > 3 and line[3].isdigit():
                n += 1
        return n or 1
    except OSError:
        return 1


def _read_cpu_pct(proc: Path, sample_s: float = 0.75) -> float | None:
    """Host CPU busy % from two /proc/stat samples (cached ~2s).

    Short windows (e.g. 150ms) read very high on bursty ROS cells; a ~1s
    window matches what operators expect from top/`uptime`.
    """
    now = time.monotonic()
    cached_at = float(_CPU_CACHE.get('at') or 0.0)
    if now - cached_at < 2.0 and _CPU_CACHE.get('pct') is not None:
        return _CPU_CACHE['pct']  # type: ignore[return-value]

    def snapshot() -> tuple[int, int] | None:
        try:
            line = (proc / 'stat').read_text(encoding='utf-8').splitlines()[0]
            parts = [int(x) for x in line.split()[1:]]
            if len(parts) < 4:
                return None
            idle = parts[3] + (parts[4] if len(parts) > 4 else 0)
            total = sum(parts)
            return idle, total
        except (OSError, ValueError, IndexError):
            return None

    a = snapshot()
    if not a:
        return None
    time.sleep(sample_s)
    b = snapshot()
    if not b:
        return None
    idle_d = b[0] - a[0]
    total_d = b[1] - a[1]
    if total_d <= 0:
        return None
    busy = max(0.0, min(100.0, 100.0 * (1.0 - idle_d / total_d)))
    pct = round(busy, 1)
    _CPU_CACHE['at'] = time.monotonic()
    _CPU_CACHE['pct'] = pct
    return pct


def _read_loadavg(proc: Path) -> dict[str, float | int | None]:
    try:
        parts = (proc / 'loadavg').read_text(encoding='utf-8').split()
        load1 = float(parts[0])
        load5 = float(parts[1])
        load15 = float(parts[2])
        cores = _nproc(proc)
        # Load relative to core count (can exceed 100% when runnable > CPUs).
        pressure = round(100.0 * load1 / max(cores, 1), 1)
        return {
            'load1': load1,
            'load5': load5,
            'load15': load15,
            'cpu_cores': cores,
            'load_pressure_pct': pressure,
        }
    except (OSError, IndexError, ValueError):
        return {
            'load1': None,
            'load5': None,
            'load15': None,
            'cpu_cores': None,
            'load_pressure_pct': None,
        }


def _read_mem(proc: Path) -> dict[str, float | int | None]:
    try:
        info: dict[str, int] = {}
        for line in (proc / 'meminfo').read_text(encoding='utf-8').splitlines():
            if ':' not in line:
                continue
            key, raw = line.split(':', 1)
            num = raw.strip().split()[0]
            info[key] = int(num) * 1024  # kB → bytes
        total = info.get('MemTotal')
        available = info.get('MemAvailable')
        if total and available is not None and total > 0:
            used = total - available
            return {
                'mem_total_bytes': total,
                'mem_used_bytes': used,
                'mem_pct': round(100.0 * used / total, 1),
            }
    except (OSError, ValueError, IndexError):
        pass
    return {'mem_total_bytes': None, 'mem_used_bytes': None, 'mem_pct': None}


def _read_temp_c(sys_root: Path) -> float | None:
    zones = sorted(sys_root.glob('class/thermal/thermal_zone*/temp'))
    for path in zones:
        try:
            raw = int(path.read_text(encoding='utf-8').strip())
            # millidegrees on Linux thermal zones
            value = raw / 1000.0 if raw > 200 else float(raw)
            if 0 < value < 150:
                return round(value, 1)
        except (OSError, ValueError):
            continue
    return None


def _read_disk(path: str) -> dict[str, float | int | None]:
    try:
        usage = os.statvfs(path)
        total = usage.f_frsize * usage.f_blocks
        free = usage.f_frsize * usage.f_bavail
        used = total - free
        if total <= 0:
            raise OSError('empty filesystem')
        return {
            'disk_path': path,
            'disk_total_bytes': total,
            'disk_used_bytes': used,
            'disk_pct': round(100.0 * used / total, 1),
        }
    except OSError:
        return {
            'disk_path': path,
            'disk_total_bytes': None,
            'disk_used_bytes': None,
            'disk_pct': None,
        }


def _parse_node_exporter(text: str) -> dict[str, Any]:
    """Best-effort gauges from node_exporter text exposition."""
    gauges: dict[str, float] = {}
    for line in text.splitlines():
        if not line or line.startswith('#') or '{' not in line and ' ' not in line:
            # bare metric
            parts = line.split()
            if len(parts) == 2:
                try:
                    gauges[parts[0]] = float(parts[1])
                except ValueError:
                    pass
            continue
        # Keep first-match simple gauges we care about.
        try:
            name_labels, value_s = line.rsplit(' ', 1)
            value = float(value_s)
        except ValueError:
            continue
        if name_labels == 'node_load1' or name_labels.startswith('node_load1{'):
            gauges['load1'] = value
        elif name_labels == 'node_load5' or name_labels.startswith('node_load5{'):
            gauges['load5'] = value
        elif name_labels == 'node_load15' or name_labels.startswith('node_load15{'):
            gauges['load15'] = value
        elif name_labels.startswith('node_memory_MemTotal_bytes'):
            gauges['mem_total'] = value
        elif name_labels.startswith('node_memory_MemAvailable_bytes'):
            gauges['mem_avail'] = value
        elif (
            name_labels.startswith('node_filesystem_size_bytes{')
            and 'mountpoint="/"' in name_labels
            and 'fstype="rootfs"' not in name_labels
        ):
            gauges['disk_total'] = value
        elif (
            name_labels.startswith('node_filesystem_avail_bytes{')
            and 'mountpoint="/"' in name_labels
            and 'fstype="rootfs"' not in name_labels
        ):
            gauges['disk_avail'] = value
        elif name_labels.startswith('node_thermal_zone_temp') or (
            'node_hwmon_temp_celsius' in name_labels and 'temp1' in name_labels
        ):
            if 'temp_c' not in gauges and 0 < value < 150:
                gauges['temp_c'] = value

    out: dict[str, Any] = {'source': 'node_exporter'}
    if 'load1' in gauges:
        out['load1'] = gauges['load1']
        out['load5'] = gauges.get('load5')
        out['load15'] = gauges.get('load15')
    if 'mem_total' in gauges and 'mem_avail' in gauges and gauges['mem_total'] > 0:
        used = gauges['mem_total'] - gauges['mem_avail']
        out['mem_total_bytes'] = int(gauges['mem_total'])
        out['mem_used_bytes'] = int(used)
        out['mem_pct'] = round(100.0 * used / gauges['mem_total'], 1)
    if 'disk_total' in gauges and 'disk_avail' in gauges and gauges['disk_total'] > 0:
        used = gauges['disk_total'] - gauges['disk_avail']
        out['disk_path'] = '/'
        out['disk_total_bytes'] = int(gauges['disk_total'])
        out['disk_used_bytes'] = int(used)
        out['disk_pct'] = round(100.0 * used / gauges['disk_total'], 1)
    if 'temp_c' in gauges:
        out['temp_c'] = round(gauges['temp_c'], 1)
    return out


def _scrape_node_exporter() -> dict[str, Any] | None:
    url = (
        os.environ.get('NODE_EXPORTER_URL') or 'http://host.docker.internal:9100/metrics'
    ).strip()
    if not url:
        return None
    try:
        req = request.Request(url, method='GET')
        with request.urlopen(req, timeout=1.5) as resp:
            text = resp.read().decode('utf-8', errors='replace')
        parsed = _parse_node_exporter(text)
        # CPU still from /proc sample (node_exporter needs a rate()).
        return parsed
    except Exception:  # noqa: BLE001
        return None


def _collect_host_metrics() -> dict[str, Any]:
    """Pi (or API host) system metrics for the Controls dashboard."""
    proc = _proc_root()
    sys_root = _sys_root()
    metrics: dict[str, Any] = {
        'source': f'proc:{proc}',
        'cpu_pct': _read_cpu_pct(proc),
        **_read_loadavg(proc),
        **_read_mem(proc),
        'temp_c': _read_temp_c(sys_root),
    }
    # Prefer /data (robot store) when present; else root / host root.
    disk_path = '/data' if Path('/data').is_dir() else '/'
    for candidate in ('/host/root', disk_path, '/'):
        if candidate == '/host/root' and not Path(candidate).is_dir():
            continue
        disk = _read_disk(candidate if candidate != '/host/root' else '/host/root')
        if disk.get('disk_total_bytes'):
            metrics.update(disk)
            break
    else:
        metrics.update(_read_disk(disk_path))

    ne = _scrape_node_exporter()
    if ne:
        # Prefer host exporter for mem/disk/load/temp; keep local cpu sample.
        for key in (
            'load1',
            'load5',
            'load15',
            'mem_pct',
            'mem_used_bytes',
            'mem_total_bytes',
            'disk_pct',
            'disk_used_bytes',
            'disk_total_bytes',
            'disk_path',
            'temp_c',
        ):
            if ne.get(key) is not None:
                metrics[key] = ne[key]
        metrics['source'] = 'node_exporter+proc'
    return metrics


@app.get('/host_info')
def host_info() -> dict:
    identity = _load_device_identity()
    now = datetime.now(tz=timezone.utc)
    return {
        'hostname': socket.gethostname(),
        'device_id': identity.get('device_id'),
        'robot_id': identity.get('robot_id'),
        'robot_type': identity.get('robot_type'),
        'site_id': identity.get('site_id'),
        'image_tag': _load_image_tag(),
        'profile_id': _load_profile_id(),
        # UTC clock on the API host (Pi in robot-prod). Used by the dashboard to
        # surface Pi vs browser vs Niryo time skew (MoveIt is sensitive to this).
        'utc_unix': now.timestamp(),
        'utc_iso': now.isoformat(timespec='milliseconds').replace('+00:00', 'Z'),
        'metrics': _collect_host_metrics(),
    }


def _http_probe(url: str, timeout_s: float = 2.0) -> dict[str, Any]:
    started = time.monotonic()
    try:
        req = request.Request(url, method='GET')
        with request.urlopen(req, timeout=timeout_s) as resp:
            code = getattr(resp, 'status', None) or resp.getcode()
            latency_ms = round((time.monotonic() - started) * 1000)
            return {
                'reachable': True,
                'http_status': int(code),
                'latency_ms': latency_ms,
                'error': None,
            }
    except error.HTTPError as exc:
        # Condor often returns 404 on `/` — still means the listener is up.
        latency_ms = round((time.monotonic() - started) * 1000)
        return {
            'reachable': True,
            'http_status': int(exc.code),
            'latency_ms': latency_ms,
            'error': None,
        }
    except Exception as exc:  # noqa: BLE001 — surface any probe failure to UI
        latency_ms = round((time.monotonic() - started) * 1000)
        return {
            'reachable': False,
            'http_status': None,
            'latency_ms': latency_ms,
            'error': str(exc)[:200],
        }


def _condor_base_url() -> str:
    override = (os.environ.get('CONDOR_AGENT_URL') or '').strip()
    if override:
        return override.rstrip('/')
    parsed = parse.urlparse(WEIGHMENT_URL)
    if parsed.scheme and parsed.netloc:
        return f'{parsed.scheme}://{parsed.netloc}'
    return 'http://127.0.0.1:5002'


def _newest_condor_log(root: Path) -> Path | None:
    if not root.is_dir():
        return None
    newest: Path | None = None
    newest_mtime = -1.0
    try:
        for path in root.rglob('*.log'):
            try:
                mtime = path.stat().st_mtime
            except OSError:
                continue
            if mtime > newest_mtime:
                newest = path
                newest_mtime = mtime
    except OSError:
        return None
    return newest


def _parse_condor_websocket(log_path: Path | None) -> dict[str, Any]:
    """Infer Condor cloud WS health from the agent's file logger."""
    if log_path is None:
        return {
            'status': 'unknown',
            'detail': 'no log file under /data/condor-agent/logs',
            'log_path': None,
            'age_s': None,
        }
    try:
        raw = log_path.read_bytes()
        # Windows agent logs may use UTF-16; fall back to latin-1.
        text = None
        for encoding in ('utf-8', 'utf-16', 'latin-1'):
            try:
                text = raw.decode(encoding)
                break
            except UnicodeDecodeError:
                continue
        if text is None:
            text = raw.decode('latin-1', errors='replace')
        tail = text[-12000:]
        age_s = round(time.time() - log_path.stat().st_mtime, 1)
    except OSError as exc:
        return {
            'status': 'unknown',
            'detail': f'log read failed: {exc}',
            'log_path': str(log_path),
            'age_s': None,
        }

    lower = tail.lower()
    # Prefer the most recent decisive line.
    lines = [ln.strip() for ln in tail.splitlines() if ln.strip()]
    last_ws = None
    for line in reversed(lines):
        if 'websocket' in line.lower() or 'register' in line.lower():
            last_ws = line
            break

    if re.search(r'register[_ ]?error|registration failed|unauthorized', lower):
        status = 'disconnected'
        detail = last_ws or 'registration error in log'
    elif re.search(
        r'websocket:\s*(waiting for messages|connected|connection established)',
        lower,
    ):
        # Stale "Waiting" lines after a long gap mean the process may be hung.
        status = 'connected' if age_s is not None and age_s < 90 else 'stale'
        detail = last_ws or 'WebSocket healthy'
    elif re.search(r'websocket:\s*(disconnected|closed|error)', lower):
        status = 'disconnected'
        detail = last_ws or 'WebSocket disconnected'
    else:
        status = 'unknown'
        detail = last_ws or 'no WebSocket lines in recent log'

    return {
        'status': status,
        'detail': detail[:240],
        'log_path': str(log_path),
        'age_s': age_s,
    }


@app.get('/integrations/status')
def integrations_status() -> dict:
    """Fleet/MES link health for the robot dashboard Controls panel.

    Condor agent:
      - HTTP listener (WEIGHMENT_URL host, often :5002)
      - Cloud WebSocket inferred from persistent agent logs under /data
    Webhook:
      - GET /health on the local webhook_service
    """
    condor_base = _condor_base_url()
    webhook_health = (
        os.environ.get('WEBHOOK_HEALTH_URL') or 'http://webhook_service:5000/health'
    ).strip()

    condor_http = _http_probe(f'{condor_base}/')
    webhook = _http_probe(webhook_health)

    log_roots = [
        Path(p)
        for p in (
            os.environ.get('CONDOR_LOG_ROOT') or '',
            '/data/condor-agent/logs/promtek-condor-rhapsodi-agent',
            '/data/condor-agent/logs',
            '/data/condor-agent',
        )
        if p
    ]
    log_file = None
    for root in log_roots:
        log_file = _newest_condor_log(root)
        if log_file is not None:
            break
    ws = _parse_condor_websocket(log_file)

    if not condor_http['reachable']:
        condor_status = 'disconnected'
    elif ws['status'] == 'disconnected':
        condor_status = 'disconnected'
    elif ws['status'] == 'stale':
        condor_status = 'stale'
    elif ws['status'] == 'connected':
        condor_status = 'connected'
    else:
        # HTTP listener up — weighment posts can reach the agent even if we
        # cannot read the Windows-style log tree from this container mount.
        condor_status = 'connected'

    webhook_status = (
        'connected'
        if webhook['reachable'] and webhook['http_status'] == 200
        else ('degraded' if webhook['reachable'] else 'disconnected')
    )

    return {
        'checked_at': datetime.now(tz=timezone.utc)
        .isoformat(timespec='milliseconds')
        .replace('+00:00', 'Z'),
        'condor': {
            'status': condor_status,
            'base_url': condor_base,
            'http': condor_http,
            'websocket': ws,
        },
        'webhook': {
            'status': webhook_status,
            'health_url': webhook_health,
            'http': webhook,
        },
    }


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
        'mes_timeseries_sent': row.mes_timeseries_sent,
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
        'mes_timeseries_sent': row.mes_timeseries_sent,
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
        'error_message': run_row.error_message,
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
    """True while the robot is moving (starting/running)."""
    return (
        db.query(RobotWeightmentRun)
        .filter(RobotWeightmentRun.status.in_(['starting', 'running']))
        .order_by(RobotWeightmentRun.id.desc())
        .first()
    )


def has_mode_switch_blocker(db) -> RobotWeightmentRun | None:
    """Block mode changes while a run still owns MES outbound work.

    Covers robot motion *and* post-run processing / failed MES sends so a
    switch to mock-local/lightsout cannot route Condor posts through NullMesClient.
    """
    return (
        db.query(RobotWeightmentRun)
        .filter(
            RobotWeightmentRun.status.in_(
                ['starting', 'running', 'awaiting_processing', 'mes_send_failed']
            )
        )
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
    weight_tolerance_g = target_weight_g * 0.02
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
        'weight_tolerance_g': weight_tolerance_g,
        'expected_lot': row.lot_code or '',
        'mode': 'webhook',
    }


def start_robot_run_for_weightment_row(
    db, row: WebhookWeightment, *, contract: dict | None = None
) -> dict:
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
    contract = contract if contract is not None else build_robot_run_contract(db, row)
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


def batch_timeseries_already_sent(db, batch_id: str) -> bool:
    rows = (
        db.query(WebhookWeightment)
        .filter(WebhookWeightment.batch_id == batch_id)
        .all()
    )
    return bool(rows) and all(row.mes_timeseries_sent for row in rows)


def mark_batch_timeseries_sent(db, batch_id: str) -> None:
    (
        db.query(WebhookWeightment)
        .filter(WebhookWeightment.batch_id == batch_id)
        .update(
            {WebhookWeightment.mes_timeseries_sent: True},
            synchronize_session=False,
        )
    )
    (
        db.query(RobotWeightmentRun)
        .filter(RobotWeightmentRun.batch_id == batch_id)
        .update(
            {RobotWeightmentRun.mes_timeseries_sent: True},
            synchronize_session=False,
        )
    )


def list_timeseries_pending_batch_ids(db) -> list[str]:
    rows = (
        db.query(
            WebhookWeightment.batch_id,
            func.bool_and(WebhookWeightment.weightment_completed).label('all_completed'),
            func.bool_and(WebhookWeightment.mes_timeseries_sent).label('all_sent'),
        )
        .filter(WebhookWeightment.batch_id.isnot(None))
        .group_by(WebhookWeightment.batch_id)
        .all()
    )
    return [
        str(row.batch_id)
        for row in rows
        if row.all_completed and not row.all_sent
    ]


def send_batch_timeseries_to_mes(db, batch_id: str) -> dict:
    if batch_timeseries_already_sent(db, batch_id):
        logger.info(
            'Skipping batch timeseries send: batch_id=%s already_sent=true',
            batch_id,
        )
        return {
            'ok': True,
            'skipped': True,
            'reason': 'already_sent',
            'batchId': batch_id,
            'itemCount': 0,
        }

    # Only skip when every row in the batch is mock — never poison a Condor
    # batch if a mock row somehow shared a batch_id (defensive; mock ids are
    # negative so this should not happen in practice).
    batch_rows = (
        db.query(WebhookWeightment)
        .filter(WebhookWeightment.batch_id == batch_id)
        .all()
    )
    if batch_rows and all(is_mock_event_id(item.event_id) for item in batch_rows):
        mark_batch_timeseries_sent(db, batch_id)
        db.commit()
        logger.info(
            'Skipping batch timeseries send for mock-only batch_id=%s rows=%s',
            batch_id,
            len(batch_rows),
        )
        return {
            'ok': True,
            'skipped': True,
            'reason': 'mock_event',
            'batchId': batch_id,
            'itemCount': 0,
        }

    # mes-generic null sink: never emit Condor timeseries for those batches.
    if (
        batch_rows
        and all(is_mes_generic_event_id(item.event_id) for item in batch_rows)
        and mes_generic_sink_name() == 'null'
    ):
        mark_batch_timeseries_sent(db, batch_id)
        db.commit()
        logger.info(
            'Skipping batch timeseries send for mes-generic null-sink '
            'batch_id=%s rows=%s',
            batch_id,
            len(batch_rows),
        )
        return {
            'ok': True,
            'skipped': True,
            'reason': 'mes_generic_null_sink',
            'batchId': batch_id,
            'itemCount': 0,
        }

    runs = (
        db.query(RobotWeightmentRun)
        .join(
            WebhookWeightment,
            WebhookWeightment.id == RobotWeightmentRun.weightment_id,
        )
        .filter(
            RobotWeightmentRun.batch_id == batch_id,
            RobotWeightmentRun.parquet_path.isnot(None),
            WebhookWeightment.weightment_completed.is_(True),
        )
        .order_by(RobotWeightmentRun.id.asc())
        .all()
    )
    items = build_timeseries_items_from_runs(batch_id, runs)
    if not items:
        logger.warning(
            'Batch timeseries send skipped: batch_id=%s no weight items from %s parquet files',
            batch_id,
            len(runs),
        )
        return {
            'ok': False,
            'skipped': False,
            'reason': 'no_weight_items',
            'batchId': batch_id,
            'itemCount': 0,
            'parquetFiles': len(runs),
        }

    payload = build_timeseries_payload(batch_id, items)
    # Webhook/MES-family path: bind Condor client explicitly so a later
    # runtime mode switch cannot null-route in-flight batch timeseries.
    mes_client = get_mes_client(OperatingMode.MES_CONDOR)
    try:
        response = mes_client.post_timeseries(
            payload,
            timeout_seconds=TIMESERIES_TIMEOUT_SECONDS,
        )
    except HTTPException as exc:
        logger.error(
            'Batch timeseries send failed: batch_id=%s item_count=%s detail=%s',
            batch_id,
            len(items),
            exc.detail,
        )
        return {
            'ok': False,
            'skipped': False,
            'reason': 'downstream_error',
            'batchId': batch_id,
            'itemCount': len(items),
            'detail': exc.detail,
        }

    mark_batch_timeseries_sent(db, batch_id)
    db.commit()
    logger.info(
        'Batch timeseries send succeeded: batch_id=%s item_count=%s',
        batch_id,
        len(items),
    )
    return {
        'ok': True,
        'skipped': False,
        'batchId': batch_id,
        'itemCount': len(items),
        'response': response,
    }


def send_weightment_to_mes(db, row: WebhookWeightment) -> dict:
    # Mock-local synthetic rows must never call Condor (webhook path binds
    # CondorMesClient explicitly, bypassing NullMesClient).
    if is_mock_event_id(row.event_id):
        row.weightment_completed = True
        db.commit()
        logger.info(
            'Skipping MES weighment send for mock event_id=%s weightment_id=%s',
            row.event_id,
            row.id,
        )
        return {
            'weightmentId': row.id,
            'locationId': None,
            'weighmentResponse': {
                'skipped': True,
                'reason': 'mock_event',
                'eventId': row.event_id,
            },
            'batchEndSent': False,
            'batchEndResponse': None,
            'timeseriesResult': None,
            'skippedMes': True,
        }
    # mes-generic + MES_GENERIC_SINK=null: no Condor traffic / no allocation req.
    if is_mes_generic_event_id(row.event_id) and mes_generic_sink_name() == 'null':
        row.weightment_completed = True
        timeseries_result = None
        if row.batch_id is not None:
            related = (
                db.query(WebhookWeightment)
                .filter(WebhookWeightment.batch_id == row.batch_id)
                .all()
            )
            if related and all(item.weightment_completed for item in related):
                timeseries_result = send_batch_timeseries_to_mes(
                    db, str(row.batch_id)
                )
        else:
            row.mes_timeseries_sent = True
        db.commit()
        logger.info(
            'Skipping MES weighment send for mes-generic null sink '
            'event_id=%s weightment_id=%s',
            row.event_id,
            row.id,
        )
        return {
            'weightmentId': row.id,
            'locationId': None,
            'weighmentResponse': {
                'skipped': True,
                'reason': 'mes_generic_null_sink',
                'eventId': row.event_id,
            },
            'batchEndSent': False,
            'batchEndResponse': None,
            'timeseriesResult': timeseries_result,
            'skippedMes': True,
        }
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
    # Bind client by row provenance so a runtime mode switch cannot
    # null-route Promtek rows (or Condor-route mes-generic null-sink rows).
    if is_mes_generic_event_id(row.event_id):
        mes_client = get_mes_client(OperatingMode.MES_GENERIC)
    else:
        mes_client = get_mes_client(OperatingMode.MES_CONDOR)
    weighment_response = mes_client.post_weighment(weighment_payload)

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
            batch_end_response = mes_client.post_batch_end(
                {
                    'batchId': batch_id_int,
                    'endUtc': row.end_utc,
                },
            )
            batch_end_sent = True
            try:
                timeseries_result = send_batch_timeseries_to_mes(db, str(row.batch_id))
            except Exception:
                logger.exception(
                    'Unexpected batch timeseries send failure: batch_id=%s',
                    row.batch_id,
                )
                timeseries_result = {
                    'ok': False,
                    'reason': 'unexpected_error',
                    'batchId': str(row.batch_id),
                }
        else:
            timeseries_result = None
    else:
        timeseries_result = None

    return {
        'weightmentId': row.id,
        'locationId': int(location_row.stock_item_location_id),
        'weighmentResponse': weighment_response,
        'batchEndSent': batch_end_sent,
        'batchEndResponse': batch_end_response,
        'timeseriesResult': timeseries_result,
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
    if is_mock_event_id(weightment_row.event_id) or is_mock_event_id(
        run_row.event_id
    ):
        weightment_row.weightment_completed = True
        run_row.status = 'succeeded'
        run_row.error_message = None
        run_row.mes_weighment_sent = True
        run_row.mes_batch_end_sent = False
        db.commit()
        db.refresh(run_row)
        logger.info(
            'Skipping MES send for mock run: run_id=%s weightment_id=%s event_id=%s',
            run_row.id,
            weightment_row.id,
            weightment_row.event_id,
        )
        return {
            'ok': True,
            'status': run_row.status,
            'skippedMes': True,
            'run': serialize_robot_run(run_row),
        }
    try:
        logger.info(
            'Sending processed weightment to MES: run_id=%s weightment_id=%s event_id=%s batch_id=%s',
            run_row.id,
            weightment_row.id,
            weightment_row.event_id,
            weightment_row.batch_id,
        )
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
    logger.info(
        'Processed weightment send succeeded: run_id=%s weightment_id=%s batch_end_sent=%s',
        run_row.id,
        weightment_row.id,
        bool(send_result['batchEndSent']),
    )
    next_batch_run = maybe_start_next_batch_weightment(weightment_row.event_id)
    return {
        'ok': True,
        'status': run_row.status,
        'run': serialize_robot_run(run_row),
        'sendResult': send_result,
        'nextBatchRun': next_batch_run,
    }


def complete_processed_weightment_in_background(robot_run_id: int) -> None:
    db = SessionLocal()
    try:
        logger.info(
            'Background processed completion started: robot_run_id=%s',
            robot_run_id,
        )
        run_row = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.id == robot_run_id)
            .first()
        )
        if run_row is None:
            logger.warning(
                'Background processed completion skipped: missing run robot_run_id=%s',
                robot_run_id,
            )
            return
        if run_row.status != 'awaiting_processing' or run_row.mes_weighment_sent:
            logger.info(
                'Background processed completion skipped: run_id=%s status=%s mes_sent=%s',
                run_row.id,
                run_row.status,
                run_row.mes_weighment_sent,
            )
            return
        weightment_row = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.id == run_row.weightment_id)
            .first()
        )
        if weightment_row is None:
            logger.warning(
                'Background processed completion skipped: missing weightment run_id=%s weightment_id=%s',
                run_row.id,
                run_row.weightment_id,
            )
            return
        send_processed_weightment_to_mes(db, run_row, weightment_row)
        logger.info(
            'Background processed completion finished: run_id=%s weightment_id=%s',
            run_row.id,
            weightment_row.id,
        )
    except Exception:
        db.rollback()
        logger.exception(
            'Failed to complete processed weightment run in background: run_id=%s',
            robot_run_id,
        )
    finally:
        db.close()


def schedule_processed_weightment_completion(robot_run_id: int) -> None:
    worker = threading.Thread(
        target=complete_processed_weightment_in_background,
        args=(robot_run_id,),
        name=f'processed-completion-{robot_run_id}',
        daemon=True,
    )
    worker.start()
    logger.info(
        'Started processed completion worker thread: run_id=%s thread=%s',
        robot_run_id,
        worker.name,
    )


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
    run_id = int(payload['run_id'])
    contract = ((payload.get('result_payload') or {}).get('contract')) or {}
    if run_id == LIGHTSOUT_ROSBRIDGE_RUN_ID or contract.get('kind') == 'lightsout':
        get_lightsout_session().clear()
        logger.info(
            'Cleared lightsout session after orchestrator state=%s',
            ((payload.get('result_payload') or {}).get('run_state')),
        )
        return

    db = SessionLocal()
    try:
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
        logger.info(
            'Received processed callback: run_id=%s robot_weightment_run_id=%s '
            'run_db_id=%s mcap_path=%s parquet_path=%s',
            req.run_id,
            req.robot_weightment_run_id,
            req.run_db_id,
            req.mcap_path,
            req.parquet_path,
        )
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
            logger.info(
                'Scheduling background processed completion: run_id=%s processed_id=%s weightment_id=%s',
                run_row.id,
                processed_id,
                run_row.weightment_id,
            )
            schedule_processed_weightment_completion(run_row.id)
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
    time_from: str | None = None,
    time_to: str | None = None,
    time_sort: str = 'desc',
    time_sort_field: str = 'start',
) -> dict:
    db = SessionLocal()
    try:
        safe_limit = max(limit, 0)
        safe_offset = max(offset, 0)
        start_from_ns = datetime_to_ns(parse_request_datetime(time_from))
        start_to_ns = datetime_to_ns(parse_request_datetime(time_to))
        if time_sort not in {'asc', 'desc'}:
            raise HTTPException(
                status_code=400, detail='time_sort must be "asc" or "desc"'
            )
        if time_sort_field not in {'start', 'end'}:
            raise HTTPException(
                status_code=400,
                detail='time_sort_field must be "start" or "end"',
            )

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
        if start_from_ns is not None:
            base_query = base_query.filter(
                Run.start_time_ns.is_not(None), Run.start_time_ns >= start_from_ns
            )
        if start_to_ns is not None:
            base_query = base_query.filter(
                Run.start_time_ns.is_not(None), Run.start_time_ns <= start_to_ns
            )

        total = base_query.count()

        sort_column = Run.start_time_ns if time_sort_field == 'start' else Run.end_time_ns
        if time_sort == 'asc':
            rows_query = base_query.order_by(
                sort_column.is_(None),
                sort_column.asc(),
                LightsOutProcessed.id.asc(),
            )
        else:
            rows_query = base_query.order_by(
                sort_column.is_(None),
                sort_column.desc(),
                LightsOutProcessed.id.desc(),
            )
        rows_query = rows_query.offset(safe_offset)
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
                    'overshoot_g': signed_final_error_g(
                        processed_row.target_weight_g,
                        processed_row.final_weight_g,
                        processed_row.overshoot_g,
                    ),
                    'scoop_duration_s': processed_row.scoop_duration_s,
                    'pour_duration_s': processed_row.pour_duration_s,
                    'parquet_path': processed_row.parquet_path,
                    'stop_on': processed_row.stop_on,
                    'stop_value': processed_row.stop_value,
                    'stop_reason': processed_row.stop_reason,
                    'powder_id': processed_row.powder_id,
                    'powder_name': processed_row.powder_name,
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
def list_webhook_weightment_summary(
    limit: int = 20,
    offset: int = 0,
    status: str = 'all',
    batch_query: str | None = None,
    time_from: str | None = None,
    time_to: str | None = None,
    time_sort: str = 'desc',
) -> dict:
    db = SessionLocal()
    try:
        safe_limit = max(limit, 0)
        safe_offset = max(offset, 0)
        filter_time_from = parse_request_datetime(time_from)
        filter_time_to = parse_request_datetime(time_to)
        normalized_status = status or 'all'
        if normalized_status not in {'all', 'completed', 'not_completed'}:
            raise HTTPException(
                status_code=400,
                detail='status must be "all", "completed", or "not_completed"',
            )
        if time_sort not in {'asc', 'desc'}:
            raise HTTPException(
                status_code=400, detail='time_sort must be "asc" or "desc"'
            )
        rows = (
            db.query(WebhookWeightment)
            .order_by(WebhookWeightment.id.desc())
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
        query_text = (batch_query or '').strip().lower()
        if query_text:
            data = [
                summary
                for summary in data
                if query_text in (summary.get('batch_id') or '').lower()
                or query_text in (summary.get('event_id') or '').lower()
            ]
        if normalized_status == 'completed':
            data = [summary for summary in data if summary.get('completed')]
        elif normalized_status == 'not_completed':
            data = [summary for summary in data if not summary.get('completed')]

        filtered_data = []
        for summary in data:
            sent_dt = parse_stored_datetime(summary.get('sent_utc'))
            if filter_time_from is not None and (
                sent_dt is None or sent_dt < filter_time_from
            ):
                continue
            if filter_time_to is not None and (
                sent_dt is None or sent_dt > filter_time_to
            ):
                continue
            enriched = dict(summary)
            enriched['_sent_dt'] = sent_dt
            filtered_data.append(enriched)
        data = filtered_data

        non_null_data = [summary for summary in data if summary['_sent_dt'] is not None]
        null_data = [summary for summary in data if summary['_sent_dt'] is None]
        non_null_data.sort(
            key=lambda summary: summary['_sent_dt'],
            reverse=time_sort == 'desc',
        )
        data = non_null_data + null_data
        total = len(data)
        if safe_limit > 0:
            data = data[safe_offset : safe_offset + safe_limit]
        else:
            data = data[safe_offset:]
        for summary in data:
            summary.pop('_sent_dt', None)
    finally:
        db.close()
    return {
        'rows': data,
        'total': total,
        'limit': safe_limit,
        'offset': safe_offset,
    }


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
                        signed_final_error_g(
                            processed_row.target_weight_g,
                            processed_row.final_weight_g,
                            processed_row.overshoot_g,
                        )
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
                    'mes_timeseries_sent': row.mes_timeseries_sent,
                    'robot_mes_timeseries_sent': run_row.mes_timeseries_sent
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
                    'mes_timeseries_sent': row.mes_timeseries_sent,
                    'robot_mes_timeseries_sent': run_row.mes_timeseries_sent
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


@app.get('/batches/timeseries_pending')
def get_timeseries_pending_batches() -> dict:
    db = SessionLocal()
    try:
        batch_ids = list_timeseries_pending_batch_ids(db)
        return {'batchIds': batch_ids, 'count': len(batch_ids)}
    finally:
        db.close()


@app.post('/batches/{batch_id}/send_timeseries')
def send_batch_timeseries(batch_id: str) -> dict:
    db = SessionLocal()
    try:
        rows = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.batch_id == batch_id)
            .all()
        )
        if not rows:
            raise HTTPException(status_code=404, detail='Batch not found')
        if not all(row.weightment_completed for row in rows):
            raise HTTPException(
                status_code=400,
                detail='Batch is not fully completed; cannot send timeseries yet',
            )
        result = send_batch_timeseries_to_mes(db, batch_id)
        if not result.get('ok') and not result.get('skipped'):
            raise HTTPException(
                status_code=502,
                detail=result.get('detail') or result.get('reason'),
            )
        return result
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
