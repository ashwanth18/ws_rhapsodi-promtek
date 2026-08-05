import json
from datetime import datetime, timezone
from typing import Any, Dict, Optional

from sqlalchemy.orm import Session

from .models import (
    Artifact,
    LightsOutProcessed,
    RobotWeightmentRun,
    Run,
    WebhookWeightment,
)


def _metadata_field(payload: Dict[str, Any], key: str) -> Optional[Any]:
    """Pull a field from top-level payload or nested metadata_json."""
    if payload.get(key) is not None:
        return payload.get(key)
    raw = payload.get('metadata_json')
    if not raw:
        return None
    try:
        meta = json.loads(raw) if isinstance(raw, str) else raw
    except (TypeError, ValueError, json.JSONDecodeError):
        return None
    if isinstance(meta, dict):
        return meta.get(key)
    return None


def _upsert_artifact(
    db: Session, run_db_id: int, artifact_type: str, path: str | None
) -> None:
    if not path:
        return
    artifact = (
        db.query(Artifact)
        .filter(
            Artifact.run_db_id == run_db_id,
            Artifact.artifact_type == artifact_type,
        )
        .first()
    )
    if artifact is None:
        artifact = Artifact(
            run_db_id=run_db_id,
            artifact_type=artifact_type,
            path=path,
        )
        db.add(artifact)
    else:
        artifact.path = path


def _ns_to_iso_utc(value: int | None) -> str | None:
    if value is None:
        return None
    try:
        return datetime.fromtimestamp(value / 1e9, tz=timezone.utc).isoformat()
    except (OverflowError, OSError, ValueError, TypeError):
        return None


def _ns_to_datetime_utc(value: int | None) -> datetime | None:
    if value is None:
        return None
    try:
        return datetime.fromtimestamp(value / 1e9, tz=timezone.utc)
    except (OverflowError, OSError, ValueError, TypeError):
        return None


def store_processed_run(
    db: Session, payload: Dict[str, Any]
) -> LightsOutProcessed:
    run = None
    if payload.get('run_db_id'):
        run = db.query(Run).filter(Run.id == payload['run_db_id']).first()
    run_key = _metadata_field(payload, 'run_key')
    environment = _metadata_field(payload, 'environment')
    mode = payload.get('mode') or _metadata_field(payload, 'mode')
    label_fields = {
        'powder_id': payload.get('powder_id') or _metadata_field(payload, 'powder_id'),
        'powder_name': payload.get('powder_name')
        or _metadata_field(payload, 'powder_name'),
        'lot_code': payload.get('lot_code') or _metadata_field(payload, 'lot_code'),
        'operator': payload.get('operator') or _metadata_field(payload, 'operator'),
        'notes': payload.get('notes') or _metadata_field(payload, 'notes'),
        'episodes_total': payload.get('episodes_total')
        or _metadata_field(payload, 'episodes_total'),
        'scooped_mass_g': payload.get('scooped_mass_g')
        or _metadata_field(payload, 'scooped_mass_g'),
        'target_mode': payload.get('target_mode')
        or _metadata_field(payload, 'target_mode'),
        'target_fraction': payload.get('target_fraction')
        or _metadata_field(payload, 'target_fraction'),
        'pour_outcome': payload.get('pour_outcome')
        or _metadata_field(payload, 'pour_outcome'),
        'rng_seed': payload.get('rng_seed') or _metadata_field(payload, 'rng_seed'),
        'stop_on': payload.get('stop_on') or _metadata_field(payload, 'stop_on'),
        'stop_value': payload.get('stop_value')
        if payload.get('stop_value') is not None
        else _metadata_field(payload, 'stop_value'),
        'stop_reason': payload.get('stop_reason')
        or _metadata_field(payload, 'stop_reason'),
        'layout_id': payload.get('layout_id') or _metadata_field(payload, 'layout_id'),
        'layout_hash': payload.get('layout_hash')
        or _metadata_field(payload, 'layout_hash'),
        'poses_hash': payload.get('poses_hash') or _metadata_field(payload, 'poses_hash'),
        'tool_id': payload.get('tool_id') or _metadata_field(payload, 'tool_id'),
        'authored_in': payload.get('authored_in')
        or _metadata_field(payload, 'authored_in'),
    }
    if not run:
        run = Run(
            robot_id=payload.get('robot_id'),
            run_id=payload.get('run_id'),
            run_key=run_key,
            batch_id=payload.get('batch_id'),
            ingredient_id=payload.get('ingredient_id'),
            episode_index=payload.get('episode_index'),
            mode=mode,
            environment=environment,
            start_time_ns=payload.get('start_time_ns'),
            end_time_ns=payload.get('end_time_ns'),
            metadata_json=payload.get('metadata_json'),
            **label_fields,
        )
        db.add(run)
        db.flush()
    else:
        run.robot_id = payload.get('robot_id') or run.robot_id
        run.run_id = payload.get('run_id') or run.run_id
        run.run_key = run_key or run.run_key
        run.batch_id = payload.get('batch_id') or run.batch_id
        run.ingredient_id = payload.get('ingredient_id') or run.ingredient_id
        run.episode_index = payload.get('episode_index')
        run.mode = mode or run.mode
        run.environment = environment or run.environment
        run.start_time_ns = payload.get('start_time_ns') or run.start_time_ns
        run.end_time_ns = payload.get('end_time_ns') or run.end_time_ns
        run.metadata_json = payload.get('metadata_json') or run.metadata_json
        for key, value in label_fields.items():
            if value is not None:
                setattr(run, key, value)
    processed = LightsOutProcessed(
        run_db_id=run.id,
        robot_id=payload.get('robot_id'),
        run_id=payload.get('run_id'),
        batch_id=payload.get('batch_id'),
        ingredient_id=payload.get('ingredient_id'),
        episode_index=payload.get('episode_index'),
        mode=payload.get('mode'),
        target_weight_g=payload.get('target_weight_g'),
        baseline_weight_g=payload.get('baseline_weight_g'),
        final_weight_g=payload.get('final_weight_g'),
        net_weight_g=payload.get('net_weight_g'),
        max_weight_g=payload.get('max_weight_g'),
        overshoot_g=payload.get('overshoot_g'),
        avg_flow_rate_g_s=payload.get('avg_flow_rate_g_s'),
        total_episode_time_s=payload.get('total_episode_time_s'),
        pour_duration_s=payload.get('pour_duration_s'),
        scoop_duration_s=payload.get('scoop_duration_s'),
        settle_time_s=payload.get('settle_time_s'),
        parquet_path=payload.get('parquet_path'),
        phase_events_json=payload.get('phase_events_json'),
        features_json=payload.get('features_json'),
        **label_fields,
    )
    db.add(processed)
    db.flush()
    _upsert_artifact(db, run.id, 'mcap', payload.get('mcap_path'))
    _upsert_artifact(db, run.id, 'parquet', payload.get('parquet_path'))
    robot_weightment_run_id = payload.get('robot_weightment_run_id')
    robot_run = None
    if robot_weightment_run_id:
        robot_run = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.id == robot_weightment_run_id)
            .first()
        )
    elif payload.get('run_id'):
        robot_run = (
            db.query(RobotWeightmentRun)
            .filter(RobotWeightmentRun.trace_run_id == payload.get('run_id'))
            .first()
        )
    if robot_run is not None:
        robot_run.trace_run_id = payload.get('run_id') or robot_run.trace_run_id
        robot_run.processed_run_db_id = run.id
        robot_run.processed_id = processed.id
        robot_run.mcap_path = payload.get('mcap_path') or robot_run.mcap_path
        robot_run.parquet_path = payload.get('parquet_path') or robot_run.parquet_path
        processed_start_utc = _ns_to_iso_utc(run.start_time_ns)
        processed_end_utc = _ns_to_iso_utc(run.end_time_ns)
        processed_started_at = _ns_to_datetime_utc(run.start_time_ns)
        processed_finished_at = _ns_to_datetime_utc(run.end_time_ns)
        if processed_start_utc is not None:
            robot_run.start_utc = processed_start_utc
        if processed_end_utc is not None:
            robot_run.end_utc = processed_end_utc
        if processed_started_at is not None:
            robot_run.started_at = processed_started_at
        if processed_finished_at is not None:
            robot_run.finished_at = processed_finished_at
        final_weight_g = payload.get('final_weight_g')
        webhook_weightment = (
            db.query(WebhookWeightment)
            .filter(WebhookWeightment.id == robot_run.weightment_id)
            .first()
        )
        if webhook_weightment is not None:
            if processed_start_utc is not None:
                webhook_weightment.start_utc = processed_start_utc
            if processed_end_utc is not None:
                webhook_weightment.end_utc = processed_end_utc
        if final_weight_g is not None:
            actual_weight_kg = float(final_weight_g) / 1000.0
            robot_run.actual_weight_kg = actual_weight_kg
            if webhook_weightment is not None:
                webhook_weightment.actual_weight_kg = actual_weight_kg
    db.commit()
    db.refresh(processed)
    return processed
