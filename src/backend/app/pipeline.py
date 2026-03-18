from datetime import datetime, timezone
from typing import Any, Dict

from sqlalchemy.orm import Session

from .models import (
    Artifact,
    LightsOutProcessed,
    RobotWeightmentRun,
    Run,
    WebhookWeightment,
)


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
    if not run:
        run = Run(
            robot_id=payload.get('robot_id'),
            run_id=payload.get('run_id'),
            batch_id=payload.get('batch_id'),
            ingredient_id=payload.get('ingredient_id'),
            episode_index=payload.get('episode_index'),
            mode=payload.get('mode'),
            start_time_ns=payload.get('start_time_ns'),
            end_time_ns=payload.get('end_time_ns'),
            metadata_json=payload.get('metadata_json'),
        )
        db.add(run)
        db.flush()
    else:
        run.robot_id = payload.get('robot_id') or run.robot_id
        run.run_id = payload.get('run_id') or run.run_id
        run.batch_id = payload.get('batch_id') or run.batch_id
        run.ingredient_id = payload.get('ingredient_id') or run.ingredient_id
        run.episode_index = payload.get('episode_index')
        run.mode = payload.get('mode') or run.mode
        run.start_time_ns = payload.get('start_time_ns') or run.start_time_ns
        run.end_time_ns = payload.get('end_time_ns') or run.end_time_ns
        run.metadata_json = payload.get('metadata_json') or run.metadata_json
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
