from typing import Any, Dict

from sqlalchemy.orm import Session

from .models import LightsOutProcessed, Run


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
        )
        db.add(run)
        db.flush()
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
    db.commit()
    db.refresh(processed)
    return processed
