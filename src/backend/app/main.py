import json
import os

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from .database import engine, SessionLocal
from .models import Base, LightsOutProcessed, Run
from .pipeline import store_processed_run
from .schemas import ProcessedRequest, ProcessedResponse

Base.metadata.create_all(bind=engine)

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


@app.get('/health')
def health() -> dict:
    return {'status': 'ok'}


@app.post('/processed', response_model=ProcessedResponse)
def processed(req: ProcessedRequest) -> ProcessedResponse:
    db = SessionLocal()
    try:
        processed_row = store_processed_run(db, req.model_dump())
    except Exception as exc:
        db.rollback()
        raise HTTPException(status_code=500, detail=str(exc)) from exc
    finally:
        db.close()
    return ProcessedResponse(
        processed_id=processed_row.id, run_db_id=processed_row.run_db_id
    )


@app.get('/lightsout_processed')
def list_lightsout_processed(limit: int = 50) -> dict:
    db = SessionLocal()
    try:
        rows = (
            db.query(LightsOutProcessed, Run)
            .join(Run, LightsOutProcessed.run_db_id == Run.id)
            .order_by(LightsOutProcessed.id.desc())
            .limit(limit)
            .all()
        )
        data = []
        for processed_row, run_row in rows:
            data.append(
                {
                    'id': processed_row.id,
                    'run_db_id': processed_row.run_db_id,
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
    return {'rows': data}
