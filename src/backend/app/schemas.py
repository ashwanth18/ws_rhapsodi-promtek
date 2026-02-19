from typing import Optional

from pydantic import BaseModel


class ProcessedRequest(BaseModel):
    run_db_id: Optional[int] = None
    robot_id: Optional[str] = None
    run_id: Optional[str] = None
    batch_id: Optional[str] = None
    ingredient_id: Optional[str] = None
    episode_index: Optional[int] = None
    mode: Optional[str] = None
    target_weight_g: Optional[float] = None
    baseline_weight_g: Optional[float] = None
    final_weight_g: Optional[float] = None
    net_weight_g: Optional[float] = None
    max_weight_g: Optional[float] = None
    overshoot_g: Optional[float] = None
    avg_flow_rate_g_s: Optional[float] = None
    total_episode_time_s: Optional[float] = None
    pour_duration_s: Optional[float] = None
    scoop_duration_s: Optional[float] = None
    settle_time_s: Optional[float] = None
    start_time_ns: Optional[int] = None
    end_time_ns: Optional[int] = None
    parquet_path: Optional[str] = None
    phase_events_json: Optional[str] = None
    features_json: Optional[str] = None


class ProcessedResponse(BaseModel):
    processed_id: int
    run_db_id: int
