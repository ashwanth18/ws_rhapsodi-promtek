from typing import Any, Optional

from pydantic import BaseModel


class RuntimeModeRequest(BaseModel):
    mode: str
    environment: str = 'real'


class MockLocalRunRequest(BaseModel):
    """Operator-triggered single-location test run (mock-local mode)."""

    target_weight_g: float
    tolerance_g: Optional[float] = None
    location_code: Optional[str] = None
    pickup_target_name: Optional[str] = None
    weigh_target_name: Optional[str] = None
    return_target_name: Optional[str] = None


class LightsoutRunRequest(BaseModel):
    """Operator-triggered lights-out training run (lightsout mode)."""

    powder_name: str
    target_weight_g: float
    episodes: int
    batch_id: str = ''
    cycle_end_limit: str = ''
    # Opt-in: LightsOut tree skips ExecuteScoop unless true (safe default).
    enable_scoop: bool = False


class ProcessedRequest(BaseModel):
    run_db_id: Optional[int] = None
    robot_weightment_run_id: Optional[int] = None
    robot_id: Optional[str] = None
    run_id: Optional[str] = None
    run_key: Optional[str] = None
    batch_id: Optional[str] = None
    ingredient_id: Optional[str] = None
    weightment_id: Optional[str] = None
    location_id: Optional[str] = None
    location_code: Optional[str] = None
    episode_index: Optional[int] = None
    mode: Optional[str] = None
    environment: Optional[str] = None
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
    mcap_path: Optional[str] = None
    parquet_path: Optional[str] = None
    metadata_json: Optional[str] = None
    phase_events_json: Optional[str] = None
    features_json: Optional[str] = None


class ProcessedResponse(BaseModel):
    processed_id: int
    run_db_id: int


class RobotRunCompletionRequest(BaseModel):
    success: bool
    actual_weight_kg: Optional[float] = None
    final_scale_weight_g: Optional[float] = None
    start_utc: Optional[str] = None
    end_utc: Optional[str] = None
    energy_kwh: Optional[int] = 0
    error_message: Optional[str] = None
    result_payload: Optional[dict[str, Any]] = None
