from sqlalchemy import (
    BigInteger,
    Boolean,
    Column,
    DateTime,
    Float,
    ForeignKey,
    Integer,
    String,
    Text,
    func,
)
from sqlalchemy.orm import declarative_base, relationship


Base = declarative_base()


class Run(Base):
    __tablename__ = 'runs'

    id = Column(Integer, primary_key=True, index=True)
    robot_id = Column(String, index=True, nullable=True)
    run_id = Column(String, index=True, nullable=True)
    # Stable edge/uploader identity (run folder basename). See RunSpec.
    run_key = Column(String, index=True, nullable=True)
    batch_id = Column(String, index=True, nullable=True)
    ingredient_id = Column(String, index=True, nullable=True)
    episode_index = Column(Integer, nullable=True)
    mode = Column(String, index=True, nullable=True)
    # real | sim — orthogonal to mode (see docs/MODES.md).
    environment = Column(String, index=True, nullable=True)
    start_time_ns = Column(BigInteger, nullable=True)
    end_time_ns = Column(BigInteger, nullable=True)
    metadata_json = Column(Text, nullable=True)

    artifacts = relationship(
        'Artifact', back_populates='run', cascade='all, delete-orphan'
    )


class Artifact(Base):
    __tablename__ = 'artifacts'

    id = Column(Integer, primary_key=True, index=True)
    run_db_id = Column(Integer, ForeignKey('runs.id'), index=True)
    artifact_type = Column(String, nullable=False)
    path = Column(String, nullable=False)

    run = relationship('Run', back_populates='artifacts')


class LightsOutProcessed(Base):
    __tablename__ = 'lightsout_processed'

    id = Column(Integer, primary_key=True, index=True)
    run_db_id = Column(Integer, ForeignKey('runs.id'), index=True)
    robot_id = Column(String, index=True, nullable=True)
    run_id = Column(String, index=True, nullable=True)
    batch_id = Column(String, index=True, nullable=True)
    ingredient_id = Column(String, index=True, nullable=True)
    episode_index = Column(Integer, nullable=True)
    mode = Column(String, index=True, nullable=True)

    target_weight_g = Column(Float, nullable=True)
    baseline_weight_g = Column(Float, nullable=True)
    final_weight_g = Column(Float, nullable=True)
    net_weight_g = Column(Float, nullable=True)
    max_weight_g = Column(Float, nullable=True)
    overshoot_g = Column(Float, nullable=True)
    avg_flow_rate_g_s = Column(Float, nullable=True)
    total_episode_time_s = Column(Float, nullable=True)
    pour_duration_s = Column(Float, nullable=True)
    scoop_duration_s = Column(Float, nullable=True)
    settle_time_s = Column(Float, nullable=True)

    parquet_path = Column(String, nullable=True)
    phase_events_json = Column(Text, nullable=True)
    features_json = Column(Text, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())

    run = relationship('Run')


class WebhookWeightment(Base):
    __tablename__ = 'webhook_weightments'

    id = Column(Integer, primary_key=True, index=True)
    event_id = Column(String, index=True, nullable=False)
    sent_utc = Column(String, nullable=True)
    user_id = Column(String, nullable=True)
    site_id = Column(String, index=True, nullable=True)
    batch_id = Column(String, index=True, nullable=True)
    batch_number = Column(String, index=True, nullable=True)
    work_order_id = Column(String, index=True, nullable=True)
    batch_target_quantity = Column(Float, nullable=True)
    ingredient_id = Column(String, index=True, nullable=True)
    target_weight_kg = Column(Float, nullable=True)
    actual_weight_kg = Column(Float, nullable=True)
    weightment_completed = Column(Boolean, nullable=False, default=False)
    mes_timeseries_sent = Column(Boolean, nullable=False, default=False)
    batch_auto_run_enabled = Column(Boolean, nullable=False, default=False)
    start_utc = Column(String, nullable=True)
    end_utc = Column(String, nullable=True)
    energy_kwh = Column(Integer, nullable=True)
    lot_code = Column(String, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())


class StockLocationAllocation(Base):
    __tablename__ = 'stock_location_allocations'

    id = Column(Integer, primary_key=True, index=True)
    event_id = Column(String, index=True, nullable=False)
    context_id = Column(String, nullable=True)
    site_id = Column(String, index=True, nullable=True)
    created_utc = Column(String, nullable=True)
    stock_item_location_id = Column(Integer, index=True, nullable=True)
    stock_item_location_code = Column(String, index=True, nullable=True)
    stock_item_id = Column(String, index=True, nullable=True)
    stock_item_code = Column(String, nullable=True)
    stock_item_name = Column(String, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())


class RobotWeightmentRun(Base):
    __tablename__ = 'robot_weightment_runs'

    id = Column(Integer, primary_key=True, index=True)
    weightment_id = Column(
        Integer, ForeignKey('webhook_weightments.id'), index=True, nullable=False
    )
    event_id = Column(String, index=True, nullable=True)
    batch_id = Column(String, index=True, nullable=True)
    ingredient_id = Column(String, index=True, nullable=True)
    site_id = Column(String, index=True, nullable=True)
    stock_location_id = Column(Integer, index=True, nullable=True)
    stock_location_code = Column(String, index=True, nullable=True)
    ingredient_name = Column(String, nullable=True)
    pickup_target_name = Column(String, nullable=True)
    weigh_target_name = Column(String, nullable=True)
    return_target_name = Column(String, nullable=True)
    target_weight_kg = Column(Float, nullable=True)
    target_weight_g = Column(Float, nullable=True)
    weight_tolerance_g = Column(Float, nullable=True)
    expected_lot = Column(String, nullable=True)
    trace_run_id = Column(String, index=True, nullable=True)
    status = Column(String, index=True, nullable=False, default='starting')
    request_payload_json = Column(Text, nullable=True)
    result_payload_json = Column(Text, nullable=True)
    error_message = Column(Text, nullable=True)
    requested_at = Column(DateTime(timezone=True), server_default=func.now())
    started_at = Column(DateTime(timezone=True), nullable=True)
    finished_at = Column(DateTime(timezone=True), nullable=True)
    start_utc = Column(String, nullable=True)
    end_utc = Column(String, nullable=True)
    actual_weight_kg = Column(Float, nullable=True)
    energy_kwh = Column(Integer, nullable=True)
    processed_run_db_id = Column(Integer, ForeignKey('runs.id'), nullable=True)
    processed_id = Column(Integer, ForeignKey('lightsout_processed.id'), nullable=True)
    mcap_path = Column(String, nullable=True)
    parquet_path = Column(String, nullable=True)
    mes_weighment_sent = Column(Boolean, nullable=False, default=False)
    mes_batch_end_sent = Column(Boolean, nullable=False, default=False)
    mes_timeseries_sent = Column(Boolean, nullable=False, default=False)
