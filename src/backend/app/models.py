from sqlalchemy import (
    BigInteger,
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
    batch_id = Column(String, index=True, nullable=True)
    ingredient_id = Column(String, index=True, nullable=True)
    episode_index = Column(Integer, nullable=True)
    mode = Column(String, index=True, nullable=True)
    start_time_ns = Column(BigInteger, nullable=True)
    end_time_ns = Column(BigInteger, nullable=True)

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
