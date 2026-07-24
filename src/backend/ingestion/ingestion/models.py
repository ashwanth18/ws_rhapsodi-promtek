"""SQLAlchemy models for the Rhapsodi data-collection ingestion channel.

Deliberately its own `Base`/table set, separate from `src/backend/app`'s
MES/webhook models - this service has its own lifecycle and should stay
independently deployable, even though (per the plan) it shares the same
underlying Postgres instance rather than standing up a new database
technology.
"""
from sqlalchemy import (
    BigInteger,
    Column,
    DateTime,
    Integer,
    String,
    Text,
    UniqueConstraint,
    func,
)
from sqlalchemy.orm import declarative_base

Base = declarative_base()


class IngestedRun(Base):
    """One row per run_key the edge has synced Tier 0 for. Tier 0 content
    (metadata.json, events.jsonl) is small and queried often enough to be
    worth storing inline; features.parquet is a real file and goes to
    the object store, with only its key recorded here.
    """

    __tablename__ = 'ingested_runs'

    id = Column(Integer, primary_key=True, index=True)
    run_key = Column(String, unique=True, index=True, nullable=False)
    robot_id = Column(String, index=True, nullable=True)
    device_id = Column(String, index=True, nullable=True)
    metadata_json = Column(Text, nullable=True)
    events_jsonl = Column(Text, nullable=True)
    features_parquet_key = Column(String, nullable=True)
    tier0_received_at = Column(
        DateTime(timezone=True), server_default=func.now()
    )
    tier1_completed_at = Column(DateTime(timezone=True), nullable=True)


class IngestedBlob(Base):
    """One row per finalized Tier-1 blob (a single file out of a run's
    MCAP bag directory, or a future vision/ file). Only written once the
    blob passes its sha256 check on finalize - a blob mid-upload has no
    row here yet, only bytes sitting in the object store's `.part` file.
    """

    __tablename__ = 'ingested_blobs'
    __table_args__ = (UniqueConstraint('run_key', 'blob_key'),)

    id = Column(Integer, primary_key=True, index=True)
    run_key = Column(String, index=True, nullable=False)
    blob_key = Column(String, nullable=False)
    object_store_key = Column(String, nullable=False)
    size_bytes = Column(BigInteger, nullable=True)
    sha256 = Column(String, nullable=True)
    finalized_at = Column(DateTime(timezone=True), server_default=func.now())


class FleetHealthEvent(Base):
    """One row per line the fleet-wide health.jsonl uplink has sent -
    the queryable, cross-device counterpart to each Pi's local
    health.jsonl/events.jsonl, and the input `incident-detection`'s rules
    engine reads from.
    """

    __tablename__ = 'fleet_health_events'

    id = Column(Integer, primary_key=True, index=True)
    device_id = Column(String, index=True, nullable=False)
    stamp_sec = Column(BigInteger, nullable=True)
    stamp_nanosec = Column(BigInteger, nullable=True)
    component = Column(String, index=True, nullable=True)
    severity = Column(String, index=True, nullable=True)
    code = Column(String, index=True, nullable=True)
    message = Column(Text, nullable=True)
    context_json = Column(Text, nullable=True)
    received_at = Column(DateTime(timezone=True), server_default=func.now())
