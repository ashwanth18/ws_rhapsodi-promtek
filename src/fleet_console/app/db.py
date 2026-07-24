from __future__ import annotations

import os
from datetime import datetime, timezone

from sqlalchemy import Boolean, Column, DateTime, Integer, String, Text, create_engine
from sqlalchemy.orm import declarative_base, sessionmaker

DB_PATH = os.environ.get(
    'FLEET_DB_PATH',
    os.path.join(os.environ.get('FLEET_DATA_DIR', '/data'), 'fleet.db'),
)
os.makedirs(os.path.dirname(DB_PATH) or '.', exist_ok=True)

engine = create_engine(f'sqlite:///{DB_PATH}', connect_args={'check_same_thread': False})
SessionLocal = sessionmaker(bind=engine, autoflush=False, autocommit=False)
Base = declarative_base()


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


class Deployment(Base):
    __tablename__ = 'deployments'

    id = Column(Integer, primary_key=True, autoincrement=True)
    device_id = Column(String(128), nullable=False, index=True)
    action = Column(String(32), nullable=False)  # provision | deploy | build
    robot_type = Column(String(64), nullable=True)
    site_id = Column(String(64), nullable=True)
    profile_id = Column(String(128), nullable=True)
    tracked_branch = Column(String(256), nullable=True)
    image_tag = Column(String(64), nullable=False, default='')
    status = Column(String(32), nullable=False, default='running')
    # running | success | failed | rolled_back
    requested_by = Column(String(128), nullable=True)
    log_path = Column(String(512), nullable=True)
    error_message = Column(Text, nullable=True)
    started_at = Column(DateTime, nullable=False, default=utc_now)
    finished_at = Column(DateTime, nullable=True)


class DeviceTarget(Base):
    """Desired state for a device (independent of Tailscale inventory)."""

    __tablename__ = 'device_targets'

    device_id = Column(String(128), primary_key=True)
    tracked_branch = Column(String(256), nullable=False, default='main')
    profile_id = Column(String(128), nullable=False, default='prod-niryo')
    pinned_image_tag = Column(String(64), nullable=True)
    auto_update = Column(Boolean, nullable=False, default=False)
    robot_type = Column(String(64), nullable=True)
    site_id = Column(String(64), nullable=True)
    updated_at = Column(DateTime, nullable=False, default=utc_now)


def init_db() -> None:
    Base.metadata.create_all(bind=engine)
