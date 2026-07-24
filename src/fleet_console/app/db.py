from __future__ import annotations

import os
from datetime import datetime, timezone

from sqlalchemy import (
    Boolean,
    Column,
    DateTime,
    ForeignKey,
    Integer,
    String,
    Text,
    create_engine,
)
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


class Release(Base):
    """Verified build reported by CI (or manually). Only unit agents may apply."""

    __tablename__ = 'releases'

    id = Column(Integer, primary_key=True, autoincrement=True)
    branch = Column(String(256), nullable=False, index=True)
    git_sha = Column(String(64), nullable=False, index=True)
    status = Column(String(32), nullable=False, default='success')
    # success | failed
    images = Column(Text, nullable=True)  # JSON: {role: full-tag, ...}
    image_registry = Column(String(256), nullable=True)
    workflow_run_url = Column(String(512), nullable=True)
    error_message = Column(Text, nullable=True)
    duration_seconds = Column(Integer, nullable=True)
    timings = Column(Text, nullable=True)  # JSON: build-timings.json payload
    reported_at = Column(DateTime, nullable=False, default=utc_now)


class Deployment(Base):
    __tablename__ = 'deployments'

    id = Column(Integer, primary_key=True, autoincrement=True)
    device_id = Column(String(128), nullable=False, index=True)
    action = Column(String(32), nullable=False)
    # provision | deploy | build | reconcile
    robot_type = Column(String(64), nullable=True)
    site_id = Column(String(64), nullable=True)
    profile_id = Column(String(128), nullable=True)
    tracked_branch = Column(String(256), nullable=True)
    release_id = Column(Integer, ForeignKey('releases.id'), nullable=True)
    image_tag = Column(String(64), nullable=False, default='')
    status = Column(String(32), nullable=False, default='running')
    # running | success | failed | rolled_back | applying | converged
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
    release_id = Column(Integer, ForeignKey('releases.id'), nullable=True)
    auto_update = Column(Boolean, nullable=False, default=False)
    robot_type = Column(String(64), nullable=True)
    site_id = Column(String(64), nullable=True)
    agent_token = Column(String(128), nullable=True, unique=True, index=True)
    # Last agent self-report
    agent_status = Column(String(64), nullable=True)
    agent_message = Column(Text, nullable=True)
    agent_applied_release_id = Column(Integer, ForeignKey('releases.id'), nullable=True)
    agent_reported_at = Column(DateTime, nullable=True)
    updated_at = Column(DateTime, nullable=False, default=utc_now)


def init_db() -> None:
    Base.metadata.create_all(bind=engine)
    # Lightweight SQLite migration for columns added after first ship.
    with engine.connect() as conn:
        cols = {
            row[1]
            for row in conn.exec_driver_sql('PRAGMA table_info(device_targets)').fetchall()
        }
        migrations = []
        if 'release_id' not in cols:
            migrations.append(
                'ALTER TABLE device_targets ADD COLUMN release_id INTEGER'
            )
        if 'agent_token' not in cols:
            migrations.append(
                'ALTER TABLE device_targets ADD COLUMN agent_token VARCHAR(128)'
            )
        if 'agent_status' not in cols:
            migrations.append(
                'ALTER TABLE device_targets ADD COLUMN agent_status VARCHAR(64)'
            )
        if 'agent_message' not in cols:
            migrations.append(
                'ALTER TABLE device_targets ADD COLUMN agent_message TEXT'
            )
        if 'agent_applied_release_id' not in cols:
            migrations.append(
                'ALTER TABLE device_targets ADD COLUMN agent_applied_release_id INTEGER'
            )
        if 'agent_reported_at' not in cols:
            migrations.append(
                'ALTER TABLE device_targets ADD COLUMN agent_reported_at DATETIME'
            )
        # Drop legacy pinned_image_tag usage by leaving the column if present
        # (SQLite cannot DROP COLUMN reliably on older versions).
        for stmt in migrations:
            conn.exec_driver_sql(stmt)
        conn.commit()

        dep_cols = {
            row[1]
            for row in conn.exec_driver_sql('PRAGMA table_info(deployments)').fetchall()
        }
        if 'release_id' not in dep_cols:
            conn.exec_driver_sql(
                'ALTER TABLE deployments ADD COLUMN release_id INTEGER'
            )
            conn.commit()

        rel_cols = {
            row[1]
            for row in conn.exec_driver_sql('PRAGMA table_info(releases)').fetchall()
        }
        if 'duration_seconds' not in rel_cols:
            conn.exec_driver_sql(
                'ALTER TABLE releases ADD COLUMN duration_seconds INTEGER'
            )
            conn.commit()
        if 'timings' not in rel_cols:
            conn.exec_driver_sql('ALTER TABLE releases ADD COLUMN timings TEXT')
            conn.commit()
