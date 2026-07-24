import os

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

# Shares the same Postgres instance as src/backend/app (extending the
# existing SQLAlchemy setup per the plan, not introducing a new database
# technology) but its own tables/Base (ingestion.models) and its own
# deployable service - "fully separate from condor_agent/MES" is a
# service-boundary decision, not a database one.
DATABASE_URL = os.environ.get(
    'DATABASE_URL',
    'postgresql://postgres:postgres@db:5432/robot_data',
)

engine = create_engine(DATABASE_URL, pool_pre_ping=True)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
