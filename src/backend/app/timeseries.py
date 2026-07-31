import logging
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import pandas as pd

from .models import RobotWeightmentRun

logger = logging.getLogger('uvicorn.error')

# Condor / Promtek gateway rejects placeholder IDs like "RHAPSODI_1" with
# HTTP 400. Defaults match the Pi Condor agent appsettings
# (OrganisationId / SiteId / UserId); override per-site via env.
TIMESERIES_ORGANISATION_ID = os.environ.get(
    'TIMESERIES_ORGANISATION_ID',
    '6647A41A-8FEB-4082-86F4-3D14964161AD',
)
TIMESERIES_SITE_ID = os.environ.get(
    'TIMESERIES_SITE_ID',
    '76454DA2-683C-4706-A88E-4E2FC6773685',
)
TIMESERIES_USER_ID = os.environ.get(
    'TIMESERIES_USER_ID',
    '15de0997-1508-4f2e-91de-14745f295d9e',
)
TIMESERIES_CONTEXT_TYPE = os.environ.get('TIMESERIES_CONTEXT_TYPE', 'batch')
TIMESERIES_METRIC_WEIGHT = os.environ.get(
    'TIMESERIES_METRIC_WEIGHT', 'weight_g'
)
PATH_MAP_FROM = os.environ.get('PATH_MAP_FROM', '').strip()
PATH_MAP_TO = os.environ.get('PATH_MAP_TO', '').strip()


def remap_path(path: Path) -> Path:
    if not PATH_MAP_FROM or not PATH_MAP_TO:
        return path
    try:
        rel = path.resolve().relative_to(Path(PATH_MAP_FROM).resolve())
    except ValueError:
        return path
    return Path(PATH_MAP_TO) / rel


def ns_to_recorded_utc(log_time_ns: int) -> str:
    dt = datetime.fromtimestamp(log_time_ns / 1e9, tz=timezone.utc)
    return dt.isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def build_weight_item(
    *,
    batch_id: str,
    log_time_ns: int,
    weight_g: float,
) -> dict[str, Any]:
    return {
        'metricCode': TIMESERIES_METRIC_WEIGHT,
        'recordedUtc': ns_to_recorded_utc(int(log_time_ns)),
        'value': float(weight_g),
        'contextId': str(batch_id),
        'contextType': TIMESERIES_CONTEXT_TYPE,
    }


def build_timeseries_items_from_parquet(
    batch_id: str,
    parquet_path: str | Path,
) -> list[dict[str, Any]]:
    path = Path(parquet_path).expanduser()
    if not path.exists():
        path = remap_path(path)
    if not path.exists():
        logger.warning(
            'Skipping missing parquet for batch_id=%s path=%s',
            batch_id,
            parquet_path,
        )
        return []

    try:
        df = pd.read_parquet(path)
    except Exception:
        logger.exception(
            'Failed to read parquet for batch_id=%s path=%s',
            batch_id,
            path,
        )
        return []

    if 'log_time_ns' not in df.columns or 'weight_g' not in df.columns:
        logger.warning(
            'Parquet missing required columns for batch_id=%s path=%s '
            'columns=%s',
            batch_id,
            path,
            list(df.columns),
        )
        return []

    weight_rows = df[df['weight_g'].notna()].sort_values('log_time_ns')
    items: list[dict[str, Any]] = []
    for row in weight_rows.itertuples(index=False):
        try:
            items.append(
                build_weight_item(
                    batch_id=batch_id,
                    log_time_ns=int(row.log_time_ns),
                    weight_g=float(row.weight_g),
                )
            )
        except (TypeError, ValueError):
            continue
    return items


def build_timeseries_items_from_runs(
    batch_id: str,
    runs: list[RobotWeightmentRun],
) -> list[dict[str, Any]]:
    items: list[dict[str, Any]] = []
    seen_paths: set[str] = set()
    for run in runs:
        if not run.parquet_path:
            continue
        parquet_path = str(run.parquet_path)
        if parquet_path in seen_paths:
            continue
        seen_paths.add(parquet_path)
        items.extend(
            build_timeseries_items_from_parquet(batch_id, parquet_path)
        )
    items.sort(key=lambda item: item['recordedUtc'])
    return items


def build_timeseries_payload(
    batch_id: str, items: list[dict[str, Any]]
) -> dict[str, Any]:
    return {
        'organisationId': TIMESERIES_ORGANISATION_ID,
        'siteId': TIMESERIES_SITE_ID,
        'userId': TIMESERIES_USER_ID,
        'items': items,
    }
