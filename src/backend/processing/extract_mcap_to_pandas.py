import base64
import json
import os
from pathlib import Path
from typing import Any, Dict, List, Tuple

import pandas as pd
import psycopg2
from mcap.reader import make_reader


def _db_connect() -> psycopg2.extensions.connection:
    dsn = os.environ.get(
        'DATABASE_URL',
        'postgresql://postgres:postgres@localhost:5432/robot_data',
    )
    return psycopg2.connect(dsn)


def _find_mcap_file(bag_path: Path) -> Path:
    if bag_path.is_file():
        return bag_path
    if not bag_path.exists():
        raise FileNotFoundError(f'Bag path not found: {bag_path}')
    mcap_files = sorted(bag_path.glob('*.mcap'))
    if not mcap_files:
        raise FileNotFoundError(f'No .mcap files found in {bag_path}')
    return mcap_files[0]


def _load_run_metadata(run_db_id: int) -> Tuple[Dict[str, Any], str]:
    with _db_connect() as conn:
        with conn.cursor() as cur:
            cur.execute(
                """
                SELECT runs.metadata_json, artifacts.path
                FROM runs
                JOIN artifacts ON artifacts.run_db_id = runs.id
                WHERE runs.id = %s AND artifacts.artifact_type = 'mcap'
                """,
                (run_db_id,),
            )
            row = cur.fetchone()
            if not row:
                raise RuntimeError(f'Run {run_db_id} not found')
            metadata_json, bag_path = row
    metadata = json.loads(metadata_json or '{}')
    return metadata, bag_path


def extract_run(run_db_id: int) -> pd.DataFrame:
    metadata, bag_path = _load_run_metadata(run_db_id)
    bag = _find_mcap_file(Path(bag_path))

    rows: List[Dict[str, Any]] = []
    with open(bag, 'rb') as f:
        reader = make_reader(f)
        for schema, channel, message in reader.iter_messages():
            rows.append(
                {
                    'topic': channel.topic,
                    'schema': schema.name if schema else None,
                    'log_time_ns': message.log_time,
                    'publish_time_ns': message.publish_time,
                    'payload_base64': base64.b64encode(
                        message.data
                    ).decode('ascii'),
                }
            )

    df = pd.DataFrame(rows)
    for k, v in metadata.items():
        df[f'meta_{k}'] = json.dumps(v) if isinstance(v, dict) else v
    return df


if __name__ == '__main__':
    run_id = int(os.environ.get('RUN_DB_ID', '1'))
    out_path = os.environ.get('OUT_PATH', f'run_{run_id}.parquet')
    df = extract_run(run_id)
    df.to_parquet(out_path, index=False)
    print(f'Wrote {len(df)} rows to {out_path}')
