import json
import os
from pathlib import Path
from typing import Any, Dict

import requests
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel


BACKEND_URL = os.environ.get('BACKEND_URL', 'http://backend:8000')
PROCESSING_URL = os.environ.get('PROCESSING_URL', 'http://processing:8002')


class IngestFolderRequest(BaseModel):
    run_folder: str


app = FastAPI(title='Rhapsodi Ingestion Service')


def _load_metadata(run_folder: str) -> Dict[str, Any]:
    folder = Path(run_folder).expanduser().resolve()
    metadata_path = folder / 'metadata.json'
    if not metadata_path.exists():
        raise FileNotFoundError(f'metadata.json not found in {folder}')
    metadata_json = metadata_path.read_text()
    metadata = json.loads(metadata_json or '{}')
    bag_path = metadata.get('bag_path') or str(folder / 'data')
    return {
        'metadata_json': metadata_json,
        'bag_path': bag_path,
        'metadata_path': str(metadata_path),
    }


@app.post('/ingest')
def ingest(req: IngestFolderRequest) -> Dict[str, Any]:
    try:
        payload = _load_metadata(req.run_folder)
    except FileNotFoundError as exc:
        raise HTTPException(status_code=404, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc

    try:
        resp = requests.post(f'{BACKEND_URL}/ingest', json=payload, timeout=10)
        resp.raise_for_status()
        ingest_result = resp.json()
    except Exception as exc:
        raise HTTPException(status_code=502, detail=str(exc)) from exc

    run_db_id = ingest_result.get('run_db_id')
    if run_db_id:
        try:
            requests.post(
                f'{PROCESSING_URL}/process',
                json={'run_db_id': run_db_id, 'run_folder': req.run_folder},
                timeout=10,
            ).raise_for_status()
        except Exception as exc:
            raise HTTPException(status_code=502, detail=str(exc)) from exc
    return ingest_result


