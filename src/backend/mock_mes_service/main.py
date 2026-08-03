import json
import logging
import os
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from fastapi import FastAPI


logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"))
logger = logging.getLogger("mock_mes_service")

DATA_ROOT = Path(os.environ.get("DATA_ROOT", "/data")) / "mes_mock"
DATA_ROOT.mkdir(parents=True, exist_ok=True)

app = FastAPI(title="Mock MES Service")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def append_jsonl(filename: str, payload: dict[str, Any]) -> None:
    path = DATA_ROOT / filename
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(payload) + "\n")


@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}


@app.post("/batch/weighment")
def batch_weighment(payload: dict[str, Any]) -> dict[str, Any]:
    logger.info("Received mock /batch/weighment payload: %s", payload)
    append_jsonl(
        "weighment.jsonl",
        {
            "receivedAt": utc_now(),
            "payload": payload,
        },
    )
    return {
        "accepted": True,
        "endpoint": "batch/weighment",
        "receivedAt": utc_now(),
    }


@app.post("/batch/end")
def batch_end(payload: dict[str, Any]) -> dict[str, Any]:
    logger.info("Received mock /batch/end payload: %s", payload)
    append_jsonl(
        "batch_end.jsonl",
        {
            "receivedAt": utc_now(),
            "payload": payload,
        },
    )
    return {
        "accepted": True,
        "endpoint": "batch/end",
        "receivedAt": utc_now(),
    }


@app.post("/timeseries")
def timeseries(payload: dict[str, Any]) -> dict[str, Any]:
    # Condor contract uses `metrics`; accept legacy `items` in mock too.
    item_count = len(payload.get("metrics") or payload.get("items") or [])
    logger.info(
        "Received mock /timeseries payload: item_count=%s payload=%s",
        item_count,
        payload,
    )
    append_jsonl(
        "timeseries.jsonl",
        {
            "receivedAt": utc_now(),
            "itemCount": item_count,
            "payload": payload,
        },
    )
    return {
        "accepted": True,
        "endpoint": "timeseries",
        "itemCount": item_count,
        "receivedAt": utc_now(),
    }
