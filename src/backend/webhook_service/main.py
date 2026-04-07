import json
import logging
import os
from datetime import datetime, timezone
from typing import Any

from fastapi import FastAPI
from sqlalchemy import func

from database import SessionLocal, engine
from models import Base, StockLocationAllocation, WebhookWeightment


logging.basicConfig(level=os.environ.get("LOG_LEVEL", "INFO"))
logger = logging.getLogger("webhook_service")

Base.metadata.create_all(bind=engine)

app = FastAPI(title="Rhapsodi Webhook Service")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def log_request(event_type: str, payload: dict[str, Any]) -> None:
    logger.info(
        "%s request body:\n%s",
        event_type,
        json.dumps(payload, indent=2),
    )


def to_float(value: Any) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def extract_parameter_target(
    parameters: list[dict[str, Any]], parameter_code: str
) -> Any | None:
    for parameter in parameters:
        code = parameter.get("batchInstructionParameterCode") or parameter.get(
            "BatchInstructionParameterCode"
        )
        if code == parameter_code:
            return parameter.get("target") or parameter.get("Target")
    return None


def build_weightments(payload: dict[str, Any]) -> list[WebhookWeightment]:
    batch = payload.get("batch") or {}
    lines = batch.get("lines") or []

    event_id = payload.get("eventId") or payload.get("EventId")
    sent_utc = payload.get("sentUtc") or payload.get("SentUtc")
    user_id = payload.get("userId") or payload.get("UserId")
    site_id = payload.get("siteId") or payload.get("SiteId")

    batch_id = batch.get("id")
    batch_number = batch.get("batchNumber")
    work_order_id = batch.get("workOrderId")
    batch_target_quantity = to_float(batch.get("targetQuantity"))

    weightments: list[WebhookWeightment] = []
    for line in lines:
        parameters = line.get("parameters") or []
        ingredient_id = extract_parameter_target(parameters, "Ingredient")
        target_weight_kg = to_float(extract_parameter_target(parameters, "Quantity"))
        weightments.append(
            WebhookWeightment(
                event_id=str(event_id) if event_id is not None else "",
                sent_utc=str(sent_utc) if sent_utc is not None else None,
                user_id=str(user_id) if user_id is not None else None,
                site_id=str(site_id) if site_id is not None else None,
                batch_id=str(batch_id) if batch_id is not None else None,
                batch_number=str(batch_number) if batch_number is not None else None,
                work_order_id=str(work_order_id)
                if work_order_id is not None
                else None,
                batch_target_quantity=batch_target_quantity,
                ingredient_id=str(ingredient_id)
                if ingredient_id is not None
                else None,
                target_weight_kg=target_weight_kg,
                weightment_completed=False,
                batch_auto_run_enabled=False,
            )
        )
    return weightments


def build_stock_location_allocations(
    payload: dict[str, Any],
) -> list[StockLocationAllocation]:
    event_id = payload.get("eventId") or payload.get("EventId")
    context_id = payload.get("contextId") or payload.get("ContextId")
    site_id = payload.get("siteId") or payload.get("SiteId")
    created_utc = payload.get("createdUtc") or payload.get("CreatedUtc")
    location_id = payload.get("stockItemLocationId") or payload.get("StockItemLocationId")
    location_code = payload.get("stockItemLocationCode") or payload.get(
        "StockItemLocationCode"
    )
    allocated_stock_items = payload.get("allocatedStockItems") or payload.get(
        "AllocatedStockItems"
    ) or []

    rows: list[StockLocationAllocation] = []
    for stock_item in allocated_stock_items:
        stock_item_id = stock_item.get("id") or stock_item.get("Id")
        stock_item_code = stock_item.get("code") or stock_item.get("Code")
        stock_item_name = stock_item.get("name") or stock_item.get("Name")
        rows.append(
            StockLocationAllocation(
                event_id=str(event_id) if event_id is not None else "",
                context_id=str(context_id) if context_id is not None else None,
                site_id=str(site_id) if site_id is not None else None,
                created_utc=str(created_utc) if created_utc is not None else None,
                stock_item_location_id=int(location_id)
                if location_id is not None
                else None,
                stock_item_location_code=str(location_code)
                if location_code is not None
                else None,
                stock_item_id=str(stock_item_id)
                if stock_item_id is not None
                else None,
                stock_item_code=str(stock_item_code)
                if stock_item_code is not None
                else None,
                stock_item_name=str(stock_item_name)
                if stock_item_name is not None
                else None,
            )
        )
    return rows


@app.get("/health")
def health() -> dict[str, str]:
    return {"status": "ok"}


@app.post("/api/BatchReleasedEvent")
def batch_released_event(payload: dict[str, Any]) -> dict[str, Any]:
    event_id = payload.get("eventId") or payload.get("EventId")
    logger.info(
        "Received BatchReleasedEvent event_id=%s",
        event_id,
    )
    log_request("BatchReleasedEvent", payload)
    weightments = build_weightments(payload)
    inserted_rows = 0
    db = SessionLocal()
    try:
        existing_count = 0
        if event_id:
            existing_count = (
                db.query(func.count(WebhookWeightment.id))
                .filter(WebhookWeightment.event_id == str(event_id))
                .scalar()
            )
        if existing_count:
            logger.info(
                "Skipping duplicate BatchReleasedEvent event_id=%s existing_rows=%s",
                event_id,
                existing_count,
            )
            return {
                "accepted": True,
                "duplicate": True,
                "eventType": "BatchReleasedEvent",
                "receivedAt": utc_now(),
                "insertedRows": 0,
            }

        if weightments:
            db.add_all(weightments)
            db.commit()
            inserted_rows = len(weightments)
        else:
            logger.info("No weightment rows parsed for event_id=%s", event_id)
        logger.info(
            "Stored BatchReleasedEvent event_id=%s inserted_rows=%s",
            event_id,
            inserted_rows,
        )
    finally:
        db.close()
    return {
        "accepted": True,
        "duplicate": False,
        "eventType": "BatchReleasedEvent",
        "receivedAt": utc_now(),
        "insertedRows": inserted_rows,
    }


@app.post("/api/StockItemLocationAllocatedEvent")
def stock_item_location_allocated_event(
    payload: dict[str, Any],
) -> dict[str, Any]:
    event_id = payload.get("eventId") or payload.get("EventId")
    logger.info(
        "Received StockItemLocationAllocatedEvent event_id=%s",
        event_id,
    )
    log_request("StockItemLocationAllocatedEvent", payload)
    rows = build_stock_location_allocations(payload)
    inserted_rows = 0
    db = SessionLocal()
    try:
        existing_count = 0
        if event_id:
            existing_count = (
                db.query(func.count(StockLocationAllocation.id))
                .filter(StockLocationAllocation.event_id == str(event_id))
                .scalar()
            )
        if existing_count:
            logger.info(
                "Skipping duplicate StockItemLocationAllocatedEvent event_id=%s existing_rows=%s",
                event_id,
                existing_count,
            )
            return {
                "accepted": True,
                "duplicate": True,
                "eventType": "StockItemLocationAllocatedEvent",
                "receivedAt": utc_now(),
                "insertedRows": 0,
            }

        if rows:
            db.add_all(rows)
            db.commit()
            inserted_rows = len(rows)
        else:
            logger.info(
                "No stock location rows parsed for event_id=%s",
                event_id,
            )
        logger.info(
            "Stored StockItemLocationAllocatedEvent event_id=%s inserted_rows=%s",
            event_id,
            inserted_rows,
        )
    finally:
        db.close()
    return {
        "accepted": True,
        "duplicate": False,
        "eventType": "StockItemLocationAllocatedEvent",
        "receivedAt": utc_now(),
        "insertedRows": inserted_rows,
    }
