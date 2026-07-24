# Batch weightment integration service

## Overview

This project now uses a webhook-driven, row-based integration instead of a queue-based one.

The system sits between:

- an upstream batch system that emits Promtek webhooks
- the dashboard and backend in this repo
- the robot orchestrator running through ROS
- a ROS-native start adapter that accepts HTTP and calls ROS services with `rclpy`
- a downstream API that accepts `/batch/weighment` and `/batch/end`

## Current Architecture

At a high level, the workflow is:

1. Promtek sends `BatchReleasedEvent` and `StockItemLocationAllocatedEvent` to `webhook_service`.
2. `webhook_service` stores one database row per weightment and stores stock-location allocation rows.
3. `backend` serves the dashboard and remains the source of truth for operator actions and robot-run state.
4. The dashboard shows grouped webhook events and per-weightment detail rows.
5. The operator clicks `Run Robot` for one incomplete weightment row.
6. The backend resolves the latest stock location, maps it to robot target names, converts kg to g, and calls the robot start adapter over HTTP.
7. The robot runs one BT execution for that one weightment row.
8. The adapter calls `/bt_start_webhook_weightment` through native ROS `rclpy`.
9. The backend watches robot completion state through rosbridge topics.
10. After robot success, the backend updates the stored weightment row and posts `/batch/weighment`.
11. When all weightments for the batch are complete, the backend posts `/batch/end`.

## Services And Ports

- `webhook_service`: receives upstream webhook events
- `backend`: FastAPI API used by the dashboard, usually on `http://localhost:8000`
- `robot_start_adapter`: ROS-native FastAPI control service, usually on `http://localhost:8010`
- `rosbridge_websocket`: central ROS bridge, usually on `ws://localhost:9090`
- downstream weighment endpoint: `http://localhost:5002/batch/weighment`
- downstream batch-end endpoint: `http://localhost:5002/batch/end`

## Stored State

Important persistent state now lives in the database:

- `webhook_weightments`
  - one row per weightment extracted from `BatchReleasedEvent`
- `stock_location_allocations`
  - stock item to location mapping from `StockItemLocationAllocatedEvent`
- `robot_weightment_runs`
  - one robot execution record per attempted robot run

Webhook deduplication is still based on the upstream `eventId`.

## Webhook Endpoints

The upstream system should send webhook events to `webhook_service` routes:

- `POST /api/BatchReleasedEvent`
- `POST /api/StockItemLocationAllocatedEvent`

Behavior of `BatchReleasedEvent` ingestion:

- prints the raw webhook body to the terminal
- extracts batch metadata and lines
- creates one `webhook_weightments` row per weightment
- stores `event_id` for deduplication

Behavior of `StockItemLocationAllocatedEvent` ingestion:

- prints the raw webhook body to the terminal
- extracts location and allocated stock item data
- stores rows in `stock_location_allocations`
- later allows the backend to resolve `ingredient_id` to `location_id`

## Backend API

Important backend routes:

- `GET /webhook_weightments/summary`
  - grouped event summary for the dashboard
- `GET /webhook_weightments/{event_id}`
  - detailed weightment rows for one event
- `POST /webhook_weightments/{weightment_id}/run_robot`
  - starts one robot run for one weightment row
- `POST /webhook_weightments/{weightment_id}/send`
  - manual fallback that sends directly to MES without robot execution
- `GET /robot_weightment_runs`
  - robot-run status records

## Backend To Robot Contract

The backend translates one stored weightment row into one robot-ready request.

Main mapping:

- `ingredient_id` + `site_id` -> latest stock location allocation row
- `stock_item_location_id` or `stock_item_location_code` -> robot target mapping
- `target_weight_kg` -> `target_weight_g`

The BT start contract is still the ROS service:

- `/bt_start_webhook_weightment`
- type: `robot_common_msgs/srv/StartWebhookWeightment`

Key fields:

- `weightment_id`
- `batch_id`
- `ingredient_id`
- `location_id`
- `location_code`
- `pickup_target_name`
- `weigh_target_name`
- `return_target_name`
- `target_weight_g`
- `tolerance_g`
- `expected_lot`

## Event Mapping

### BatchReleasedEvent

The current ingestion path supports the real payload shape you shared.

Important fields mapped into `webhook_weightments`:

- `eventId` -> `event_id`
- `sentUtc` -> `sent_utc`
- `userId` -> `user_id`
- `siteId` -> `site_id`
- `batch.id` -> `batch_id`
- `batch.batchNumber` -> `batch_number`
- `batch.workOrderId` -> `work_order_id`
- `batch.targetQuantity` -> `batch_target_quantity`
- `lines[].parameters[Ingredient].target` -> `ingredient_id`
- `lines[].parameters[Quantity].target` -> `target_weight_kg`

### StockItemLocationAllocatedEvent

Important fields mapped into `stock_location_allocations`:

- `eventId` -> `event_id`
- `siteId` -> `site_id`
- `stockItemLocationId` -> `stock_item_location_id`
- `stockItemLocationCode` -> `stock_item_location_code`
- `allocatedStockItems[].id` -> `stock_item_id`
- `allocatedStockItems[].code` -> `stock_item_code`
- `allocatedStockItems[].name` -> `stock_item_name`

## Downstream Request Payloads

### Weighment Request

After successful robot completion, the backend posts:

```json
{
  "batchId": 22461,
  "stockItemId": 1327,
  "locationId": 92,
  "lotCode": "",
  "targetKg": 0.15,
  "actualKg": 0.15,
  "startUtc": "2026-03-12T12:38:31.0968685Z",
  "endUtc": "2026-03-12T12:45:00.000Z",
  "energyKwh": 0
}
```

### Batch End Request

When the full batch is complete, the backend posts:

```json
{
  "batchId": 22461,
  "endUtc": "2026-03-12T12:45:00.000Z"
}
```

## Robot Completion Behavior

The system does not mark a weightment complete just because the operator pressed a button.

Instead:

- `Run Robot` starts a robot execution request
- the robot executes one BT run
- the backend start request goes through the ROS-native adapter
- completion state comes back through rosbridge topics
- the backend updates:
  - `actual_weight_kg`
  - `start_utc`
  - `end_utc`
  - `energy_kwh`
  - robot run status
- only then does the backend send MES `/batch/weighment`

## Production Considerations

For production, consider adding:

- authentication and authorization for operator actions
- webhook authentication or signature validation
- structured logging
- observability and metrics
- retry and dead-letter behavior for downstream API failures
- monitoring for rosbridge connectivity
- monitoring for adapter health and ROS service availability
- automated tests for webhook parsing, robot-start logic, and completion handling
