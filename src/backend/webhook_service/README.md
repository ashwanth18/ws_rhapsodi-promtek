# Webhook Service

`webhook_service` is the inbound integration service for Promtek MES events.

It is still called `webhook_service` because its job is to receive upstream webhook traffic, but the operator-facing product language in the dashboard now uses production-style terms such as `Batches`, `Batch Details`, and `Batch Execution Dashboard`.

## Naming Guidance

Use these terms when discussing the system:

- `webhook_service`: the inbound FastAPI service that accepts Promtek events
- `batch`: the operator-facing concept shown in the UI
- `weightment`: one row within a batch that the robot can execute
- `location allocation`: the MES event that maps stock items to physical locations

There is currently a deliberate split between external UI wording and internal compatibility names:

- the UI says `Batches`
- some backend routes and database tables still use `webhook_*`
- this is intentional for now to avoid a broader backend refactor

## What This Service Does

This service currently:

- exposes webhook endpoints for Promtek events
- logs the full incoming JSON payload in pretty-printed form
- parses `BatchReleasedEvent` into one database row per weightment
- parses `StockItemLocationAllocatedEvent` into location allocation rows
- deduplicates events by upstream `eventId`
- returns a small acknowledgement payload

This service does not:

- directly start the robot
- decide operator actions
- send `/batch/weighment` or `/batch/end` downstream

Those responsibilities belong to the main `backend` service.

## Endpoints

- `GET /health`
- `POST /api/BatchReleasedEvent`
- `POST /api/StockItemLocationAllocatedEvent`

When run through Docker Compose in this repo, the service is typically reachable at:

- `http://localhost:5000/health`
- `http://localhost:5000/api/BatchReleasedEvent`
- `http://localhost:5000/api/StockItemLocationAllocatedEvent`

## Runtime Behavior

### `BatchReleasedEvent`

On receipt, the service:

- logs the raw request body
- extracts top-level event metadata
- reads `batch.lines`
- extracts the `Ingredient` and `Quantity` parameter targets from each line
- creates one `webhook_weightments` row per line
- marks each new row as not completed

Important mappings:

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

### `StockItemLocationAllocatedEvent`

On receipt, the service:

- logs the raw request body
- extracts the stock item location metadata
- expands `allocatedStockItems`
- creates one `stock_location_allocations` row per allocated stock item

Important mappings:

- `eventId` -> `event_id`
- `contextId` -> `context_id`
- `siteId` -> `site_id`
- `createdUtc` -> `created_utc`
- `stockItemLocationId` -> `stock_item_location_id`
- `stockItemLocationCode` -> `stock_item_location_code`
- `allocatedStockItems[].id` -> `stock_item_id`
- `allocatedStockItems[].code` -> `stock_item_code`
- `allocatedStockItems[].name` -> `stock_item_name`

## Database State Written By This Service

This service writes to:

- `webhook_weightments`
  - one row per weightment extracted from `BatchReleasedEvent`
- `stock_location_allocations`
  - one row per allocated stock item extracted from `StockItemLocationAllocatedEvent`

Deduplication is based on the upstream `eventId`.

If a `BatchReleasedEvent` with the same `eventId` is received again, the service acknowledges it as a duplicate and does not insert the rows again. The same duplicate protection applies to `StockItemLocationAllocatedEvent`.

## Example Log Output

For each accepted webhook, the terminal shows:

- a short log line with the event id
- the full request body as formatted JSON

Example:

```text
Received BatchReleasedEvent event_id=568146dd-d903-406b-83c9-201b583b39e7
BatchReleasedEvent request body:
{
  "eventId": "568146dd-d903-406b-83c9-201b583b39e7",
  "sender": "promtek-service-batch",
  "batch": {
    "...": "..."
  }
}
```

## Relationship To The Backend

After this service writes the source-of-truth rows, the main `backend` service takes over. In practice:

1. `webhook_service` stores incoming batch and allocation events.
2. `backend` serves the dashboard APIs and operator actions.
3. The UI groups stored rows as `Batches`.
4. The backend resolves location allocations, starts one robot run per weightment, waits for completion and processed MCAP results, then sends downstream MES callbacks.

So the naming split is:

- inbound integration and compatibility layer: `webhook_service`, `webhook_weightments`
- operator-facing product language: `Batches`, `Batch Details`, `Batch Execution`

## Running The Service

The container runs:

```bash
uvicorn main:app --host 0.0.0.0 --port 5000
```

With Docker Compose, the service is normally available at `http://localhost:5000`.
