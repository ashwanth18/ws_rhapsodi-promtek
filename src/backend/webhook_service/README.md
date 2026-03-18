# Webhook Service

This service is a small FastAPI app that receives Promtek webhook events and prints the incoming JSON payloads to the terminal.

## Current Behavior

The service currently does three things:

- exposes webhook endpoints
- logs the incoming webhook body in pretty-printed JSON
- returns a small JSON acknowledgement

It does not currently:

- store webhook payloads
- transform the payload into an internal queue
- write to a database

## Endpoints

- `GET /health`
- `POST /api/BatchReleasedEvent`
- `POST /api/StockItemLocationAllocatedEvent`

When run through Docker in this repo, the service listens on:

- `http://localhost:5000/health`
- `http://localhost:5000/api/BatchReleasedEvent`
- `http://localhost:5000/api/StockItemLocationAllocatedEvent`

## What Gets Printed

For each webhook, the terminal shows:

- a short log line with the `eventId`
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

## BatchReleasedEvent Structure

The payload is naturally hierarchical and is easiest to read in four levels:

1. Event metadata
2. Batch
3. Batch lines
4. Line parameters

### 1. Event Metadata

Top-level fields describe the webhook itself:

- `sentUtc`: when the event was sent
- `userId`: user related to the event
- `siteId`: plant/site identifier
- `eventId`: unique event identifier
- `sender`: source system
- `batch`: the actual batch payload

### 2. Batch

The `batch` object describes the production batch:

- `id`: batch id
- `batchNumber`: human-readable batch number
- `createdUtc`
- `startTimeUtc`
- `controlSystemCode`
- `workOrderId`
- `scheduleItemId`
- `startingScheduleItemId`
- `routeWorkCentreId`
- `finishedProductId`
- `targetQuantity`
- `baseFormula`
- `batchStatus`
- `lines`

### 3. Batch Lines

`batch.lines` is the ordered list of work steps for the batch.

Each line usually contains:

- `id`: batch line id
- `sequence`: execution order
- `batchInstructionCode`: step type, for example `MAT_ADD`
- `weigherLocationId`
- `weigherLocationCode`
- `weigherId`
- `weigherCode`
- `batchLineStatus`
- `parameters`

### 4. Line Parameters

Each line has a `parameters` list. These are typed name/value inputs for the step.

In your example, the important ones are:

- `Ingredient`: material or stock item id
- `Quantity`: target amount to add
- `Weigher`: target weighing device id

Example simplified view:

```yaml
event:
  eventId: 568146dd-d903-406b-83c9-201b583b39e7
  sender: promtek-service-batch
  siteId: 76454da2-683c-4706-a88e-4e2fc6773685

batch:
  id: 22675
  batchNumber: 7
  targetQuantity: 1.0
  formula:
    code: F001
    name: Formula 1
  lines:
    - lineId: 503000
      sequence: 0
      instruction: MAT_ADD
      weigherCode: RW1
      ingredientId: 1330
      quantity: 0.5000000
    - lineId: 503001
      sequence: 1
      instruction: MAT_ADD
      weigherCode: RW1
      ingredientId: 1329
      quantity: 0.2300000
```

## Most Important Fields

If you want to extract the actionable information from a `BatchReleasedEvent`, the most important fields are:

- `eventId`: useful for deduplication and traceability
- `siteId`: useful for system/site routing
- `batch.id`: primary batch identifier
- `batch.batchNumber`: human-readable batch reference
- `batch.workOrderId`: production order reference
- `batch.baseFormula`: formula metadata
- `batch.targetQuantity`: intended total batch amount
- `batch.lines[].sequence`: operation order
- `batch.lines[].batchInstructionCode`: operation type
- `batch.lines[].parameters`: line-level inputs
- `Ingredient`: which material to use
- `Quantity`: how much to add
- `Weigher`: which weighing device to use

## Suggested Internal Structure

If you later want to process this payload in code, a cleaner internal structure is to flatten each line's `parameters` array into named fields.

For example, instead of storing:

```json
{
  "parameters": [
    { "batchInstructionParameterCode": "Ingredient", "target": "1330" },
    { "batchInstructionParameterCode": "Quantity", "target": "0.5000000" },
    { "batchInstructionParameterCode": "Weigher", "target": "16" }
  ]
}
```

you can convert it into:

```json
{
  "lineId": 503000,
  "sequence": 0,
  "instruction": "MAT_ADD",
  "ingredientId": 1330,
  "quantity": 0.5,
  "weigherId": 16,
  "weigherCode": "RW1"
}
```

That makes downstream logic much easier for robot execution, queue building, and UI display.

## Running The Service

The Docker image runs:

```bash
uvicorn main:app --host 0.0.0.0 --port 5000
```

With Docker Compose, you can reach it at `http://localhost:5000`.
