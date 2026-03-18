# Webhook Run Observability

## Suggested file name

`webhook-run-observability.md`

This is a good name because the document is specifically about:

- webhook-triggered robot runs
- observability and trace capture
- how MCAP, processing, backend, and dashboard fit together

## Why this exists

The webhook integration now runs the robot one weightment at a time.

That means each weightment run can produce two different kinds of data:

1. business-state data
   - batch id
   - ingredient id
   - location id
   - target weight
   - completion state
   - downstream MES send state
2. robot trace data
   - MCAP bag
   - processed Parquet
   - time-series weight data
   - phase transitions
   - derived performance features

The robot trace data is extremely valuable because it gives us the full execution history of the run, not just the final row stored in `webhook_weightments`.

## What we can do with this information

### 1. Trace every webhook weightment to one robot run

We can now treat:

- one `webhook_weightments` row
- one `robot_weightment_runs` row
- one MCAP trace
- one processed feature set

as a single chain of evidence.

This makes debugging and review much easier.

### 2. Improve the source of truth for completion data

The quick completion signal still comes from ROS runtime state.

But the processed MCAP can provide a more authoritative post-run summary:

- actual/final weight
- start time
- end time
- pour duration
- scoop duration
- settle time
- overshoot

That means the backend can eventually use:

- ROS runtime state for immediate completion
- processed MCAP features for authoritative metrics and auditability

### 3. Build run quality analytics

The processed data already supports useful quality signals:

- overshoot
- average flow rate
- total duration
- settle time
- target vs final weight

This can become:

- a run quality score
- warning badges for unstable runs
- ingredient-specific or location-specific diagnostics

### 4. Support replay and debugging

Because the MCAP contains weight, phases, system status, joint states, and static transforms, it can be used for:

- Foxglove playback
- failed-run investigation
- regression comparison between BT versions
- performance tuning of scoop and pour behavior

### 5. Enable optimization over time

Across many runs, the data can show:

- which ingredients are harder to pour accurately
- which stock locations correlate with poor behavior
- which target weights lead to overshoot
- whether a BT or controller change made things better or worse

## Dedicated webhook metadata approach

We do not want webhook runs to reuse `lightsout_training/*` metadata topics.

Instead, webhook runs use their own dedicated ROS trace metadata:

- `/webhook_run/active`
- `/webhook_run/metadata`
- `/webhook_run/phase`

This keeps:

- lights-out experiments
- webhook production-style runs

cleanly separated.

## Current end-to-end webhook trace flow

1. The dashboard calls `POST /webhook_weightments/{id}/run_robot`.
2. The backend creates a `robot_weightment_runs` row.
3. The backend generates a `trace_run_id` like `webhook-run-<id>`.
4. The backend calls a ROS-native HTTP adapter service for the start request.
5. The adapter calls `/bt_start_webhook_weightment` through `rclpy` in the ROS environment.
6. The orchestrator publishes webhook-specific trace metadata and phase events.
7. The backend watches completion and live weight through rosbridge topics.
8. `data_collection_manager` records a dedicated MCAP for that webhook run.
9. When the webhook run ends, `data_collection_manager` finalizes the bag and triggers processing.
10. The processing service extracts features and posts them back to the backend.
11. The backend links the processed results and artifact paths back onto `robot_weightment_runs`.
12. The dashboard can display trace availability next to the webhook weightment.

## Current stored trace linkage

The backend now links trace information back to the robot run record:

- `trace_run_id`
- `processed_run_db_id`
- `processed_id`
- `mcap_path`
- `parquet_path`

This gives the webhook detail page a way to show whether a trace is available for a given weightment run.

## Troubleshooting

### Issue: webhook weightment detail page stuck on `Refreshing...`

#### 502 symptom

After starting a new webhook weighment, the dashboard detail page could get stuck showing:

- `Refreshing...`

At the same time:

- `GET /webhook_weightments/{event_id}` could hang for many seconds or time out
- the latest `robot_weightment_runs` row often remained in `starting`
- PostgreSQL showed backend sessions in `idle in transaction`

#### Observed database evidence

When the issue was active, `pg_stat_activity` showed queries like:

- `SELECT robot_weightment_runs.id ...`

with session state:

- `idle in transaction`

This was the main clue that the backend was leaving transactions open while waiting on non-database work.

#### 502 root cause

There were two related backend problems:

1. rosbridge reset work was being triggered from the stale-run reconciliation path used by the detail endpoint
   - if rosbridge cleanup blocked, the request thread for `GET /webhook_weightments/{event_id}` could block too
2. `POST /webhook_weightments/{weightment_id}/run_robot` still touched SQLAlchemy ORM objects after `db.commit()`
   - because SQLAlchemy expires ORM fields on commit, reading `run_row.id` after commit triggered a fresh `SELECT`
   - that reopened a database transaction
   - the request then waited on `rosbridge_robot_client.start_run(...)` while PostgreSQL sat `idle in transaction`
   - later detail-page requests could hang behind that leaked session state

#### 502 fix

The backend was updated in two ways:

1. rosbridge reset was made non-blocking
   - the client now detaches the current websocket/topic handles immediately
   - disposal of the old rosbridge connection happens in a background thread
   - stale-run reconciliation now uses this non-blocking reset path
2. the `run_robot` route was changed so it does not touch expired ORM fields before the external rosbridge call
   - scalar IDs are captured before the external call
   - the route no longer reopens a database transaction just to read `run_row.id`

#### 502 result

After these fixes:

- `GET /webhook_weightments/{event_id}` returned normally again
- the webhook detail page stopped hanging on `Refreshing...`
- PostgreSQL no longer accumulated `idle in transaction` sessions from this path

#### If this appears again

Check these first:

- `docker logs` for the backend container
- `pg_stat_activity` for `idle in transaction` sessions
- whether the latest `robot_weightment_runs` row is stuck in `starting`

If the detail route is hanging again, the most likely regression is that some backend path is still:

- holding a DB session open across an external call
- or performing blocking rosbridge cleanup directly inside a request handler

### Issue: webhook start returns `502` with adapter `504` timeout

#### Symptom

After pressing `Run Robot`, the backend could fail immediately with a start error like:

- `502: Downstream request failed (http://host.docker.internal:8010/start_webhook_weightment): 504 {"detail":"Service /bt_start_webhook_weightment call timed out after 15.0s"}`

At the same time:

- the weightment stayed in `starting`
- the dashboard looked like the robot never began
- the backend log showed `POST /webhook_weightments/{id}/run_robot HTTP/1.1" 502 Bad Gateway`
- the adapter log showed `POST /start_webhook_weightment HTTP/1.1" 504 Gateway Timeout`

#### Root cause

The backend-to-adapter HTTP path was healthy.

The actual failure was one level deeper:

1. the backend successfully called the ROS-native HTTP adapter
2. the adapter successfully discovered `/bt_start_webhook_weightment`
3. the adapter then called the ROS service through `rclpy`
4. the service call never received a response before the timeout

This was confusing because service discovery worked correctly.

The key diagnostic result was:

- the same `ros2 service call /bt_start_webhook_weightment ...` succeeded immediately from the host
- the same request from inside the `robot_start_adapter` container hung until timeout
- forcing the adapter container to use `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` made the exact same call succeed

So the issue was not:

- the backend request format
- the BT service callback itself
- or the adapter application logic

It was a Fast DDS transport/runtime problem affecting request/response traffic from the container.

#### Fix

The adapter runtime was updated to force UDP transport:

- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`

This was added to:

- `docker-compose.yml` for `robot_start_adapter`
- `docker-compose.lightsout.yml` for `robot_start_adapter`

After recreating the adapter container with that environment, the native ROS service call from inside the container completed normally again.

#### Result

After the transport fix:

- `POST /start_webhook_weightment` stopped returning `504`
- `POST /webhook_weightments/{id}/run_robot` stopped failing with downstream `502`
- webhook starts progressed out of `starting` again
- the adapter-based start path became reliable without falling back to rosbridge control

#### If this appears again

Check these in order:

- adapter health: `GET http://localhost:8010/health`
- adapter logs for `504 Gateway Timeout`
- whether host-side `ros2 service call /bt_start_webhook_weightment ...` succeeds
- whether the same call from inside the adapter container hangs
- whether `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` is present in the adapter container environment

If host calls succeed but container calls hang, the likely regression is in DDS transport/runtime configuration, not in the webhook payload mapping.

### Issue: `data_collection_manager` reports `Failed to process run: HTTP Error 502: Bad Gateway`

#### Symptom

After MCAP recording stopped, `data_collection_manager` logged:

- `Failed to process run: HTTP Error 502: Bad Gateway`

The processing service also showed:

- `POST /process HTTP/1.1" 502 Bad Gateway`

But the robot run itself could already appear completed in the backend.

#### Root cause

The failure happened on the callback path:

1. `data_collection_manager` called the processing service
2. the processing service successfully extracted features and called `POST /processed` on the backend
3. the backend stored the processed data successfully
4. the backend then crashed while building the HTTP response

The backend exception was:

- `sqlalchemy.orm.exc.DetachedInstanceError`

Specifically, `/processed` returned `processed_row.id` and `processed_row.run_db_id` after session work had completed, and SQLAlchemy attempted to refresh an ORM instance that was no longer safely bound for attribute loading.

#### Fix

The backend `/processed` route now captures plain scalar values immediately:

- `processed_id`
- `run_db_id`

and returns those scalars instead of reading ORM attributes at the end of the request.

#### Result

After the fix:

- the processing service no longer receives a backend `500` that gets translated into `502`
- `data_collection_manager` should stop reporting this false processing failure
- the response status now matches the actual stored result

#### Important note

In the failing version, the processed run was often already stored before the response crashed.

So this specific error could be misleading:

- the UI might already show `processed_id`
- `robot_weightment_runs.processed_id` could already be populated
- MES send flags could already be `true`

## Recommended next steps

### Short term

- show a direct "Open Logs" or trace details action from the webhook detail page
- add a mode filter in `LogsPage` so webhook runs can be isolated quickly
- expose processed overshoot/final-weight data directly in the webhook detail response

### Medium term

- add a dedicated processed table or richer schema for webhook runs if needed
- add direct trace playback support
- make processed MCAP features the authoritative post-run metrics for robot completion

### Longer term

- build ingredient-level and location-level quality dashboards
- compare BT versions across runs
- add anomaly detection or adaptive control based on historical traces
