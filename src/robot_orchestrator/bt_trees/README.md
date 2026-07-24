# BT Orchestrator Integration Notes

This document explains how `main.xml` in this folder fits with the current webhook, backend, dashboard, and robot setup.

## Webhook-Driven Flow Now Implemented

The webhook integration now uses a dedicated tree instead of pushing Promtek data into `main.xml`.

New pieces:

- `webhook_weightment.xml`
- `/bt_start_webhook_weightment`
- `/orchestrator/run_state`
- central `rosbridge_websocket`

The runtime flow is:

1. dashboard calls backend `POST /webhook_weightments/{id}/run_robot`
2. backend creates one `robot_weightment_runs` row with a translated robot contract
3. backend calls `/bt_start_webhook_weightment` through rosbridge
5. `webhook_weightment.xml` runs one scoop -> weigh -> pour cycle
6. backend watches `/orchestrator/run_state` and `/weight` through rosbridge
7. backend updates `webhook_weightments`, sends `/batch/weighment`, and if needed `/batch/end`

## What `main.xml` Does

The behavior tree in `main.xml` runs the powder handling flow for one queued task at a time.

At a high level it does this:

1. fetches a batch queue
2. loops through ingredient tasks
3. projects each task into blackboard keys
4. moves to the QR position
5. scans the QR code
6. moves to the ingredient container
7. moves to the scale
8. pours until the target is reached
9. runs cleanup motions

The tree currently uses these key BT nodes:

- `FetchBatch`
- `LoopContainerSpec`
- `ProjectContainer`
- `MoveTo`
- `ScanQr`
- `PourToTarget`
- `QueueStatus`
- `Vibrate`

## Current Tree Assumptions

The current tree is not built directly around Promtek webhook payloads. It expects robot-ready task data.

In particular, it assumes each task can provide:

- `container_name`
- `qr_target_name`
- `expected_lot`
- `container_target_g`
- valid named robot targets such as `scale` and `test2`

This means the tree already expects a translated robot task, not raw webhook data.

## What The Current Webhook Flow Provides

The current webhook/database/dashboard setup stores one row per weightment with fields such as:

- `event_id`
- `sent_utc`
- `batch_id`
- `ingredient_id`
- `target_weight_kg`
- `lot_code`
- `weightment_completed`

This is useful for batch tracking and operator actions, but it is still missing some robot-facing information that `main.xml` expects.

## Main Integration Gap

The main gap is:

`webhook weightment row` is not the same thing as a `BT container task`

Today, the webhook flow knows:

- which batch is running
- which ingredient is needed
- how much mass is needed

But the BT needs:

- which named robot target corresponds to that ingredient or stock location
- which QR target should be used
- what lot code is expected for scanning
- the weight target in the unit expected by the tree

So the missing piece is an adapter layer.

## Recommended Architecture

The cleanest design is:

1. webhook service receives `BatchReleasedEvent`
2. backend stores one row per weightment
3. dashboard shows the rows
4. operator chooses the next row or batch to run
5. backend converts that row into a BT/orchestrator request
6. orchestrator runs the robot flow
7. robot result comes back to backend
8. backend marks the row complete
9. backend sends downstream `/batch/weighment`
10. when all rows in the batch are complete, backend sends `/batch/end`

The important point is:

- the dashboard should not directly translate webhook data into robot commands
- the backend should own the translation
- the backend should also remain the owner of completion state and downstream API calls

## Best Way To Use `main.xml` With The Current Setup

The easiest way to make the current tree work is to run it one weightment at a time.

That means:

- one row in `webhook_weightments`
- becomes one robot task
- becomes one BT run

This matches the way your database is already structured and avoids forcing the BT to understand full webhook batch payloads directly.

## Mapping You Need

To make a stored weightment usable by the BT, you need a mapping layer such as:

- `ingredient_id` or stock location -> `container_name`
- `container_name` -> `qr_target_name`
- row `lot_code` -> `expected_lot`
- `target_weight_kg` -> `container_target_g` or whatever unit contract you decide

Without this mapping, the tree does not know where to move for a given webhook row.

## Units

The stored webhook rows currently use kilograms:

- `target_weight_kg`
- `actual_weight_kg`

The BT naming strongly suggests grams:

- `container_target_g`
- internal scale-related naming

So you should define one explicit conversion rule. For example:

- backend stores kg
- backend converts kg -> g before starting the BT
- robot side runs in g
- backend converts back if needed for downstream systems

Do not leave the unit contract implicit.

## Lot Code And QR

The tree uses:

- `ScanQr expected_lot="{expected_lot}"`

But `BatchReleasedEvent` does not naturally provide a lot code.

So for QR verification to work, `expected_lot` must come from one of these:

- allocation event data
- another inventory lookup
- operator input
- a temporary default value for development

If no real lot code is available, then the QR step will either need:

- a placeholder expected value
- to be bypassed in a development tree
- or to be populated from another source before the BT starts

## What Needs To Happen In Backend

The backend should provide an adapter endpoint, for example:

- `POST /webhook_weightments/{id}/run_robot`

That endpoint should:

1. load the selected DB row
2. map ingredient or location to robot target names
3. resolve the expected lot
4. convert target mass to the BT unit
5. call the orchestrator start service

This keeps the business logic in one place and avoids duplicating robot translation logic in the dashboard.

## What Needs To Come Back From Robot

After the robot finishes, the backend needs a structured completion result.

A useful result payload would contain:

- `weightment_id`
- `batch_id`
- `ingredient_id`
- `success`
- `actual_weight_kg`
- `lot_code`
- `start_utc`
- `end_utc`
- `energy_kwh`

Then the backend can:

- update the `webhook_weightments` row
- mark it completed
- send downstream `/batch/weighment`
- automatically send `/batch/end` when the full batch is done

## Practical Short-Term Plan

The fastest path with minimal changes is:

1. keep `main.xml` mostly unchanged
2. start the BT one weightment at a time
3. add a static mapping from ingredient or location to robot target names
4. feed `expected_lot` from the stored row or a temporary default
5. update the backend when the robot run finishes

This lets the existing tree work without redesigning the whole orchestrator.

## Current Webhook Contract

The explicit robot-start contract is now `robot_common_msgs/srv/StartWebhookWeightment`:

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

This keeps the backend responsible for translation while the BT stays focused on robot execution.

## Longer-Term Improvements

Later, you can improve the integration by:

- persisting stock allocation location data in the database
- mapping location codes directly to robot targets
- replacing static target-name mapping with pose-based motion if needed
- adding a proper robot completion callback/API
- allowing batch-level auto-advance through all stored rows
- making QR verification use real lot data instead of defaults

## Summary

`main.xml` can work with the current setup, but it needs a translation layer.

The BT is already a good robot execution flow. The main thing missing is a bridge from:

- stored webhook weightment row

to:

- robot-ready BT task

The cleanest approach is to build that bridge in the backend, keep the dashboard simple, and let the backend remain responsible for marking completion and sending downstream weighment/batch-end events.
