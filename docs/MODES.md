# Operating modes

Canonical reference for how Rhapsodi describes *what kind of job* a
robot is running. Pair with [DATA_ARCHITECTURE.md](DATA_ARCHITECTURE.md)
for on-disk layout and uplink.

## Three axes

| Axis | Values | Meaning |
|---|---|---|
| **Environment** | `real` \| `sim` | Physical cell vs simulated stack. Sim is laptop-only; never a fleet profile on a Pi. |
| **Mode** | `mes-condor` \| `mes-generic` \| `mock-local` \| `lightsout` | Job family / business contract. |
| **Device** | Pi / Jetson / x86 host class | Hardware + compose device file (`compose/devices/…`). Orthogonal to mode. |

A run is identified by the intersection of these axes plus a stable
`run_key` (the local run-folder basename).

## Mode families

### MES family (`mes-condor`, `mes-generic`, `mock-local`)

Share one behavior-tree / start-service identity: **WebhookWeightment**
(`webhook_weightment.xml`, `/bt_start_webhook_weightment`).

- Phase topic: `/webhook_run/phase`
- Condor/MES wiring differs by mode; the robot job shape does not.

### Lights-out (`lightsout`)

Separate tree: **LightsOut** (`lightsout.xml`).

- Phase topic: `/lightsout_training/phase`
- Training episodes, not MES weighments.

## RunSpec

`src/backend/app/run_spec.py` defines the shared contract:

- `mode`, `environment`, `run_key`
- `target_weight_g`, `tolerance_g` (orchestrator honors `tolerance_g` when `> 0`)
- optional MES fields: `location_code`, `location_id`, `batch_id`,
  `ingredient_id`, `weightment_id`, `expected_lot`
- `tree_id` — `WebhookWeightment` or `LightsOut`
- `phase_topic` — derived from mode
- `to_metadata_dict()` — seed for run `metadata.json` (`schema_version: "1"`)

## On-disk layout

Single root `DATA_OUTPUT_ROOT` (default `/data/runs` for `environment=real`).
Runs go under `{output_root}/{mode}/…`, falling back to
`{output_root}/unknown/` when mode is unset. Details in
[DATA_ARCHITECTURE.md](DATA_ARCHITECTURE.md).

When `environment=sim`, the resolved recorder root prefers (in order):

1. `SIM_DATA_OUTPUT_ROOT` (explicit override)
2. `DATA_OUTPUT_ROOT` (if set)
3. `/tmp/rhapsodi-sim/runs` (laptop default)

`GET /runtime/capabilities` includes `data_output_root` for the active
environment.

## Hot-switch

Phase 2: the orchestrator registers all share `bt_trees/*.xml` at startup
and creates a tree per run from the start-service `tree_id` (no restart).

Phase 3: backend **ModeManager** (`src/backend/app/modes/`) exposes
`GET/PUT /runtime/mode` and `GET /runtime/capabilities`. Active mode is
persisted at `RUNTIME_MODE_PATH` (default `/data/runtime_mode.json`).
Mode changes are refused with HTTP 409 while `has_active_robot_run` is
true. Outbound MES posts go through `MesClient` (`CondorMesClient` vs
`NullMesClient` for mock-local / lights-out). `BT_TREE` / `tree_file` is
an optional preferred-tree hint only.

Phase 4: `POST /modes/mock/runs` (active mode must already be
`mock-local`, else 409) inserts a synthetic `WebhookWeightment` with
`event_id` prefix `mock-` and starts via the production
`robot_start_adapter` / `/bt_start_webhook_weightment` path. Completion
short-circuits Condor when `event_id` starts with `mock-`. Dashboard
**Test** page (`/test`) drives mode switch + mock starts.

Phase 5: `POST /modes/lightsout/runs` (active mode must already be
`lightsout`, else 409) starts training via
`robot_start_adapter` `/start_lightsout` → `/bt_start_lightsout`.
Uses `NullMesClient` (no Condor). Blackboard `enable_scoop` (default
false) gates `ExecuteScoop` in `lightsout.xml`. Dashboard **Training**
page (`/training`) drives mode switch + starts.

Phase 6: **mes-generic** is config-driven. Inbound events go to
`POST /modes/mes-generic/events` (active mode must be `mes-generic`,
else 409). An inbound adapter registry normalizes external payloads into
the same internal weightment shape as Condor/`webhook_weightments`:

| Adapter | Env | Behavior |
|---|---|---|
| `condor` (default) | `MES_GENERIC_INBOUND_ADAPTER=condor` | Promtek `BatchReleasedEvent` shape (same keys/types as `webhook_service`) |
| `generic_json` | `MES_GENERIC_INBOUND_ADAPTER=generic_json` | Flat/simple JSON; fields mapped by `MES_GENERIC_FIELD_MAP_JSON` |

Outbound sink (`MES_GENERIC_SINK`):

| Value | Client |
|---|---|
| `condor` (default) | `CondorMesClient` (same URLs as mes-condor) |
| `null` | `NullMesClient` (no Condor traffic) |

Stored rows use an `event_id` prefix `generic-` so completion routing binds
the mes-generic sink even if the runtime mode later changes. **mes-condor**
still uses `webhook_service` → `BatchReleasedEvent` unchanged.

## Phase 7: laptop sim environment

`environment=sim` uses the same `GET/PUT /runtime/mode` APIs as `real`, but
is **laptop-only**:

| Guard | Behavior |
|---|---|
| `SIM_ALLOWED` | Default `0`. Must be `1` for sim. Set only in `docker-compose.sim.yml` (or local docs). Pi5 / robot-prod profiles force `0`. |
| Device class | Even if `SIM_ALLOWED=1`, ModeManager refuses sim when `DEVICE_CLASS`, `ROBOT_TYPE`, or `device.yaml` (`device_class` / `robot_type`) is `pi5`. |

**Modes that allow sim** (via `allowed_environments`): `mock-local`,
`lightsout`. MES modes (`mes-condor`, `mes-generic`) stay `real`-only.

**Null MES sink:** when the active runtime environment is `sim`,
`get_mes_client()` always returns `NullMesClient` (no Condor traffic),
regardless of mode.

### Laptop compose

```bash
docker compose -f docker-compose.sim.yml up -d --build
curl -s localhost:8000/runtime/capabilities
curl -s -X PUT localhost:8000/runtime/mode \
  -H 'content-type: application/json' \
  -d '{"mode":"mock-local","environment":"sim"}'
```

`docker-compose.sim.yml` sets `SIM_ALLOWED=1`, `ENVIRONMENT=sim`,
`DEVICE_CLASS=x86`, and sim data roots under `/tmp/rhapsodi-sim/…`.
`compose/devices/pi5.yml` / `robot-prod.env.example` keep `SIM_ALLOWED=0`
and `DEVICE_CLASS=pi5`.
