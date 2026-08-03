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

Single root `DATA_OUTPUT_ROOT` (default `/data/runs`). Runs go under
`{output_root}/{mode}/…`, falling back to `{output_root}/unknown/` when
mode is unset. Details in [DATA_ARCHITECTURE.md](DATA_ARCHITECTURE.md).

## Hot-switch (Phase 3)

Runtime mode changes go through a future **ModeManager** (not in this
phase). Until then, profile / `BT_TREE` / compose selection still pick
the active tree at deploy time.
