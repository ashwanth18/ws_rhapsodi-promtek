# Data hygiene notes (Phase 1 audit)

Observed on `niryo-rhapsodi` during the data-export / modes work:

## Persistent and correct

- Bind mount: `/opt/rhapsodi/ws_rhapsodi-promtek/data` → `/data` (survives container recreate).
- Active root: `/data/runs/manifest.sqlite` (0 run rows until first recorded cycle).
- Exports staging: `/data/exports` (created lazily by backend export API).

## Stale leftovers — confirm before deleting

| Path | Notes |
|------|-------|
| `/opt/rhapsodi/ws_rhapsodi-promtek/data/webhook/manifest.sqlite` | Pre-collapse layout leftover. Safe to remove after confirming no process still points `DATA_OUTPUT_ROOT` at `/data/webhook`. |
| Docker volume `ws_rhapsodi-promtek_db_data` | Orphan from an older compose project name. Active DB volume is `rhapsodi-promtek-robot-prod_db_data`. Confirm with `docker volume inspect` before `docker volume rm`. |

Do **not** auto-delete these; ask the operator first.
