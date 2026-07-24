#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TEMPLATE_DIR="${ROOT_DIR}/scripts/webhook_sim_test"

BACKEND_URL="${BACKEND_URL:-http://localhost:8000}"
WEBHOOK_URL="${WEBHOOK_URL:-http://localhost:5000}"
MES_MOCK_URL="${MES_MOCK_URL:-http://localhost:5002}"

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "${TMP_DIR}"' EXIT

wait_for_url() {
  local url="$1"
  local label="$2"
  echo "Waiting for ${label} at ${url} ..."
  for _ in $(seq 1 120); do
    if curl -fsS "${url}" >/dev/null 2>&1; then
      echo "${label} is ready"
      return 0
    fi
    sleep 1
  done
  echo "Timed out waiting for ${label} at ${url}" >&2
  return 1
}

wait_for_url "${BACKEND_URL}/health" "backend"
wait_for_url "${WEBHOOK_URL}/health" "webhook_service"
wait_for_url "${MES_MOCK_URL}/health" "mes_mock"

export TEMPLATE_DIR TMP_DIR
python3 - <<'PY'
import json
import os
import uuid
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path

template_dir = Path(os.environ["TEMPLATE_DIR"])
tmp_dir = Path(os.environ["TMP_DIR"])

now = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
batch_template = json.loads((template_dir / "BatchReleasedEvent.sample.json").read_text())
alloc_template = json.loads((template_dir / "StockItemLocationAllocatedEvent.sample.json").read_text())

batch_payload = deepcopy(batch_template)
alloc_payload = deepcopy(alloc_template)

event_suffix = uuid.uuid4().hex[:8]
batch_id = 300000 + int(event_suffix[:4], 16)

batch_payload["eventId"] = str(uuid.uuid4())
batch_payload["sentUtc"] = now
batch_payload["batch"]["id"] = batch_id
batch_payload["batch"]["batchNumber"] = f"SIM-BATCH-{batch_id}"
batch_payload["batch"]["workOrderId"] = f"SIM-WO-{event_suffix}"

alloc_payload["eventId"] = str(uuid.uuid4())
alloc_payload["contextId"] = f"sim-context-{event_suffix}"
alloc_payload["createdUtc"] = now

(tmp_dir / "batch.json").write_text(json.dumps(batch_payload, indent=2))
(tmp_dir / "allocation.json").write_text(json.dumps(alloc_payload, indent=2))
print(batch_payload["eventId"])
print(batch_id)
PY

BATCH_EVENT_ID="$(python3 - <<'PY'
import json, os
from pathlib import Path
tmp_dir = Path(os.environ["TMP_DIR"])
payload = json.loads((tmp_dir / "batch.json").read_text())
print(payload["eventId"])
PY
)"

echo "Posting StockItemLocationAllocatedEvent ..."
curl -fsS \
  -H "Content-Type: application/json" \
  -X POST \
  --data @"${TMP_DIR}/allocation.json" \
  "${WEBHOOK_URL}/api/StockItemLocationAllocatedEvent"
echo

echo "Posting BatchReleasedEvent ..."
curl -fsS \
  -H "Content-Type: application/json" \
  -X POST \
  --data @"${TMP_DIR}/batch.json" \
  "${WEBHOOK_URL}/api/BatchReleasedEvent"
echo

echo "Resolving created weightment row for event_id=${BATCH_EVENT_ID} ..."
WEIGHTMENT_ID="$(
  BACKEND_URL="${BACKEND_URL}" BATCH_EVENT_ID="${BATCH_EVENT_ID}" python3 - <<'PY'
import json
import os
import urllib.request

backend_url = os.environ["BACKEND_URL"]
event_id = os.environ["BATCH_EVENT_ID"]
with urllib.request.urlopen(f"{backend_url}/webhook_weightments?limit=200") as response:
    payload = json.load(response)
rows = payload.get("rows", [])
matches = [row for row in rows if row.get("event_id") == event_id]
if not matches:
    raise SystemExit("No webhook weightment rows found for new test event")
print(matches[0]["id"])
PY
)"

echo "Starting robot run for weightment_id=${WEIGHTMENT_ID} ..."
curl -fsS \
  -H "Content-Type: application/json" \
  -X POST \
  "${BACKEND_URL}/webhook_weightments/${WEIGHTMENT_ID}/run_robot"
echo

cat <<EOF

Simulation webhook test started.

Useful follow-up commands:
  curl -fsS ${BACKEND_URL}/webhook_weightments?limit=20 | python3 -m json.tool
  curl -fsS ${BACKEND_URL}/robot_weightment_runs?limit=20 | python3 -m json.tool
  curl -fsS ${MES_MOCK_URL}/health

Dashboard:
  http://localhost:8080

Test event_id:
  ${BATCH_EVENT_ID}

Weightment id:
  ${WEIGHTMENT_ID}
EOF
