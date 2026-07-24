#!/usr/bin/env bash
# Fleet health check over Tailscale SSH — for operators and Cursor agents.
#
# Usage:
#   bash scripts/fleet_health_check.sh
#   bash scripts/fleet_health_check.sh rhapsodi-pi5 jetson
#   FLEET_HOSTS="rhapsodi-pi5" bash scripts/fleet_health_check.sh
#
# Exit codes:
#   0 = all hosts healthy
#   1 = one or more faults found
set -euo pipefail

SSH_USER="${SSH_USER:-admin}"
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=8 -o StrictHostKeyChecking=accept-new)

discover_hosts() {
  if [[ $# -gt 0 ]]; then
    printf '%s\n' "$@"
    return
  fi
  if [[ -n "${FLEET_HOSTS:-}" ]]; then
    # shellcheck disable=SC2086
    printf '%s\n' ${FLEET_HOSTS}
    return
  fi
  python3 - <<'PY'
import json, subprocess
data = json.loads(subprocess.check_output(["tailscale", "status", "--json"], text=True))
peers = list((data.get("Peer") or {}).values())
if data.get("Self"):
    peers.append(data["Self"])
for d in peers:
    name = (d.get("DNSName") or d.get("HostName") or "").rstrip(".").split(".")[0]
    tags = d.get("Tags") or []
    if not name:
        continue
    if "tag:robot" in tags or name.startswith("rhapsodi") or name.endswith("-pi5"):
        print(name)
PY
}

check_host() {
  local host="$1"
  local candidates=()
  case "${host}" in
    rhapsodi-pi5|niryo-rhapsodi) candidates+=(niryo-rhapsodi "admin@rhapsodi-pi5" "admin@${host}") ;;
    jetson) candidates+=("ashwanth@jetson" jetson "ashwanth@${host}") ;;
    *) candidates+=("${host}" "admin@${host}" "ashwanth@${host}") ;;
  esac

  echo "---- ${host} ----"
  local ssh_target=""
  local cand
  for cand in "${candidates[@]}"; do
    if ssh "${SSH_OPTS[@]}" "${cand}" 'true' 2>/dev/null; then
      ssh_target="${cand}"
      break
    fi
  done
  if [[ -z "${ssh_target}" ]]; then
    echo "FAULT: cannot SSH to ${host} (tried: ${candidates[*]})"
    return 1
  fi
  echo "ssh_target=${ssh_target}"

  # shellcheck disable=SC2029
  ssh "${SSH_OPTS[@]}" "${ssh_target}" bash -s <<'REMOTE'
set -euo pipefail
faults=0
echo "hostname=$(hostname) arch=$(uname -m) uptime=$(uptime -p 2>/dev/null || uptime)"
df -h / | tail -1

if command -v docker >/dev/null 2>&1; then
  if docker info >/dev/null 2>&1; then
    echo "docker=ok"
    unhealthy="$(docker ps -a --filter health=unhealthy --format '{{.Names}}' 2>/dev/null || true)"
    if [[ -n "${unhealthy}" ]]; then
      echo "FAULT: unhealthy containers: ${unhealthy}"
      faults=1
    fi
    exited="$(docker ps -a --filter status=exited --format '{{.Names}} ({{.Status}})' 2>/dev/null | head -20 || true)"
    while IFS= read -r line; do
      [[ -z "${line}" ]] && continue
      case "${line}" in
        *backend*|*rosbridge*|*webhook*|*dashboard*|*processing*|*orchestrator*|*pouring*|*scooping*)
          echo "FAULT: exited stack container: ${line}"
          faults=1
          ;;
      esac
    done <<< "${exited}"
  else
    echo "FAULT: docker installed but not usable (group/permissions?)"
    faults=1
  fi
else
  echo "WARN: docker not installed"
fi

if curl -fsS --max-time 3 http://127.0.0.1:8000/health >/dev/null 2>&1 \
  || curl -fsS --max-time 3 http://127.0.0.1:8000/docs >/dev/null 2>&1; then
  echo "backend=ok"
else
  echo "WARN: backend health endpoint not reachable on :8000"
fi

if curl -fsS --max-time 3 http://127.0.0.1:9100/metrics >/dev/null 2>&1; then
  echo "node_exporter=ok"
else
  echo "WARN: node_exporter not reachable on :9100"
fi

if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then
  for c in $(docker ps --format '{{.Names}}' 2>/dev/null | head -30); do
    errs="$(docker logs --since 5m "$c" 2>&1 | grep -iE 'error|fatal|panic|traceback' | tail -3 || true)"
    if [[ -n "${errs}" ]]; then
      echo "FAULT: recent errors in ${c}:"
      echo "${errs}"
      faults=1
    fi
  done
fi

exit "${faults}"
REMOTE
}

main() {
  mapfile -t hosts < <(discover_hosts "$@")
  if [[ "${#hosts[@]}" -eq 0 ]]; then
    echo "No fleet hosts discovered. Pass hosts explicitly or set FLEET_HOSTS." >&2
    exit 1
  fi
  echo "Checking ${#hosts[@]} host(s): ${hosts[*]}"
  failed=0
  for h in "${hosts[@]}"; do
    if ! check_host "${h}"; then
      failed=1
    fi
    echo
  done
  if [[ "${failed}" -ne 0 ]]; then
    echo "RESULT: faults detected"
    exit 1
  fi
  echo "RESULT: all checked hosts healthy"
}

main "$@"
