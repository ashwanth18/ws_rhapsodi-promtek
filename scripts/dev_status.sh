#!/usr/bin/env bash
# Read-only doctor for the cell-layout / scoop authoring loop.
# HTTP GETs + git status only — never commands robot motion.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PI_HTTP="${ROBOT_HTTP_URL:-http://${ROBOT_HOST:-rhapsodi-pi5}:8000}"
LAYOUTS_DIR="${CELL_LAYOUTS_DIR:-$ROOT/config/layouts}"

section() { printf '\n\033[1m%s\033[0m\n' "$1"; }
ok() { printf '  \033[32m✓\033[0m %s\n' "$1"; }
warn() { printf '  \033[33m!\033[0m %s\n' "$1"; }
bad() { printf '  \033[31m✗\033[0m %s\n' "$1"; }

section "Local tree"
if git -C "$ROOT" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  branch="$(git -C "$ROOT" rev-parse --abbrev-ref HEAD)"
  ok "branch $branch"
  dirty_layouts="$(git -C "$ROOT" status --porcelain -- config/layouts config/models 2>/dev/null || true)"
  if [[ -n "$dirty_layouts" ]]; then
    warn "uncommitted layout/model edits:"
    printf '%s\n' "$dirty_layouts" | sed 's/^/    /'
  else
    ok "config/layouts + config/models clean"
  fi
else
  warn "not a git checkout"
fi

section "Layout validation"
if python3 "$ROOT/scripts/validate_layouts.py" >/tmp/rhapsodi-validate-layouts.txt 2>&1; then
  ok "validate-layouts passed ($(grep -c '^valid:' /tmp/rhapsodi-validate-layouts.txt || true) files)"
else
  bad "validate-layouts failed"
  sed 's/^/    /' /tmp/rhapsodi-validate-layouts.txt | tail -n 20
fi

section "Pose export freshness"
stale=0
for layout in "$LAYOUTS_DIR"/*.yaml; do
  [[ -f "$layout" ]] || continue
  id="$(basename "$layout" .yaml)"
  poses="$LAYOUTS_DIR/$id/poses.yaml"
  if [[ ! -f "$poses" ]]; then
    warn "$id: missing poses.yaml"
    stale=1
    continue
  fi
  if [[ "$layout" -nt "$poses" ]]; then
    warn "$id: layout YAML newer than poses.yaml (re-export scoop poses?)"
    stale=1
  else
    ok "$id: poses.yaml is up to date vs layout YAML"
  fi
done
if [[ "$stale" -eq 0 ]]; then
  ok "no stale pose exports detected"
fi

section "Device layout ($PI_HTTP)"
if layouts_json="$(curl --fail --silent --show-error --max-time 3 "$PI_HTTP/layouts" 2>/tmp/rhapsodi-status-curl.err)"; then
  python3 - "$layouts_json" <<'PY'
import json, sys
data = json.loads(sys.argv[1])
active = data.get("active_layout_id") or "(none)"
hash_ = data.get("active_layout_hash") or "(none)"
preview = data.get("preview")
print(f"  active_layout_id={active}")
print(f"  active_layout_hash={hash_}")
print(f"  preview={preview}")
if preview or hash_ == "preview":
    print("  ! PREVIEW geometry active — save + re-apply before production runs")
PY
else
  warn "could not reach $PI_HTTP/layouts (offline or wrong host)"
  [[ -f /tmp/rhapsodi-status-curl.err ]] && sed 's/^/    /' /tmp/rhapsodi-status-curl.err
fi

section "Suggested next step"
if [[ -n "${dirty_layouts:-}" ]]; then
  echo "  1. Finish authoring in RViz (make author), Save Layout / Export Poses"
  echo "  2. make validate-layouts && commit config/layouts changes"
  echo "  3. Build/release, then apply via Fleet Console"
elif [[ "$stale" -ne 0 ]]; then
  echo "  Export scoop poses (RViz Export or make export-poses), then validate + commit"
else
  echo "  Tree looks clean. Use make author / make sim to iterate, or Fleet Console to apply."
fi

echo
echo "See docs/CELL_LAYOUT_DEV_LOOP.md for the full sequence."
