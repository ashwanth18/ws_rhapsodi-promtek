#!/usr/bin/env bash
# Authoring entry point: bench loop with the interactive cell-layout editor.
# Equivalent to: make bench LAYOUT=... ROBOT=... LAYOUT_EDIT=true
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export LAYOUT_EDIT=true
exec "$ROOT/scripts/dev_bench.sh" "$@"
