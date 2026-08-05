#!/usr/bin/env python3
"""Validate every versioned cell layout."""
from pathlib import Path
import importlib.util
import sys

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "cell_layout", ROOT / "src/backend/app/modes/cell_layout.py")
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)
load_layout = MODULE.load_layout


def main() -> int:
    layouts = sorted((ROOT / "config/layouts").glob("*.yaml"))
    failures = []
    for path in layouts:
        try:
            load_layout(path)
            print(f"valid: {path.relative_to(ROOT)}")
        except Exception as exc:  # report all invalid layouts in CI
            failures.append(f"{path}: {exc}")
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
