#!/usr/bin/env python3
"""Generate / check .vscode/tasks.json pickString options from layouts + robots."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
TASKS = ROOT / ".vscode" / "tasks.json"
ROBOTS = ROOT / "src" / "scooping_controller" / "config" / "robots.yaml"
LAYOUTS = ROOT / "config" / "layouts"


def layout_ids() -> list[str]:
    return sorted(path.stem for path in LAYOUTS.glob("*.yaml"))


def robot_ids() -> list[str]:
    data = yaml.safe_load(ROBOTS.read_text()) or {}
    return sorted(str(key) for key in (data.get("robots") or {}).keys())


def desired_inputs() -> list[dict]:
    return [
        {
            "id": "layout",
            "type": "pickString",
            "options": layout_ids(),
            "default": "dual-container" if "dual-container" in layout_ids() else layout_ids()[0],
            "description": "Cell layout",
        },
        {
            "id": "robot",
            "type": "pickString",
            "options": robot_ids(),
            "default": "niryo" if "niryo" in robot_ids() else robot_ids()[0],
            "description": "Robot arm",
        },
    ]


def sync_tasks(data: dict) -> dict:
    by_id = {item["id"]: item for item in desired_inputs()}
    inputs = []
    seen = set()
    for item in data.get("inputs") or []:
        if item.get("id") in by_id:
            inputs.append(by_id[item["id"]])
            seen.add(item["id"])
        else:
            inputs.append(item)
    for key, item in by_id.items():
        if key not in seen:
            inputs.append(item)
    data["inputs"] = inputs

    # Ensure Author / Status tasks exist.
    labels = {task.get("label") for task in data.get("tasks") or []}
    extras = []
    if "Author (bench + layout editor)" not in labels:
        extras.append(
            {
                "label": "Author (bench + layout editor)",
                "type": "shell",
                "command": "make author LAYOUT=${input:layout} ROBOT=${input:robot}",
                "options": {"cwd": "${workspaceFolder}"},
                "problemMatcher": [],
                "presentation": {"reveal": "always", "panel": "dedicated"},
            }
        )
    if "Dev status" not in labels:
        extras.append(
            {
                "label": "Dev status",
                "type": "shell",
                "command": "make status",
                "options": {"cwd": "${workspaceFolder}"},
                "problemMatcher": [],
            }
        )
    if extras:
        data.setdefault("tasks", [])
        # Insert author/status near the top after Bench.
        data["tasks"] = extras + list(data["tasks"])
    return data


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--check",
        action="store_true",
        help="Exit non-zero if committed tasks.json is stale",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="Rewrite .vscode/tasks.json pickString options",
    )
    args = parser.parse_args()
    if not args.check and not args.write:
        args.write = True

    current = json.loads(TASKS.read_text())
    updated = sync_tasks(json.loads(json.dumps(current)))  # deep copy
    desired_text = json.dumps(updated, indent=2) + "\n"
    current_text = TASKS.read_text()

    if args.check:
        # Compare only the generated inputs options for stable CI.
        current_inputs = {
            item["id"]: item.get("options")
            for item in current.get("inputs") or []
            if item.get("id") in {"layout", "robot"}
        }
        desired = {
            item["id"]: item.get("options") for item in desired_inputs()
        }
        if current_inputs != desired:
            print("stale .vscode/tasks.json pickString options:", file=sys.stderr)
            print(f"  committed={current_inputs}", file=sys.stderr)
            print(f"  expected ={desired}", file=sys.stderr)
            print("Run: python3 scripts/gen_vscode_tasks.py --write", file=sys.stderr)
            return 1
        print("ok: .vscode/tasks.json layout/robot options are current")
        return 0

    if desired_text != current_text:
        TASKS.write_text(desired_text)
        print(f"updated {TASKS.relative_to(ROOT)}")
    else:
        print(f"unchanged {TASKS.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
