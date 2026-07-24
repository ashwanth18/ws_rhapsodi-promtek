#!/usr/bin/env python3
"""Ansible dynamic inventory from Tailscale devices tagged tag:robot.

Prefer the local `tailscale status --json` output (no API token needed on a
machine already on the tailnet). Falls back to TAILSCALE_API_KEY + TAILNET
if status is unavailable.

Usage:
  ansible-inventory -i ansible/inventory/tailscale.py --list
  ansible-playbook -i ansible/inventory/tailscale.py ansible/deploy.yml
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import urllib.request


ROBOT_TAG = os.environ.get("TAILSCALE_ROBOT_TAG", "tag:robot")
BUILDER_TAG = os.environ.get("TAILSCALE_BUILDER_TAG", "tag:builder")


def _tags(device: dict) -> list[str]:
    tags = device.get("Tags") or device.get("tags") or []
    return list(tags)


def from_status() -> list[dict]:
    raw = subprocess.check_output(["tailscale", "status", "--json"], text=True)
    data = json.loads(raw)
    devices = []
    self_dev = data.get("Self")
    if self_dev:
        devices.append(self_dev)
    devices.extend((data.get("Peer") or {}).values())
    return devices


def from_api() -> list[dict]:
    api_key = os.environ["TAILSCALE_API_KEY"]
    tailnet = os.environ["TAILNET"]
    url = f"https://api.tailscale.com/api/v2/tailnet/{tailnet}/devices"
    req = urllib.request.Request(url, headers={"Authorization": f"Bearer {api_key}"})
    with urllib.request.urlopen(req, timeout=30) as resp:
        payload = json.loads(resp.read().decode())
    return payload.get("devices") or []


def host_entry(device: dict) -> tuple[str, dict] | None:
    tags = _tags(device)
    # Local status uses Caps; API uses lowercase keys.
    hostname = (
        device.get("DNSName")
        or device.get("HostName")
        or device.get("hostname")
        or device.get("Name")
        or ""
    ).rstrip(".")
    short = hostname.split(".")[0] if hostname else ""
    addrs = device.get("TailscaleIPs") or device.get("addresses") or []
    ip = next((a for a in addrs if ":" not in a), None)
    if not short or not ip:
        return None
    online = device.get("Online")
    if online is None:
        # status --json: missing Active means online-ish; treat ConnectedToControl as hint
        online = bool(device.get("Active") or device.get("Online", True))
    # Default SSH users differ per device class in this fleet.
    default_user = os.environ.get("ANSIBLE_USER")
    if not default_user:
        if short == "jetson" or short.startswith("ashwanth"):
            default_user = "ashwanth"
        else:
            default_user = "admin"
    meta = {
        "ansible_host": ip,
        "ansible_user": default_user,
        "tailscale_hostname": short,
        "tailscale_tags": tags,
        "tailscale_online": online,
    }
    return short, meta


def build_inventory(devices: list[dict]) -> dict:
    robots: dict[str, dict] = {}
    builders: dict[str, dict] = {}
    all_hosts: dict[str, dict] = {}

    for device in devices:
        tags = _tags(device)
        entry = host_entry(device)
        if not entry:
            continue
        name, meta = entry
        all_hosts[name] = meta
        if ROBOT_TAG in tags or any(t.endswith(":robot") for t in tags):
            robots[name] = meta
        if BUILDER_TAG in tags or any(t.endswith(":builder") for t in tags):
            builders[name] = meta
        # Fallback: hostname heuristics for fleets not yet tagged
        if name.startswith("rhapsodi") or name.endswith("-pi5") or "robot" in name:
            robots.setdefault(name, meta)

    return {
        "_meta": {"hostvars": {**all_hosts}},
        "all": {"children": ["robots", "builders"]},
        "robots": {"hosts": sorted(robots.keys()), "vars": {"device_role": "robot"}},
        "builders": {"hosts": sorted(builders.keys()), "vars": {"device_role": "builder"}},
    }


def main() -> None:
    if len(sys.argv) == 2 and sys.argv[1] == "--host":
        # Prefer --list only; empty host vars
        print("{}")
        return
    try:
        devices = from_status()
    except (FileNotFoundError, subprocess.CalledProcessError, OSError):
        if not (os.environ.get("TAILSCALE_API_KEY") and os.environ.get("TAILNET")):
            print(
                "tailscale status failed and TAILSCALE_API_KEY/TAILNET not set",
                file=sys.stderr,
            )
            sys.exit(1)
        devices = from_api()
    print(json.dumps(build_inventory(devices), indent=2))


if __name__ == "__main__":
    main()
