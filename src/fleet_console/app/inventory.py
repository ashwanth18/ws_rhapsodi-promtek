"""Tailscale device discovery for the Fleet Console.

Mirrors ansible/inventory/tailscale.py so the UI and Ansible see the same hosts.
"""
from __future__ import annotations

import json
import os
import subprocess
import urllib.request
from typing import Any


ROBOT_TAG = os.environ.get('TAILSCALE_ROBOT_TAG', 'tag:robot')
BUILDER_TAG = os.environ.get('TAILSCALE_BUILDER_TAG', 'tag:builder')


def _tags(device: dict) -> list[str]:
    tags = device.get('Tags') or device.get('tags') or []
    return list(tags)


def from_status() -> list[dict]:
    raw = subprocess.check_output(['tailscale', 'status', '--json'], text=True)
    data = json.loads(raw)
    devices = []
    self_dev = data.get('Self')
    if self_dev:
        devices.append(self_dev)
    devices.extend((data.get('Peer') or {}).values())
    return devices


def from_api() -> list[dict]:
    api_key = os.environ['TAILSCALE_API_KEY']
    tailnet = os.environ['TAILNET']
    url = f'https://api.tailscale.com/api/v2/tailnet/{tailnet}/devices'
    req = urllib.request.Request(
        url, headers={'Authorization': f'Bearer {api_key}'}
    )
    with urllib.request.urlopen(req, timeout=30) as resp:
        payload = json.loads(resp.read().decode())
    return payload.get('devices') or []


def _host_meta(device: dict) -> dict[str, Any] | None:
    tags = _tags(device)
    hostname = (
        device.get('DNSName')
        or device.get('HostName')
        or device.get('hostname')
        or device.get('Name')
        or ''
    ).rstrip('.')
    short = hostname.split('.')[0] if hostname else ''
    addrs = device.get('TailscaleIPs') or device.get('addresses') or []
    ip = next((a for a in addrs if ':' not in a), None)
    if not short or not ip:
        return None
    online = device.get('Online')
    if online is None:
        online = bool(device.get('Active') or device.get('Online', True))
    default_user = os.environ.get('ANSIBLE_USER')
    if not default_user:
        if short == 'jetson' or short.startswith('ashwanth'):
            default_user = 'ashwanth'
        else:
            default_user = 'admin'
    return {
        'id': short,
        'hostname': short,
        'ip': ip,
        'ansible_user': default_user,
        'tags': tags,
        'online': bool(online),
        'role': None,
    }


def list_devices() -> list[dict[str, Any]]:
    try:
        raw_devices = from_status()
    except (FileNotFoundError, subprocess.CalledProcessError, OSError):
        if not (os.environ.get('TAILSCALE_API_KEY') and os.environ.get('TAILNET')):
            return []
        raw_devices = from_api()

    out: list[dict[str, Any]] = []
    seen: set[str] = set()
    for device in raw_devices:
        meta = _host_meta(device)
        if not meta:
            continue
        name = meta['hostname']
        if name in seen:
            continue
        tags = meta['tags']
        is_robot = ROBOT_TAG in tags or any(t.endswith(':robot') for t in tags)
        is_builder = BUILDER_TAG in tags or any(
            t.endswith(':builder') for t in tags
        )
        if name.startswith('rhapsodi') or name.endswith('-pi5') or 'robot' in name:
            is_robot = True
        if not is_robot and not is_builder:
            continue
        meta['role'] = 'builder' if is_builder and not is_robot else 'robot'
        if is_builder and is_robot:
            meta['role'] = 'robot'
        seen.add(name)
        out.append(meta)
    out.sort(key=lambda d: d['hostname'])
    return out


def get_device(device_id: str) -> dict[str, Any] | None:
    for device in list_devices():
        if device['id'] == device_id or device['hostname'] == device_id:
            return device
    return None
