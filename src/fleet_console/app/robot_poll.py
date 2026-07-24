"""Live-poll each robot's local backend for identity / active run status."""
from __future__ import annotations

import asyncio
from typing import Any

import httpx

HOST_INFO_TIMEOUT = 2.5
ACTIVE_TIMEOUT = 2.5


async def poll_host_info(ip: str, port: int = 8000) -> dict[str, Any] | None:
    url = f'http://{ip}:{port}/host_info'
    try:
        async with httpx.AsyncClient(timeout=HOST_INFO_TIMEOUT) as client:
            resp = await client.get(url)
            if resp.status_code != 200:
                return None
            return resp.json()
    except Exception:
        return None


async def poll_active_run(ip: str, port: int = 8000) -> dict[str, Any] | None:
    url = f'http://{ip}:{port}/robot_weightment_runs/active'
    try:
        async with httpx.AsyncClient(timeout=ACTIVE_TIMEOUT) as client:
            resp = await client.get(url)
            if resp.status_code != 200:
                return None
            return resp.json()
    except Exception:
        return None


async def enrich_device(device: dict[str, Any]) -> dict[str, Any]:
    ip = device.get('ip')
    if not ip:
        device['host_info'] = None
        device['active_run'] = None
        device['provisioned'] = False
        return device
    host_info, active = await asyncio.gather(
        poll_host_info(ip),
        poll_active_run(ip),
    )
    device['host_info'] = host_info
    device['active_run'] = active
    # Consider provisioned if backend answers /host_info.
    device['provisioned'] = host_info is not None
    if host_info:
        device['robot_type'] = host_info.get('robot_type')
        device['site_id'] = host_info.get('site_id')
        device['device_id'] = host_info.get('device_id') or device.get('id')
        device['image_tag'] = host_info.get('image_tag')
        device['robot_id'] = host_info.get('robot_id')
        device['running_profile_id'] = host_info.get('profile_id')
    else:
        device.setdefault('robot_type', None)
        device.setdefault('site_id', None)
        device.setdefault('device_id', device.get('id'))
        device.setdefault('image_tag', None)
        device.setdefault('robot_id', None)
        device.setdefault('running_profile_id', None)
    # Backend shape: {"active": <run dict|null>}
    if isinstance(active, dict):
        device['active'] = active.get('active') is not None
    else:
        device['active'] = False
    return device


async def enrich_devices(devices: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not devices:
        return []
    return list(await asyncio.gather(*(enrich_device(d) for d in devices)))
