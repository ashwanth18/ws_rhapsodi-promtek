"""Live-poll each robot's local backend for identity / active run status."""
from __future__ import annotations

import asyncio
from typing import Any

import httpx

HOST_INFO_TIMEOUT = 2.5
ACTIVE_TIMEOUT = 2.5
RUNTIME_MODE_TIMEOUT = 2.5


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


async def poll_runtime_mode(ip: str, port: int = 8000) -> dict[str, Any] | None:
    """GET /runtime/mode → {mode, environment, active_run?}."""
    url = f'http://{ip}:{port}/runtime/mode'
    try:
        async with httpx.AsyncClient(timeout=RUNTIME_MODE_TIMEOUT) as client:
            resp = await client.get(url)
            if resp.status_code != 200:
                return None
            payload = resp.json()
            if not isinstance(payload, dict):
                return None
            return payload
    except Exception:
        return None


async def enrich_device(device: dict[str, Any]) -> dict[str, Any]:
    ip = device.get('ip')
    if not ip:
        device['host_info'] = None
        device['active_run'] = None
        device['runtime_mode'] = None
        device['active_mode'] = None
        device['environment'] = None
        device['layout_id'] = None
        device['layout_hash'] = None
        device['provisioned'] = False
        return device
    host_info, active, runtime = await asyncio.gather(
        poll_host_info(ip),
        poll_active_run(ip),
        poll_runtime_mode(ip),
    )
    device['host_info'] = host_info
    device['active_run'] = active
    device['runtime_mode'] = runtime
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
    # Live mode/environment from robot backend (preferred over agent cache).
    if isinstance(runtime, dict):
        mode = runtime.get('mode')
        env = runtime.get('environment')
        device['active_mode'] = str(mode).strip() if mode else None
        device['environment'] = str(env).strip() if env else None
        device['layout_id'] = runtime.get('layout_id')
        device['layout_hash'] = runtime.get('layout_hash')
    else:
        device['active_mode'] = None
        device['environment'] = None
        device['layout_id'] = None
        device['layout_hash'] = None
    return device


async def enrich_devices(devices: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not devices:
        return []
    return list(await asyncio.gather(*(enrich_device(d) for d in devices)))
