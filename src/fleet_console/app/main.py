"""Rhapsodi Fleet Console API — central deploy / provision control plane."""
from __future__ import annotations

import asyncio
import os
from pathlib import Path
from typing import Any, Literal

from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .ansible_runner import log_path_for, run_playbook
from .auth import require_token
from .build_runner import run_branch_build
from .db import Deployment, DeviceTarget, SessionLocal, init_db, utc_now
from .github import version_check
from .inventory import get_device, list_devices
from .profiles import get_profile, list_profiles
from .prometheus import device_metrics
from .robot_poll import enrich_device, enrich_devices

app = FastAPI(title='Rhapsodi Fleet Console', version='0.2.0')

app.add_middleware(
    CORSMiddleware,
    allow_origins=os.environ.get('CORS_ORIGINS', '*').split(','),
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

STATIC_DIR = Path(__file__).resolve().parent.parent / 'static'


class ProvisionRequest(BaseModel):
    robot_type: Literal['niryo', 'jaka']
    site_id: str = 'site-1'
    image_tag: str = Field(min_length=1)
    profile_id: str = 'prod-niryo'
    tracked_branch: str = 'main'
    device_id: str | None = None
    robot_id: str | None = None


class DeployRequest(BaseModel):
    image_tag: str = Field(min_length=1)
    profile_id: str | None = None
    tracked_branch: str | None = None


class TargetUpdateRequest(BaseModel):
    tracked_branch: str | None = None
    profile_id: str | None = None
    pinned_image_tag: str | None = None
    auto_update: bool | None = None
    robot_type: str | None = None
    site_id: str | None = None


class BuildRequest(BaseModel):
    branch: str | None = None
    deploy_after: bool = False
    profile_id: str | None = None


def _serialize_deployment(row: Deployment) -> dict[str, Any]:
    return {
        'id': row.id,
        'device_id': row.device_id,
        'action': row.action,
        'robot_type': row.robot_type,
        'site_id': row.site_id,
        'profile_id': row.profile_id,
        'tracked_branch': row.tracked_branch,
        'image_tag': row.image_tag,
        'status': row.status,
        'requested_by': row.requested_by,
        'log_path': row.log_path,
        'error_message': row.error_message,
        'started_at': row.started_at.isoformat() if row.started_at else None,
        'finished_at': row.finished_at.isoformat() if row.finished_at else None,
    }


def _serialize_target(row: DeviceTarget | None, device_id: str) -> dict[str, Any]:
    if row is None:
        return {
            'device_id': device_id,
            'tracked_branch': 'main',
            'profile_id': 'prod-niryo',
            'pinned_image_tag': None,
            'auto_update': False,
            'robot_type': None,
            'site_id': None,
            'updated_at': None,
        }
    return {
        'device_id': row.device_id,
        'tracked_branch': row.tracked_branch,
        'profile_id': row.profile_id,
        'pinned_image_tag': row.pinned_image_tag,
        'auto_update': bool(row.auto_update),
        'robot_type': row.robot_type,
        'site_id': row.site_id,
        'updated_at': row.updated_at.isoformat() if row.updated_at else None,
    }


def _get_or_create_target(
    db,
    device_id: str,
    *,
    robot_type: str | None = None,
    site_id: str | None = None,
    profile_id: str | None = None,
    tracked_branch: str | None = None,
) -> DeviceTarget:
    row = db.query(DeviceTarget).filter(DeviceTarget.device_id == device_id).first()
    if row is None:
        default_profile = (
            profile_id
            or ('prod-jaka' if robot_type == 'jaka' else 'prod-niryo')
        )
        row = DeviceTarget(
            device_id=device_id,
            tracked_branch=tracked_branch or 'main',
            profile_id=default_profile,
            robot_type=robot_type,
            site_id=site_id,
            updated_at=utc_now(),
        )
        db.add(row)
        db.commit()
        db.refresh(row)
    return row


def _upsert_target(
    device_id: str,
    *,
    tracked_branch: str | None = None,
    profile_id: str | None = None,
    pinned_image_tag: str | None = None,
    auto_update: bool | None = None,
    robot_type: str | None = None,
    site_id: str | None = None,
) -> DeviceTarget:
    db = SessionLocal()
    try:
        row = _get_or_create_target(
            db,
            device_id,
            robot_type=robot_type,
            site_id=site_id,
            profile_id=profile_id,
            tracked_branch=tracked_branch,
        )
        if tracked_branch is not None:
            row.tracked_branch = tracked_branch
        if profile_id is not None:
            if get_profile(profile_id) is None:
                raise HTTPException(status_code=400, detail=f'Unknown profile {profile_id}')
            row.profile_id = profile_id
        if pinned_image_tag is not None:
            row.pinned_image_tag = pinned_image_tag or None
        if auto_update is not None:
            row.auto_update = auto_update
        if robot_type is not None:
            row.robot_type = robot_type
        if site_id is not None:
            row.site_id = site_id
        row.updated_at = utc_now()
        db.commit()
        db.refresh(row)
        # Detach values we need after session close
        db.expunge(row)
        return row
    finally:
        db.close()


def _targets_by_device() -> dict[str, dict[str, Any]]:
    db = SessionLocal()
    try:
        rows = db.query(DeviceTarget).all()
        return {r.device_id: _serialize_target(r, r.device_id) for r in rows}
    finally:
        db.close()


def _latest_deployments_by_device() -> dict[str, dict[str, Any]]:
    db = SessionLocal()
    try:
        rows = (
            db.query(Deployment)
            .order_by(Deployment.started_at.desc())
            .limit(500)
            .all()
        )
        out: dict[str, dict[str, Any]] = {}
        for row in rows:
            if row.device_id not in out:
                out[row.device_id] = _serialize_deployment(row)
        return out
    finally:
        db.close()


def _device_has_running_job(device_id: str) -> Deployment | None:
    db = SessionLocal()
    try:
        return (
            db.query(Deployment)
            .filter(
                Deployment.device_id == device_id,
                Deployment.status == 'running',
            )
            .order_by(Deployment.started_at.desc())
            .first()
        )
    finally:
        db.close()


def _on_playbook_complete(
    deployment_id: int, rc: int | None, err: str | None
) -> None:
    db = SessionLocal()
    try:
        row = db.query(Deployment).filter(Deployment.id == deployment_id).first()
        if not row:
            return
        row.finished_at = utc_now()
        if rc == 0:
            row.status = 'success'
            row.error_message = None
            # Pin successful image as desired.
            target = (
                db.query(DeviceTarget)
                .filter(DeviceTarget.device_id == row.device_id)
                .first()
            )
            if target and row.image_tag:
                target.pinned_image_tag = row.image_tag
                if row.profile_id:
                    target.profile_id = row.profile_id
                if row.tracked_branch:
                    target.tracked_branch = row.tracked_branch
                target.updated_at = utc_now()
        else:
            row.status = 'failed'
            row.error_message = err or f'exit code {rc}'
            try:
                text = Path(row.log_path or '').read_text(encoding='utf-8')
                if (
                    'Rollback to previous IMAGE_TAG' in text
                    or 'Attempted rollback' in text
                ):
                    row.status = 'rolled_back'
            except OSError:
                pass
        db.commit()
    finally:
        db.close()


def _on_build_complete(
    deployment_id: int,
    rc: int | None,
    err: str | None,
    image_tag: str | None,
) -> None:
    db = SessionLocal()
    try:
        row = db.query(Deployment).filter(Deployment.id == deployment_id).first()
        if not row:
            return
        row.finished_at = utc_now()
        if image_tag:
            row.image_tag = image_tag
        if rc == 0:
            row.status = 'success'
            row.error_message = None
        else:
            row.status = 'failed'
            row.error_message = err or f'exit code {rc}'
        db.commit()
        device_id = row.device_id
        profile_id = row.profile_id
        tracked_branch = row.tracked_branch
        deploy_after = (row.requested_by or '').endswith('+deploy')
    finally:
        db.close()

    if rc == 0 and deploy_after and image_tag:
        # Chain a deploy of the freshly built tag.
        try:
            _start_deploy(
                device_id,
                image_tag=image_tag,
                profile_id=profile_id,
                tracked_branch=tracked_branch,
            )
        except Exception:  # noqa: BLE001
            pass


def _start_deploy(
    device_id: str,
    *,
    image_tag: str,
    profile_id: str | None,
    tracked_branch: str | None,
) -> dict[str, Any]:
    running = _device_has_running_job(device_id)
    if running:
        raise HTTPException(
            status_code=409,
            detail=f'Device already has a running job (deployment_id={running.id})',
        )
    target = _upsert_target(
        device_id,
        profile_id=profile_id,
        tracked_branch=tracked_branch,
        pinned_image_tag=image_tag,
    )
    resolved_profile = profile_id or target.profile_id
    db = SessionLocal()
    try:
        row = Deployment(
            device_id=device_id,
            action='deploy',
            robot_type=target.robot_type,
            site_id=target.site_id,
            profile_id=resolved_profile,
            tracked_branch=tracked_branch or target.tracked_branch,
            image_tag=image_tag,
            status='running',
            requested_by='fleet-console',
            started_at=utc_now(),
        )
        db.add(row)
        db.commit()
        db.refresh(row)
        deployment_id = row.id
        path = run_playbook(
            deployment_id=deployment_id,
            playbook='deploy.yml',
            limit=device_id,
            extra_vars={
                'image_tag': image_tag,
                'serial_batch': '1',
                'profile': resolved_profile,
            },
            on_complete=_on_playbook_complete,
        )
        row.log_path = str(path)
        db.commit()
        db.refresh(row)
        return _serialize_deployment(row)
    finally:
        db.close()


def _merge_metrics(device: dict[str, Any], metrics: dict[str, dict]) -> dict[str, Any]:
    hostname = device.get('hostname') or device.get('id')
    m = metrics.get(hostname or '', {})
    device['alive'] = m.get('alive')
    if device.get('alive') is None:
        device['alive'] = bool(device.get('online')) and bool(
            device.get('provisioned')
        )
    device['metrics'] = {
        'cpu_pct': m.get('cpu_pct'),
        'mem_pct': m.get('mem_pct'),
        'disk_pct': m.get('disk_pct'),
    }
    return device


def _attach_desired_and_drift(device: dict[str, Any], target: dict[str, Any]) -> None:
    device['target'] = target
    device['desired_branch'] = target.get('tracked_branch')
    device['desired_profile_id'] = target.get('profile_id')
    device['desired_image_tag'] = target.get('pinned_image_tag')
    running_profile = device.get('running_profile_id')
    running_tag = device.get('image_tag')
    profile_drift = bool(
        target.get('profile_id')
        and running_profile
        and target['profile_id'] != running_profile
    )
    version_drift = bool(
        target.get('pinned_image_tag')
        and running_tag
        and target['pinned_image_tag'] != running_tag
    )
    device['drift'] = {
        'profile': profile_drift,
        'version': version_drift,
        'any': profile_drift or version_drift,
    }


@app.on_event('startup')
def on_startup() -> None:
    init_db()


@app.get('/health')
def health() -> dict:
    return {'status': 'ok'}


@app.get('/api/profiles', dependencies=[Depends(require_token)])
def api_list_profiles(robot_type: str | None = None) -> dict:
    return {'profiles': list_profiles(robot_type=robot_type)}


@app.get('/api/devices', dependencies=[Depends(require_token)])
async def api_list_devices() -> dict:
    devices = list_devices()
    robots = [d for d in devices if d.get('role') == 'robot']
    robots = await enrich_devices(robots)
    metrics = device_metrics()
    latest = _latest_deployments_by_device()
    targets = _targets_by_device()
    for device in robots:
        _merge_metrics(device, metrics)
        device['last_deployment'] = latest.get(device['id']) or latest.get(
            device.get('hostname', '')
        )
        target = targets.get(device['id']) or _serialize_target(None, device['id'])
        _attach_desired_and_drift(device, target)
        # Best-effort update check (cached per branch).
        try:
            check = version_check(
                target.get('tracked_branch') or 'main',
                device.get('image_tag'),
            )
            device['update_available'] = check.get('update_available')
            device['latest_sha'] = check.get('latest_sha')
        except Exception:
            device['update_available'] = None
            device['latest_sha'] = None
    return {'devices': robots}


@app.get('/api/devices/{device_id}', dependencies=[Depends(require_token)])
async def api_get_device(device_id: str) -> dict:
    device = get_device(device_id)
    if not device:
        raise HTTPException(
            status_code=404, detail='Device not found in Tailscale inventory'
        )
    device = await enrich_device(device)
    _merge_metrics(device, device_metrics())
    db = SessionLocal()
    try:
        target_row = _get_or_create_target(
            db,
            device_id,
            robot_type=device.get('robot_type'),
            site_id=device.get('site_id'),
        )
        target = _serialize_target(target_row, device_id)
        rows = (
            db.query(Deployment)
            .filter(Deployment.device_id == device_id)
            .order_by(Deployment.started_at.desc())
            .limit(50)
            .all()
        )
        history = [_serialize_deployment(r) for r in rows]
    finally:
        db.close()
    _attach_desired_and_drift(device, target)
    device['deployments'] = history
    device['last_deployment'] = history[0] if history else None
    device['grafana_url'] = os.environ.get(
        'GRAFANA_PI_OVERVIEW_URL',
        'http://127.0.0.1:3001/d/pi-overview/rhapsodi-pi-overview',
    )
    try:
        check = version_check(target['tracked_branch'], device.get('image_tag'))
        device['version_check'] = check
        device['update_available'] = check.get('update_available')
        device['latest_sha'] = check.get('latest_sha')
    except Exception as exc:  # noqa: BLE001
        device['version_check'] = {'error': str(exc)}
        device['update_available'] = None
        device['latest_sha'] = None
    return {'device': device}


@app.put('/api/devices/{device_id}/target', dependencies=[Depends(require_token)])
def api_put_target(device_id: str, body: TargetUpdateRequest) -> dict:
    if get_device(device_id) is None:
        raise HTTPException(
            status_code=404, detail='Device not found in Tailscale inventory'
        )
    row = _upsert_target(
        device_id,
        tracked_branch=body.tracked_branch,
        profile_id=body.profile_id,
        pinned_image_tag=body.pinned_image_tag,
        auto_update=body.auto_update,
        robot_type=body.robot_type,
        site_id=body.site_id,
    )
    return {'target': _serialize_target(row, device_id)}


@app.get(
    '/api/devices/{device_id}/version_check',
    dependencies=[Depends(require_token)],
)
def api_version_check(device_id: str) -> dict:
    device = get_device(device_id)
    if not device:
        raise HTTPException(
            status_code=404, detail='Device not found in Tailscale inventory'
        )
    db = SessionLocal()
    try:
        target = _get_or_create_target(db, device_id)
        branch = target.tracked_branch
    finally:
        db.close()
    deployed = None
    try:
        import json as _json
        from urllib.request import urlopen

        with urlopen(f"http://{device['ip']}:8000/host_info", timeout=2.5) as resp:
            payload = _json.loads(resp.read().decode())
            deployed = payload.get('image_tag')
    except Exception:
        deployed = None
    try:
        check = version_check(branch, deployed)
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    return {'device_id': device_id, **check}


@app.post('/api/devices/{device_id}/provision', dependencies=[Depends(require_token)])
def api_provision(device_id: str, body: ProvisionRequest) -> dict:
    device = get_device(device_id)
    if not device:
        raise HTTPException(
            status_code=404, detail='Device not found in Tailscale inventory'
        )
    if get_profile(body.profile_id) is None:
        raise HTTPException(status_code=400, detail=f'Unknown profile {body.profile_id}')
    running = _device_has_running_job(device_id)
    if running:
        raise HTTPException(
            status_code=409,
            detail=f'Device already has a running job (deployment_id={running.id})',
        )
    _upsert_target(
        device_id,
        tracked_branch=body.tracked_branch,
        profile_id=body.profile_id,
        pinned_image_tag=body.image_tag,
        robot_type=body.robot_type,
        site_id=body.site_id,
    )
    db = SessionLocal()
    try:
        row = Deployment(
            device_id=device_id,
            action='provision',
            robot_type=body.robot_type,
            site_id=body.site_id,
            profile_id=body.profile_id,
            tracked_branch=body.tracked_branch,
            image_tag=body.image_tag,
            status='running',
            requested_by='fleet-console',
            started_at=utc_now(),
        )
        db.add(row)
        db.commit()
        db.refresh(row)
        deployment_id = row.id
        path = run_playbook(
            deployment_id=deployment_id,
            playbook='provision.yml',
            limit=device_id,
            extra_vars={
                'image_tag': body.image_tag,
                'robot_type': body.robot_type,
                'site_id': body.site_id,
                'device_id': body.device_id or device_id,
                'robot_id': body.robot_id or body.device_id or device_id,
                'profile': body.profile_id,
            },
            on_complete=_on_playbook_complete,
        )
        row.log_path = str(path)
        db.commit()
        db.refresh(row)
        return {'deployment': _serialize_deployment(row)}
    finally:
        db.close()


@app.post('/api/devices/{device_id}/deploy', dependencies=[Depends(require_token)])
def api_deploy(device_id: str, body: DeployRequest) -> dict:
    if get_device(device_id) is None:
        raise HTTPException(
            status_code=404, detail='Device not found in Tailscale inventory'
        )
    if body.profile_id and get_profile(body.profile_id) is None:
        raise HTTPException(status_code=400, detail=f'Unknown profile {body.profile_id}')
    deployment = _start_deploy(
        device_id,
        image_tag=body.image_tag,
        profile_id=body.profile_id,
        tracked_branch=body.tracked_branch,
    )
    return {'deployment': deployment}


@app.post('/api/devices/{device_id}/build', dependencies=[Depends(require_token)])
def api_build(device_id: str, body: BuildRequest) -> dict:
    if get_device(device_id) is None:
        raise HTTPException(
            status_code=404, detail='Device not found in Tailscale inventory'
        )
    running = _device_has_running_job(device_id)
    if running:
        raise HTTPException(
            status_code=409,
            detail=f'Device already has a running job (deployment_id={running.id})',
        )
    db = SessionLocal()
    try:
        target = _get_or_create_target(db, device_id)
        branch = (body.branch or target.tracked_branch or 'main').strip()
        profile_id = body.profile_id or target.profile_id
        row = Deployment(
            device_id=device_id,
            action='build',
            robot_type=target.robot_type,
            site_id=target.site_id,
            profile_id=profile_id,
            tracked_branch=branch,
            image_tag='',
            status='running',
            requested_by='fleet-console+deploy' if body.deploy_after else 'fleet-console',
            started_at=utc_now(),
        )
        db.add(row)
        db.commit()
        db.refresh(row)
        deployment_id = row.id
        path = run_branch_build(
            deployment_id=deployment_id,
            branch=branch,
            on_complete=_on_build_complete,
        )
        row.log_path = str(path)
        db.commit()
        db.refresh(row)
        return {'deployment': _serialize_deployment(row)}
    finally:
        db.close()


@app.get('/api/deployments', dependencies=[Depends(require_token)])
def api_list_deployments(
    device_id: str | None = None,
    status: str | None = None,
    limit: int = 100,
) -> dict:
    db = SessionLocal()
    try:
        q = db.query(Deployment).order_by(Deployment.started_at.desc())
        if device_id:
            q = q.filter(Deployment.device_id == device_id)
        if status:
            q = q.filter(Deployment.status == status)
        rows = q.limit(max(1, min(limit, 500))).all()
        return {'deployments': [_serialize_deployment(r) for r in rows]}
    finally:
        db.close()


@app.get('/api/deployments/{deployment_id}', dependencies=[Depends(require_token)])
def api_get_deployment(deployment_id: int) -> dict:
    db = SessionLocal()
    try:
        row = db.query(Deployment).filter(Deployment.id == deployment_id).first()
        if not row:
            raise HTTPException(status_code=404, detail='Deployment not found')
        return {'deployment': _serialize_deployment(row)}
    finally:
        db.close()


@app.get(
    '/api/deployments/{deployment_id}/logs',
    dependencies=[Depends(require_token)],
)
def api_get_logs(deployment_id: int, tail: int = 400) -> dict:
    db = SessionLocal()
    try:
        row = db.query(Deployment).filter(Deployment.id == deployment_id).first()
        if not row:
            raise HTTPException(status_code=404, detail='Deployment not found')
        path = Path(row.log_path or str(log_path_for(deployment_id)))
        if not path.is_file():
            return {'deployment_id': deployment_id, 'log': '', 'status': row.status}
        lines = path.read_text(encoding='utf-8', errors='replace').splitlines()
        return {
            'deployment_id': deployment_id,
            'log': '\n'.join(lines[-max(1, min(tail, 5000)) :]),
            'status': row.status,
        }
    finally:
        db.close()


@app.get(
    '/api/deployments/{deployment_id}/logs/stream',
    dependencies=[Depends(require_token)],
)
async def api_stream_logs(
    deployment_id: int, request: Request
) -> StreamingResponse:
    db = SessionLocal()
    try:
        row = db.query(Deployment).filter(Deployment.id == deployment_id).first()
        if not row:
            raise HTTPException(status_code=404, detail='Deployment not found')
        path = Path(row.log_path or str(log_path_for(deployment_id)))
    finally:
        db.close()

    async def event_stream():
        yield 'retry: 2000\n\n'
        offset = 0
        idle_ticks = 0
        while True:
            if await request.is_disconnected():
                break
            try:
                if path.is_file():
                    data = path.read_text(encoding='utf-8', errors='replace')
                    if len(data) > offset:
                        chunk = data[offset:]
                        offset = len(data)
                        for line in chunk.splitlines():
                            yield f'data: {line}\n'
                        yield '\n'
                        idle_ticks = 0
                    else:
                        idle_ticks += 1
                        yield ': keep-alive\n\n'
                else:
                    idle_ticks += 1
                    yield ': waiting-for-log\n\n'
            except OSError:
                yield 'event: error\ndata: log read failed\n\n'

            db2 = SessionLocal()
            try:
                current = (
                    db2.query(Deployment)
                    .filter(Deployment.id == deployment_id)
                    .first()
                )
                if current and current.status != 'running' and idle_ticks > 2:
                    yield f'event: done\ndata: {current.status}\n\n'
                    break
            finally:
                db2.close()
            await asyncio.sleep(1)

    return StreamingResponse(
        event_stream(),
        media_type='text/event-stream',
        headers={
            'Cache-Control': 'no-cache',
            'Connection': 'keep-alive',
            'X-Accel-Buffering': 'no',
        },
    )


@app.get('/api/image_tags', dependencies=[Depends(require_token)])
def api_image_tags() -> dict:
    db = SessionLocal()
    try:
        rows = (
            db.query(Deployment.image_tag)
            .order_by(Deployment.started_at.desc())
            .limit(100)
            .all()
        )
        tags: list[str] = []
        seen: set[str] = set()
        for (tag,) in rows:
            if tag and tag not in seen:
                seen.add(tag)
                tags.append(tag)
        default = os.environ.get('DEFAULT_IMAGE_TAG', '').strip()
        if default and default not in seen:
            tags.insert(0, default)
        return {'image_tags': tags}
    finally:
        db.close()


# Serve built UI if present (production). Dev UI uses Vite proxy.
if STATIC_DIR.is_dir() and (STATIC_DIR / 'index.html').is_file():
    app.mount('/assets', StaticFiles(directory=STATIC_DIR / 'assets'), name='assets')

    @app.get('/')
    def spa_index() -> FileResponse:
        return FileResponse(STATIC_DIR / 'index.html')

    @app.get('/{full_path:path}')
    def spa_fallback(full_path: str) -> FileResponse:
        candidate = STATIC_DIR / full_path
        if candidate.is_file():
            return FileResponse(candidate)
        return FileResponse(STATIC_DIR / 'index.html')
