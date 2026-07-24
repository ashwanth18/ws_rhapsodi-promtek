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
from .db import Deployment, SessionLocal, init_db, utc_now
from .inventory import get_device, list_devices
from .prometheus import device_metrics
from .robot_poll import enrich_device, enrich_devices

app = FastAPI(title='Rhapsodi Fleet Console', version='0.1.0')

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
    device_id: str | None = None
    robot_id: str | None = None


class DeployRequest(BaseModel):
    image_tag: str = Field(min_length=1)


def _serialize_deployment(row: Deployment) -> dict[str, Any]:
    return {
        'id': row.id,
        'device_id': row.device_id,
        'action': row.action,
        'robot_type': row.robot_type,
        'site_id': row.site_id,
        'image_tag': row.image_tag,
        'status': row.status,
        'requested_by': row.requested_by,
        'log_path': row.log_path,
        'error_message': row.error_message,
        'started_at': row.started_at.isoformat() if row.started_at else None,
        'finished_at': row.finished_at.isoformat() if row.finished_at else None,
    }


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
        else:
            # deploy.yml may have rolled back; surface as failed (alert).
            row.status = 'failed'
            row.error_message = err or f'exit code {rc}'
            # Heuristic: if log mentions rollback, mark rolled_back.
            try:
                text = Path(row.log_path or '').read_text(encoding='utf-8')
                if 'Rollback to previous IMAGE_TAG' in text or 'Attempted rollback' in text:
                    row.status = 'rolled_back'
            except OSError:
                pass
        db.commit()
    finally:
        db.close()


def _merge_metrics(device: dict[str, Any], metrics: dict[str, dict]) -> dict[str, Any]:
    hostname = device.get('hostname') or device.get('id')
    m = metrics.get(hostname or '', {})
    device['alive'] = m.get('alive')
    if device.get('alive') is None:
        # Fall back to Tailscale online + backend reachability.
        device['alive'] = bool(device.get('online')) and bool(
            device.get('provisioned')
        )
    device['metrics'] = {
        'cpu_pct': m.get('cpu_pct'),
        'mem_pct': m.get('mem_pct'),
        'disk_pct': m.get('disk_pct'),
    }
    return device


@app.on_event('startup')
def on_startup() -> None:
    init_db()


@app.get('/health')
def health() -> dict:
    return {'status': 'ok'}


@app.get('/api/devices', dependencies=[Depends(require_token)])
async def api_list_devices() -> dict:
    devices = list_devices()
    # Only show robots in the primary fleet list (builders optional later).
    robots = [d for d in devices if d.get('role') == 'robot']
    robots = await enrich_devices(robots)
    metrics = device_metrics()
    latest = _latest_deployments_by_device()
    for device in robots:
        _merge_metrics(device, metrics)
        device['last_deployment'] = latest.get(device['id']) or latest.get(
            device.get('hostname', '')
        )
    return {'devices': robots}


@app.get('/api/devices/{device_id}', dependencies=[Depends(require_token)])
async def api_get_device(device_id: str) -> dict:
    device = get_device(device_id)
    if not device:
        raise HTTPException(status_code=404, detail='Device not found in Tailscale inventory')
    device = await enrich_device(device)
    _merge_metrics(device, device_metrics())
    db = SessionLocal()
    try:
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
    device['deployments'] = history
    device['last_deployment'] = history[0] if history else None
    device['grafana_url'] = os.environ.get(
        'GRAFANA_PI_OVERVIEW_URL',
        'http://127.0.0.1:3001/d/pi-overview/rhapsodi-pi-overview',
    )
    return {'device': device}


@app.post('/api/devices/{device_id}/provision', dependencies=[Depends(require_token)])
def api_provision(device_id: str, body: ProvisionRequest) -> dict:
    device = get_device(device_id)
    if not device:
        raise HTTPException(status_code=404, detail='Device not found in Tailscale inventory')
    running = _device_has_running_job(device_id)
    if running:
        raise HTTPException(
            status_code=409,
            detail=f'Device already has a running job (deployment_id={running.id})',
        )
    db = SessionLocal()
    try:
        row = Deployment(
            device_id=device_id,
            action='provision',
            robot_type=body.robot_type,
            site_id=body.site_id,
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
    device = get_device(device_id)
    if not device:
        raise HTTPException(status_code=404, detail='Device not found in Tailscale inventory')
    running = _device_has_running_job(device_id)
    if running:
        raise HTTPException(
            status_code=409,
            detail=f'Device already has a running job (deployment_id={running.id})',
        )
    db = SessionLocal()
    try:
        row = Deployment(
            device_id=device_id,
            action='deploy',
            robot_type=None,
            site_id=None,
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
            playbook='deploy.yml',
            limit=device_id,
            extra_vars={
                'image_tag': body.image_tag,
                'serial_batch': '1',
            },
            on_complete=_on_playbook_complete,
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
                        # SSE data lines must not contain raw newlines unescaped.
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
    """Recent image tags from local deployment history (best-effort)."""
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
        # Also surface env-configured default / latest known.
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
