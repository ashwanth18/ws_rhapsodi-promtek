"""Rhapsodi Fleet Console API — pull-based control plane."""
from __future__ import annotations

import asyncio
import json
import os
import secrets
import subprocess
import time
from pathlib import Path
from typing import Any

from fastapi import Depends, FastAPI, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from .ansible_runner import cancel_playbook, log_path_for, run_playbook
from .auth import require_agent, require_ci_token, require_token
from .db import (
    CustomDashboard,
    Deployment,
    DeviceTarget,
    Release,
    SessionLocal,
    init_db,
    utc_now,
)
from .github import (
    latest_workflow_runs,
    latest_commit,
    list_branches,
    trigger_workflow,
    version_check,
)
from .inventory import get_device, list_devices
from .metric_catalog import load_metric_catalog
from .profiles import (
    compose_for_device_class,
    get_device_class,
    get_profile,
    list_device_classes,
    list_profiles,
    list_robot_types,
    profile_allows_device_class,
)
from . import alerts as alerts_mod
from . import loki as loki_mod
from .prometheus import (
    device_metrics,
    device_series,
    fleet_summary,
    label_names as prom_label_names,
    label_values as prom_label_values,
    metric_names as prom_metric_names,
    query_instant,
    query_range as prom_query_range,
)
from .robot_poll import enrich_device, enrich_devices
from . import settings_store
from .platforms import (
    infer_device_class,
    manifest_supports_platform,
    normalize_platform,
    parse_device_classes,
    parse_platforms,
    platforms_from_timings,
    release_device_classes,
    release_platforms,
    release_supports_device_class,
    release_supports_platform,
)


def _image_registry() -> str:
    return settings_store.get('image_registry', 'iserenity/rhapsodi-promtek')


def _fleet_console_url() -> str:
    return settings_store.get('fleet_console_url', 'http://127.0.0.1:8090')


def _grafana_url() -> str:
    return settings_store.get(
        'grafana_pi_overview_url',
        'http://127.0.0.1:3001/d/pi-overview/rhapsodi-pi-overview',
    )


def _write_log(deployment_id: int, text: str, *, append: bool = False) -> Path:
    path = log_path_for(deployment_id)
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = 'a' if append else 'w'
    with path.open(mode, encoding='utf-8') as fh:
        fh.write(text if text.endswith('\n') else text + '\n')
    return path


def _ts() -> str:
    return utc_now().strftime('%Y-%m-%dT%H:%M:%SZ')

app = FastAPI(title='Rhapsodi Fleet Console', version='0.3.0')

app.add_middleware(
    CORSMiddleware,
    allow_origins=os.environ.get('CORS_ORIGINS', '*').split(','),
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

STATIC_DIR = Path(__file__).resolve().parent.parent / 'static'
IMAGE_ROLES = (
    'ros-prod',
    'backend',
    'processing',
    'webhook',
    'ingestion',
    'dashboard',
    'condor-agent',
)


class ReleaseReportRequest(BaseModel):
    branch: str = 'main'
    git_sha: str = Field(min_length=4)
    status: str = 'success'  # success | failed
    images: dict[str, str] | None = None
    image_registry: str | None = None
    workflow_run_url: str | None = None
    error_message: str | None = None
    duration_seconds: int | None = None
    build_timings: dict[str, Any] | None = None
    # OCI platforms this release was built for, e.g. ["linux/amd64","linux/arm64"]
    platforms: list[str] | None = None
    # Hardware classes this release is intended for, e.g. ["pi5"]
    device_classes: list[str] | None = None
    # Commit subject / one-line changelog (from CI git log).
    subject: str | None = None


class ProvisionRequest(BaseModel):
    robot_type: str = Field(min_length=1)
    site_id: str = 'site-1'
    release_id: int
    profile_id: str = 'prod-niryo'
    tracked_branch: str = 'main'
    device_id: str | None = None
    robot_id: str | None = None


class DeployRequest(BaseModel):
    release_id: int
    profile_id: str | None = None
    tracked_branch: str | None = None


class TargetUpdateRequest(BaseModel):
    tracked_branch: str | None = None
    profile_id: str | None = None
    release_id: int | None = None
    auto_update: bool | None = None
    robot_type: str | None = None
    site_id: str | None = None
    platform: str | None = None
    device_class: str | None = None


class SettingsUpdateRequest(BaseModel):
    fleet_console_url: str | None = None
    image_registry: str | None = None
    github_repo: str | None = None
    prometheus_url: str | None = None
    loki_url: str | None = None
    alertmanager_url: str | None = None
    grafana_pi_overview_url: str | None = None
    # Secrets: omit or null = leave unchanged; empty string clears.
    fleet_api_token: str | None = None
    ci_report_token: str | None = None
    github_token: str | None = None


class SettingsTestRequest(BaseModel):
    """Optional draft values from the Settings form (unsaved)."""
    fleet_console_url: str | None = None
    image_registry: str | None = None
    github_repo: str | None = None
    prometheus_url: str | None = None
    loki_url: str | None = None
    alertmanager_url: str | None = None
    grafana_pi_overview_url: str | None = None
    fleet_api_token: str | None = None
    ci_report_token: str | None = None
    github_token: str | None = None


class DashboardCreateRequest(BaseModel):
    name: str = Field(min_length=1, max_length=256)
    scope: str = 'fleet'  # fleet | device_template
    panels: list[dict[str, Any]] | None = None


class DashboardUpdateRequest(BaseModel):
    name: str | None = None
    scope: str | None = None
    panels: list[dict[str, Any]] | None = None


class BuildRequest(BaseModel):
    branch: str | None = None


class AgentReportRequest(BaseModel):
    applied_release_id: int | None = None
    profile_id: str | None = None
    status: str = Field(min_length=1)  # success | rolled_back | failed | applying | converged
    message: str | None = None
    image_tag: str | None = None
    # Host identity for arch-aware deploy filtering
    platform: str | None = None  # linux/arm64 | linux/amd64 | aarch64 | …
    device_class: str | None = None  # pi5 | jetson | x86
    arch: str | None = None  # raw uname -m fallback
    # Runtime mode from robot backend GET /runtime/mode (heartbeat cache)
    active_mode: str | None = None
    environment: str | None = None


def _repo_root() -> Path:
    if os.environ.get('REPO_ROOT'):
        return Path(os.environ['REPO_ROOT'])
    here = Path(__file__).resolve()
    try:
        return here.parents[3]
    except IndexError:
        return Path('/repo')


def _commit_subject(sha: str | None, stored: str | None = None) -> str | None:
    """Prefer CI-stored subject; fall back to local git log."""
    if stored and stored.strip():
        return stored.strip()
    short = (sha or '').strip()
    if not short or short == 'deadbee':
        return None
    try:
        raw = subprocess.check_output(
            ['git', 'log', '-1', '--format=%s', short],
            cwd=str(_repo_root()),
            text=True,
            timeout=5,
            stderr=subprocess.DEVNULL,
        ).strip()
        return raw or None
    except (subprocess.CalledProcessError, FileNotFoundError, OSError):
        return None


def _sha_matches(a: str | None, b: str | None) -> bool:
    if not a or not b:
        return False
    left, right = a.strip(), b.strip()
    return left == right or left.startswith(right) or right.startswith(left)


def _short_sha(sha: str | None) -> str | None:
    if not sha:
        return None
    return sha.strip()[:7]


def _release_version(row: Release) -> str:
    """Human release number (monotonic DB id) + short sha — not SemVer."""
    short = _short_sha(row.git_sha) or 'unknown'
    return f'#{row.id} ({short})'


def _serialize_release(row: Release) -> dict[str, Any]:
    images = {}
    if row.images:
        try:
            images = json.loads(row.images)
        except json.JSONDecodeError:
            images = {}
    timings: dict[str, Any] = {}
    if getattr(row, 'timings', None):
        try:
            timings = json.loads(row.timings)  # type: ignore[arg-type]
        except json.JSONDecodeError:
            timings = {}
    duration = getattr(row, 'duration_seconds', None)
    if duration is None and isinstance(timings, dict):
        duration = timings.get('total_seconds')
    plats = release_platforms(row)
    classes = release_device_classes(row)
    subject = _commit_subject(row.git_sha, getattr(row, 'subject', None))
    return {
        'id': row.id,
        'branch': row.branch,
        'git_sha': row.git_sha,
        'short_sha': _short_sha(row.git_sha),
        # App-style version for operators: Release #N (shortsha). Not SemVer.
        'version': _release_version(row),
        'status': row.status,
        'images': images,
        'image_registry': row.image_registry or _image_registry(),
        'workflow_run_url': row.workflow_run_url,
        'duration_seconds': duration,
        'build_timings': timings or None,
        'platforms': plats,
        'device_classes': classes,
        'error_message': row.error_message,
        'reported_at': row.reported_at.isoformat() if row.reported_at else None,
        'subject': subject,
        'demo': row.git_sha == 'deadbee',
    }


def _serialize_deployment(row: Deployment) -> dict[str, Any]:
    return {
        'id': row.id,
        'device_id': row.device_id,
        'action': row.action,
        'robot_type': row.robot_type,
        'site_id': row.site_id,
        'profile_id': row.profile_id,
        'tracked_branch': row.tracked_branch,
        'release_id': row.release_id,
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
        inferred = infer_device_class(device_id=device_id, hostname=device_id)
        return {
            'device_id': device_id,
            'tracked_branch': 'main',
            'profile_id': 'prod-niryo',
            'release_id': None,
            'auto_update': False,
            'robot_type': None,
            'site_id': None,
            'platform': None,
            'device_class': inferred,
            'compose_file': compose_for_device_class(inferred),
            'agent_status': None,
            'agent_message': None,
            'agent_applied_release_id': None,
            'agent_reported_at': None,
            'active_mode': None,
            'environment': None,
            'has_agent_token': False,
            'updated_at': None,
        }
    device_class = row.device_class or infer_device_class(
        device_id=row.device_id,
        hostname=row.device_id,
        platform=row.platform,
    )
    return {
        'device_id': row.device_id,
        'tracked_branch': row.tracked_branch,
        'profile_id': row.profile_id,
        'release_id': row.release_id,
        'auto_update': bool(row.auto_update),
        'robot_type': row.robot_type,
        'site_id': row.site_id,
        'platform': row.platform,
        'device_class': device_class,
        'compose_file': compose_for_device_class(device_class),
        'agent_status': row.agent_status,
        'agent_message': row.agent_message,
        'agent_applied_release_id': row.agent_applied_release_id,
        'agent_reported_at': (
            row.agent_reported_at.isoformat() if row.agent_reported_at else None
        ),
        'active_mode': row.active_mode,
        'environment': row.environment,
        'has_agent_token': bool(row.agent_token),
        'updated_at': row.updated_at.isoformat() if row.updated_at else None,
    }


def _get_release(db, release_id: int) -> Release:
    row = db.query(Release).filter(Release.id == release_id).first()
    if not row:
        raise HTTPException(status_code=404, detail=f'Release {release_id} not found')
    return row


def _default_images(git_sha: str, registry: str) -> dict[str, str]:
    return {role: f'{registry}:{role}-{git_sha}' for role in IMAGE_ROLES}


_REGISTRY_PROBE_CACHE: dict[str, tuple[float, bool | None]] = {}
_REGISTRY_PROBE_TTL = 300.0  # seconds


def _registry_tag_exists(repository: str, tag: str) -> bool | None:
    """Probe image tag via local docker (uses logged-in creds for private repos).

    Returns True/False when docker can answer, or None if the check cannot run.
    Anonymous Docker Hub API is NOT used — private repos look like 404 there.
    """
    import time

    ref = f'{repository}:{tag}'
    now = time.time()
    cached = _REGISTRY_PROBE_CACHE.get(ref)
    if cached and now - cached[0] < _REGISTRY_PROBE_TTL:
        return cached[1]
    try:
        result = subprocess.run(
            ['docker', 'manifest', 'inspect', ref],
            check=False,
            capture_output=True,
            text=True,
            timeout=45,
        )
    except (FileNotFoundError, OSError, subprocess.TimeoutExpired):
        _REGISTRY_PROBE_CACHE[ref] = (now, None)
        return None
    if result.returncode == 0:
        _REGISTRY_PROBE_CACHE[ref] = (now, True)
        return True
    err = (result.stderr or result.stdout or '').lower()
    # Definite miss vs auth/network ambiguity.
    if 'no such' in err or 'not found' in err or 'manifest unknown' in err:
        _REGISTRY_PROBE_CACHE[ref] = (now, False)
        return False
    _REGISTRY_PROBE_CACHE[ref] = (now, None)
    return None


def _release_images_on_registry(release: Release) -> tuple[bool, str]:
    """Best-effort check that the primary backend image exists in the registry."""
    registry = (release.image_registry or _image_registry()).strip()
    if registry.count('/') != 1:
        return True, 'skipped (non Docker Hub-style registry)'
    tag = f'backend-{release.git_sha}'
    exists = _registry_tag_exists(registry, tag)
    if exists is True:
        return True, f'{registry}:{tag} found'
    if exists is False:
        return False, (
            f'{registry}:{tag} is missing — a git deploy-* tag is not enough; '
            f'CI must push images before this Release is deployable'
        )
    return True, 'registry check unavailable; allowing'


def _assert_release_deployable(
    release: Release,
    *,
    device_platform: str | None = None,
    device_class: str | None = None,
) -> None:
    if release.status != 'success':
        raise HTTPException(
            status_code=400,
            detail=f'Release {release.id} is not deployable (status={release.status})',
        )
    ok, reason = _release_images_on_registry(release)
    if not ok:
        raise HTTPException(
            status_code=400,
            detail=f'Release {release.id} ({release.git_sha}) not deployable: {reason}',
        )
    if not release_supports_device_class(release, device_class):
        allowed = release_device_classes(release)
        raise HTTPException(
            status_code=400,
            detail=(
                f'Release {release.id} ({release.git_sha}) is for device_classes '
                f'{allowed} but device is {device_class}'
            ),
        )
    want = normalize_platform(device_platform)
    if not want:
        return
    recorded = release_platforms(release)
    if recorded and want not in recorded:
        raise HTTPException(
            status_code=400,
            detail=(
                f'Release {release.id} ({release.git_sha}) was built for '
                f'[{", ".join(recorded)}] but device needs {want}'
            ),
        )
    registry = (release.image_registry or _image_registry()).strip()
    tag = f'backend-{release.git_sha}'
    supported, mreason = manifest_supports_platform(registry, tag, want)
    if supported is False:
        raise HTTPException(
            status_code=400,
            detail=(
                f'Release {release.id} ({release.git_sha}) not deployable on '
                f'{want}: {mreason}'
            ),
        )


def _mint_agent_token() -> str:
    return secrets.token_urlsafe(32)


def _resolve_device_platform(device_id: str) -> str | None:
    """Best-known OCI platform for a device (agent report → class heuristic)."""
    db = SessionLocal()
    try:
        row = db.query(DeviceTarget).filter(DeviceTarget.device_id == device_id).first()
        if row and row.platform:
            return normalize_platform(row.platform)
        dclass = (row.device_class if row else None) or infer_device_class(
            device_id=device_id, hostname=device_id
        )
        class_info = get_device_class(dclass)
        if class_info and class_info.get('platform'):
            return normalize_platform(class_info['platform'])
        if dclass in ('pi5', 'jetson'):
            return 'linux/arm64'
        if dclass == 'x86':
            return 'linux/amd64'
        return None
    finally:
        db.close()


def _resolve_device_class(device_id: str) -> str | None:
    db = SessionLocal()
    try:
        row = db.query(DeviceTarget).filter(DeviceTarget.device_id == device_id).first()
        if row and row.device_class:
            return row.device_class
        return infer_device_class(
            device_id=device_id,
            hostname=device_id,
            platform=row.platform if row else None,
        )
    finally:
        db.close()


def _get_or_create_target(
    db,
    device_id: str,
    *,
    robot_type: str | None = None,
    site_id: str | None = None,
    profile_id: str | None = None,
    tracked_branch: str | None = None,
    platform: str | None = None,
    device_class: str | None = None,
) -> DeviceTarget:
    row = db.query(DeviceTarget).filter(DeviceTarget.device_id == device_id).first()
    if row is None:
        default_profile = (
            profile_id
            or ('prod-jaka' if robot_type == 'jaka' else 'prod-niryo')
        )
        plat = normalize_platform(platform)
        dclass = device_class or infer_device_class(
            device_id=device_id, hostname=device_id, platform=plat
        )
        row = DeviceTarget(
            device_id=device_id,
            tracked_branch=tracked_branch or 'main',
            profile_id=default_profile,
            robot_type=robot_type,
            site_id=site_id,
            platform=plat,
            device_class=dclass,
            agent_token=_mint_agent_token(),
            updated_at=utc_now(),
        )
        db.add(row)
        db.commit()
        db.refresh(row)
    else:
        dirty = False
        if not row.agent_token:
            row.agent_token = _mint_agent_token()
            dirty = True
        if not row.device_class:
            row.device_class = infer_device_class(
                device_id=device_id,
                hostname=device_id,
                platform=row.platform,
            )
            dirty = True
        if dirty:
            db.commit()
            db.refresh(row)
    return row


def _upsert_target(
    device_id: str,
    *,
    tracked_branch: str | None = None,
    profile_id: str | None = None,
    release_id: int | None = None,
    auto_update: bool | None = None,
    robot_type: str | None = None,
    site_id: str | None = None,
    platform: str | None = None,
    device_class: str | None = None,
    allow_robot_type_change: bool = False,
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
            platform=platform,
            device_class=device_class,
        )
        if tracked_branch is not None:
            row.tracked_branch = tracked_branch
        if platform is not None:
            row.platform = normalize_platform(platform)
        if device_class is not None:
            row.device_class = device_class.strip() or infer_device_class(
                device_id=device_id, platform=row.platform
            )
            class_info = get_device_class(row.device_class)
            if class_info and not row.platform:
                row.platform = normalize_platform(class_info.get('platform'))
        if profile_id is not None:
            if get_profile(profile_id) is None:
                raise HTTPException(status_code=400, detail=f'Unknown profile {profile_id}')
            if row.device_class and not profile_allows_device_class(
                profile_id, row.device_class
            ):
                raise HTTPException(
                    status_code=400,
                    detail=(
                        f'Profile {profile_id} is not allowed on device_class '
                        f'{row.device_class}'
                    ),
                )
            row.profile_id = profile_id
        if release_id is not None:
            release = _get_release(db, release_id)
            _assert_release_deployable(
                release,
                device_platform=row.platform,
                device_class=row.device_class,
            )
            row.release_id = release_id
        if auto_update is not None:
            row.auto_update = auto_update
        if robot_type is not None:
            if row.robot_type and row.robot_type != robot_type and not allow_robot_type_change:
                raise HTTPException(
                    status_code=400,
                    detail=(
                        f'robot_type is immutable after first set '
                        f'(current={row.robot_type})'
                    ),
                )
            known = list_robot_types()
            if known and robot_type not in known:
                raise HTTPException(
                    status_code=400,
                    detail=f'Unknown robot_type {robot_type}. Known: {", ".join(known)}',
                )
            row.robot_type = robot_type
        if site_id is not None:
            row.site_id = site_id
        if not row.device_class:
            row.device_class = infer_device_class(
                device_id=device_id, platform=row.platform
            )
        row.updated_at = utc_now()
        db.commit()
        db.refresh(row)
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


def _releases_by_id(ids: set[int]) -> dict[int, dict[str, Any]]:
    if not ids:
        return {}
    db = SessionLocal()
    try:
        rows = db.query(Release).filter(Release.id.in_(ids)).all()
        return {r.id: _serialize_release(r) for r in rows}
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
        # Don't overwrite an explicit cancel already recorded by the API.
        if row.status == 'cancelled':
            if not row.finished_at:
                row.finished_at = utc_now()
            db.commit()
            return
        row.finished_at = utc_now()
        if rc == 0:
            row.status = 'success'
            row.error_message = None
        elif err and 'cancelled' in err.lower():
            row.status = 'cancelled'
            row.error_message = err
        else:
            row.status = 'failed'
            row.error_message = err or f'exit code {rc}'
            try:
                text = Path(row.log_path or '').read_text(encoding='utf-8')
                if 'cancelled by operator' in text:
                    row.status = 'cancelled'
                elif (
                    'Rollback to previous IMAGE_TAG' in text
                    or 'Attempted rollback' in text
                ):
                    row.status = 'rolled_back'
            except OSError:
                pass
        db.commit()
    finally:
        db.close()


def _sync_releases_from_git_tags() -> int:
    """Import deploy-<sha> tags that have real registry images.

    Git tags alone are not Releases — skip anything whose backend image
    is missing so the picker never offers undeliverable versions.
    """
    repo = _repo_root()
    try:
        raw = subprocess.check_output(
            ['git', 'ls-remote', '--tags', 'origin', 'deploy-*'],
            cwd=str(repo),
            text=True,
            timeout=30,
        )
    except (subprocess.CalledProcessError, FileNotFoundError, OSError):
        return 0
    created = 0
    db = SessionLocal()
    try:
        for line in raw.splitlines():
            parts = line.split()
            if len(parts) < 2:
                continue
            ref = parts[1]
            if ref.endswith('^{}'):
                continue
            tag = ref.rsplit('/', 1)[-1]
            if not tag.startswith('deploy-'):
                continue
            sha = tag[len('deploy-') :]
            if not sha or sha == 'deadbee':
                continue
            exists = (
                db.query(Release)
                .filter(Release.git_sha == sha)
                .order_by(Release.reported_at.desc())
                .first()
            )
            if exists:
                # Re-probe success rows; demote if images vanished.
                if exists.status == 'success':
                    ok, reason = _release_images_on_registry(exists)
                    if not ok:
                        exists.status = 'failed'
                        exists.error_message = reason
                continue
            images = _default_images(sha, _image_registry())
            probe = Release(
                branch='main',
                git_sha=sha,
                status='success',
                images=json.dumps(images),
                image_registry=_image_registry(),
                error_message='imported from git tag ' + tag,
                reported_at=utc_now(),
            )
            ok, reason = _release_images_on_registry(probe)
            if not ok:
                # Do not create undeliverable releases at all.
                continue
            db.add(probe)
            created += 1
        db.commit()
    finally:
        db.close()
    return created


def _prune_undeliverable_releases() -> int:
    """Mark success Releases without registry images as failed. Returns count."""
    db = SessionLocal()
    demoted = 0
    try:
        rows = (
            db.query(Release)
            .filter(Release.status == 'success')
            .order_by(Release.reported_at.desc())
            .limit(100)
            .all()
        )
        for row in rows:
            if row.git_sha == 'deadbee':
                row.status = 'failed'
                row.error_message = 'demo fake release — not on registry'
                demoted += 1
                continue
            ok, reason = _release_images_on_registry(row)
            if not ok:
                row.status = 'failed'
                row.error_message = reason
                demoted += 1
        if demoted:
            db.commit()
    finally:
        db.close()
    return demoted


def _list_picker_releases(
    *,
    branch: str | None = None,
    limit: int = 50,
    platform: str | None = None,
    device_class: str | None = None,
) -> list[dict[str, Any]]:
    """Releases the UI may offer: success + images + matching platform/class."""
    want = normalize_platform(platform)
    db = SessionLocal()
    try:
        q = (
            db.query(Release)
            .filter(Release.status == 'success')
            .filter(Release.git_sha != 'deadbee')
            .order_by(Release.reported_at.desc(), Release.id.desc())
        )
        if branch:
            q = q.filter(Release.branch == branch)
        rows = q.limit(max(1, min(limit, 200))).all()
        out: list[dict[str, Any]] = []
        dirty = False
        for row in rows:
            ok, reason = _release_images_on_registry(row)
            if not ok:
                row.status = 'failed'
                row.error_message = reason
                dirty = True
                continue
            if want and not release_supports_platform(row, want):
                continue
            if not release_supports_device_class(row, device_class):
                continue
            if want:
                registry = (row.image_registry or _image_registry()).strip()
                tag = f'backend-{row.git_sha}'
                supported, _mreason = manifest_supports_platform(registry, tag, want)
                if supported is False:
                    continue
            out.append(_serialize_release(row))
        if dirty:
            db.commit()
        return out
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


def _attach_robot_urls(device: dict[str, Any]) -> None:
    """Derive per-device robot SPA / API URLs for fleet operators.

    Prefer MagicDNS hostname (works in Tailscale browsers); fall back to the
    Tailscale IPv4. Ports match compose/devices/pi5.yml defaults.
    """
    host = (device.get('hostname') or device.get('id') or '').strip()
    ip = (device.get('ip') or '').strip()
    reach = host or ip
    if not reach:
        device['dashboard_url'] = None
        device['api_url'] = None
        return
    dash_port = (os.environ.get('ROBOT_DASHBOARD_PORT') or '8080').strip() or '8080'
    api_port = (os.environ.get('ROBOT_API_PORT') or '8000').strip() or '8000'
    device['dashboard_url'] = f'http://{reach}:{dash_port}'
    device['api_url'] = f'http://{reach}:{api_port}'
    # IP variants when hostname differs (handy if MagicDNS is slow).
    if ip and ip != reach:
        device['dashboard_url_ip'] = f'http://{ip}:{dash_port}'
        device['api_url_ip'] = f'http://{ip}:{api_port}'
    else:
        device['dashboard_url_ip'] = device['dashboard_url']
        device['api_url_ip'] = device['api_url']


def _attach_desired_and_drift(
    device: dict[str, Any],
    target: dict[str, Any],
    releases: dict[int, dict[str, Any]],
) -> None:
    device['target'] = target
    device['desired_branch'] = target.get('tracked_branch')
    device['desired_profile_id'] = target.get('profile_id')
    release_id = target.get('release_id')
    desired_release = releases.get(release_id) if release_id else None
    device['desired_release'] = desired_release
    device['desired_image_tag'] = (
        desired_release.get('git_sha') if desired_release else None
    )
    device['agent_status'] = target.get('agent_status')
    device['agent_message'] = target.get('agent_message')
    device['agent_reported_at'] = target.get('agent_reported_at')
    applied_id = target.get('agent_applied_release_id')
    applied_release = releases.get(applied_id) if applied_id else None
    device['agent_applied_release'] = applied_release

    # Prefer live /runtime/mode poll; fall back to last agent heartbeat.
    if not device.get('active_mode') and target.get('active_mode'):
        device['active_mode'] = target.get('active_mode')
    if not device.get('environment') and target.get('environment'):
        device['environment'] = target.get('environment')

    # Older backend images only return hostname from /host_info. Fall back to
    # what the agent last successfully applied so Running is not a blank dash.
    running_source = 'host_info'
    if not device.get('image_tag') and applied_release:
        device['image_tag'] = applied_release.get('git_sha')
        running_source = 'agent'
    if not device.get('running_profile_id') and target.get('agent_status') in (
        'success',
        'converged',
    ):
        # After converge, agent is on desired profile unless it reported otherwise.
        device['running_profile_id'] = target.get('profile_id')
        if running_source == 'host_info' and not device.get('image_tag'):
            running_source = 'agent'
        elif running_source != 'host_info':
            running_source = 'agent'
    device['running_source'] = running_source

    running_profile = device.get('running_profile_id')
    running_tag = device.get('image_tag')
    profile_drift = bool(
        target.get('profile_id')
        and running_profile
        and target['profile_id'] != running_profile
    )
    version_drift = bool(
        device.get('desired_image_tag')
        and running_tag
        and device['desired_image_tag'] != running_tag
        and not (running_tag or '').startswith(device['desired_image_tag'] or '')
        and not (device['desired_image_tag'] or '').startswith(running_tag or '')
    )
    # Also treat agent not yet converged as drift.
    if (
        release_id
        and applied_id
        and release_id != applied_id
        and target.get('agent_status') not in (None, 'applying')
    ):
        version_drift = True
    device['drift'] = {
        'profile': profile_drift,
        'version': version_drift,
        'any': profile_drift or version_drift,
    }


def _deployable_releases(branch: str | None = None) -> list[dict[str, Any]]:
    """Picker-grade releases (success + images on registry), newest first."""
    releases = _list_picker_releases(branch=branch, limit=100)
    if branch:
        branched = [r for r in releases if r.get('branch') == branch]
        return branched or releases
    return releases


def _attach_update_status(device: dict[str, Any], target: dict[str, Any]) -> None:
    """Set honest update fields: deployable release vs unbuilt branch tip."""
    branch = target.get('tracked_branch') or 'main'
    running = device.get('image_tag')
    desired = device.get('desired_image_tag')
    baseline = desired or running

    releases = _deployable_releases(branch)
    latest_release = releases[0] if releases else None
    current_release = None
    if baseline:
        for rel in releases:
            if _sha_matches(rel.get('git_sha'), baseline):
                current_release = rel
                break

    deployable_update = False
    if latest_release and not _sha_matches(latest_release.get('git_sha'), baseline):
        if current_release is None:
            deployable_update = True
        else:
            # Newer by reported_at / id (release table order is newest-first).
            deployable_update = latest_release['id'] != current_release['id'] and (
                (latest_release.get('reported_at') or '')
                >= (current_release.get('reported_at') or '')
            )

    device['latest_release'] = latest_release
    device['update_available'] = deployable_update
    device['latest_sha'] = (
        latest_release.get('git_sha') if deployable_update and latest_release else None
    )
    device['update_kind'] = 'release' if deployable_update else None
    device['needs_build'] = False
    device['branch_tip_sha'] = None
    device['branch_tip_message'] = None

    try:
        check = version_check(branch, running or desired)
        device['version_check'] = check
        tip = check.get('latest_sha')
        device['branch_tip_sha'] = tip
        device['branch_tip_message'] = check.get('latest_message')
        tip_is_release = bool(
            tip and any(_sha_matches(tip, r.get('git_sha')) for r in releases)
        )
        branch_ahead = bool(check.get('branch_ahead'))
        # Tip has newer commits than running, but no matching Release image yet.
        if tip and branch_ahead and not tip_is_release:
            device['needs_build'] = True
            if not deployable_update:
                device['update_kind'] = 'needs_build'
                device['latest_sha'] = tip
        # Tip behind / identical → not an update (fixes false "→ e0a9f75").
    except Exception as exc:  # noqa: BLE001
        device['version_check'] = {'error': str(exc)}


def _resolve_agent_target(target: DeviceTarget) -> dict[str, Any]:
    db = SessionLocal()
    try:
        release = None
        if target.release_id:
            release = (
                db.query(Release).filter(Release.id == target.release_id).first()
            )
        device_class = target.device_class or infer_device_class(
            device_id=target.device_id, platform=target.platform
        )
        compose_file = compose_for_device_class(device_class)
        class_info = get_device_class(device_class)
        profile = get_profile(target.profile_id) or {
            'id': target.profile_id,
            'env': {},
        }
        base = {
            'device_id': target.device_id,
            'profile_id': target.profile_id,
            'tracked_branch': target.tracked_branch,
            'robot_type': target.robot_type,
            'site_id': target.site_id,
            'platform': target.platform
            or (class_info or {}).get('platform'),
            'device_class': device_class,
            'compose_file': compose_file,
            'env': profile.get('env') or {},
            'image_registry': _image_registry(),
        }
        if release is None:
            return {
                **base,
                'release_id': None,
                'git_sha': None,
                'images': {},
                'noop': True,
            }
        images = {}
        if release.images:
            try:
                images = json.loads(release.images)
            except json.JSONDecodeError:
                images = {}
        registry = release.image_registry or _image_registry()
        if not images:
            images = _default_images(release.git_sha, registry)
        return {
            **base,
            'release_id': release.id,
            'git_sha': release.git_sha,
            'images': images,
            'image_registry': registry,
            'noop': False,
        }
    finally:
        db.close()


@app.on_event('startup')
def on_startup() -> None:
    init_db()


@app.get('/health')
def health() -> dict:
    return {'status': 'ok'}


@app.get('/api/settings', dependencies=[Depends(require_token)])
def api_get_settings() -> dict:
    return settings_store.public_view()


@app.put('/api/settings', dependencies=[Depends(require_token)])
def api_put_settings(body: SettingsUpdateRequest) -> dict:
    patch = body.model_dump(exclude_unset=True)
    for key in settings_store.SECRET_KEYS:
        if key not in patch:
            continue
        value = patch[key]
        if value is None:
            del patch[key]
            continue
        text = str(value).strip()
        # Reject masked placeholders pasted back from GET.
        if '…' in text or text.startswith('••••'):
            del patch[key]
            continue
        patch[key] = text
    settings_store.update(patch)
    return settings_store.public_view()


@app.post('/api/settings/test', dependencies=[Depends(require_token)])
def api_test_settings(
    request: Request,
    body: SettingsTestRequest | None = None,
) -> dict:
    """Probe GitHub / Prometheus / Grafana / console URL / tokens."""
    from .settings_checks import run_all

    overrides = (body.model_dump(exclude_unset=True) if body else {}) or {}
    bearer = request.headers.get('Authorization') or ''
    request_token = ''
    if bearer.lower().startswith('bearer '):
        request_token = bearer[7:].strip()
    request_token = request_token or (request.query_params.get('access_token') or '')
    return run_all(overrides, request_token=request_token or None)


@app.get('/api/profiles', dependencies=[Depends(require_token)])
def api_list_profiles(
    robot_type: str | None = None,
    device_class: str | None = None,
) -> dict:
    return {
        'profiles': list_profiles(robot_type=robot_type, device_class=device_class),
    }


@app.get('/api/device_classes', dependencies=[Depends(require_token)])
def api_list_device_classes(production_only: bool = False) -> dict:
    return {'device_classes': list_device_classes(production_only=production_only)}


@app.get('/api/robot_types', dependencies=[Depends(require_token)])
def api_robot_types() -> dict:
    return {'robot_types': list_robot_types()}


@app.get('/api/branches', dependencies=[Depends(require_token)])
def api_branches() -> dict:
    try:
        return {'branches': list_branches()}
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=502, detail=str(exc)) from exc


@app.get('/api/branches/tip', dependencies=[Depends(require_token)])
def api_branch_tip(branch: str = 'main') -> dict:
    """Newest git commit on a branch (may not have a successful Release yet).

    Query param (not path) so names like feature/fleet-console work.
    """
    try:
        tip = latest_commit(branch)
        return {'tip': tip}
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=502, detail=str(exc)) from exc


@app.get('/api/releases', dependencies=[Depends(require_token)])
def api_list_releases(
    branch: str | None = None,
    status: str | None = 'success',
    limit: int = 50,
    sync: bool = False,
    deployable_only: bool = True,
    platform: str | None = None,
    device_id: str | None = None,
    device_class: str | None = None,
) -> dict:
    """List Releases. Default: only versions whose images exist on the registry.

    When platform/device_class or device_id is set, filter to matching Releases.
    """
    synced = 0
    demoted = 0
    if sync:
        synced = _sync_releases_from_git_tags()
        demoted = _prune_undeliverable_releases()
    want = normalize_platform(platform)
    dclass = (device_class or '').strip() or None
    if device_id:
        if not want:
            want = _resolve_device_platform(device_id)
        if not dclass:
            dclass = _resolve_device_class(device_id)
    if deployable_only and (status is None or status == 'success'):
        return {
            'releases': _list_picker_releases(
                branch=branch,
                limit=limit,
                platform=want,
                device_class=dclass,
            ),
            'synced': synced,
            'demoted': demoted,
            'platform': want,
            'device_class': dclass,
        }
    db = SessionLocal()
    try:
        q = db.query(Release).order_by(Release.reported_at.desc())
        if branch:
            q = q.filter(Release.branch == branch)
        if status:
            q = q.filter(Release.status == status)
        rows = q.limit(max(1, min(limit, 200))).all()
        return {
            'releases': [_serialize_release(r) for r in rows],
            'synced': synced,
            'demoted': demoted,
        }
    finally:
        db.close()


@app.post('/api/releases/sync', dependencies=[Depends(require_token)])
def api_sync_releases() -> dict:
    """Import deploy-* tags that have registry images; prune undeliverable ones."""
    created = _sync_releases_from_git_tags()
    demoted = _prune_undeliverable_releases()
    return {
        'created': created,
        'demoted': demoted,
        'releases': _list_picker_releases(limit=100),
    }


@app.get('/api/releases/{release_id}', dependencies=[Depends(require_token)])
def api_get_release(release_id: int) -> dict:
    db = SessionLocal()
    try:
        return {'release': _serialize_release(_get_release(db, release_id))}
    finally:
        db.close()


@app.post('/api/releases/report', dependencies=[Depends(require_ci_token)])
def api_report_release(body: ReleaseReportRequest) -> dict:
    git_sha = body.git_sha.strip()
    short = git_sha[:7] if len(git_sha) >= 7 else git_sha
    registry = (body.image_registry or _image_registry()).strip()
    images = body.images or (
        _default_images(short, registry) if body.status == 'success' else {}
    )
    db = SessionLocal()
    try:
        # Upsert by (branch, git_sha) to avoid duplicates from retries.
        existing = (
            db.query(Release)
            .filter(Release.branch == body.branch, Release.git_sha == short)
            .order_by(Release.reported_at.desc())
            .first()
        )
        timings_json = (
            json.dumps(body.build_timings) if body.build_timings else None
        )
        duration = body.duration_seconds
        if duration is None and body.build_timings:
            raw_total = body.build_timings.get('total_seconds')
            if isinstance(raw_total, int):
                duration = raw_total
        plats = parse_platforms(body.platforms)
        if not plats:
            plats = platforms_from_timings(body.build_timings)
        platforms_json = json.dumps(plats) if plats else None
        classes = parse_device_classes(body.device_classes)
        # Default production robot product line when CI omits the field.
        if not classes and body.status == 'success':
            classes = ['pi5']
        classes_json = json.dumps(classes) if classes else None
        subject = (body.subject or '').strip() or _commit_subject(short)
        if existing:
            existing.status = body.status
            existing.images = json.dumps(images)
            existing.image_registry = registry
            existing.workflow_run_url = body.workflow_run_url
            existing.error_message = body.error_message
            existing.duration_seconds = duration
            existing.timings = timings_json
            if platforms_json:
                existing.platforms = platforms_json
            if classes_json:
                existing.device_classes = classes_json
            if subject:
                existing.subject = subject[:512]
            existing.reported_at = utc_now()
            db.commit()
            db.refresh(existing)
            return {'release': _serialize_release(existing), 'upserted': True}
        row = Release(
            branch=body.branch.strip() or 'main',
            git_sha=short,
            status=body.status,
            images=json.dumps(images),
            image_registry=registry,
            workflow_run_url=body.workflow_run_url,
            error_message=body.error_message,
            platforms=platforms_json,
            device_classes=classes_json,
            subject=(subject[:512] if subject else None),
            duration_seconds=duration,
            timings=timings_json,
            reported_at=utc_now(),
        )
        db.add(row)
        db.commit()
        db.refresh(row)
        return {'release': _serialize_release(row), 'upserted': False}
    finally:
        db.close()


@app.get('/api/devices', dependencies=[Depends(require_token)])
async def api_list_devices() -> dict:
    devices = list_devices()
    robots = [d for d in devices if d.get('role') == 'robot']
    robots = await enrich_devices(robots)
    metrics = device_metrics()
    latest = _latest_deployments_by_device()
    targets = _targets_by_device()
    release_ids: set[int] = set()
    for t in targets.values():
        if t.get('release_id'):
            release_ids.add(int(t['release_id']))
        if t.get('agent_applied_release_id'):
            release_ids.add(int(t['agent_applied_release_id']))
    releases = _releases_by_id(release_ids)
    for device in robots:
        _merge_metrics(device, metrics)
        device['last_deployment'] = latest.get(device['id']) or latest.get(
            device.get('hostname', '')
        )
        target = targets.get(device['id']) or _serialize_target(None, device['id'])
        _attach_desired_and_drift(device, target, releases)
        _attach_update_status(device, target)
        _attach_robot_urls(device)
        device['platform'] = target.get('platform')
        device['device_class'] = target.get('device_class')
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
            .limit(20)
            .all()
        )
        history = [_serialize_deployment(r) for r in rows]
        release_ids: set[int] = set()
        if target.get('release_id'):
            release_ids.add(int(target['release_id']))
        if target.get('agent_applied_release_id'):
            release_ids.add(int(target['agent_applied_release_id']))
        for h in history:
            if h.get('release_id'):
                release_ids.add(int(h['release_id']))
        releases = {
            r.id: _serialize_release(r)
            for r in db.query(Release).filter(Release.id.in_(release_ids)).all()
        } if release_ids else {}
    finally:
        db.close()
    _attach_desired_and_drift(device, target, releases)
    _attach_update_status(device, target)
    _attach_robot_urls(device)
    device['platform'] = target.get('platform')
    device['device_class'] = target.get('device_class')
    device['deployments'] = history
    device['last_deployment'] = history[0] if history else None
    device['grafana_url'] = _grafana_url()
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
        release_id=body.release_id,
        auto_update=body.auto_update,
        robot_type=body.robot_type,
        site_id=body.site_id,
        platform=body.platform,
        device_class=body.device_class,
        allow_robot_type_change=False,
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
        from urllib.request import urlopen

        with urlopen(f"http://{device['ip']}:8000/host_info", timeout=2.5) as resp:
            payload = json.loads(resp.read().decode())
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
    known = list_robot_types()
    if known and body.robot_type not in known:
        raise HTTPException(
            status_code=400,
            detail=f'Unknown robot_type {body.robot_type}. Known: {", ".join(known)}',
        )
    running = _device_has_running_job(device_id)
    if running:
        raise HTTPException(
            status_code=409,
            detail=f'Device already has a running job (deployment_id={running.id})',
        )

    device_platform = _resolve_device_platform(device_id) or 'linux/arm64'
    device_class = _resolve_device_class(device_id) or infer_device_class(
        device_id=device_id, hostname=device.get('hostname'), platform=device_platform
    )
    # Hostname heuristics (rhapsodi-pi5 → pi5 → arm64) apply before first agent report.
    _upsert_target(
        device_id,
        platform=device_platform,
        device_class=device_class,
    )
    db = SessionLocal()
    try:
        release = _get_release(db, body.release_id)
        _assert_release_deployable(
            release,
            device_platform=device_platform,
            device_class=device_class,
        )
        image_tag = release.git_sha
    finally:
        db.close()

    target = _upsert_target(
        device_id,
        tracked_branch=body.tracked_branch,
        profile_id=body.profile_id,
        release_id=body.release_id,
        robot_type=body.robot_type,
        site_id=body.site_id,
        platform=device_platform,
        device_class=device_class,
        allow_robot_type_change=True,
    )

    # Re-fetch token for Ansible extra_vars.
    db = SessionLocal()
    try:
        target_row = (
            db.query(DeviceTarget).filter(DeviceTarget.device_id == device_id).first()
        )
        agent_token = target_row.agent_token if target_row else None
        if not agent_token:
            target_row = _get_or_create_target(db, device_id)
            agent_token = target_row.agent_token
    finally:
        db.close()

    db = SessionLocal()
    try:
        row = Deployment(
            device_id=device_id,
            action='provision',
            robot_type=body.robot_type,
            site_id=body.site_id,
            profile_id=body.profile_id,
            tracked_branch=body.tracked_branch,
            release_id=body.release_id,
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
            playbook='provision.yml',
            limit=device_id,
            extra_vars={
                'image_tag': image_tag,
                'robot_type': body.robot_type,
                'site_id': body.site_id,
                'device_id': body.device_id or device_id,
                'robot_id': body.robot_id or body.device_id or device_id,
                'profile': body.profile_id,
                'agent_token': agent_token or '',
                'fleet_console_url': _fleet_console_url(),
                'release_id': str(body.release_id),
            },
            on_complete=_on_playbook_complete,
        )
        row.log_path = str(path)
        db.commit()
        db.refresh(row)
        return {
            'deployment': _serialize_deployment(row),
            'target': _serialize_target(target, device_id),
        }
    finally:
        db.close()


@app.post('/api/devices/{device_id}/deploy', dependencies=[Depends(require_token)])
def api_deploy(device_id: str, body: DeployRequest) -> dict:
    """Set desired state — job stays running until fleet-agent reports reconcile."""
    if get_device(device_id) is None:
        raise HTTPException(
            status_code=404, detail='Device not found in Tailscale inventory'
        )
    if body.profile_id and get_profile(body.profile_id) is None:
        raise HTTPException(status_code=400, detail=f'Unknown profile {body.profile_id}')

    running = _device_has_running_job(device_id)
    if running:
        raise HTTPException(
            status_code=409,
            detail=f'Device already has a running job (deployment_id={running.id})',
        )

    device_platform = _resolve_device_platform(device_id)
    device_class = _resolve_device_class(device_id)
    db = SessionLocal()
    try:
        release = _get_release(db, body.release_id)
        _assert_release_deployable(
            release,
            device_platform=device_platform,
            device_class=device_class,
        )
        image_tag = release.git_sha
        branch = body.tracked_branch or release.branch
    finally:
        db.close()

    target = _upsert_target(
        device_id,
        profile_id=body.profile_id,
        tracked_branch=branch,
        release_id=body.release_id,
        platform=device_platform,
        device_class=device_class,
    )
    profile_id = body.profile_id or target.profile_id

    db = SessionLocal()
    try:
        row = Deployment(
            device_id=device_id,
            action='deploy',
            robot_type=target.robot_type,
            site_id=target.site_id,
            profile_id=profile_id,
            tracked_branch=branch,
            release_id=body.release_id,
            image_tag=image_tag,
            status='running',
            requested_by='fleet-console',
            error_message='Awaiting fleet-agent reconcile',
            started_at=utc_now(),
            finished_at=None,
        )
        db.add(row)
        db.commit()
        db.refresh(row)
        deployment_id = row.id
        path = _write_log(
            deployment_id,
            '\n'.join(
                [
                    f'[{_ts()}] PLAY [Pull-based deploy]',
                    f'[{_ts()}] TASK [Set desired state on Fleet Console]',
                    f'[{_ts()}] ok: device={device_id}',
                    f'[{_ts()}]     release_id={body.release_id} git_sha={image_tag}',
                    f'[{_ts()}]     profile_id={profile_id} branch={branch}',
                    f'[{_ts()}] ',
                    f'[{_ts()}] TASK [Wait for fleet-agent on device]',
                    f'[{_ts()}] => Desired state is set. This job will stay running until',
                    f'[{_ts()}]    the on-device fleet-agent polls /api/agent/target, applies',
                    f'[{_ts()}]    the release (compose pull/up + health), and reports back.',
                    f'[{_ts()}] ',
                    f'[{_ts()}] NOTE: If this never advances, fleet-agent is not installed or',
                    f'[{_ts()}]       cannot reach this console. Re-run Flash install / provision',
                    f'[{_ts()}]       so the systemd unit and AGENT_TOKEN are present on the Pi.',
                    f'[{_ts()}] waiting for agent report…',
                ]
            ),
        )
        row.log_path = str(path)
        db.commit()
        db.refresh(row)
        return {
            'deployment': _serialize_deployment(row),
            'target': _serialize_target(target, device_id),
        }
    finally:
        db.close()


def _dispatch_build(
    *,
    branch: str,
    device_id: str = 'fleet',
    robot_type: str | None = None,
    site_id: str | None = None,
    profile_id: str | None = None,
) -> dict:
    """Record a build job and dispatch GitHub Actions workflow_dispatch."""
    branch = (branch or 'main').strip()
    if not branch:
        raise HTTPException(status_code=400, detail='branch is required')
    db = SessionLocal()
    try:
        row = Deployment(
            device_id=device_id,
            action='build',
            robot_type=robot_type,
            site_id=site_id,
            profile_id=profile_id,
            tracked_branch=branch,
            image_tag='',
            status='running',
            requested_by='fleet-console',
            started_at=utc_now(),
        )
        db.add(row)
        db.commit()
        db.refresh(row)
        deployment_id = row.id
    finally:
        db.close()

    try:
        result = trigger_workflow(branch)
        db = SessionLocal()
        try:
            row = db.query(Deployment).filter(Deployment.id == deployment_id).first()
            if row:
                row.status = 'success'
                row.finished_at = utc_now()
                row.error_message = result.get('message')
                row.log_path = None
                db.commit()
                db.refresh(row)
                return {
                    'deployment': _serialize_deployment(row),
                    'workflow': result,
                }
        finally:
            db.close()
    except Exception as exc:  # noqa: BLE001
        db = SessionLocal()
        try:
            row = db.query(Deployment).filter(Deployment.id == deployment_id).first()
            if row:
                row.status = 'failed'
                row.finished_at = utc_now()
                row.error_message = str(exc)
                db.commit()
                db.refresh(row)
                return {'deployment': _serialize_deployment(row), 'workflow': None}
        finally:
            db.close()
        raise HTTPException(status_code=502, detail=str(exc)) from exc
    raise HTTPException(status_code=500, detail='Build dispatch failed unexpectedly')


@app.post('/api/builds', dependencies=[Depends(require_token)])
def api_fleet_build(body: BuildRequest) -> dict:
    """Fleet-level CI build — not tied to a single device."""
    return _dispatch_build(branch=body.branch or 'main', device_id='fleet')


@app.post('/api/devices/{device_id}/build', dependencies=[Depends(require_token)])
def api_build(device_id: str, body: BuildRequest) -> dict:
    """Legacy: device-scoped build. Prefer POST /api/builds."""
    if get_device(device_id) is None:
        raise HTTPException(
            status_code=404, detail='Device not found in Tailscale inventory'
        )
    db = SessionLocal()
    try:
        target = _get_or_create_target(db, device_id)
        branch = (body.branch or target.tracked_branch or 'main').strip()
        return _dispatch_build(
            branch=branch,
            device_id=device_id,
            robot_type=target.robot_type,
            site_id=target.site_id,
            profile_id=target.profile_id,
        )
    finally:
        db.close()


@app.get('/api/workflow_runs', dependencies=[Depends(require_token)])
def api_workflow_runs(branch: str | None = None, limit: int = 20) -> dict:
    try:
        return {'runs': latest_workflow_runs(branch=branch, limit=limit)}
    except Exception as exc:  # noqa: BLE001
        raise HTTPException(status_code=502, detail=str(exc)) from exc


@app.get('/api/agent/target')
def api_agent_target(target: DeviceTarget = Depends(require_agent)) -> dict:
    return {'target': _resolve_agent_target(target)}


@app.post('/api/agent/report')
def api_agent_report(
    body: AgentReportRequest,
    target: DeviceTarget = Depends(require_agent),
) -> dict:
    db = SessionLocal()
    try:
        row = (
            db.query(DeviceTarget)
            .filter(DeviceTarget.device_id == target.device_id)
            .first()
        )
        if not row:
            raise HTTPException(status_code=404, detail='Device target missing')
        row.agent_status = body.status
        row.agent_message = body.message
        row.agent_reported_at = utc_now()
        if body.applied_release_id is not None:
            row.agent_applied_release_id = body.applied_release_id
        if body.active_mode:
            row.active_mode = body.active_mode.strip()
        if body.environment:
            row.environment = body.environment.strip()
        plat = normalize_platform(body.platform) or normalize_platform(body.arch)
        if plat:
            row.platform = plat
        if body.device_class:
            row.device_class = body.device_class.strip()
        elif plat and not row.device_class:
            row.device_class = infer_device_class(
                device_id=row.device_id, platform=plat
            )
        db.commit()

        image_tag = body.image_tag or ''
        if body.applied_release_id and not image_tag:
            release = (
                db.query(Release)
                .filter(Release.id == body.applied_release_id)
                .first()
            )
            if release:
                image_tag = release.git_sha

        mapped_status = (
            'success'
            if body.status in ('success', 'converged')
            else (
                'rolled_back'
                if body.status == 'rolled_back'
                else ('running' if body.status == 'applying' else 'failed')
            )
        )

        msg = body.message or body.status
        open_jobs = (
            db.query(Deployment)
            .filter(
                Deployment.device_id == target.device_id,
                Deployment.status == 'running',
                Deployment.action.in_(('deploy', 'reconcile')),
            )
            .order_by(Deployment.started_at.desc())
            .all()
        )

        # Append into the newest open deploy job (if any) for the console stream.
        open_deploy = next((j for j in open_jobs if j.action == 'deploy'), None)
        if open_deploy is not None:
            dep_id = open_deploy.id
            _write_log(
                dep_id,
                f'[{_ts()}] AGENT [{body.status}] release={body.applied_release_id or "-"} '
                f'profile={body.profile_id or open_deploy.profile_id} '
                f'tag={image_tag or "-"} :: {msg}',
                append=True,
            )
            if not open_deploy.log_path:
                open_deploy.log_path = str(log_path_for(dep_id))

        if mapped_status == 'running':
            # Heartbeat: update newest open reconcile, or create one.
            open_reconcile = next(
                (j for j in open_jobs if j.action == 'reconcile'), None
            )
            if open_reconcile is not None:
                open_reconcile.error_message = msg
                open_reconcile.image_tag = image_tag or open_reconcile.image_tag
                if body.applied_release_id:
                    open_reconcile.release_id = body.applied_release_id
            else:
                db.add(
                    Deployment(
                        device_id=target.device_id,
                        action='reconcile',
                        robot_type=row.robot_type,
                        site_id=row.site_id,
                        profile_id=body.profile_id or row.profile_id,
                        tracked_branch=row.tracked_branch,
                        release_id=body.applied_release_id,
                        image_tag=image_tag,
                        status='running',
                        requested_by='fleet-agent',
                        error_message=msg,
                        started_at=utc_now(),
                    )
                )
            if open_deploy is not None:
                open_deploy.error_message = msg
            db.commit()
        else:
            # Terminal agent status: close EVERY open deploy/reconcile for this device
            # so the UI never stays stuck on "running" after converge.
            if open_deploy is not None:
                if mapped_status == 'success':
                    _write_log(
                        open_deploy.id,
                        f'[{_ts()}] PLAY RECAP: agent converged ok=1 failed=0\n'
                        f'[{_ts()}] Deploy complete via fleet-agent.',
                        append=True,
                    )
                elif mapped_status == 'rolled_back':
                    _write_log(
                        open_deploy.id,
                        f'[{_ts()}] PLAY RECAP: agent rolled back after health failure',
                        append=True,
                    )
                else:
                    _write_log(
                        open_deploy.id,
                        f'[{_ts()}] PLAY RECAP: agent failed :: {msg}',
                        append=True,
                    )

            closed_any = False
            for job in open_jobs:
                job.status = mapped_status
                job.finished_at = utc_now()
                job.error_message = msg
                if body.applied_release_id:
                    job.release_id = body.applied_release_id
                if image_tag:
                    job.image_tag = image_tag
                closed_any = True

            # Skip spam: identical converged with nothing open.
            if not (
                body.status == 'converged'
                and not closed_any
                and row.release_id == body.applied_release_id
            ):
                if not closed_any:
                    db.add(
                        Deployment(
                            device_id=target.device_id,
                            action='reconcile',
                            robot_type=row.robot_type,
                            site_id=row.site_id,
                            profile_id=body.profile_id or row.profile_id,
                            tracked_branch=row.tracked_branch,
                            release_id=body.applied_release_id,
                            image_tag=image_tag,
                            status=mapped_status,
                            requested_by='fleet-agent',
                            error_message=msg,
                            started_at=utc_now(),
                            finished_at=utc_now(),
                        )
                    )
            db.commit()

        db.refresh(row)
        return {
            'ok': True,
            'target': _serialize_target(row, target.device_id),
        }
    finally:
        db.close()


@app.get('/api/deployments', dependencies=[Depends(require_token)])
def api_list_deployments(
    device_id: str | None = None,
    status: str | None = None,
    page: int = 1,
    limit: int = 20,
) -> dict:
    page = max(1, page)
    limit = max(1, min(limit, 20))
    db = SessionLocal()
    try:
        q = db.query(Deployment).order_by(Deployment.started_at.desc())
        if device_id:
            q = q.filter(Deployment.device_id == device_id)
        if status:
            q = q.filter(Deployment.status == status)
        total = q.count()
        pages = max(1, (total + limit - 1) // limit) if total else 1
        if page > pages:
            page = pages
        rows = q.offset((page - 1) * limit).limit(limit).all()
        return {
            'deployments': [_serialize_deployment(r) for r in rows],
            'page': page,
            'limit': limit,
            'total': total,
            'pages': pages,
        }
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


@app.post(
    '/api/deployments/{deployment_id}/cancel',
    dependencies=[Depends(require_token)],
)
def api_cancel_deployment(deployment_id: int) -> dict:
    """Cancel a running provision/deploy/build job."""
    db = SessionLocal()
    try:
        row = db.query(Deployment).filter(Deployment.id == deployment_id).first()
        if not row:
            raise HTTPException(status_code=404, detail='Deployment not found')
        if row.status not in ('running',):
            raise HTTPException(
                status_code=409,
                detail=f'Job is not running (status={row.status})',
            )
        signalled = False
        if row.action in ('provision',):
            signalled = cancel_playbook(deployment_id)
        # Always mark cancelled so SSE stream ends even if process already gone.
        row.status = 'cancelled'
        row.finished_at = utc_now()
        row.error_message = 'cancelled by operator'
        db.commit()
        db.refresh(row)
        try:
            _write_log(
                deployment_id,
                f'[{_ts()}] CANCELLED by operator'
                + (' (ansible process signalled)' if signalled else ''),
                append=True,
            )
            if not row.log_path:
                row.log_path = str(log_path_for(deployment_id))
                db.commit()
                db.refresh(row)
        except OSError:
            pass
        return {
            'deployment': _serialize_deployment(row),
            'signalled': signalled,
        }
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
            return {
                'deployment_id': deployment_id,
                'log': row.error_message or '',
                'status': row.status,
            }
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


def _serialize_dashboard(row: CustomDashboard) -> dict[str, Any]:
    try:
        panels = json.loads(row.panels or '[]')
    except json.JSONDecodeError:
        panels = []
    if not isinstance(panels, list):
        panels = []
    return {
        'id': row.id,
        'name': row.name,
        'scope': row.scope,
        'panels': panels,
        'created_at': row.created_at.isoformat() if row.created_at else None,
        'updated_at': row.updated_at.isoformat() if row.updated_at else None,
    }


def _substitute_device(expr: str, device_id: str | None) -> str:
    if not device_id:
        return expr
    return (
        expr.replace('$device_id', device_id)
        .replace('${device_id}', device_id)
        .replace('$instance', device_id)
    )


@app.get('/api/metrics/catalog', dependencies=[Depends(require_token)])
def api_metrics_catalog() -> dict:
    """Curated named metrics (PromQL/LogQL pre-baked) for the panel picker."""
    return {'metrics': load_metric_catalog()}


@app.get('/api/metrics/fleet', dependencies=[Depends(require_token)])
def api_metrics_fleet() -> dict:
    summary = fleet_summary()
    summary['alert_counts'] = alerts_mod.alert_counts_by_instance()
    summary['alerts_total'] = sum(summary['alert_counts'].values())
    return summary


@app.get(
    '/api/metrics/devices/{device_id}/series',
    dependencies=[Depends(require_token)],
)
def api_device_series(
    device_id: str,
    since: str = '1h',
    step: str = '30s',
) -> dict:
    # parse_since returns absolute unix start; convert to duration.
    since_seconds = max(60, int(time.time()) - loki_mod.parse_since(since, 3600))
    return device_series(device_id, since_seconds=since_seconds, step=step)


@app.get('/api/alerts', dependencies=[Depends(require_token)])
def api_alerts(instance: str | None = None) -> dict:
    return {
        'alerts': alerts_mod.list_alerts(instance=instance),
        'counts_by_instance': alerts_mod.alert_counts_by_instance(),
    }


@app.get(
    '/api/logs/devices/{device_id}/containers',
    dependencies=[Depends(require_token)],
)
def api_device_log_containers(device_id: str) -> dict:
    return {
        'device_id': device_id,
        'containers': loki_mod.containers_for_host(device_id),
    }


@app.get(
    '/api/logs/devices/{device_id}/query',
    dependencies=[Depends(require_token)],
)
def api_device_logs_query(
    device_id: str,
    container: str | None = None,
    q: str | None = None,
    since: str = '15m',
    start: float | None = None,
    end: float | None = None,
    limit: int = 500,
    direction: str = 'backward',
) -> dict:
    return loki_mod.device_logs(
        device_id,
        container=container,
        q=q,
        since=since,
        start=start,
        end=end,
        limit=max(1, min(limit, 5000)),
        direction=direction,
    )


@app.get('/api/query', dependencies=[Depends(require_token)])
def api_query_proxy(
    ds: str,
    expr: str,
    start: float | None = None,
    end: float | None = None,
    step: str = '30s',
    limit: int = 500,
    direction: str = 'backward',
    device_id: str | None = None,
) -> dict:
    """Generic Prometheus / Loki query proxy for custom dashboard panels."""
    resolved = _substitute_device(expr, device_id)
    now = time.time()
    end_ts = float(end if end is not None else now)
    start_ts = float(start if start is not None else end_ts - 3600)
    ds_norm = (ds or '').strip().lower()
    if ds_norm in ('prometheus', 'prom'):
        if start is None and end is None:
            # Instant query when no range given — useful for stat/table panels.
            if step == 'instant' or step == '0':
                return {
                    'ds': 'prometheus',
                    'expr': resolved,
                    **query_instant(resolved),
                }
        return {
            'ds': 'prometheus',
            'expr': resolved,
            'start': start_ts,
            'end': end_ts,
            **prom_query_range(resolved, start=start_ts, end=end_ts, step=step),
        }
    if ds_norm == 'loki':
        payload = loki_mod.query_range(
            resolved,
            start=start_ts,
            end=end_ts,
            limit=max(1, min(limit, 5000)),
            direction=direction,
        )
        lines = loki_mod.flatten_lines(payload.get('result') or [])
        return {
            'ds': 'loki',
            'expr': resolved,
            'start': start_ts,
            'end': end_ts,
            'status': payload.get('status'),
            'resultType': payload.get('resultType'),
            'result': payload.get('result'),
            'lines': lines,
        }
    raise HTTPException(
        status_code=400,
        detail='ds must be prometheus or loki',
    )


@app.get(
    '/api/query/meta/prometheus/metrics',
    dependencies=[Depends(require_token)],
)
def api_meta_prom_metrics(match: str | None = None, limit: int = 2000) -> dict:
    return {'metrics': prom_metric_names(prefix=match, limit=limit)}


@app.get(
    '/api/query/meta/prometheus/labels',
    dependencies=[Depends(require_token)],
)
def api_meta_prom_labels(metric: str | None = None) -> dict:
    return {'labels': prom_label_names(metric=metric)}


@app.get(
    '/api/query/meta/prometheus/label-values',
    dependencies=[Depends(require_token)],
)
def api_meta_prom_label_values(
    label: str,
    metric: str | None = None,
    match: str | None = None,
) -> dict:
    if not label.strip():
        raise HTTPException(status_code=400, detail='label is required')
    return {
        'label': label,
        'values': prom_label_values(label, metric=metric, match=match),
    }


@app.get(
    '/api/query/meta/loki/labels',
    dependencies=[Depends(require_token)],
)
def api_meta_loki_labels() -> dict:
    return {'labels': loki_mod.labels()}


@app.get(
    '/api/query/meta/loki/label-values',
    dependencies=[Depends(require_token)],
)
def api_meta_loki_label_values(
    label: str,
    match: str | None = None,
) -> dict:
    if not label.strip():
        raise HTTPException(status_code=400, detail='label is required')
    return {
        'label': label,
        'values': loki_mod.label_values(label, match=match),
    }


@app.get('/api/dashboards', dependencies=[Depends(require_token)])
def api_list_dashboards(scope: str | None = None) -> dict:
    db = SessionLocal()
    try:
        q = db.query(CustomDashboard).order_by(CustomDashboard.updated_at.desc())
        if scope:
            q = q.filter(CustomDashboard.scope == scope)
        return {'dashboards': [_serialize_dashboard(r) for r in q.all()]}
    finally:
        db.close()


@app.post('/api/dashboards', dependencies=[Depends(require_token)])
def api_create_dashboard(body: DashboardCreateRequest) -> dict:
    scope = (body.scope or 'fleet').strip()
    if scope not in ('fleet', 'device_template'):
        raise HTTPException(
            status_code=400, detail='scope must be fleet or device_template'
        )
    panels = body.panels if isinstance(body.panels, list) else []
    db = SessionLocal()
    try:
        row = CustomDashboard(
            name=body.name.strip(),
            scope=scope,
            panels=json.dumps(panels),
            created_at=utc_now(),
            updated_at=utc_now(),
        )
        db.add(row)
        db.commit()
        db.refresh(row)
        return {'dashboard': _serialize_dashboard(row)}
    finally:
        db.close()


@app.get('/api/dashboards/{dashboard_id}', dependencies=[Depends(require_token)])
def api_get_dashboard(dashboard_id: int) -> dict:
    db = SessionLocal()
    try:
        row = (
            db.query(CustomDashboard)
            .filter(CustomDashboard.id == dashboard_id)
            .first()
        )
        if not row:
            raise HTTPException(status_code=404, detail='Dashboard not found')
        return {'dashboard': _serialize_dashboard(row)}
    finally:
        db.close()


@app.put('/api/dashboards/{dashboard_id}', dependencies=[Depends(require_token)])
def api_update_dashboard(dashboard_id: int, body: DashboardUpdateRequest) -> dict:
    db = SessionLocal()
    try:
        row = (
            db.query(CustomDashboard)
            .filter(CustomDashboard.id == dashboard_id)
            .first()
        )
        if not row:
            raise HTTPException(status_code=404, detail='Dashboard not found')
        if body.name is not None:
            name = body.name.strip()
            if not name:
                raise HTTPException(status_code=400, detail='name cannot be empty')
            row.name = name
        if body.scope is not None:
            scope = body.scope.strip()
            if scope not in ('fleet', 'device_template'):
                raise HTTPException(
                    status_code=400, detail='scope must be fleet or device_template'
                )
            row.scope = scope
        if body.panels is not None:
            if not isinstance(body.panels, list):
                raise HTTPException(status_code=400, detail='panels must be a list')
            row.panels = json.dumps(body.panels)
        row.updated_at = utc_now()
        db.commit()
        db.refresh(row)
        return {'dashboard': _serialize_dashboard(row)}
    finally:
        db.close()


@app.delete('/api/dashboards/{dashboard_id}', dependencies=[Depends(require_token)])
def api_delete_dashboard(dashboard_id: int) -> dict:
    db = SessionLocal()
    try:
        row = (
            db.query(CustomDashboard)
            .filter(CustomDashboard.id == dashboard_id)
            .first()
        )
        if not row:
            raise HTTPException(status_code=404, detail='Dashboard not found')
        db.delete(row)
        db.commit()
        return {'ok': True, 'id': dashboard_id}
    finally:
        db.close()


@app.post(
    '/api/dashboards/{dashboard_id}/duplicate',
    dependencies=[Depends(require_token)],
)
def api_duplicate_dashboard(dashboard_id: int) -> dict:
    db = SessionLocal()
    try:
        row = (
            db.query(CustomDashboard)
            .filter(CustomDashboard.id == dashboard_id)
            .first()
        )
        if not row:
            raise HTTPException(status_code=404, detail='Dashboard not found')
        copy = CustomDashboard(
            name=f'{row.name} (copy)',
            scope=row.scope,
            panels=row.panels,
            created_at=utc_now(),
            updated_at=utc_now(),
        )
        db.add(copy)
        db.commit()
        db.refresh(copy)
        return {'dashboard': _serialize_dashboard(copy)}
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
