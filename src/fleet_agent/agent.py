#!/usr/bin/env python3
"""Rhapsodi fleet-agent — pull desired state from Fleet Console and converge."""
from __future__ import annotations

import json
import logging
import os
import shutil
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

from envfile import read_env, write_env

LOG = logging.getLogger('fleet-agent')

WORKSPACE_DIR = Path(
    os.environ.get('WORKSPACE_DIR', '/opt/rhapsodi/ws_rhapsodi-promtek')
)
ENV_FILE = Path(os.environ.get('ENV_FILE', str(WORKSPACE_DIR / 'robot-prod.env')))
VERSION_FILE = Path(
    os.environ.get('VERSION_FILE', str(WORKSPACE_DIR / '.rhapsodi-version'))
)
LAST_GOOD_FILE = Path(
    os.environ.get('LAST_GOOD_FILE', str(WORKSPACE_DIR / '.rhapsodi-last-good'))
)
FLEET_CONSOLE_URL = os.environ.get(
    'FLEET_CONSOLE_URL', 'http://127.0.0.1:8090'
).rstrip('/')
AGENT_TOKEN = os.environ.get('AGENT_TOKEN', '').strip()
POLL_INTERVAL = int(os.environ.get('POLL_INTERVAL_SECONDS', '30'))
HEALTH_URL = os.environ.get('HEALTH_URL', 'http://127.0.0.1:8000/health')
HEALTH_RETRIES = int(os.environ.get('HEALTH_RETRIES', '30'))
HEALTH_DELAY = int(os.environ.get('HEALTH_DELAY_SECONDS', '5'))
GIT_REMOTE = os.environ.get('GIT_REMOTE', 'origin')
IMAGE_REGISTRY_DEFAULT = os.environ.get(
    'IMAGE_REGISTRY', 'iserenity/rhapsodi-promtek'
)


def _request(
    method: str,
    path: str,
    body: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if not AGENT_TOKEN:
        raise RuntimeError('AGENT_TOKEN is not set')
    url = f'{FLEET_CONSOLE_URL}{path}'
    data = None
    headers = {
        'Authorization': f'Bearer {AGENT_TOKEN}',
        'Accept': 'application/json',
        'User-Agent': 'rhapsodi-fleet-agent',
    }
    if body is not None:
        data = json.dumps(body).encode()
        headers['Content-Type'] = 'application/json'
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    try:
        with urllib.request.urlopen(req, timeout=30) as resp:
            raw = resp.read().decode()
            return json.loads(raw) if raw.strip() else {}
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode(errors='replace')[:300]
        raise RuntimeError(f'{method} {path} -> {exc.code}: {detail}') from exc


def fetch_desired() -> dict[str, Any]:
    payload = _request('GET', '/api/agent/target')
    return payload.get('target') or {}


def report(
    *,
    status: str,
    message: str | None = None,
    applied_release_id: int | None = None,
    profile_id: str | None = None,
    image_tag: str | None = None,
) -> None:
    try:
        _request(
            'POST',
            '/api/agent/report',
            {
                'status': status,
                'message': message,
                'applied_release_id': applied_release_id,
                'profile_id': profile_id,
                'image_tag': image_tag,
            },
        )
    except Exception as exc:  # noqa: BLE001
        LOG.warning('Failed to report status=%s: %s', status, exc)


def load_local_state() -> dict[str, Any]:
    if not VERSION_FILE.is_file():
        return {}
    try:
        return json.loads(VERSION_FILE.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return {}


def save_local_state(state: dict[str, Any]) -> None:
    VERSION_FILE.write_text(json.dumps(state, indent=2) + '\n', encoding='utf-8')


def save_last_good(state: dict[str, Any]) -> None:
    LAST_GOOD_FILE.write_text(json.dumps(state, indent=2) + '\n', encoding='utf-8')


def load_last_good() -> dict[str, Any]:
    if not LAST_GOOD_FILE.is_file():
        return {}
    try:
        return json.loads(LAST_GOOD_FILE.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError):
        return {}


def needs_apply(desired: dict[str, Any], current: dict[str, Any]) -> bool:
    if desired.get('noop'):
        return False
    if not desired.get('release_id'):
        return False
    if str(current.get('release_id') or '') != str(desired.get('release_id') or ''):
        return True
    if (current.get('profile_id') or '') != (desired.get('profile_id') or ''):
        return True
    if (current.get('image_tag') or '') != (desired.get('git_sha') or ''):
        return True
    return False


def _run(
    cmd: list[str],
    *,
    cwd: Path | None = None,
    env: dict[str, str] | None = None,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    LOG.info('$ %s', ' '.join(cmd))
    merged = os.environ.copy()
    if env:
        merged.update(env)
    result = subprocess.run(
        cmd,
        cwd=str(cwd or WORKSPACE_DIR),
        check=False,
        env=merged,
        text=True,
        capture_output=True,
    )
    if result.stdout:
        for line in result.stdout.splitlines()[-40:]:
            LOG.info('  | %s', line)
    if result.stderr:
        for line in result.stderr.splitlines()[-40:]:
            LOG.warning('  ! %s', line)
    if check and result.returncode != 0:
        detail = (result.stderr or result.stdout or '').strip()
        # Prefer the useful docker/git lines over a bare exit code.
        tail = detail[-800:] if detail else '(no output)'
        raise RuntimeError(
            f'Command failed (rc={result.returncode}): {" ".join(cmd)}\n{tail}'
        )
    return result


def _ensure_https_origin() -> None:
    """Prefer HTTPS origin — Pi hosts usually have no GitHub SSH deploy key."""
    try:
        result = _run(['git', 'remote', 'get-url', GIT_REMOTE], check=False)
    except Exception:  # noqa: BLE001
        return
    url = (result.stdout or '').strip()
    if url.startswith('git@github.com:'):
        https = 'https://github.com/' + url[len('git@github.com:') :]
        LOG.info('Rewriting %s remote to HTTPS (no SSH key on device)', GIT_REMOTE)
        _run(['git', 'remote', 'set-url', GIT_REMOTE, https], check=False)


def checkout_deploy_tag(git_sha: str) -> None:
    tag = f'deploy-{git_sha}'
    # Fast path: tag already present from a prior provision/fetch.
    local = _run(
        ['git', 'rev-parse', '-q', '--verify', f'refs/tags/{tag}'],
        check=False,
    )
    if local.returncode == 0:
        _run(['git', 'checkout', '--force', tag])
        return

    _ensure_https_origin()
    git_env = {'GIT_TERMINAL_PROMPT': '0'}
    fetched = _run(
        [
            'git',
            'fetch',
            '--depth',
            '1',
            GIT_REMOTE,
            f'refs/tags/{tag}:refs/tags/{tag}',
        ],
        env=git_env,
        check=False,
    )
    if fetched.returncode != 0:
        fetched = _run(
            ['git', 'fetch', '--depth', '1', GIT_REMOTE, tag],
            env=git_env,
            check=False,
        )
    if fetched.returncode != 0:
        err = (fetched.stderr or fetched.stdout or '').strip()
        raise RuntimeError(
            f'git fetch {tag} failed (rc={fetched.returncode}). '
            f'Is the deploy tag published, and can this device reach GitHub over HTTPS? '
            f'{err[:400]}'
        )
    _run(['git', 'checkout', '--force', tag])


def render_env(desired: dict[str, Any]) -> None:
    git_sha = desired['git_sha']
    registry = desired.get('image_registry') or IMAGE_REGISTRY_DEFAULT
    images = desired.get('images') or {}
    profile_env = desired.get('env') or {}
    compose_file = desired.get('compose_file') or 'docker-compose.robot-prod.yml'

    # Ensure env file exists from example on first run.
    if not ENV_FILE.is_file():
        example = WORKSPACE_DIR / 'robot-prod.env.example'
        if example.is_file():
            shutil.copy(example, ENV_FILE)
        else:
            ENV_FILE.write_text('', encoding='utf-8')

    updates: dict[str, str] = {
        'IMAGE_TAG': git_sha,
        'PROFILE_ID': desired.get('profile_id') or '',
        'COMPOSE_FILE': compose_file,
        'ROS_PROD_IMAGE': images.get('ros-prod')
        or f'{registry}:ros-prod-{git_sha}',
        'BACKEND_IMAGE': images.get('backend')
        or f'{registry}:backend-{git_sha}',
        'PROCESSING_IMAGE': images.get('processing')
        or f'{registry}:processing-{git_sha}',
        'WEBHOOK_IMAGE': images.get('webhook')
        or f'{registry}:webhook-{git_sha}',
        'DASHBOARD_IMAGE': images.get('dashboard')
        or f'{registry}:dashboard-{git_sha}',
        'CONDOR_AGENT_IMAGE': images.get('condor-agent')
        or f'{registry}:condor-agent-{git_sha}',
    }
    for key, value in profile_env.items():
        updates[str(key)] = str(value)
    write_env(ENV_FILE, updates)


def compose_file_path(desired: dict[str, Any]) -> Path:
    name = desired.get('compose_file') or 'docker-compose.robot-prod.yml'
    return WORKSPACE_DIR / name


def compose_up(desired: dict[str, Any]) -> None:
    compose = compose_file_path(desired)
    if not compose.is_file():
        raise FileNotFoundError(f'Missing compose file {compose}')
    _run(
        [
            'docker',
            'compose',
            '--env-file',
            str(ENV_FILE),
            '-f',
            str(compose),
            'pull',
        ]
    )
    _run(
        [
            'docker',
            'compose',
            '--env-file',
            str(ENV_FILE),
            '-f',
            str(compose),
            'up',
            '-d',
            '--remove-orphans',
        ]
    )
    exporters = WORKSPACE_DIR / 'monitoring/exporters/docker-compose.exporters.yml'
    if exporters.is_file():
        try:
            _run(['docker', 'compose', '-f', str(exporters), 'up', '-d'])
        except subprocess.CalledProcessError as exc:
            LOG.warning('Exporters compose failed (non-fatal): %s', exc)


def wait_health() -> bool:
    for attempt in range(1, HEALTH_RETRIES + 1):
        try:
            with urllib.request.urlopen(HEALTH_URL, timeout=5) as resp:
                if 200 <= resp.status < 300:
                    LOG.info('Health OK on attempt %s', attempt)
                    return True
        except Exception as exc:  # noqa: BLE001
            LOG.info('Health wait %s/%s: %s', attempt, HEALTH_RETRIES, exc)
        time.sleep(HEALTH_DELAY)
    return False


def apply_desired(desired: dict[str, Any]) -> dict[str, Any]:
    git_sha = desired['git_sha']
    release_id = desired['release_id']
    profile_id = desired.get('profile_id')
    report(
        status='applying',
        message=f'Applying release {release_id} ({git_sha}) profile={profile_id}',
        applied_release_id=release_id,
        profile_id=profile_id,
        image_tag=git_sha,
    )
    checkout_deploy_tag(git_sha)
    render_env(desired)
    compose_up(desired)
    if not wait_health():
        raise RuntimeError(f'Health check failed after applying {git_sha}')
    state = {
        'release_id': release_id,
        'image_tag': git_sha,
        'profile_id': profile_id,
        'deployed_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'robot_type': desired.get('robot_type'),
        'site_id': desired.get('site_id'),
        # Full desired snapshot so rollback needs no yaml/catalog.
        'desired': desired,
    }
    save_local_state(state)
    save_last_good(state)
    return state


def rollback_last_good() -> dict[str, Any] | None:
    last = load_last_good()
    if not last.get('image_tag') and not (last.get('desired') or {}).get('git_sha'):
        LOG.error('No last-good state available for rollback')
        return None
    LOG.warning('Rolling back to last-good release=%s', last.get('release_id'))
    desired = last.get('desired')
    if not isinstance(desired, dict) or not desired.get('git_sha'):
        # Legacy fallback from env file only.
        env = read_env(ENV_FILE)
        registry = IMAGE_REGISTRY_DEFAULT
        git_sha = last['image_tag']
        desired = {
            'release_id': last.get('release_id'),
            'git_sha': git_sha,
            'profile_id': last.get('profile_id'),
            'compose_file': env.get('COMPOSE_FILE')
            or 'docker-compose.robot-prod.yml',
            'image_registry': registry,
            'env': {},
            'images': {
                'ros-prod': env.get('ROS_PROD_IMAGE')
                or f'{registry}:ros-prod-{git_sha}',
                'backend': env.get('BACKEND_IMAGE')
                or f'{registry}:backend-{git_sha}',
                'processing': env.get('PROCESSING_IMAGE')
                or f'{registry}:processing-{git_sha}',
                'webhook': env.get('WEBHOOK_IMAGE')
                or f'{registry}:webhook-{git_sha}',
                'dashboard': env.get('DASHBOARD_IMAGE')
                or f'{registry}:dashboard-{git_sha}',
                'condor-agent': env.get('CONDOR_AGENT_IMAGE')
                or f'{registry}:condor-agent-{git_sha}',
            },
            'robot_type': last.get('robot_type'),
            'site_id': last.get('site_id'),
        }
    git_sha = desired['git_sha']
    checkout_deploy_tag(git_sha)
    render_env(desired)
    compose_up(desired)
    if not wait_health():
        raise RuntimeError('Health check failed after rollback')
    state = {
        'release_id': desired.get('release_id'),
        'image_tag': git_sha,
        'profile_id': desired.get('profile_id'),
        'deployed_at': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        'robot_type': desired.get('robot_type'),
        'site_id': desired.get('site_id'),
        'desired': desired,
    }
    save_local_state(state)
    return state


def reconcile_once() -> None:
    desired = fetch_desired()
    current = load_local_state()
    if not needs_apply(desired, current):
        if desired.get('release_id'):
            report(
                status='converged',
                message='Already at desired state',
                applied_release_id=desired.get('release_id'),
                profile_id=desired.get('profile_id'),
                image_tag=desired.get('git_sha'),
            )
        return

    LOG.info(
        'Drift detected: desired release=%s profile=%s (current release=%s profile=%s)',
        desired.get('release_id'),
        desired.get('profile_id'),
        current.get('release_id'),
        current.get('profile_id'),
    )
    try:
        state = apply_desired(desired)
        report(
            status='success',
            message='Applied desired state',
            applied_release_id=state.get('release_id'),
            profile_id=state.get('profile_id'),
            image_tag=state.get('image_tag'),
        )
    except Exception as exc:  # noqa: BLE001
        LOG.exception('Apply failed: %s', exc)
        try:
            rolled = rollback_last_good()
            report(
                status='rolled_back',
                message=f'Apply failed ({exc}); rolled back to {rolled}',
                applied_release_id=(rolled or {}).get('release_id'),
                profile_id=(rolled or {}).get('profile_id'),
                image_tag=(rolled or {}).get('image_tag'),
            )
        except Exception as rb_exc:  # noqa: BLE001
            LOG.exception('Rollback also failed: %s', rb_exc)
            report(
                status='failed',
                message=f'Apply failed ({exc}); rollback failed ({rb_exc})',
                applied_release_id=desired.get('release_id'),
                profile_id=desired.get('profile_id'),
                image_tag=desired.get('git_sha'),
            )


def main() -> int:
    logging.basicConfig(
        level=os.environ.get('LOG_LEVEL', 'INFO'),
        format='%(asctime)s %(levelname)s %(name)s: %(message)s',
    )
    if not AGENT_TOKEN:
        LOG.error('AGENT_TOKEN is required')
        return 2
    if not WORKSPACE_DIR.is_dir():
        LOG.error('WORKSPACE_DIR does not exist: %s', WORKSPACE_DIR)
        return 2

    LOG.info(
        'Starting fleet-agent console=%s workspace=%s poll=%ss',
        FLEET_CONSOLE_URL,
        WORKSPACE_DIR,
        POLL_INTERVAL,
    )
    while True:
        try:
            reconcile_once()
        except Exception as exc:  # noqa: BLE001
            LOG.exception('Reconcile error: %s', exc)
            report(status='failed', message=str(exc))
        time.sleep(POLL_INTERVAL)


if __name__ == '__main__':
    sys.exit(main())
