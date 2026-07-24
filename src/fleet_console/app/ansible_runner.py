"""Run ansible-playbook as a subprocess and capture logs for SSE streaming."""
from __future__ import annotations

import os
import signal
import subprocess
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

REPO_ROOT = Path(
    os.environ.get(
        'REPO_ROOT',
        str(Path(__file__).resolve().parents[3]),
    )
)
ANSIBLE_DIR = Path(os.environ.get('ANSIBLE_DIR', str(REPO_ROOT / 'ansible')))
LOG_DIR = Path(os.environ.get('FLEET_LOG_DIR', '/data/logs'))
LOG_DIR.mkdir(parents=True, exist_ok=True)

INVENTORY = str(ANSIBLE_DIR / 'inventory' / 'tailscale.py')

# deployment_id -> Popen
_PROCS: dict[int, subprocess.Popen] = {}
_PROCS_LOCK = threading.Lock()


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def log_path_for(deployment_id: int) -> Path:
    return LOG_DIR / f'deployment-{deployment_id}.log'


def build_command(
    playbook: str,
    limit: str,
    extra_vars: dict[str, str],
) -> list[str]:
    playbook_path = str(ANSIBLE_DIR / playbook)
    cmd = [
        'ansible-playbook',
        '-i',
        INVENTORY,
        playbook_path,
        '--limit',
        limit,
    ]
    for key, value in extra_vars.items():
        if value is None:
            continue
        cmd.extend(['-e', f'{key}={value}'])
    return cmd


def cancel_playbook(deployment_id: int) -> bool:
    """Terminate a running ansible-playbook for deployment_id. Returns True if signalled."""
    signalled = False
    with _PROCS_LOCK:
        proc = _PROCS.get(deployment_id)
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            signalled = True
        except (ProcessLookupError, PermissionError, OSError):
            try:
                proc.terminate()
                signalled = True
            except Exception:  # noqa: BLE001
                pass
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except (ProcessLookupError, PermissionError, OSError):
                try:
                    proc.kill()
                except Exception:  # noqa: BLE001
                    pass
        with _PROCS_LOCK:
            _PROCS.pop(deployment_id, None)
    else:
        # Fallback: kill orphaned ansible from a previous console process.
        try:
            subprocess.run(
                ['pkill', '-f', f'ansible-playbook .*deployment.id={deployment_id}'],
                check=False,
                timeout=5,
            )
        except Exception:  # noqa: BLE001
            pass
        try:
            # Broad fallback — only one provision/deploy at a time per console.
            rc = subprocess.run(
                ['pkill', '-f', 'ansible-playbook .*/ansible/provision.yml'],
                check=False,
                timeout=5,
            )
            if rc.returncode == 0:
                signalled = True
            rc2 = subprocess.run(
                ['pkill', '-f', 'ansible-playbook .*/ansible/deploy.yml'],
                check=False,
                timeout=5,
            )
            if rc2.returncode == 0:
                signalled = True
        except Exception:  # noqa: BLE001
            pass
    try:
        with open(log_path_for(deployment_id), 'a', encoding='utf-8') as log:
            log.write(f'\n# cancelled {_utc_stamp()} by operator\n')
    except OSError:
        pass
    return signalled

def run_playbook(
    *,
    deployment_id: int,
    playbook: str,
    limit: str,
    extra_vars: dict[str, str],
    on_complete: Callable[[int, int | None, str | None], None],
) -> Path:
    """Start ansible-playbook in a daemon thread; return log path immediately."""
    path = log_path_for(deployment_id)
    cmd = build_command(playbook, limit, extra_vars)
    env = os.environ.copy()
    env.setdefault('ANSIBLE_CONFIG', str(ANSIBLE_DIR / 'ansible.cfg'))

    def _worker() -> None:
        rc: int | None = None
        err: str | None = None
        cancelled = False
        try:
            with open(path, 'a', encoding='utf-8') as log:
                log.write(f'# started {_utc_stamp()}\n')
                log.write(f'# cmd: {" ".join(cmd)}\n')
                log.flush()
                proc = subprocess.Popen(
                    cmd,
                    cwd=str(ANSIBLE_DIR),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    env=env,
                    text=True,
                    start_new_session=True,  # own process group for cancel
                )
                with _PROCS_LOCK:
                    _PROCS[deployment_id] = proc
                rc = proc.wait()
                with _PROCS_LOCK:
                    _PROCS.pop(deployment_id, None)
                # Detect cancel: negative signal / our cancel marker.
                if rc is not None and rc < 0:
                    cancelled = True
                    err = 'cancelled by operator'
                try:
                    text = path.read_text(encoding='utf-8', errors='replace')
                    if 'cancelled' in text[-400:]:
                        cancelled = True
                        err = 'cancelled by operator'
                except OSError:
                    pass
                log.write(f'\n# finished {_utc_stamp()} rc={rc}\n')
                log.flush()
                if cancelled:
                    rc = -1
                    err = 'cancelled by operator'
                elif rc != 0:
                    err = f'ansible-playbook exited with code {rc}'
        except Exception as exc:  # noqa: BLE001
            err = str(exc)
            try:
                with open(path, 'a', encoding='utf-8') as log:
                    log.write(f'\n# error: {err}\n')
            except OSError:
                pass
            rc = 1
            with _PROCS_LOCK:
                _PROCS.pop(deployment_id, None)
        on_complete(deployment_id, rc, err)

    thread = threading.Thread(
        target=_worker,
        name=f'ansible-{deployment_id}',
        daemon=True,
    )
    thread.start()
    return path
