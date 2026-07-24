"""Run ansible-playbook as a subprocess and capture logs for SSE streaming."""
from __future__ import annotations

import os
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
    # Ensure inventory script is executable / findable.
    env.setdefault('ANSIBLE_CONFIG', str(ANSIBLE_DIR / 'ansible.cfg'))

    def _worker() -> None:
        rc: int | None = None
        err: str | None = None
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
                )
                rc = proc.wait()
                log.write(f'\n# finished {_utc_stamp()} rc={rc}\n')
                log.flush()
                if rc != 0:
                    err = f'ansible-playbook exited with code {rc}'
        except Exception as exc:  # noqa: BLE001
            err = str(exc)
            try:
                with open(path, 'a', encoding='utf-8') as log:
                    log.write(f'\n# error: {err}\n')
            except OSError:
                pass
            rc = 1
        on_complete(deployment_id, rc, err)

    thread = threading.Thread(
        target=_worker,
        name=f'ansible-{deployment_id}',
        daemon=True,
    )
    thread.start()
    return path
