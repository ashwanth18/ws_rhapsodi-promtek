"""Run buildx_push_images.sh + publish_deploy_bundle.sh for a branch."""
from __future__ import annotations

import os
import subprocess
import tempfile
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable

REPO_ROOT = Path(
    os.environ.get('REPO_ROOT', str(Path(__file__).resolve().parents[3]))
)
LOG_DIR = Path(os.environ.get('FLEET_LOG_DIR', '/data/logs'))
LOG_DIR.mkdir(parents=True, exist_ok=True)


def _utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def log_path_for(deployment_id: int) -> Path:
    return LOG_DIR / f'deployment-{deployment_id}.log'


def run_branch_build(
    *,
    deployment_id: int,
    branch: str,
    on_complete: Callable[[int, int | None, str | None, str | None], None],
) -> Path:
    """Build+publish images for branch HEAD in a temp worktree; return log path.

    on_complete(deployment_id, rc, err, image_tag)
    """
    path = log_path_for(deployment_id)
    branch = branch.strip()

    def _worker() -> None:
        rc: int | None = None
        err: str | None = None
        image_tag: str | None = None
        worktree: Path | None = None
        try:
            with open(path, 'a', encoding='utf-8') as log:
                log.write(f'# started {_utc_stamp()}\n')
                log.write(f'# build branch={branch}\n')
                log.flush()

                # Fetch + resolve SHA from the mounted monorepo.
                fetch = subprocess.run(
                    ['git', 'fetch', 'origin', branch],
                    cwd=str(REPO_ROOT),
                    capture_output=True,
                    text=True,
                )
                log.write(fetch.stdout or '')
                log.write(fetch.stderr or '')
                log.flush()
                if fetch.returncode != 0:
                    raise RuntimeError(f'git fetch origin {branch} failed')

                sha_proc = subprocess.run(
                    ['git', 'rev-parse', '--short', f'origin/{branch}'],
                    cwd=str(REPO_ROOT),
                    capture_output=True,
                    text=True,
                    check=True,
                )
                image_tag = sha_proc.stdout.strip()
                log.write(f'# resolved origin/{branch} -> {image_tag}\n')
                log.flush()

                worktree = Path(tempfile.mkdtemp(prefix='fleet-build-'))
                add = subprocess.run(
                    [
                        'git',
                        'worktree',
                        'add',
                        '--detach',
                        str(worktree),
                        f'origin/{branch}',
                    ],
                    cwd=str(REPO_ROOT),
                    capture_output=True,
                    text=True,
                )
                log.write(add.stdout or '')
                log.write(add.stderr or '')
                log.flush()
                if add.returncode != 0:
                    raise RuntimeError('git worktree add failed')

                env = os.environ.copy()
                env.setdefault('BUILDER_NAME', 'multiarch')
                build = subprocess.run(
                    ['bash', 'scripts/buildx_push_images.sh'],
                    cwd=str(worktree),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    env=env,
                    text=True,
                )
                if build.returncode != 0:
                    raise RuntimeError(
                        f'buildx_push_images.sh exited {build.returncode}'
                    )

                publish = subprocess.run(
                    ['bash', 'scripts/publish_deploy_bundle.sh'],
                    cwd=str(worktree),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    env=env,
                    text=True,
                )
                if publish.returncode != 0:
                    raise RuntimeError(
                        f'publish_deploy_bundle.sh exited {publish.returncode}'
                    )

                log.write(f'\n# finished {_utc_stamp()} rc=0 image_tag={image_tag}\n')
                log.flush()
                rc = 0
        except Exception as exc:  # noqa: BLE001
            err = str(exc)
            rc = 1
            try:
                with open(path, 'a', encoding='utf-8') as log:
                    log.write(f'\n# error: {err}\n')
            except OSError:
                pass
        finally:
            if worktree is not None:
                subprocess.run(
                    ['git', 'worktree', 'remove', '--force', str(worktree)],
                    cwd=str(REPO_ROOT),
                    capture_output=True,
                )
            on_complete(deployment_id, rc, err, image_tag)

    thread = threading.Thread(
        target=_worker, name=f'build-{deployment_id}', daemon=True
    )
    thread.start()
    return path
