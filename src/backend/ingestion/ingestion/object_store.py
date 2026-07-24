"""Blob storage for Tier-1 uploads (and the occasional Tier-0
features.parquet), abstracted behind a small interface so the local
filesystem implementation used today can be swapped for S3/MinIO later
without touching `main.py`'s upload/resume/finalize logic.

Every key is a plain relative path (e.g. ``"{run_key}/{blob_key}"``); the
in-progress/partial upload for a key lives alongside it with a
``.part`` suffix so a half-received blob and a finalized one are never
confused with each other, and a failed checksum can delete just the
`.part` file to force the client to restart clean (see
``main.py``'s finalize handler).
"""
from __future__ import annotations

import hashlib
import shutil
from pathlib import Path
from typing import Protocol


class ObjectStore(Protocol):
    def part_size(self, key: str) -> int: ...

    def append_part(self, key: str, offset: int, data: bytes) -> int: ...

    def part_sha256(self, key: str) -> str: ...

    def finalize_part(self, key: str) -> None: ...

    def delete_part(self, key: str) -> None: ...

    def write(self, key: str, data: bytes) -> None: ...

    def read(self, key: str) -> bytes: ...

    def exists(self, key: str) -> bool: ...


class LocalFilesystemObjectStore:
    """Stores every key under `root`, mirroring the key's path segments as
    real directories - good enough for a single-node deployment and for
    unit tests; a future S3/MinIO-backed implementation would satisfy the
    same interface.
    """

    def __init__(self, root: Path) -> None:
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)

    def _final_path(self, key: str) -> Path:
        return self.root / key

    def _part_path(self, key: str) -> Path:
        return self.root / f'{key}.part'

    def part_size(self, key: str) -> int:
        part_path = self._part_path(key)
        if part_path.is_file():
            return part_path.stat().st_size
        final_path = self._final_path(key)
        if final_path.is_file():
            # Already finalized in a prior pass - report its full size so
            # a client that retries the same blob sees "fully received"
            # rather than starting over.
            return final_path.stat().st_size
        return 0

    def append_part(self, key: str, offset: int, data: bytes) -> int:
        part_path = self._part_path(key)
        part_path.parent.mkdir(parents=True, exist_ok=True)
        current_size = part_path.stat().st_size if part_path.is_file() else 0
        if offset != current_size:
            raise ValueError(
                f'offset mismatch for {key}: expected {current_size}, '
                f'got {offset}'
            )
        with open(part_path, 'ab') as f:
            f.write(data)
        return part_path.stat().st_size

    def part_sha256(self, key: str) -> str:
        path = self._part_path(key)
        if not path.is_file():
            path = self._final_path(key)
        digest = hashlib.sha256()
        with open(path, 'rb') as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b''):
                digest.update(chunk)
        return digest.hexdigest()

    def finalize_part(self, key: str) -> None:
        part_path = self._part_path(key)
        final_path = self._final_path(key)
        final_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(part_path), str(final_path))

    def delete_part(self, key: str) -> None:
        part_path = self._part_path(key)
        if part_path.is_file():
            part_path.unlink()

    def write(self, key: str, data: bytes) -> None:
        final_path = self._final_path(key)
        final_path.parent.mkdir(parents=True, exist_ok=True)
        final_path.write_bytes(data)

    def read(self, key: str) -> bytes:
        return self._final_path(key).read_bytes()

    def exists(self, key: str) -> bool:
        return self._final_path(key).is_file()
