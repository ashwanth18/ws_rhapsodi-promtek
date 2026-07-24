"""HTTP client for the store-and-forward uplink (uplink-daemon).

Talks to the Rhapsodi ingestion service (`central-ingestion-service`,
built next) over a small versioned wire protocol designed for the
"small local storage + intermittent connectivity" edge deployment target
called out in the plan:

- Fleet-wide append-only log (``health.jsonl``): tailed incrementally by
  byte offset, no checksum needed — a resend of a few already-seen lines
  after a crash is harmless (the receiving side just appends text).
- Tier 0 (``metadata.json``, ``features.parquet``, ``events.jsonl``): one
  multipart POST per run — small enough that whole-file-at-once is fine.
- Tier 1 (``run.mcap`` bag directories, future ``vision/``): each regular
  file inside is uploaded individually as a "blob" with byte-offset
  resume (ask the server how much it already has, send the rest) and a
  server-side sha256 check on finalize, so a bag that's mostly uploaded
  before a WiFi drop resumes instead of restarting from zero, and a
  corrupted transfer is caught before the local copy is ever deleted.

This module only implements the *client* side of the protocol; see
`central-ingestion-service` for the matching FastAPI receiver. Every
method raises `UplinkError` on failure (never a bare `urllib`/socket
exception) so callers have one exception type to catch.
"""
from __future__ import annotations

import hashlib
import json
import mimetypes
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional

DEFAULT_CHUNK_SIZE = 4 * 1024 * 1024  # 4 MiB
DEFAULT_TIMEOUT_SECONDS = 30.0


class UplinkError(RuntimeError):
    """Raised for any uplink HTTP/transport failure. Always safe to catch
    and retry on the next pass — nothing about this client's state is
    corrupted by a failed call."""


def sha256_of_file(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass
class UplinkClient:
    base_url: str
    timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS

    def __post_init__(self) -> None:
        self.base_url = self.base_url.rstrip('/')

    def _request(
        self,
        method: str,
        path: str,
        data: Optional[bytes] = None,
        headers: Optional[Dict[str, str]] = None,
    ) -> bytes:
        url = f'{self.base_url}{path}'
        req = urllib.request.Request(
            url, data=data, headers=headers or {}, method=method
        )
        try:
            with urllib.request.urlopen(
                req, timeout=self.timeout_seconds
            ) as resp:
                return resp.read()
        except urllib.error.HTTPError as exc:
            body = exc.read().decode('utf-8', errors='replace')
            raise UplinkError(
                f'{method} {url} -> HTTP {exc.code}: {body[:500]}'
            ) from exc
        except (urllib.error.URLError, OSError, TimeoutError) as exc:
            raise UplinkError(f'{method} {url} -> {exc!r}') from exc

    def _request_json(self, method: str, path: str, payload: Any) -> Dict[str, Any]:
        body = json.dumps(payload).encode('utf-8')
        raw = self._request(
            method, path, data=body, headers={'Content-Type': 'application/json'}
        )
        try:
            return json.loads(raw.decode('utf-8')) if raw else {}
        except json.JSONDecodeError as exc:
            raise UplinkError(f'Non-JSON response from {method} {path}') from exc

    # --- Fleet-wide append-only log sync -------------------------------

    def append_fleet_log(self, device_id: str, log_name: str, text: str) -> None:
        """Append `text` (new lines only — offset bookkeeping is the
        caller's job) to the fleet-wide log named `log_name` (e.g.
        "health") for `device_id`."""
        self._request(
            'POST',
            f'/v1/fleet/{device_id}/{log_name}',
            data=text.encode('utf-8'),
            headers={'Content-Type': 'text/plain; charset=utf-8'},
        )

    # --- Tier 0 (small, whole-file) -------------------------------------

    def sync_tier0(
        self,
        run_key: str,
        robot_id: str,
        device_id: str,
        files: Dict[str, Path],
    ) -> None:
        """Upload every Tier-0 file for `run_key` in one multipart POST.
        `files` maps a stable field name (e.g. "metadata_json") to the
        file's local path; missing files are skipped, not an error (not
        every run has produced a features.parquet yet, for instance).
        """
        boundary = uuid.uuid4().hex
        parts = []
        for field_name, file_path in files.items():
            if not file_path.is_file():
                continue
            content_type = (
                mimetypes.guess_type(file_path.name)[0]
                or 'application/octet-stream'
            )
            parts.append(
                f'--{boundary}\r\n'
                f'Content-Disposition: form-data; name="{field_name}"; '
                f'filename="{file_path.name}"\r\n'
                f'Content-Type: {content_type}\r\n\r\n'.encode('utf-8')
                + file_path.read_bytes()
                + b'\r\n'
            )
        for field_name, value in (
            ('robot_id', robot_id),
            ('device_id', device_id),
        ):
            parts.append(
                f'--{boundary}\r\n'
                f'Content-Disposition: form-data; name="{field_name}"\r\n\r\n'
                f'{value}\r\n'.encode('utf-8')
            )
        parts.append(f'--{boundary}--\r\n'.encode('utf-8'))
        body = b''.join(parts)
        self._request(
            'POST',
            f'/v1/runs/{run_key}/tier0',
            data=body,
            headers={
                'Content-Type': f'multipart/form-data; boundary={boundary}'
            },
        )

    # --- Tier 1 (bulky, resumable, checksum-verified) -------------------

    def blob_status(self, run_key: str, blob_key: str) -> int:
        """Bytes the server already has for this blob (0 if unseen) —
        the resume offset for the next `upload_blob_chunk` call."""
        result = self._request_json(
            'GET', f'/v1/runs/{run_key}/tier1/{blob_key}/status', {}
        )
        return int(result.get('received_bytes', 0))

    def upload_blob(
        self,
        run_key: str,
        blob_key: str,
        file_path: Path,
        chunk_size: int = DEFAULT_CHUNK_SIZE,
    ) -> None:
        """Upload `file_path` as `blob_key`, resuming from whatever byte
        offset the server reports, then finalize with a sha256 check.
        Raises UplinkError (without partial local side effects) if the
        server rejects the checksum on finalize — the caller should
        simply retry on the next pass, which restarts the blob from
        offset 0 server-side.
        """
        size = file_path.stat().st_size
        offset = self.blob_status(run_key, blob_key)
        if offset > size:
            # Server's partial upload is somehow larger than the source
            # file now claims to be — never trust it, restart clean.
            offset = 0
        with open(file_path, 'rb') as f:
            f.seek(offset)
            while offset < size:
                chunk = f.read(chunk_size)
                if not chunk:
                    break
                self._request(
                    'PUT',
                    f'/v1/runs/{run_key}/tier1/{blob_key}',
                    data=chunk,
                    headers={
                        'Content-Type': 'application/octet-stream',
                        'X-Upload-Offset': str(offset),
                    },
                )
                offset += len(chunk)
        digest = sha256_of_file(file_path)
        result = self._request_json(
            'POST',
            f'/v1/runs/{run_key}/tier1/{blob_key}/finalize',
            {'sha256': digest, 'size': size},
        )
        if not result.get('acked'):
            raise UplinkError(
                f'Server did not ack finalize for {run_key}/{blob_key}: '
                f'{result}'
            )

    def complete_tier1(self, run_key: str) -> None:
        """Tell the server every Tier-1 blob for this run has been
        finalized, so it can mark the run fully ingested. Only call this
        after every `upload_blob` for the run has succeeded."""
        result = self._request_json(
            'POST', f'/v1/runs/{run_key}/tier1/complete', {}
        )
        if not result.get('acked'):
            raise UplinkError(
                f'Server did not ack tier1 completion for {run_key}: '
                f'{result}'
            )
