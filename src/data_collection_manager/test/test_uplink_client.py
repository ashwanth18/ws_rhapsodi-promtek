"""Exercises UplinkClient against a real (if minimal) HTTP server, so the
wire protocol itself — not a mocked call boundary — is under test. The
server here is a deliberately small stand-in for the eventual
central-ingestion-service; it implements just enough of the protocol
(byte-offset resume, sha256 finalize) to prove the client's behavior is
correct.
"""
import hashlib
import http.server
import json
import threading
from pathlib import Path

import pytest

from data_collection_manager.uplink_client import UplinkClient, UplinkError


class _FakeIngestionHandler(http.server.BaseHTTPRequestHandler):
    # Shared across all requests to this test server instance.
    tier0_uploads = {}
    fleet_logs = {}
    blobs = {}  # blob_key -> bytearray
    finalized_blobs = set()
    completed_runs = set()
    fail_next_finalize = False

    def log_message(self, *args):  # noqa: D401 - silence test server logs
        pass

    def _send_json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length) if length else b''

        if self.path.split('?')[0].endswith('/tier0'):
            run_key = self.path.split('/')[3]
            type(self).tier0_uploads[run_key] = body
            self._send_json(200, {'status': 'ok'})
            return

        if '/fleet/' in self.path:
            device_id = self.path.split('/')[3]
            log_name = self.path.split('/')[4]
            key = f'{device_id}/{log_name}'
            type(self).fleet_logs[key] = (
                type(self).fleet_logs.get(key, b'') + body
            )
            self._send_json(200, {'received_bytes': len(body)})
            return

        if self.path.endswith('/finalize'):
            blob_key = self._extract_blob_key('/finalize')
            payload = json.loads(body.decode('utf-8'))
            data = bytes(type(self).blobs.get(blob_key, bytearray()))
            actual_sha = hashlib.sha256(data).hexdigest()
            if type(self).fail_next_finalize:
                type(self).fail_next_finalize = False
                self._send_json(409, {'error': 'forced_failure'})
                return
            if (
                actual_sha == payload.get('sha256')
                and len(data) == payload.get('size')
            ):
                type(self).finalized_blobs.add(blob_key)
                self._send_json(200, {'acked': True})
            else:
                self._send_json(409, {'error': 'checksum_mismatch'})
            return

        if self.path.endswith('/tier1/complete'):
            run_key = self.path.split('/')[3]
            type(self).completed_runs.add(run_key)
            self._send_json(200, {'acked': True})
            return

        self._send_json(404, {'error': 'not_found'})

    def do_PUT(self):
        length = int(self.headers.get('Content-Length', 0))
        body = self.rfile.read(length) if length else b''
        blob_key = self._extract_blob_key('')
        offset = int(self.headers.get('X-Upload-Offset', 0))
        buf = type(self).blobs.setdefault(blob_key, bytearray())
        if offset != len(buf):
            self._send_json(
                409,
                {
                    'error': 'offset_mismatch',
                    'received_bytes': len(buf),
                },
            )
            return
        buf.extend(body)
        self._send_json(200, {'received_bytes': len(buf)})

    def do_GET(self):
        if self.path.endswith('/status'):
            blob_key = self._extract_blob_key('/status')
            received = len(type(self).blobs.get(blob_key, b''))
            self._send_json(200, {'received_bytes': received})
            return
        self._send_json(404, {'error': 'not_found'})

    def _extract_blob_key(self, suffix: str) -> str:
        path = self.path
        if suffix:
            path = path[: -len(suffix)]
        # /v1/runs/{run_key}/tier1/{blob_key...}
        prefix_marker = '/tier1/'
        idx = path.index(prefix_marker) + len(prefix_marker)
        run_key = path.split('/')[3]
        blob_path = path[idx:]
        return f'{run_key}::{blob_path}'


@pytest.fixture()
def server():
    _FakeIngestionHandler.tier0_uploads = {}
    _FakeIngestionHandler.fleet_logs = {}
    _FakeIngestionHandler.blobs = {}
    _FakeIngestionHandler.finalized_blobs = set()
    _FakeIngestionHandler.completed_runs = set()
    _FakeIngestionHandler.fail_next_finalize = False

    httpd = http.server.HTTPServer(('127.0.0.1', 0), _FakeIngestionHandler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    try:
        yield httpd
    finally:
        httpd.shutdown()
        thread.join(timeout=5)


def _client(server) -> UplinkClient:
    port = server.server_address[1]
    return UplinkClient(base_url=f'http://127.0.0.1:{port}', timeout_seconds=5.0)


def test_sync_tier0_uploads_files_and_identity(server, tmp_path):
    metadata_path = tmp_path / 'metadata.json'
    metadata_path.write_text('{"run_id": "abc"}')
    client = _client(server)

    client.sync_tier0(
        'run-1', robot_id='robot-1', device_id='robot-1',
        files={'metadata_json': metadata_path},
    )

    uploaded = _FakeIngestionHandler.tier0_uploads['run-1']
    assert b'metadata.json' in uploaded
    assert b'"run_id": "abc"' in uploaded
    assert b'robot-1' in uploaded


def test_append_fleet_log(server):
    client = _client(server)
    client.append_fleet_log('robot-1', 'health', 'line1\nline2\n')
    assert _FakeIngestionHandler.fleet_logs['robot-1/health'] == (
        b'line1\nline2\n'
    )


def test_upload_blob_full_round_trip(server, tmp_path):
    blob_file = tmp_path / 'episode_1_0.mcap'
    blob_file.write_bytes(b'x' * (5 * 1024 * 1024 + 123))
    client = _client(server)

    client.upload_blob('run-1', '0/episode_1_0.mcap', blob_file, chunk_size=1024 * 1024)

    assert 'run-1::0/episode_1_0.mcap' in _FakeIngestionHandler.finalized_blobs
    assert len(_FakeIngestionHandler.blobs['run-1::0/episode_1_0.mcap']) == (
        5 * 1024 * 1024 + 123
    )


def test_upload_blob_resumes_from_server_reported_offset(server, tmp_path):
    blob_file = tmp_path / 'episode_1_0.mcap'
    content = b'y' * (3 * 1024 * 1024)
    blob_file.write_bytes(content)

    # Simulate a prior partial upload already sitting on the server.
    _FakeIngestionHandler.blobs['run-1::0/episode_1_0.mcap'] = bytearray(
        content[: 1024 * 1024]
    )

    client = _client(server)
    client.upload_blob('run-1', '0/episode_1_0.mcap', blob_file, chunk_size=512 * 1024)

    assert bytes(_FakeIngestionHandler.blobs['run-1::0/episode_1_0.mcap']) == content
    assert 'run-1::0/episode_1_0.mcap' in _FakeIngestionHandler.finalized_blobs


def test_upload_blob_raises_on_checksum_rejection(server, tmp_path):
    blob_file = tmp_path / 'f.bin'
    blob_file.write_bytes(b'z' * 1000)
    _FakeIngestionHandler.fail_next_finalize = True
    client = _client(server)

    with pytest.raises(UplinkError):
        client.upload_blob('run-1', '0/f.bin', blob_file)


def test_complete_tier1_marks_run_complete_on_server(server):
    client = _client(server)
    client.complete_tier1('run-1')
    assert 'run-1' in _FakeIngestionHandler.completed_runs


def test_request_raises_uplink_error_on_connection_failure():
    client = UplinkClient(base_url='http://127.0.0.1:1', timeout_seconds=1.0)
    with pytest.raises(UplinkError):
        client.complete_tier1('run-1')
