"""Exercises the ingestion FastAPI app end-to-end through its HTTP
surface (TestClient), against a temp-file object store and an
in-memory SQLite DB standing in for Postgres - the models use only
portable column types, so this is a faithful stand-in for the wire
protocol and persistence behavior without needing a running Postgres.
"""
import hashlib
import json

import pytest
from fastapi.testclient import TestClient
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from ingestion import database, main
from ingestion.models import Base, FleetHealthEvent, IngestedBlob, IngestedRun
from ingestion.object_store import LocalFilesystemObjectStore


@pytest.fixture()
def client(tmp_path, monkeypatch):
    # StaticPool: a bare `sqlite:///:memory:` engine hands out a fresh,
    # empty in-memory database per connection, so `create_all` and the
    # request-handling sessions would otherwise never see the same
    # database - StaticPool pins them all to one connection.
    engine = create_engine(
        'sqlite:///:memory:',
        connect_args={'check_same_thread': False},
        poolclass=StaticPool,
    )
    TestSessionLocal = sessionmaker(
        autocommit=False, autoflush=False, bind=engine
    )
    Base.metadata.create_all(bind=engine)
    monkeypatch.setattr(database, 'engine', engine)
    monkeypatch.setattr(database, 'SessionLocal', TestSessionLocal)
    monkeypatch.setattr(main, 'engine', engine)
    monkeypatch.setattr(main, 'SessionLocal', TestSessionLocal)

    store = LocalFilesystemObjectStore(tmp_path / 'store')
    monkeypatch.setattr(main, '_store', store)

    with TestClient(main.app) as test_client:
        yield test_client, TestSessionLocal, store


def _session(client_fixture):
    _, session_factory, _ = client_fixture
    return session_factory()


def test_health_check(client):
    test_client, _, _ = client
    resp = test_client.get('/health')
    assert resp.status_code == 200
    assert resp.json() == {'status': 'ok'}


def test_append_fleet_log_parses_jsonl_into_rows(client):
    test_client, _, _ = client
    lines = '\n'.join(
        [
            json.dumps(
                {
                    'stamp_sec': 100,
                    'stamp_nanosec': 0,
                    'component': 'weighing_scale_driver',
                    'severity': 'WARN',
                    'code': 'scale_timeout',
                    'message': 'no reading in 2s',
                    'context': {'attempt': 3},
                }
            ),
            'not valid json',
            json.dumps(
                {
                    'component': 'pouring_controller',
                    'severity': 'ERROR',
                    'code': 'pour_overshoot',
                    'message': 'overshoot',
                }
            ),
        ]
    )

    resp = test_client.post(
        '/v1/fleet/robot-1/health',
        content=lines,
        headers={'Content-Type': 'text/plain'},
    )
    assert resp.status_code == 200
    assert resp.json() == {'received_lines': 3, 'skipped_lines': 1}

    session = _session(client)
    try:
        rows = session.query(FleetHealthEvent).order_by(
            FleetHealthEvent.id
        ).all()
        assert len(rows) == 2
        assert rows[0].device_id == 'robot-1'
        assert rows[0].code == 'scale_timeout'
        assert json.loads(rows[0].context_json) == {'attempt': 3}
        assert rows[1].code == 'pour_overshoot'
    finally:
        session.close()


def test_unknown_fleet_log_name_is_404(client):
    test_client, _, _ = client
    resp = test_client.post('/v1/fleet/robot-1/nonsense', content='x')
    assert resp.status_code == 404


def test_sync_tier0_persists_run_and_object_store_parquet(client, tmp_path):
    test_client, _, store = client
    metadata_file = tmp_path / 'metadata.json'
    metadata_file.write_text('{"run_id": "abc"}')
    events_file = tmp_path / 'events.jsonl'
    events_file.write_text('{"code": "a"}\n')
    parquet_file = tmp_path / 'features.parquet'
    parquet_file.write_bytes(b'\x00parquet-bytes')

    resp = test_client.post(
        '/v1/runs/run-1/tier0',
        data={'robot_id': 'robot-1', 'device_id': 'robot-1'},
        files={
            'metadata_json': ('metadata.json', metadata_file.read_bytes()),
            'events_jsonl': ('events.jsonl', events_file.read_bytes()),
            'features_parquet': (
                'features.parquet',
                parquet_file.read_bytes(),
            ),
        },
    )
    assert resp.status_code == 200
    assert resp.json() == {'status': 'ok'}

    session = _session(client)
    try:
        run = session.query(IngestedRun).filter_by(run_key='run-1').one()
        assert run.robot_id == 'robot-1'
        assert json.loads(run.metadata_json) == {'run_id': 'abc'}
        assert run.events_jsonl == '{"code": "a"}\n'
        assert run.features_parquet_key == 'run-1/tier0/features.parquet'
    finally:
        session.close()

    assert store.read('run-1/tier0/features.parquet') == (
        b'\x00parquet-bytes'
    )


def test_sync_tier0_is_upsert_across_two_calls(client, tmp_path):
    test_client, _, _ = client
    metadata_file = tmp_path / 'metadata.json'
    metadata_file.write_text('{"phase": "initial"}')
    test_client.post(
        '/v1/runs/run-1/tier0',
        data={'robot_id': 'robot-1', 'device_id': 'robot-1'},
        files={'metadata_json': ('metadata.json', metadata_file.read_bytes())},
    )

    events_file = tmp_path / 'events.jsonl'
    events_file.write_text('{"code": "run_ended"}\n')
    test_client.post(
        '/v1/runs/run-1/tier0',
        data={'robot_id': 'robot-1', 'device_id': 'robot-1'},
        files={'events_jsonl': ('events.jsonl', events_file.read_bytes())},
    )

    session = _session(client)
    try:
        runs = session.query(IngestedRun).filter_by(run_key='run-1').all()
        assert len(runs) == 1
        assert json.loads(runs[0].metadata_json) == {'phase': 'initial'}
        assert runs[0].events_jsonl == '{"code": "run_ended"}\n'
    finally:
        session.close()


def test_blob_status_is_zero_for_unseen_blob(client):
    test_client, _, _ = client
    resp = test_client.get('/v1/runs/run-1/tier1/0/episode_1_0.mcap/status')
    assert resp.status_code == 200
    assert resp.json() == {'received_bytes': 0}


def test_upload_blob_full_round_trip_and_finalize(client):
    test_client, _, store = client
    content = b'x' * (2 * 1024 * 1024 + 17)

    put_resp = test_client.put(
        '/v1/runs/run-1/tier1/0/episode_1_0.mcap',
        content=content,
        headers={
            'Content-Type': 'application/octet-stream',
            'X-Upload-Offset': '0',
        },
    )
    assert put_resp.status_code == 200
    assert put_resp.json() == {'received_bytes': len(content)}

    digest = hashlib.sha256(content).hexdigest()
    finalize_resp = test_client.post(
        '/v1/runs/run-1/tier1/0/episode_1_0.mcap/finalize',
        json={'sha256': digest, 'size': len(content)},
    )
    assert finalize_resp.status_code == 200
    assert finalize_resp.json() == {'acked': True}
    assert store.read('run-1/tier1/0/episode_1_0.mcap') == content

    session = _session(client)
    try:
        blob = (
            session.query(IngestedBlob)
            .filter_by(run_key='run-1', blob_key='0/episode_1_0.mcap')
            .one()
        )
        assert blob.size_bytes == len(content)
        assert blob.sha256 == digest
    finally:
        session.close()


def test_upload_blob_resumes_from_reported_offset(client):
    test_client, _, _ = client
    first_chunk = b'a' * 1000

    test_client.put(
        '/v1/runs/run-1/tier1/f.bin',
        content=first_chunk,
        headers={'X-Upload-Offset': '0'},
    )

    status = test_client.get('/v1/runs/run-1/tier1/f.bin/status')
    assert status.json() == {'received_bytes': 1000}

    second_chunk = b'b' * 500
    put_resp = test_client.put(
        '/v1/runs/run-1/tier1/f.bin',
        content=second_chunk,
        headers={'X-Upload-Offset': '1000'},
    )
    assert put_resp.json() == {'received_bytes': 1500}


def test_upload_blob_offset_mismatch_returns_409_with_actual_size(client):
    test_client, _, _ = client
    test_client.put(
        '/v1/runs/run-1/tier1/f.bin',
        content=b'a' * 100,
        headers={'X-Upload-Offset': '0'},
    )

    resp = test_client.put(
        '/v1/runs/run-1/tier1/f.bin',
        content=b'b' * 50,
        headers={'X-Upload-Offset': '999'},
    )
    assert resp.status_code == 409
    assert resp.json()['detail'] == {'received_bytes': 100}


def test_finalize_checksum_mismatch_deletes_partial_and_forces_restart(
    client,
):
    test_client, _, store = client
    test_client.put(
        '/v1/runs/run-1/tier1/f.bin',
        content=b'corrupted',
        headers={'X-Upload-Offset': '0'},
    )

    resp = test_client.post(
        '/v1/runs/run-1/tier1/f.bin/finalize',
        json={'sha256': 'not-the-real-hash', 'size': 9},
    )
    assert resp.status_code == 409

    # The partial upload was deleted server-side, so a fresh status check
    # reports zero bytes, driving the client to restart from scratch.
    status = test_client.get('/v1/runs/run-1/tier1/f.bin/status')
    assert status.json() == {'received_bytes': 0}
    assert not store.exists('run-1/tier1/f.bin')


def test_complete_tier1_marks_run_and_requires_prior_tier0(client, tmp_path):
    test_client, _, _ = client

    missing_run_resp = test_client.post('/v1/runs/no-such-run/tier1/complete')
    assert missing_run_resp.status_code == 404

    metadata_file = tmp_path / 'metadata.json'
    metadata_file.write_text('{}')
    test_client.post(
        '/v1/runs/run-1/tier0',
        data={'robot_id': 'robot-1', 'device_id': 'robot-1'},
        files={'metadata_json': ('metadata.json', metadata_file.read_bytes())},
    )

    resp = test_client.post('/v1/runs/run-1/tier1/complete')
    assert resp.status_code == 200
    assert resp.json() == {'acked': True}

    session = _session(client)
    try:
        run = session.query(IngestedRun).filter_by(run_key='run-1').one()
        assert run.tier1_completed_at is not None
    finally:
        session.close()
