from pathlib import Path

from data_collection_manager.manifest import TIER_0, TIER_1, RunManifest
from data_collection_manager.uplink_client import UplinkError
from data_collection_manager.uplink_daemon import run_uplink_pass


class FakeClient:
    def __init__(self):
        self.fleet_appends = []
        self.tier0_calls = []
        self.uploaded_blobs = []
        self.completed_runs = []
        self.fail_tier0_for = set()
        self.fail_blob_for = set()

    def append_fleet_log(self, device_id, log_name, text):
        self.fleet_appends.append((device_id, log_name, text))

    def sync_tier0(self, run_key, robot_id, device_id, files):
        if run_key in self.fail_tier0_for:
            raise UplinkError('forced tier0 failure')
        self.tier0_calls.append((run_key, robot_id, device_id, set(files)))

    def upload_blob(self, run_key, blob_key, file_path):
        if run_key in self.fail_blob_for:
            raise UplinkError('forced blob failure')
        self.uploaded_blobs.append((run_key, blob_key))

    def complete_tier1(self, run_key):
        self.completed_runs.append(run_key)


def _make_run(manifest, tmp_path, run_key, with_bag=True):
    run_dir = tmp_path / run_key
    run_dir.mkdir(parents=True)
    metadata_path = run_dir / 'metadata.json'
    metadata_path.write_text('{}')
    manifest.upsert_run(run_key, 'robot-1', run_dir, device_id='robot-1')
    manifest.record_artifact(run_key, TIER_0, metadata_path)
    if with_bag:
        bag_dir = run_dir / 'episode_1'
        bag_dir.mkdir()
        (bag_dir / 'episode_1_0.mcap').write_bytes(b'x' * 100)
        manifest.record_artifact(run_key, TIER_1, bag_dir)
    return run_dir


def test_tier0_sync_marks_run_synced(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    _make_run(manifest, tmp_path, 'run-1', with_bag=False)
    client = FakeClient()

    summary = run_uplink_pass(manifest, client, device_id='robot-1')

    assert summary.tier0_synced_run_keys == ['run-1']
    assert manifest.get_run('run-1').tier0_synced_at is not None
    assert client.tier0_calls[0][0] == 'run-1'


def test_tier1_uploaded_once_tier0_has_synced(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    _make_run(manifest, tmp_path, 'run-1')
    client = FakeClient()

    # Tier 0 for this run has no prior state, so it syncs first; since
    # that happens before this same pass re-queries eligible Tier-1 runs,
    # Tier 1 for a freshly-tier0-synced run can proceed in the same pass
    # instead of waiting a full extra tick.
    summary = run_uplink_pass(manifest, client, device_id='robot-1')
    assert summary.tier0_synced_run_keys == ['run-1']
    assert summary.tier1_acked_run_keys == ['run-1']
    assert client.completed_runs == ['run-1']
    assert manifest.get_run('run-1').tier1_acked_at is not None


def test_tier1_not_attempted_before_tier0_has_synced(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    _make_run(manifest, tmp_path, 'run-1')
    client = FakeClient()
    client.fail_tier0_for = {'run-1'}

    summary = run_uplink_pass(manifest, client, device_id='robot-1')

    assert summary.failed_run_keys == ['run-1']
    assert summary.tier1_acked_run_keys == []
    assert client.uploaded_blobs == []
    assert manifest.get_run('run-1').tier0_synced_at is None


def test_failed_tier0_does_not_block_other_runs(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    _make_run(manifest, tmp_path, 'run-bad', with_bag=False)
    _make_run(manifest, tmp_path, 'run-good', with_bag=False)
    client = FakeClient()
    client.fail_tier0_for = {'run-bad'}

    summary = run_uplink_pass(manifest, client, device_id='robot-1')

    assert summary.tier0_synced_run_keys == ['run-good']
    assert summary.failed_run_keys == ['run-bad']
    assert manifest.get_run('run-bad').tier0_synced_at is None
    assert manifest.get_run('run-good').tier0_synced_at is not None


def test_failed_tier1_upload_does_not_ack(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    _make_run(manifest, tmp_path, 'run-1')
    client = FakeClient()
    client.fail_blob_for = {'run-1'}

    summary = run_uplink_pass(manifest, client, device_id='robot-1')

    # Tier 0 still succeeds independently of the Tier-1 blob failure.
    assert summary.tier0_synced_run_keys == ['run-1']
    assert summary.failed_run_keys == ['run-1']
    assert manifest.get_run('run-1').tier1_acked_at is None
    assert client.completed_runs == []


def test_fleet_health_log_synced_incrementally(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    fleet_log = tmp_path / 'health.jsonl'
    fleet_log.write_text('{"code": "a"}\n')
    client = FakeClient()

    summary1 = run_uplink_pass(
        manifest, client, device_id='robot-1', fleet_log_path=fleet_log
    )
    assert summary1.fleet_log_synced_bytes == len('{"code": "a"}\n')
    assert client.fleet_appends[-1][2] == '{"code": "a"}\n'

    # No new bytes since last sync -> no-op second pass.
    summary2 = run_uplink_pass(
        manifest, client, device_id='robot-1', fleet_log_path=fleet_log
    )
    assert summary2.fleet_log_synced_bytes == 0
    assert len(client.fleet_appends) == 1

    # New line appended -> only the new bytes are sent.
    with open(fleet_log, 'a') as f:
        f.write('{"code": "b"}\n')
    summary3 = run_uplink_pass(
        manifest, client, device_id='robot-1', fleet_log_path=fleet_log
    )
    assert summary3.fleet_log_synced_bytes == len('{"code": "b"}\n')
    assert client.fleet_appends[-1][2] == '{"code": "b"}\n'


def test_missing_fleet_log_is_not_an_error(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    client = FakeClient()
    summary = run_uplink_pass(
        manifest,
        client,
        device_id='robot-1',
        fleet_log_path=tmp_path / 'does_not_exist.jsonl',
    )
    assert summary.fleet_log_synced_bytes == 0
    assert client.fleet_appends == []
