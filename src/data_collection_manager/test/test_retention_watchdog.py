from data_collection_manager.manifest import TIER_0, TIER_1, RunManifest
from data_collection_manager.retention_watchdog import run_retention_pass


def _make_run_with_tier1_dir(manifest, tmp_path, run_key, acked, anomaly=False):
    run_dir = tmp_path / run_key
    bag_dir = run_dir / 'episode_1'
    bag_dir.mkdir(parents=True)
    (bag_dir / 'episode_1_0.mcap').write_bytes(b'x' * 100)
    (bag_dir / 'metadata.yaml').write_bytes(b'y' * 10)
    metadata_path = run_dir / 'metadata.json'
    metadata_path.write_text('{}')

    manifest.upsert_run(run_key, 'robot-1', run_dir, source='lightsout')
    manifest.record_artifact(run_key, TIER_0, metadata_path)
    manifest.record_artifact(run_key, TIER_1, bag_dir)
    if anomaly:
        manifest.flag_anomaly(run_key, 'pour_overshoot')
    if acked:
        manifest.mark_tier1_acked(run_key)
    return run_dir, bag_dir


def test_prunes_acked_run_and_keeps_tier0(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    run_dir, bag_dir = _make_run_with_tier1_dir(
        manifest, tmp_path, 'run-acked', acked=True
    )

    summary = run_retention_pass(manifest)

    assert summary.pruned_run_keys == ['run-acked']
    assert summary.bytes_freed == 110
    assert not bag_dir.exists()
    assert (run_dir / 'metadata.json').exists()  # Tier 0 untouched
    run = manifest.get_run('run-acked')
    assert run.tier1_pruned_at is not None
    assert run.tier1_bytes_freed == 110


def test_does_not_prune_unacked_run(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    run_dir, bag_dir = _make_run_with_tier1_dir(
        manifest, tmp_path, 'run-unacked', acked=False
    )

    summary = run_retention_pass(manifest)

    assert summary.pruned_run_keys == []
    assert bag_dir.exists()
    assert summary.unacked_run_count == 1
    assert summary.unacked_bytes == 110


def test_does_not_prune_anomaly_flagged_run_even_if_acked(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    run_dir, bag_dir = _make_run_with_tier1_dir(
        manifest, tmp_path, 'run-anomaly', acked=True, anomaly=True
    )

    summary = run_retention_pass(manifest)

    assert summary.pruned_run_keys == []
    assert bag_dir.exists()


def test_unacked_backlog_triggers_warn_health_event(tmp_path):
    manifest = RunManifest(tmp_path / 'manifest.sqlite')
    _make_run_with_tier1_dir(manifest, tmp_path, 'run-unacked', acked=False)

    captured = []

    class FakeHealth:
        def warn(self, code, message, context=None):
            captured.append(('warn', code, message, context))

        def info(self, code, message, context=None):
            captured.append(('info', code, message, context))

        def error(self, code, message, context=None):
            captured.append(('error', code, message, context))

    run_retention_pass(manifest, health=FakeHealth(), unacked_warn_bytes=1)

    warn_events = [c for c in captured if c[0] == 'warn']
    assert len(warn_events) == 1
    assert warn_events[0][1] == 'retention_tier1_backlog_high'
