from data_collection_manager.manifest import TIER_0, TIER_1, RunManifest


def _manifest(tmp_path):
    return RunManifest(tmp_path / 'manifest.sqlite')


def test_upsert_run_is_idempotent(tmp_path):
    manifest = _manifest(tmp_path)
    manifest.upsert_run(
        'run-1', 'robot-1', tmp_path / 'runs' / 'run-1', source='lightsout'
    )
    manifest.upsert_run(
        'run-1', 'robot-1', tmp_path / 'runs' / 'run-1-should-be-ignored'
    )
    run = manifest.get_run('run-1')
    assert run is not None
    assert run.run_folder == str(tmp_path / 'runs' / 'run-1')
    assert len(manifest.list_runs()) == 1


def test_record_artifact_and_list_by_tier(tmp_path):
    manifest = _manifest(tmp_path)
    manifest.upsert_run('run-1', 'robot-1', tmp_path / 'runs' / 'run-1')
    manifest.record_artifact('run-1', TIER_0, tmp_path / 'metadata.json')
    manifest.record_artifact('run-1', TIER_1, tmp_path / 'episode_1')

    tier0 = manifest.list_artifacts('run-1', tier=TIER_0)
    tier1 = manifest.list_artifacts('run-1', tier=TIER_1)
    assert [a.path for a in tier0] == [str(tmp_path / 'metadata.json')]
    assert [a.path for a in tier1] == [str(tmp_path / 'episode_1')]


def test_prunable_requires_ack_and_excludes_anomalies(tmp_path):
    manifest = _manifest(tmp_path)
    manifest.upsert_run('run-unacked', 'robot-1', tmp_path / 'r1')
    manifest.upsert_run('run-acked', 'robot-1', tmp_path / 'r2')
    manifest.upsert_run('run-anomaly', 'robot-1', tmp_path / 'r3')

    manifest.mark_tier1_acked('run-acked')
    manifest.mark_tier1_acked('run-anomaly')
    manifest.flag_anomaly('run-anomaly', 'pour_overshoot')

    prunable = manifest.list_prunable_tier1_runs()
    assert [r.run_key for r in prunable] == ['run-acked']

    unacked = manifest.list_unacked_tier1_runs()
    assert {r.run_key for r in unacked} == {'run-unacked'}


def test_prunable_ordered_oldest_first(tmp_path):
    manifest = _manifest(tmp_path)
    manifest.upsert_run(
        'run-newer', 'robot-1', tmp_path / 'a', created_at='2026-01-02T00:00:00+00:00'
    )
    manifest.upsert_run(
        'run-older', 'robot-1', tmp_path / 'b', created_at='2026-01-01T00:00:00+00:00'
    )
    manifest.mark_tier1_acked('run-newer')
    manifest.mark_tier1_acked('run-older')

    prunable = manifest.list_prunable_tier1_runs()
    assert [r.run_key for r in prunable] == ['run-older', 'run-newer']


def test_mark_tier1_pruned_removes_from_prunable_list(tmp_path):
    manifest = _manifest(tmp_path)
    manifest.upsert_run('run-1', 'robot-1', tmp_path / 'r1')
    manifest.mark_tier1_acked('run-1')
    assert len(manifest.list_prunable_tier1_runs()) == 1

    manifest.mark_tier1_pruned('run-1', bytes_freed=1234)
    assert manifest.list_prunable_tier1_runs() == []
    run = manifest.get_run('run-1')
    assert run.tier1_pruned_at is not None
    assert run.tier1_bytes_freed == 1234
