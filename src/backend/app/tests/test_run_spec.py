from app.run_spec import (
    Environment,
    OperatingMode,
    TREE_LIGHTSOUT,
    TREE_WEBHOOK_WEIGHTMENT,
    RunSpec,
)


def test_mes_family_defaults_and_phase_topic():
    spec = RunSpec(
        mode=OperatingMode.MES_CONDOR,
        run_key='robot-1_run-abc',
        target_weight_g=100.0,
        tolerance_g=2.0,
        batch_id='B1',
    )
    assert spec.environment == Environment.REAL
    assert spec.tree_id == TREE_WEBHOOK_WEIGHTMENT
    assert spec.phase_topic == '/webhook_run/phase'
    assert spec.is_mes_family
    meta = spec.to_metadata_dict()
    assert meta['schema_version'] == '1'
    assert meta['mode'] == 'mes-condor'
    assert meta['environment'] == 'real'
    assert meta['run_key'] == 'robot-1_run-abc'
    assert meta['batch_id'] == 'B1'
    assert 'weightment_id' not in meta


def test_lightsout_tree_and_phase_topic():
    spec = RunSpec(
        mode='lightsout',
        environment='sim',
        run_key='lo-1',
        target_weight_g=50.0,
        tolerance_g=1.0,
    )
    assert spec.mode == OperatingMode.LIGHTSOUT
    assert spec.environment == Environment.SIM
    assert spec.tree_id == TREE_LIGHTSOUT
    assert spec.phase_topic == '/lightsout_training/phase'
    assert not spec.is_mes_family


def test_explicit_tree_id_preserved():
    spec = RunSpec(
        mode=OperatingMode.MOCK_LOCAL,
        run_key='mock-1',
        target_weight_g=10.0,
        tolerance_g=0.5,
        tree_id='CustomTree',
    )
    assert spec.tree_id == 'CustomTree'
