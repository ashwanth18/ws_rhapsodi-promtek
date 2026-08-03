from ingestion.rules import evaluate_health_event


def test_dds_transport_timeout_signature():
    candidates = evaluate_health_event(
        {
            'code': 'robot_start_service_call_timeout',
            'message': 'Service /bt_start_webhook_weightment call timed out after 15.0s',
        }
    )
    assert [c.signature_id for c in candidates] == ['dds_transport_timeout']
    assert candidates[0].severity == 'ERROR'


def test_pour_overshoot_below_threshold_is_ordinary_warning():
    candidates = evaluate_health_event(
        {
            'code': 'pour_overshoot',
            'context': {'target_weight': 100.0, 'final_net_g': 101.5},
        }
    )
    assert [c.signature_id for c in candidates] == ['pour_overshoot']
    assert candidates[0].severity == 'WARN'
    assert candidates[0].evidence['overshoot_g'] == 1.5


def test_pour_overshoot_above_threshold_is_severe():
    candidates = evaluate_health_event(
        {
            'code': 'pour_overshoot',
            'context': {'target_weight': 100.0, 'final_net_g': 110.0},
        }
    )
    assert [c.signature_id for c in candidates] == ['pour_overshoot_severe']
    assert candidates[0].severity == 'ERROR'
    assert candidates[0].evidence['overshoot_g'] == 10.0


def test_pour_overshoot_without_context_still_detected():
    candidates = evaluate_health_event({'code': 'pour_overshoot'})
    assert [c.signature_id for c in candidates] == ['pour_overshoot']
    assert candidates[0].evidence['overshoot_g'] is None


def test_pour_stall_and_abort_codes_all_map_to_one_signature():
    for code in (
        'pour_no_progress_timeout',
        'pour_stale_weight_abort',
        'pour_stale_weight_during_settle',
    ):
        candidates = evaluate_health_event({'code': code})
        assert [c.signature_id for c in candidates] == ['pour_stalled_abort']


def test_micro_ros_and_scale_staleness_signatures():
    assert [
        c.signature_id
        for c in evaluate_health_event({'code': 'microros_heartbeat_stale'})
    ] == ['microros_heartbeat_stale']
    assert [
        c.signature_id
        for c in evaluate_health_event({'code': 'scale_readings_stale'})
    ] == ['scale_readings_stale']


def test_detached_instance_error_matches_on_message_not_code():
    candidates = evaluate_health_event(
        {
            'code': 'backend_exception',
            'message': (
                'sqlalchemy.orm.exc.DetachedInstanceError: Instance '
                '<LightsOutProcessed> is not bound to a Session'
            ),
        }
    )
    assert [c.signature_id for c in candidates] == [
        'sqlalchemy_detached_instance_error'
    ]


def test_idle_in_transaction_matches_case_insensitively():
    candidates = evaluate_health_event(
        {'code': 'db_state', 'message': 'session state: IDLE IN TRANSACTION'}
    )
    assert [c.signature_id for c in candidates] == [
        'postgres_idle_in_transaction'
    ]


def test_unknown_event_matches_nothing():
    assert evaluate_health_event({'code': 'something_benign'}) == []


def test_event_can_match_multiple_detectors():
    # A message that happens to carry both a known code and a coincidental
    # substring match should surface both incidents, not just one.
    candidates = evaluate_health_event(
        {
            'code': 'pour_overshoot',
            'message': 'overshoot logged while session was idle in transaction',
            'context': {'target_weight': 100.0, 'final_net_g': 101.0},
        }
    )
    signatures = {c.signature_id for c in candidates}
    assert signatures == {'pour_overshoot', 'postgres_idle_in_transaction'}
