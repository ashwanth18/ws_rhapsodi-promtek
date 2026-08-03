from datetime import datetime, timedelta, timezone

from ingestion.kb_draft_job import (
    draft_kb,
    render_signature_section,
    summarize_incidents,
    upsert_sections,
)
from ingestion.kb_signatures import SIGNATURES
from ingestion.models import Incident


def _incident(
    id_,
    signature_id,
    severity='WARN',
    device_id='robot-1',
    run_key=None,
    detected_at=None,
):
    return Incident(
        id=id_,
        signature_id=signature_id,
        title='x',
        severity=severity,
        device_id=device_id,
        run_key=run_key,
        detected_at=detected_at or datetime.now(timezone.utc),
    )


def test_summarize_incidents_groups_by_resolved_signature():
    now = datetime.now(timezone.utc)
    incidents = [
        _incident(1, 'pour_overshoot', severity='WARN', detected_at=now),
        _incident(
            2,
            'pour_overshoot_severe',
            severity='ERROR',
            detected_at=now + timedelta(minutes=1),
        ),
    ]
    summaries = summarize_incidents(incidents)
    # Both fold into the same 'pour_overshoot' KB-resolved key.
    assert set(summaries) == {'pour_overshoot'}
    summary = summaries['pour_overshoot']
    assert summary.count == 2
    assert summary.max_severity == 'ERROR'
    assert summary.first_seen == now
    assert summary.last_seen == now + timedelta(minutes=1)


def test_summarize_incidents_collects_unique_devices_and_runs():
    incidents = [
        _incident(1, 'scale_readings_stale', device_id='robot-a', run_key='run-1'),
        _incident(2, 'scale_readings_stale', device_id='robot-a', run_key='run-2'),
        _incident(3, 'scale_readings_stale', device_id='robot-b', run_key=None),
    ]
    summary = summarize_incidents(incidents)['scale_readings_stale']
    assert summary.count == 3
    assert summary.device_ids == ['robot-a', 'robot-b']
    assert summary.run_keys == ['run-1', 'run-2']


def test_render_signature_section_includes_occurrence_stats():
    knowledge = SIGNATURES['dds_transport_timeout']
    incidents = [_incident(1, 'dds_transport_timeout', severity='ERROR')]
    summary = summarize_incidents(incidents)['dds_transport_timeout']
    rendered = render_signature_section(knowledge, summary)
    assert '<!-- kb:begin:dds_transport_timeout -->' in rendered
    assert '<!-- kb:end:dds_transport_timeout -->' in rendered
    assert 'Observed 1x' in rendered
    assert knowledge.fix in rendered


def test_render_signature_section_without_occurrence_notes_preemptive():
    knowledge = SIGNATURES['microros_heartbeat_stale']
    rendered = render_signature_section(knowledge, None)
    assert 'Not yet observed' in rendered


def test_upsert_sections_appends_new_signature_to_empty_file():
    content = upsert_sections(
        '', 'HEADER\n', {'pour_overshoot': 'SECTION-A'}
    )
    assert content.startswith('HEADER')
    assert 'SECTION-A' in content


def test_upsert_sections_replaces_existing_marked_section_in_place():
    existing = (
        'HEADER\n\n'
        'Some human-written intro paragraph.\n\n'
        '<!-- kb:begin:pour_overshoot -->\nOLD CONTENT\n'
        '<!-- kb:end:pour_overshoot -->\n\n'
        'Some other human-written section that must survive.\n'
    )
    updated = upsert_sections(
        existing,
        'HEADER\n',
        {
            'pour_overshoot': (
                '<!-- kb:begin:pour_overshoot -->\nNEW CONTENT\n'
                '<!-- kb:end:pour_overshoot -->'
            )
        },
    )
    assert 'OLD CONTENT' not in updated
    assert 'NEW CONTENT' in updated
    assert 'Some human-written intro paragraph.' in updated
    assert 'Some other human-written section that must survive.' in updated


def test_upsert_sections_appends_a_second_signature_without_disturbing_first():
    existing = (
        'HEADER\n\n<!-- kb:begin:sig_a -->\nA\n<!-- kb:end:sig_a -->\n'
    )
    updated = upsert_sections(
        existing,
        'HEADER\n',
        {
            'sig_a': '<!-- kb:begin:sig_a -->\nA\n<!-- kb:end:sig_a -->',
            'sig_b': '<!-- kb:begin:sig_b -->\nB\n<!-- kb:end:sig_b -->',
        },
    )
    assert '<!-- kb:begin:sig_a -->\nA\n<!-- kb:end:sig_a -->' in updated
    assert '<!-- kb:begin:sig_b -->\nB\n<!-- kb:end:sig_b -->' in updated


def test_upsert_sections_handles_replacement_text_containing_backslashes():
    # Regression guard: re.sub interprets backslash-escapes in string
    # replacements (e.g. \1); rendered prose with a literal backslash
    # (e.g. a Windows-style path or regex example) must survive intact.
    existing = '<!-- kb:begin:sig_a -->\nOLD\n<!-- kb:end:sig_a -->\n'
    updated = upsert_sections(
        existing,
        'HEADER\n',
        {
            'sig_a': (
                '<!-- kb:begin:sig_a -->\nC:\\Users\\weird\\path '
                'and \\1 literally\n<!-- kb:end:sig_a -->'
            )
        },
    )
    assert 'C:\\Users\\weird\\path and \\1 literally' in updated


def test_draft_kb_marks_only_incidents_with_known_signatures_as_drafted():
    incidents = [
        _incident(1, 'pour_overshoot'),
        _incident(2, 'totally_unknown_signature'),
    ]
    _, _, drafted_ids = draft_kb(incidents)
    assert drafted_ids == [1]


def test_draft_kb_includes_every_known_signature_even_with_no_incidents():
    rule_content, troubleshooting_content, drafted_ids = draft_kb([])
    for signature_id in SIGNATURES:
        assert f'kb:begin:{signature_id}' in rule_content
        assert f'kb:begin:{signature_id}' in troubleshooting_content
    assert drafted_ids == []


def test_draft_kb_is_idempotent_on_second_pass_with_same_incidents():
    incidents = [_incident(1, 'scale_readings_stale')]
    rule_1, ts_1, _ = draft_kb(incidents)
    rule_2, ts_2, _ = draft_kb(incidents, rule_1, ts_1)
    assert rule_1 == rule_2
    assert ts_1 == ts_2
