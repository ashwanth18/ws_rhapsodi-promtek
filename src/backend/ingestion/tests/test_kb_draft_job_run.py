"""Integration tests for `kb_draft_job.run()` - the DB + filesystem
glue around the pure `draft_kb` rendering tested in
test_kb_draft_job.py. Same SQLite/StaticPool stand-in for Postgres as
test_ingestion_service.py.
"""
import pytest
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.pool import StaticPool

from ingestion import kb_draft_job
from ingestion.models import Base, Incident
from ingestion.kb_signatures import SIGNATURES


@pytest.fixture()
def db_session_factory(monkeypatch):
    engine = create_engine(
        'sqlite:///:memory:',
        connect_args={'check_same_thread': False},
        poolclass=StaticPool,
    )
    TestSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
    Base.metadata.create_all(bind=engine)
    monkeypatch.setattr(kb_draft_job, 'SessionLocal', TestSessionLocal)
    return TestSessionLocal


def test_first_run_creates_both_files_with_every_known_signature(
    db_session_factory, tmp_path
):
    kb_draft_job.run(tmp_path)

    rule_path = tmp_path / '.cursor' / 'rules' / 'robot-fault-patterns.mdc'
    troubleshooting_path = tmp_path / 'TROUBLESHOOTING.md'
    assert rule_path.is_file()
    assert troubleshooting_path.is_file()

    rule_content = rule_path.read_text()
    assert rule_content.startswith('---\ndescription:')
    for signature_id in SIGNATURES:
        assert f'kb:begin:{signature_id}' in rule_content
        assert f'kb:begin:{signature_id}' in troubleshooting_path.read_text()


def test_run_marks_kb_drafted_at_only_for_known_signatures(
    db_session_factory, tmp_path
):
    session = db_session_factory()
    session.add(
        Incident(
            signature_id='pour_overshoot',
            title='t',
            severity='WARN',
            device_id='robot-1',
        )
    )
    session.add(
        Incident(
            signature_id='totally_unknown_signature',
            title='t',
            severity='WARN',
            device_id='robot-1',
        )
    )
    session.commit()
    session.close()

    kb_draft_job.run(tmp_path)

    session = db_session_factory()
    try:
        known = (
            session.query(Incident)
            .filter_by(signature_id='pour_overshoot')
            .one()
        )
        unknown = (
            session.query(Incident)
            .filter_by(signature_id='totally_unknown_signature')
            .one()
        )
        assert known.kb_drafted_at is not None
        assert unknown.kb_drafted_at is None
    finally:
        session.close()


def test_second_run_with_no_new_incidents_is_byte_identical(
    db_session_factory, tmp_path
):
    session = db_session_factory()
    session.add(
        Incident(
            signature_id='scale_readings_stale',
            title='t',
            severity='WARN',
            device_id='robot-1',
        )
    )
    session.commit()
    session.close()

    kb_draft_job.run(tmp_path)
    rule_path = tmp_path / '.cursor' / 'rules' / 'robot-fault-patterns.mdc'
    troubleshooting_path = tmp_path / 'TROUBLESHOOTING.md'
    first_rule = rule_path.read_text()
    first_troubleshooting = troubleshooting_path.read_text()

    kb_draft_job.run(tmp_path)
    assert rule_path.read_text() == first_rule
    assert troubleshooting_path.read_text() == first_troubleshooting


def test_occurrence_count_does_not_regress_after_incident_gets_drafted(
    db_session_factory, tmp_path
):
    """Regression guard for the bug where filtering to kb_drafted_at IS
    NULL for rendering would make an already-drafted incident's
    contribution to the occurrence count disappear on the next run."""
    session = db_session_factory()
    session.add(
        Incident(
            signature_id='pour_overshoot',
            title='t',
            severity='WARN',
            device_id='robot-1',
        )
    )
    session.commit()
    session.close()

    kb_draft_job.run(tmp_path)  # marks that incident kb_drafted_at

    session = db_session_factory()
    assert (
        session.query(Incident).filter_by(signature_id='pour_overshoot').one()
    ).kb_drafted_at is not None
    session.close()

    # Second run with zero new incidents must still report the original
    # occurrence, not silently regress to "not yet observed".
    kb_draft_job.run(tmp_path)
    rule_content = (tmp_path / '.cursor' / 'rules' / 'robot-fault-patterns.mdc').read_text()
    assert 'Observed 1x' in rule_content
    assert 'Not yet observed' not in rule_content.split(
        'kb:begin:pour_overshoot'
    )[1].split('kb:end:pour_overshoot')[0]


def test_new_incident_on_later_run_increments_existing_occurrence_count(
    db_session_factory, tmp_path
):
    session = db_session_factory()
    session.add(
        Incident(
            signature_id='pour_overshoot',
            title='t',
            severity='WARN',
            device_id='robot-1',
        )
    )
    session.commit()
    session.close()

    kb_draft_job.run(tmp_path)

    session = db_session_factory()
    session.add(
        Incident(
            signature_id='pour_overshoot',
            title='t',
            severity='ERROR',
            device_id='robot-2',
        )
    )
    session.commit()
    session.close()

    kb_draft_job.run(tmp_path)
    rule_content = (tmp_path / '.cursor' / 'rules' / 'robot-fault-patterns.mdc').read_text()
    section = rule_content.split('kb:begin:pour_overshoot')[1].split(
        'kb:end:pour_overshoot'
    )[0]
    assert 'Observed 2x' in section
    assert 'robot-1' in section and 'robot-2' in section


def test_preserves_hand_written_content_outside_marked_sections(
    db_session_factory, tmp_path
):
    rule_path = tmp_path / '.cursor' / 'rules' / 'robot-fault-patterns.mdc'
    rule_path.parent.mkdir(parents=True)
    rule_path.write_text(
        '---\ndescription: old\nglobs: **/*\nalwaysApply: false\n---\n\n'
        '# Robot fault patterns\n\n'
        'A hand-written note a human added between automated runs.\n'
    )

    kb_draft_job.run(tmp_path)

    content = rule_path.read_text()
    assert 'A hand-written note a human added between automated runs.' in content
    for signature_id in SIGNATURES:
        assert f'kb:begin:{signature_id}' in content
