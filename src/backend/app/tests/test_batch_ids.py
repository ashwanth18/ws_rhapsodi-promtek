"""Tests for incremental session-label helpers."""

from app.modes.batch_ids import prefix_for_mode, suggest_next_batch_id


def test_prefix_for_known_modes():
    assert prefix_for_mode('lightsout') == 'LO'
    assert prefix_for_mode('mock-local') == 'MOCK'
    assert prefix_for_mode('nope') is None


def test_next_batch_id_empty_starts_at_0001():
    batch_id, previous = suggest_next_batch_id('LO', [])
    assert batch_id == 'LO-0001'
    assert previous is None


def test_next_batch_id_increments_past_max():
    batch_id, previous = suggest_next_batch_id(
        'LO', ['LO-0003', 'morning run']
    )
    assert batch_id == 'LO-0004'
    assert previous == 'LO-0003'


def test_next_batch_id_skips_gaps_to_max_plus_one():
    batch_id, previous = suggest_next_batch_id('LO', ['LO-0001', 'LO-0005'])
    assert batch_id == 'LO-0006'
    assert previous == 'LO-0005'


def test_next_batch_id_case_insensitive_prefix():
    batch_id, previous = suggest_next_batch_id('LO', ['lo-0002'])
    assert batch_id == 'LO-0003'
    assert previous == 'LO-0002'
