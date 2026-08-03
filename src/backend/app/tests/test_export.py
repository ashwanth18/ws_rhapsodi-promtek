"""Export API helper tests (no DB required)."""

from __future__ import annotations

from pathlib import Path

import pytest
from fastapi import HTTPException

from app.export import RUNS_CSV_FIELDS, _safe_artifact_path, runs_to_csv


def test_safe_artifact_path_rejects_traversal(tmp_path: Path) -> None:
    data_root = tmp_path / 'runs'
    data_root.mkdir()
    safe = data_root / 'episode-1' / 'metadata.json'
    safe.parent.mkdir()
    safe.write_text('{}', encoding='utf-8')

    resolved = _safe_artifact_path(safe, data_root=data_root)
    assert resolved == safe.resolve()

    with pytest.raises(HTTPException) as exc:
        _safe_artifact_path('/etc/passwd', data_root=data_root)
    assert exc.value.status_code == 403


def test_runs_csv_header() -> None:
    csv_text = runs_to_csv([])
    header = csv_text.splitlines()[0]
    assert header == ','.join(RUNS_CSV_FIELDS)


def test_runs_csv_includes_artifact_flags() -> None:
    csv_text = runs_to_csv(
        [
            {
                'run_key': 'rk-1',
                'run_id': 'rid-1',
                'mode': 'lightsout',
                'batch_id': 'b1',
                'episode_index': 1,
                'powder_id': 'alumina',
                'powder_name': 'Alumina',
                'lot_code': 'L1',
                'operator': 'op',
                'target_weight_g': 250.0,
                'final_weight_g': 248.0,
                'net_weight_g': 248.0,
                'start_time_ns': 100,
                'end_time_ns': 200,
                'artifacts': {
                    'metadata': {'present': True},
                    'events': {'present': False},
                    'parquet': {'present': True},
                    'mcap': {'present': False},
                },
            }
        ]
    )
    lines = csv_text.strip().splitlines()
    assert len(lines) == 2
    assert 'metadata_present' in lines[0]
    assert 'True' in lines[1]
