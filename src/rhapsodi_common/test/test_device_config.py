from pathlib import Path

import rhapsodi_common.device_config as device_config
from rhapsodi_common.device_config import load_device_config


def test_fallback_when_no_config_file_anywhere(tmp_path, monkeypatch):
    # Isolate from this repo's real config/device.yaml, which the normal
    # walk-up discovery would otherwise find when tests run from a checkout.
    missing = tmp_path / 'does_not_exist.yaml'
    monkeypatch.setattr(
        device_config, '_candidate_paths', lambda explicit: [missing]
    )
    config = load_device_config(str(missing))
    assert config.is_fallback
    assert config.robot_id == 'robot-1'
    assert config.processing_url == 'http://localhost:8002/process'


def test_loads_values_from_yaml(tmp_path, monkeypatch):
    monkeypatch.delenv('RHAPSODI_DEVICE_CONFIG', raising=False)
    config_path = tmp_path / 'device.yaml'
    config_path.write_text(
        """
device:
  device_id: robot-7-pi5
  robot_id: robot-7
  robot_type: jaka
  site_id: site-42
  fleet:
    processing_url: http://localhost:8002/process
    ingestion_url: https://ingest.rhapsodi.example/v1/ingest
"""
    )
    config = load_device_config(str(config_path))
    assert not config.is_fallback
    assert config.device_id == 'robot-7-pi5'
    assert config.robot_id == 'robot-7'
    assert config.robot_type == 'jaka'
    assert config.site_id == 'site-42'
    assert config.ingestion_url == 'https://ingest.rhapsodi.example/v1/ingest'
    assert config.source_path == Path(config_path)


def test_env_var_is_used_when_no_explicit_path(tmp_path, monkeypatch):
    config_path = tmp_path / 'device.yaml'
    config_path.write_text(
        'device:\n  device_id: env-robot\n  robot_id: env-robot\n'
    )
    monkeypatch.setenv('RHAPSODI_DEVICE_CONFIG', str(config_path))
    config = load_device_config()
    assert config.device_id == 'env-robot'
