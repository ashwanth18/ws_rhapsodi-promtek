from pathlib import Path

from data_collection_manager.recording_profiles import (
    RecordingProfiles,
    load_recording_profiles,
)


def test_fallback_when_no_config_file(monkeypatch):
    monkeypatch.setattr(
        'data_collection_manager.recording_profiles._candidate_paths',
        lambda explicit: [Path('/nonexistent/recording_profiles.yaml')],
    )
    profiles = load_recording_profiles()
    assert profiles.is_fallback
    assert '/weight' in profiles.all_topics()
    assert '/joint_states' in profiles.all_topics()


def test_loads_real_repo_config():
    profiles = load_recording_profiles(
        path=str(
            Path(__file__).resolve().parents[3]
            / 'config'
            / 'recording_profiles.yaml'
        )
    )
    assert not profiles.is_fallback
    assert set(profiles.profiles.keys()) == {
        'always_on',
        'pour',
        'scoop',
        'transport',
    }
    assert profiles.topics_for('pour') == [
        '/weight',
        '/vibration/intensity',
        '/incline_control',
        '/valve_control',
        '/pour_status',
    ]
    assert '/camera/color/image_raw/compressed' in profiles.topics_for(
        'scoop'
    )
    all_topics = profiles.all_topics()
    # Pour and scoop telemetry must actually make it into the recorded
    # set - this is the whole point of recording-profiles.
    assert '/vibration/intensity' in all_topics
    assert '/camera/depth/image_rect_raw' in all_topics
    assert '/system/health_events' in all_topics


def test_all_topics_deduplicates_across_profiles():
    profiles = RecordingProfiles(
        profiles={
            'always_on': ['/a', '/b'],
            'scoop': ['/b', '/c'],
        }
    )
    assert profiles.all_topics() == ['/a', '/b', '/c']


def test_malformed_yaml_falls_back(tmp_path, monkeypatch):
    bad_file = tmp_path / 'recording_profiles.yaml'
    bad_file.write_text('not: [valid, - yaml structure\n')
    monkeypatch.setattr(
        'data_collection_manager.recording_profiles._candidate_paths',
        lambda explicit: [bad_file],
    )
    profiles = load_recording_profiles()
    assert profiles.is_fallback


def test_profile_without_topics_key_is_ignored(tmp_path):
    config_file = tmp_path / 'recording_profiles.yaml'
    config_file.write_text(
        'always_on:\n'
        '  topics: ["/a"]\n'
        'empty_profile:\n'
        '  description: "no topics key"\n'
    )
    profiles = load_recording_profiles(path=str(config_file))
    assert not profiles.is_fallback
    assert profiles.profiles == {'always_on': ['/a']}
