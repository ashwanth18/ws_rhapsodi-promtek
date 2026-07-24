"""Native MCAP recorder used by DataCollectionManager (recorder-v2).

Replaces the historical ``subprocess.Popen(['ros2', 'bag', 'record', ...])``
approach with the same in-process API the ``ros2 bag record`` CLI itself
uses (``rosbag2_py.Recorder``). This gives:

- Real error surfacing: start/stop failures raise/are reported instead of
  disappearing into a subprocess's redirected-to-DEVNULL stdout/stderr.
- Native MCAP chunk compression (zstd) via the mcap storage plugin's
  ``zstd_fast`` preset, instead of shelling out to a CLI subprocess that
  could be orphaned if the parent node dies uncleanly.
- A single place to hook safe flush/close on SIGTERM (see ``main()``) and
  to publish onto ``/system/health_events`` when recording can't start or
  stop cleanly, instead of a network blip / crash silently dropping data.

Residual risk, documented rather than hidden: this recorder now runs
in-process instead of in an isolated subprocess. If the underlying C++
Recorder's own thread hits an unrecoverable error, it can still bring down
this process. Container-level ``restart: unless-stopped``
(docker-compose*.yml) is the backstop for that case, same as it already is
for every other node in this repo — this module's job is to make the
*known* failure paths (start/stop, SIGTERM) safe and observable, not to
promise a crash can never happen.
"""
from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence

import rosbag2_py

from rhapsodi_common.health import HealthEventPublisher

# "zstd_fast" chunk-compresses each MCAP chunk with zstd at the fastest
# compression level, keeping the file a normal, directly-playable .mcap
# (unlike RecordOptions.compression_mode='file', which would wrap the
# entire file in an opaque .mcap.zstd container that needs decompression
# before Foxglove / `ros2 bag play` can open it).
_STORAGE_PRESET_PROFILE = 'zstd_fast'


class RecorderStartError(RuntimeError):
    """Raised when the native recorder fails to start."""


class RecorderV2:
    """One instance per active recording (one per episode/webhook run)."""

    def __init__(self, health: Optional[HealthEventPublisher] = None) -> None:
        self._health = health
        self._recorder: Optional[rosbag2_py.Recorder] = None
        self._bag_path: Optional[Path] = None

    @property
    def is_recording(self) -> bool:
        return self._recorder is not None

    def start(self, bag_path: Path, topics: Sequence[str]) -> None:
        """Start recording `topics` into an MCAP bag at `bag_path`.

        Raises RecorderStartError on failure (and publishes a CRITICAL
        health event first) so callers can decide not to proceed as if
        recording were happening.
        """
        if self._recorder is not None:
            raise RecorderStartError(
                f'Recorder already active for {self._bag_path}; '
                'call stop() before starting a new one'
            )
        topic_list: List[str] = list(topics)
        try:
            storage_options = rosbag2_py.StorageOptions(
                uri=str(bag_path), storage_id='mcap'
            )
            storage_options.storage_preset_profile = _STORAGE_PRESET_PROFILE
            record_options = rosbag2_py.RecordOptions()
            record_options.topics = topic_list
            record_options.all_topics = False

            recorder = rosbag2_py.Recorder(
                storage_options, record_options, 'warn', 'data_collection_recorder'
            )
            recorder.record()
            recorder.start_spin()
        except Exception as exc:  # noqa: BLE001 - must surface, never swallow
            if self._health is not None:
                self._health.critical(
                    'recorder_start_failed',
                    f'Failed to start MCAP recording at {bag_path}: {exc!r}',
                    {'bag_path': str(bag_path), 'topics': topic_list},
                )
            raise RecorderStartError(str(exc)) from exc

        self._recorder = recorder
        self._bag_path = bag_path

    def stop(self) -> None:
        """Stop recording, if active. Never raises: a stop failure is
        reported as an ERROR health event (a bag that fails to close
        cleanly may still be partially readable) rather than crashing the
        node mid-shutdown.
        """
        if self._recorder is None:
            return
        bag_path = self._bag_path
        try:
            self._recorder.stop()
            self._recorder.stop_spin()
        except Exception as exc:  # noqa: BLE001
            if self._health is not None:
                self._health.error(
                    'recorder_stop_failed',
                    f'Failed to cleanly stop MCAP recording at {bag_path}: {exc!r}',
                    {'bag_path': str(bag_path) if bag_path else None},
                )
        finally:
            self._recorder = None
            self._bag_path = None
