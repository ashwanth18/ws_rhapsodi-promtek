from pathlib import Path

import rclpy
from rclpy.node import Node

from data_collection_manager.recorder_v2 import RecorderStartError, RecorderV2
from rhapsodi_common.health import HealthEventPublisher


def test_start_failure_publishes_health_event_and_raises(tmp_path):
    rclpy.init()
    try:
        node = Node('test_recorder_v2_node')
        try:
            health = HealthEventPublisher(node, 'test_component')
            captured = {}
            health._publisher.publish = lambda msg: captured.setdefault(
                'msg', msg
            )
            recorder = RecorderV2(health=health)

            # A path with a null byte is guaranteed invalid for any
            # filesystem and triggers a real failure inside rosbag2_py,
            # exercising the actual exception -> health event -> raise path
            # rather than a mocked one.
            bad_path = Path(str(tmp_path / 'bag\x00invalid'))
            try:
                recorder.start(bad_path, ['/weight'])
                assert False, 'expected RecorderStartError'
            except RecorderStartError:
                pass

            assert not recorder.is_recording
            assert 'msg' in captured
            assert captured['msg'].code == 'recorder_start_failed'
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_start_then_stop_round_trip(tmp_path):
    rclpy.init()
    try:
        node = Node('test_recorder_v2_roundtrip_node')
        try:
            recorder = RecorderV2()
            bag_path = tmp_path / 'run1'
            recorder.start(bag_path, ['/weight'])
            assert recorder.is_recording
            recorder.stop()
            assert not recorder.is_recording
            assert (bag_path / 'metadata.yaml').exists()
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_stop_without_start_is_a_noop():
    recorder = RecorderV2()
    recorder.stop()  # must not raise
    assert not recorder.is_recording
