import json

import rclpy
from rclpy.node import Node
from robot_common_msgs.msg import HealthEvent

from rhapsodi_common.health_log import HealthEventLogger


def _make_event(code='unit_test_code', severity=None, context=None):
    msg = HealthEvent()
    msg.device_id = 'robot-test'
    msg.component = 'unit_test_component'
    msg.severity = severity if severity is not None else HealthEvent.WARN
    msg.code = code
    msg.message = 'something happened'
    msg.context_json = json.dumps(context or {})
    return msg


def test_event_always_appends_to_fleet_log(tmp_path):
    rclpy.init()
    try:
        node = Node('test_health_log_fleet_node')
        try:
            fleet_log = tmp_path / 'health.jsonl'
            logger = HealthEventLogger(node, fleet_log_path=fleet_log)
            logger._on_event(_make_event())

            lines = fleet_log.read_text().splitlines()
            assert len(lines) == 1
            record = json.loads(lines[0])
            assert record['device_id'] == 'robot-test'
            assert record['severity'] == 'WARN'
            assert record['code'] == 'unit_test_code'
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_event_also_appends_to_active_run_log(tmp_path):
    rclpy.init()
    try:
        node = Node('test_health_log_run_node')
        try:
            fleet_log = tmp_path / 'health.jsonl'
            run_log = tmp_path / 'run1' / 'events.jsonl'
            logger = HealthEventLogger(node, fleet_log_path=fleet_log)

            # No active run yet: only the fleet log gets the event.
            logger._on_event(_make_event(code='before_run'))
            assert not run_log.exists()

            logger.set_run_log(run_log)
            logger._on_event(_make_event(code='during_run'))
            assert len(run_log.read_text().splitlines()) == 1
            assert len(fleet_log.read_text().splitlines()) == 2

            logger.clear_run_log()
            logger._on_event(_make_event(code='after_run'))
            # Run log unchanged, fleet log keeps growing.
            assert len(run_log.read_text().splitlines()) == 1
            assert len(fleet_log.read_text().splitlines()) == 3
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_malformed_context_json_is_preserved_not_dropped(tmp_path):
    rclpy.init()
    try:
        node = Node('test_health_log_malformed_node')
        try:
            fleet_log = tmp_path / 'health.jsonl'
            logger = HealthEventLogger(node, fleet_log_path=fleet_log)
            msg = _make_event()
            msg.context_json = 'not valid json'
            logger._on_event(msg)

            record = json.loads(fleet_log.read_text().splitlines()[0])
            assert record['context'] == {
                '_context_json_raw': 'not valid json'
            }
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
