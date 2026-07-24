import json

import rclpy
from rclpy.node import Node

from rhapsodi_common.health import HealthEventPublisher, WARN


def test_publish_fills_expected_fields():
    rclpy.init()
    try:
        node = Node('test_health_event_publisher_node')
        try:
            publisher = HealthEventPublisher(
                node, 'unit_test_component', device_id='robot-test'
            )
            captured = {}
            publisher._publisher.publish = lambda msg: captured.setdefault(
                'msg', msg
            )
            publisher.warn(
                'unit_test_code', 'something went wrong', {'foo': 'bar'}
            )
            msg = captured['msg']
            assert msg.device_id == 'robot-test'
            assert msg.component == 'unit_test_component'
            assert msg.severity == WARN
            assert msg.code == 'unit_test_code'
            assert msg.message == 'something went wrong'
            assert json.loads(msg.context_json) == {'foo': 'bar'}
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


def test_severity_constants_match_message_definition():
    from robot_common_msgs.msg import HealthEvent

    assert WARN == HealthEvent.WARN


def test_context_json_serializes_dict():
    rclpy.init()
    try:
        node = Node('test_health_event_publisher_json_node')
        try:
            publisher = HealthEventPublisher(node, 'unit_test_component')
            captured = {}

            def fake_publish(msg):
                captured['msg'] = msg

            publisher._publisher.publish = fake_publish
            publisher.error('code', 'msg', {'a': 1, 'b': [1, 2]})
            assert json.loads(captured['msg'].context_json) == {
                'a': 1,
                'b': [1, 2],
            }
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()
