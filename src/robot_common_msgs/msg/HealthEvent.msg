# Structured fault/health signal for cross-subsystem fault-finding.
#
# Every subsystem that used to only log to its own container (scale driver,
# micro-ROS connectivity, camera lifecycle, pour controller, orchestrator BT
# failures, robot_start_adapter timeouts, ...) publishes onto
# /system/health_events instead, so "why did this run fail" and "has this
# Pi been flaky" are answerable without grepping `docker logs` per service.
#
# RecorderV2 (data_collection_manager) captures this stream into every run's
# events.jsonl (run-scoped) and a rolling fleet-level health.jsonl
# (device-scoped). See src/rhapsodi_common for the publisher helpers
# (Python: rhapsodi_common.health, C++: rhapsodi_common_cpp/health_event_publisher.hpp).

builtin_interfaces/Time stamp

# Matches config/device.yaml device_id for the Pi that produced this event.
string device_id

# Short, stable subsystem name, e.g. "weighing_scale_driver", "pour_server",
# "robot_orchestrator", "microros_health_watchdog", "robot_start_adapter".
string component

uint8 severity
uint8 DEBUG = 0
uint8 INFO = 1
uint8 WARN = 2
uint8 ERROR = 3
uint8 CRITICAL = 4

# Short, machine-matchable code for the incident-detection rules engine,
# e.g. "serial_open_failed", "pour_no_progress_timeout", "bt_tree_failure".
# Prefer snake_case and stability over time (this is the join key for
# TROUBLESHOOTING.md / .cursor/rules/robot-fault-patterns.mdc).
string code

# Human-readable summary, safe to show directly in a dashboard or log line.
string message

# Optional structured context, JSON-encoded object (may be empty string).
# e.g. {"port": "/dev/ttyUSB0", "baud": 115200}
string context_json
