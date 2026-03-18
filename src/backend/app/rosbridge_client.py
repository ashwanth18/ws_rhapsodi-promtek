import logging
import os
import threading
from datetime import datetime, timezone
from typing import Any, Callable

import roslibpy


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


logger = logging.getLogger(__name__)


class RosbridgeRobotClient:
    def __init__(self) -> None:
        self.host = os.environ.get('ROSBRIDGE_HOST', 'localhost')
        self.port = int(os.environ.get('ROSBRIDGE_PORT', '9090'))
        self.run_state_topic = os.environ.get(
            'ROSBRIDGE_RUN_STATE_TOPIC', '/orchestrator/run_state'
        )
        self.weight_topic = os.environ.get('ROSBRIDGE_WEIGHT_TOPIC', '/weight')

        self._lock = threading.RLock()
        self._ros: roslibpy.Ros | None = None
        self._run_state_sub: roslibpy.Topic | None = None
        self._weight_sub: roslibpy.Topic | None = None
        self._completion_handler: Callable[[dict[str, Any]], None] | None = None
        self.latest_weight_g = 0.0
        self.active_run: dict[str, Any] | None = None

    def set_completion_handler(
        self, handler: Callable[[dict[str, Any]], None]
    ) -> None:
        self._completion_handler = handler

    def _detach_locked(
        self,
    ) -> tuple[
        roslibpy.Ros | None,
        roslibpy.Topic | None,
        roslibpy.Topic | None,
    ]:
        ros = self._ros
        run_state_sub = self._run_state_sub
        weight_sub = self._weight_sub
        self._ros = None
        self._run_state_sub = None
        self._weight_sub = None
        self.active_run = None
        return ros, run_state_sub, weight_sub

    def _dispose_handles(
        self,
        ros: roslibpy.Ros | None,
        run_state_sub: roslibpy.Topic | None,
        weight_sub: roslibpy.Topic | None,
    ) -> None:
        for topic in (run_state_sub, weight_sub):
            if topic is None:
                continue
            try:
                topic.unsubscribe()
            except Exception:
                logger.exception('Failed to unsubscribe rosbridge topic')
        if ros is not None:
            try:
                ros.close()
            except Exception:
                logger.exception('Failed to close rosbridge connection')

    def reset(self, background: bool = False) -> None:
        with self._lock:
            handles = self._detach_locked()
        if background:
            threading.Thread(
                target=self._dispose_handles, args=handles, daemon=True
            ).start()
            return
        self._dispose_handles(*handles)

    def _ensure_connected(self) -> None:
        with self._lock:
            if self._ros is not None and self._ros.is_connected:
                return
            if self._ros is not None:
                handles = self._detach_locked()
                threading.Thread(
                    target=self._dispose_handles, args=handles, daemon=True
                ).start()

            self._ros = roslibpy.Ros(host=self.host, port=self.port)
            self._ros.run()

            run_state_type = self._ros.get_topic_type(self.run_state_topic)
            self._run_state_sub = roslibpy.Topic(
                self._ros, self.run_state_topic, run_state_type
            )
            self._run_state_sub.subscribe(self._on_run_state)

            weight_type = self._ros.get_topic_type(self.weight_topic)
            self._weight_sub = roslibpy.Topic(
                self._ros, self.weight_topic, weight_type
            )
            self._weight_sub.subscribe(self._on_weight)

    def ensure_monitoring(self) -> None:
        self._ensure_connected()

    def ensure_monitoring_async(self) -> None:
        threading.Thread(target=self.ensure_monitoring, daemon=True).start()

    def register_active_run(
        self, run_id: int, contract: dict[str, Any], service_response: dict[str, Any]
    ) -> None:
        with self._lock:
            self.latest_weight_g = 0.0
            self.active_run = {
                'run_id': run_id,
                'contract': contract,
                'start_utc': utc_now(),
                'service_response': service_response,
            }
        self.ensure_monitoring_async()

    def _on_weight(self, message: dict[str, Any]) -> None:
        value = message.get('data')
        if value is None:
            return
        try:
            self.latest_weight_g = float(value)
        except (TypeError, ValueError):
            return

    def _on_run_state(self, message: dict[str, Any]) -> None:
        state = str(message.get('data') or '').strip().lower()
        if state not in {'succeeded', 'failed', 'stopped'}:
            return

        with self._lock:
            if self.active_run is None:
                return
            run = self.active_run
            self.active_run = None

        payload = {
            'run_id': int(run['run_id']),
            'success': state == 'succeeded',
            'actual_weight_kg': self.latest_weight_g / 1000.0,
            'final_scale_weight_g': self.latest_weight_g,
            'start_utc': run['start_utc'],
            'end_utc': utc_now(),
            'energy_kwh': 0,
            'error_message': None
            if state == 'succeeded'
            else f'Orchestrator finished with state {state}',
            'result_payload': {
                'run_state': state,
                'latest_weight_g': self.latest_weight_g,
                'contract': run['contract'],
                'service_response': run['service_response'],
            },
        }
        if self._completion_handler is not None:
            self._completion_handler(payload)


rosbridge_robot_client = RosbridgeRobotClient()
