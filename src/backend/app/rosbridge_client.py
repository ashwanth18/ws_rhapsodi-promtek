import logging
import os
import threading
import time
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
        self.failure_reason_topic = os.environ.get(
            'ROSBRIDGE_FAILURE_REASON_TOPIC', '/orchestrator/failure_reason'
        )
        self.weight_topic = os.environ.get('ROSBRIDGE_WEIGHT_TOPIC', '/weight')

        self._lock = threading.RLock()
        self._ros: roslibpy.Ros | None = None
        self._run_state_sub: roslibpy.Topic | None = None
        self._failure_reason_sub: roslibpy.Topic | None = None
        self._weight_sub: roslibpy.Topic | None = None
        self._completion_handler: Callable[[dict[str, Any]], None] | None = None
        self.latest_weight_g = 0.0
        self.latest_failure_reason = ''
        self.latest_run_state = ''
        self.active_run: dict[str, Any] | None = None
        # /orchestrator/run_state is latched. Subscribing while a run is active
        # replays the previous run's terminal state. Drop that first terminal.
        # A later one still counts: tree-create failure publishes "failed"
        # without a preceding "running".
        self._seen_running_for_active = False
        self._dropped_latched_terminal = False

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
        roslibpy.Topic | None,
    ]:
        ros = self._ros
        run_state_sub = self._run_state_sub
        failure_reason_sub = self._failure_reason_sub
        weight_sub = self._weight_sub
        self._ros = None
        self._run_state_sub = None
        self._failure_reason_sub = None
        self._weight_sub = None
        self.active_run = None
        return ros, run_state_sub, failure_reason_sub, weight_sub

    def _dispose_handles(
        self,
        ros: roslibpy.Ros | None,
        run_state_sub: roslibpy.Topic | None,
        failure_reason_sub: roslibpy.Topic | None,
        weight_sub: roslibpy.Topic | None,
    ) -> None:
        for topic in (run_state_sub, failure_reason_sub, weight_sub):
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
        # Do not hold _lock across rosbridge calls. get_topic_type waits on the
        # Twisted reactor, and a stuck reactor then blocks every request that
        # reconciles runs, which leaves the dashboard on Loading.
        with self._lock:
            if self._ros is not None and self._ros.is_connected:
                return
            stale = None
            if self._ros is not None:
                stale = self._detach_locked()
            ros = roslibpy.Ros(host=self.host, port=self.port)
            self._ros = ros
        if stale is not None:
            threading.Thread(
                target=self._dispose_handles, args=stale, daemon=True
            ).start()
        try:
            ros.run(timeout=5)
            if not ros.is_connected:
                raise TimeoutError('rosbridge did not connect')
            state_topic = roslibpy.Topic(ros, self.run_state_topic, 'std_msgs/msg/String')
            reason_topic = roslibpy.Topic(
                ros, self.failure_reason_topic, 'std_msgs/msg/String'
            )
            weight_topic = roslibpy.Topic(ros, self.weight_topic, 'std_msgs/msg/Float64')
            state_topic.subscribe(self._on_run_state)
            reason_topic.subscribe(self._on_failure_reason)
            weight_topic.subscribe(self._on_weight)
        except Exception:
            logger.exception('Rosbridge monitoring connect failed')
            with self._lock:
                if self._ros is ros:
                    self._ros = None
            try:
                ros.close()
            except Exception:
                logger.exception('Failed to close rosbridge after connect failure')
            return
        with self._lock:
            if self._ros is not ros:
                return
            self._run_state_sub = state_topic
            self._failure_reason_sub = reason_topic
            self._weight_sub = weight_topic

    def ensure_monitoring(self) -> None:
        self._ensure_connected()

    def ensure_monitoring_async(self) -> None:
        threading.Thread(target=self.ensure_monitoring, daemon=True).start()

    def register_active_run(
        self, run_id: int, contract: dict[str, Any], service_response: dict[str, Any]
    ) -> None:
        with self._lock:
            self.latest_weight_g = 0.0
            self.latest_failure_reason = ''
            self._seen_running_for_active = False
            self._dropped_latched_terminal = False
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

    def _on_failure_reason(self, message: dict[str, Any]) -> None:
        reason = str(message.get('data') or '').strip()
        with self._lock:
            self.latest_failure_reason = reason

    def _wait_for_failure_reason(self, timeout_s: float = 0.5) -> str:
        """Orchestrator publishes failure_reason just before run_state; wait briefly."""
        deadline = time.monotonic() + timeout_s
        while True:
            with self._lock:
                reason = self.latest_failure_reason
            if reason:
                return reason
            if time.monotonic() >= deadline:
                return ''
            time.sleep(0.05)

    def read_orchestrator_outcome(self, timeout_s: float = 2.0) -> tuple[str, str]:
        """Read the latched orchestrator run state.

        A wedged rosbridge call must not block the API thread. The dashboard
        stays on Loading when every worker is stuck behind that call.
        """
        with self._lock:
            if self.active_run is not None and not self._seen_running_for_active:
                return '', ''
            state = self.latest_run_state
            reason = self.latest_failure_reason
        if state in {'succeeded', 'failed', 'stopped'} and (state == 'succeeded' or reason):
            return state, reason

        observed: dict[str, str] = {'state': '', 'reason': ''}

        def _read() -> None:
            ros = roslibpy.Ros(host=self.host, port=self.port)
            try:
                ros.run(timeout=timeout_s)
                if not ros.is_connected:
                    return

                def on_state(message: dict[str, Any]) -> None:
                    observed['state'] = str(message.get('data') or '').strip().lower()

                def on_reason(message: dict[str, Any]) -> None:
                    observed['reason'] = str(message.get('data') or '').strip()

                state_topic = roslibpy.Topic(
                    ros, self.run_state_topic, 'std_msgs/msg/String'
                )
                reason_topic = roslibpy.Topic(
                    ros, self.failure_reason_topic, 'std_msgs/msg/String'
                )
                state_topic.subscribe(on_state)
                reason_topic.subscribe(on_reason)
                deadline = time.monotonic() + timeout_s
                while time.monotonic() < deadline:
                    if observed['state'] in {'succeeded', 'failed', 'stopped'} and (
                        observed['state'] == 'succeeded' or observed['reason']
                    ):
                        break
                    time.sleep(0.05)
            except Exception:
                logger.exception('Failed to read latched orchestrator outcome')
            finally:
                try:
                    ros.close()
                except Exception:
                    logger.exception('Failed to close orchestrator outcome connection')

        worker = threading.Thread(target=_read, daemon=True)
        worker.start()
        worker.join(timeout_s + 0.5)
        if worker.is_alive():
            logger.warning('Orchestrator outcome read exceeded %.1fs', timeout_s)
        return observed['state'], observed['reason']

    def _on_run_state(self, message: dict[str, Any]) -> None:
        state = str(message.get('data') or '').strip().lower()
        with self._lock:
            if (
                self.active_run is not None
                and not self._seen_running_for_active
                and not self._dropped_latched_terminal
                and state in {'succeeded', 'failed', 'stopped'}
            ):
                self._dropped_latched_terminal = True
                logger.info(
                    'Ignoring latched orchestrator state %s for run %s; '
                    'this run has not published running yet',
                    state,
                    self.active_run.get('run_id'),
                )
                return
            self.latest_run_state = state
            if state == 'running' and self.active_run is not None:
                self._seen_running_for_active = True
        if state not in {'succeeded', 'failed', 'stopped'}:
            return

        with self._lock:
            if self.active_run is None:
                return
            if (
                not self._seen_running_for_active
                and not self._dropped_latched_terminal
            ):
                return
            run = self.active_run
            self.active_run = None
            self._seen_running_for_active = False
            self._dropped_latched_terminal = False

        failure_reason = ''
        if state in {'failed', 'stopped'}:
            failure_reason = self._wait_for_failure_reason()

        if state == 'succeeded':
            error_message = None
        elif failure_reason:
            error_message = failure_reason
        else:
            error_message = f'Orchestrator finished with state {state}'

        payload = {
            'run_id': int(run['run_id']),
            'success': state == 'succeeded',
            'actual_weight_kg': self.latest_weight_g / 1000.0,
            'final_scale_weight_g': self.latest_weight_g,
            'start_utc': run['start_utc'],
            'end_utc': utc_now(),
            'energy_kwh': 0,
            'error_message': error_message,
            'result_payload': {
                'run_state': state,
                'latest_weight_g': self.latest_weight_g,
                'failure_reason': failure_reason or None,
                'contract': run['contract'],
                'service_response': run['service_response'],
            },
        }
        if self._completion_handler is not None:
            self._completion_handler(payload)


rosbridge_robot_client = RosbridgeRobotClient()
