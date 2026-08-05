import os
import threading
from contextlib import asynccontextmanager

import rclpy
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from robot_common_msgs.srv import ApplyCellLayout, StartLightsOut, StartWebhookWeightment

from rhapsodi_common.health import HealthEventPublisher


class StartWebhookWeightmentRequest(BaseModel):
    run_id: str
    weightment_id: str
    batch_id: str
    ingredient_id: str
    location_id: str
    location_code: str = ''
    pickup_target_name: str
    weigh_target_name: str
    return_target_name: str
    target_weight_g: float
    tolerance_g: float
    expected_lot: str = ''


class StartWebhookWeightmentResponse(BaseModel):
    accepted: bool
    message: str


class StartLightsOutRequest(BaseModel):
    powder_id: str = ''
    powder_name: str
    container_target: str = ''
    pour_target: str = ''
    lot_code: str = ''
    operator: str = ''
    notes: str = ''
    target_weight_g: float
    episodes: int
    batch_id: str = ''
    enable_scoop: bool = True
    stop_on: str = ''
    stop_value: float = 0.0
    target_mode: str = 'fixed'
    target_fractions: list[float] = []
    min_scooped_g: float = 20.0
    target_min_g: float = 0.0
    target_max_g: float = 0.0
    rng_seed: int = 0


class StartLightsOutResponse(BaseModel):
    accepted: bool
    message: str


class ApplyCellLayoutRequest(BaseModel):
    layout_id: str


class ApplyCellLayoutResponse(BaseModel):
    success: bool
    message: str
    layout_hash: str
    preflight_ok: bool


class WebhookRobotStarter:
    def __init__(self) -> None:
        self.service_name = os.environ.get(
            'ROBOT_START_SERVICE_NAME', '/bt_start_webhook_weightment'
        )
        self.lightsout_service_name = os.environ.get(
            'ROBOT_LIGHTSOUT_SERVICE_NAME', '/bt_start_lightsout'
        )
        self.wait_timeout_seconds = float(
            os.environ.get('ROBOT_START_SERVICE_WAIT_TIMEOUT_SECONDS', '5')
        )
        self.call_timeout_seconds = float(
            os.environ.get('ROBOT_START_SERVICE_CALL_TIMEOUT_SECONDS', '15')
        )
        self._lock = threading.Lock()
        self._started = False
        self._node = None
        self._client = None
        self._lightsout_client = None
        self._layout_client = None
        self._health = None

    def start(self) -> None:
        if self._started:
            return
        rclpy.init(args=None)
        self._node = rclpy.create_node('webhook_robot_start_adapter')
        self._client = self._node.create_client(
            StartWebhookWeightment, self.service_name
        )
        self._lightsout_client = self._node.create_client(
            StartLightsOut, self.lightsout_service_name
        )
        self._layout_client = self._node.create_client(
            ApplyCellLayout, '/cell_layout/apply'
        )
        self._health = HealthEventPublisher(
            self._node, 'robot_start_adapter'
        )
        self._started = True

    def stop(self) -> None:
        if not self._started:
            return
        if self._node is not None:
            self._node.destroy_node()
        rclpy.shutdown()
        self._started = False
        self._node = None
        self._client = None
        self._lightsout_client = None
        self._layout_client = None

    def service_ready(self) -> bool:
        if not self._started or self._client is None:
            return False
        return bool(
            self._client.wait_for_service(
                timeout_sec=max(self.wait_timeout_seconds, 0.1)
            )
        )

    def lightsout_service_ready(self) -> bool:
        if not self._started or self._lightsout_client is None:
            return False
        return bool(
            self._lightsout_client.wait_for_service(
                timeout_sec=max(self.wait_timeout_seconds, 0.1)
            )
        )

    def start_webhook_weightment(
        self, payload: StartWebhookWeightmentRequest
    ) -> StartWebhookWeightmentResponse:
        if not self._started or self._node is None or self._client is None:
            raise RuntimeError('ROS adapter is not initialized')
        with self._lock:
            if not self._client.wait_for_service(
                timeout_sec=self.wait_timeout_seconds
            ):
                self._health.error(
                    'robot_start_service_unavailable',
                    f'Service {self.service_name} not available after '
                    f'{self.wait_timeout_seconds}s',
                    {
                        'service_name': self.service_name,
                        'run_id': payload.run_id,
                        'weightment_id': payload.weightment_id,
                    },
                )
                raise TimeoutError(
                    f'Service {self.service_name} not available after '
                    f'{self.wait_timeout_seconds}s'
                )

            req = StartWebhookWeightment.Request()
            req.run_id = payload.run_id
            req.weightment_id = payload.weightment_id
            req.batch_id = payload.batch_id
            req.ingredient_id = payload.ingredient_id
            req.location_id = payload.location_id
            req.location_code = payload.location_code
            req.pickup_target_name = payload.pickup_target_name
            req.weigh_target_name = payload.weigh_target_name
            req.return_target_name = payload.return_target_name
            req.target_weight_g = float(payload.target_weight_g)
            req.tolerance_g = float(payload.tolerance_g)
            req.expected_lot = payload.expected_lot

            future = self._client.call_async(req)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=self.call_timeout_seconds
            )
            if not future.done():
                self._health.error(
                    'robot_start_service_call_timeout',
                    f'Service {self.service_name} call timed out after '
                    f'{self.call_timeout_seconds}s',
                    {
                        'service_name': self.service_name,
                        'run_id': payload.run_id,
                        'weightment_id': payload.weightment_id,
                    },
                )
                raise TimeoutError(
                    f'Service {self.service_name} call timed out after '
                    f'{self.call_timeout_seconds}s'
                )
            response = future.result()
            if response is None:
                self._health.error(
                    'robot_start_service_empty_response',
                    f'Service {self.service_name} returned no response',
                    {'service_name': self.service_name, 'run_id': payload.run_id},
                )
                raise RuntimeError(
                    f'Service {self.service_name} returned no response'
                )
            if not response.accepted:
                self._health.warn(
                    'robot_start_rejected',
                    f'Service {self.service_name} rejected the run: '
                    f'{response.message}',
                    {'service_name': self.service_name, 'run_id': payload.run_id},
                )
            return StartWebhookWeightmentResponse(
                accepted=bool(response.accepted),
                message=str(response.message or ''),
            )

    def start_lightsout(
        self, payload: StartLightsOutRequest
    ) -> StartLightsOutResponse:
        if (
            not self._started
            or self._node is None
            or self._lightsout_client is None
        ):
            raise RuntimeError('ROS adapter is not initialized')
        with self._lock:
            if not self._lightsout_client.wait_for_service(
                timeout_sec=self.wait_timeout_seconds
            ):
                self._health.error(
                    'robot_start_service_unavailable',
                    f'Service {self.lightsout_service_name} not available after '
                    f'{self.wait_timeout_seconds}s',
                    {
                        'service_name': self.lightsout_service_name,
                        'batch_id': payload.batch_id,
                    },
                )
                raise TimeoutError(
                    f'Service {self.lightsout_service_name} not available after '
                    f'{self.wait_timeout_seconds}s'
                )

            req = StartLightsOut.Request()
            req.powder_id = payload.powder_id
            req.powder_name = payload.powder_name
            req.container_target = payload.container_target
            req.pour_target = payload.pour_target
            req.lot_code = payload.lot_code
            req.operator_name = payload.operator
            req.notes = payload.notes
            req.target_weight_g = float(payload.target_weight_g)
            req.episodes = int(payload.episodes)
            req.batch_id = payload.batch_id
            req.enable_scoop = bool(payload.enable_scoop)
            req.stop_on = payload.stop_on
            req.stop_value = float(payload.stop_value)
            req.target_mode = payload.target_mode
            req.target_fractions = [float(v) for v in payload.target_fractions]
            req.min_scooped_g = float(payload.min_scooped_g)
            req.target_min_g = float(payload.target_min_g)
            req.target_max_g = float(payload.target_max_g)
            req.rng_seed = int(payload.rng_seed)

            future = self._lightsout_client.call_async(req)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=self.call_timeout_seconds
            )
            if not future.done():
                self._health.error(
                    'robot_start_service_call_timeout',
                    f'Service {self.lightsout_service_name} call timed out after '
                    f'{self.call_timeout_seconds}s',
                    {
                        'service_name': self.lightsout_service_name,
                        'batch_id': payload.batch_id,
                    },
                )
                raise TimeoutError(
                    f'Service {self.lightsout_service_name} call timed out after '
                    f'{self.call_timeout_seconds}s'
                )
            response = future.result()
            if response is None:
                self._health.error(
                    'robot_start_service_empty_response',
                    f'Service {self.lightsout_service_name} returned no response',
                    {'service_name': self.lightsout_service_name},
                )
                raise RuntimeError(
                    f'Service {self.lightsout_service_name} returned no response'
                )
            if not response.accepted:
                self._health.warn(
                    'robot_start_rejected',
                    f'Service {self.lightsout_service_name} rejected the run: '
                    f'{response.message}',
                    {
                        'service_name': self.lightsout_service_name,
                        'batch_id': payload.batch_id,
                    },
                )
            return StartLightsOutResponse(
                accepted=bool(response.accepted),
                message=str(response.message or ''),
            )

    def apply_cell_layout(
        self, payload: ApplyCellLayoutRequest
    ) -> ApplyCellLayoutResponse:
        if not self._started or self._node is None or self._layout_client is None:
            raise RuntimeError('ROS adapter is not initialized')
        with self._lock:
            if not self._layout_client.wait_for_service(
                timeout_sec=self.wait_timeout_seconds
            ):
                raise TimeoutError(
                    '/cell_layout/apply not available after '
                    f'{self.wait_timeout_seconds}s'
                )
            req = ApplyCellLayout.Request()
            req.layout_id = payload.layout_id
            future = self._layout_client.call_async(req)
            rclpy.spin_until_future_complete(
                self._node, future, timeout_sec=self.call_timeout_seconds
            )
            if not future.done():
                raise TimeoutError(
                    '/cell_layout/apply call timed out after '
                    f'{self.call_timeout_seconds}s'
                )
            response = future.result()
            if response is None:
                raise RuntimeError('/cell_layout/apply returned no response')
            return ApplyCellLayoutResponse(
                success=bool(response.success),
                message=str(response.message or ''),
                layout_hash=str(response.layout_hash or ''),
                preflight_ok=bool(response.preflight_ok),
            )


starter = WebhookRobotStarter()


@asynccontextmanager
async def lifespan(_: FastAPI):
    starter.start()
    try:
        yield
    finally:
        starter.stop()


app = FastAPI(title='Webhook Robot Start Adapter', lifespan=lifespan)


@app.get('/health')
def health() -> dict:
    return {
        'status': 'ok',
        'service_ready': starter.service_ready(),
        'lightsout_service_ready': starter.lightsout_service_ready(),
    }


@app.post(
    '/start_webhook_weightment', response_model=StartWebhookWeightmentResponse
)
def start_webhook_weightment(
    payload: StartWebhookWeightmentRequest,
) -> StartWebhookWeightmentResponse:
    try:
        return starter.start_webhook_weightment(payload)
    except TimeoutError as exc:
        raise HTTPException(status_code=504, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.post('/start_lightsout', response_model=StartLightsOutResponse)
def start_lightsout(payload: StartLightsOutRequest) -> StartLightsOutResponse:
    try:
        return starter.start_lightsout(payload)
    except TimeoutError as exc:
        raise HTTPException(status_code=504, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc


@app.post('/apply_cell_layout', response_model=ApplyCellLayoutResponse)
def apply_cell_layout(payload: ApplyCellLayoutRequest) -> ApplyCellLayoutResponse:
    try:
        return starter.apply_cell_layout(payload)
    except TimeoutError as exc:
        raise HTTPException(status_code=504, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=500, detail=str(exc)) from exc
