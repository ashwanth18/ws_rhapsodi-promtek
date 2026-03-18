import os
import threading
from contextlib import asynccontextmanager

import rclpy
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from robot_common_msgs.srv import StartWebhookWeightment


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


class WebhookRobotStarter:
    def __init__(self) -> None:
        self.service_name = os.environ.get(
            'ROBOT_START_SERVICE_NAME', '/bt_start_webhook_weightment'
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

    def start(self) -> None:
        if self._started:
            return
        rclpy.init(args=None)
        self._node = rclpy.create_node('webhook_robot_start_adapter')
        self._client = self._node.create_client(
            StartWebhookWeightment, self.service_name
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

    def service_ready(self) -> bool:
        if not self._started or self._client is None:
            return False
        return bool(
            self._client.wait_for_service(
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
                raise TimeoutError(
                    f'Service {self.service_name} call timed out after '
                    f'{self.call_timeout_seconds}s'
                )
            response = future.result()
            if response is None:
                raise RuntimeError(
                    f'Service {self.service_name} returned no response'
                )
            return StartWebhookWeightmentResponse(
                accepted=bool(response.accepted),
                message=str(response.message or ''),
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
    return {'status': 'ok', 'service_ready': starter.service_ready()}


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
