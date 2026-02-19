import os
import signal
import subprocess
from typing import List, Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class ScaleLauncher(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("weighing_scale_launcher")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("topic", "weight")
        self._proc: Optional[subprocess.Popen] = None

        self.get_logger().info("Scale launcher ready (lifecycle node).")

    def _build_command(self) -> List[str]:
        port = str(self.get_parameter("port").value)
        baud = str(self.get_parameter("baud").value)
        topic = str(self.get_parameter("topic").value)
        return [
            "ros2",
            "launch",
            "weighing_scale_driver",
            "weighing_scale.launch.py",
            f"port:={port}",
            f"baud:={baud}",
            f"topic:={topic}",
        ]

    def _is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _start_process(self) -> bool:
        if self._is_running():
            return True
        try:
            cmd = self._build_command()
            self.get_logger().info(f"Starting scale: {' '.join(cmd)}")
            self._proc = subprocess.Popen(
                cmd,
                start_new_session=True,
            )
            return True
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to start scale: {exc}")
            return False

    def _stop_process(self) -> bool:
        if not self._is_running():
            return True
        try:
            assert self._proc is not None
            os.killpg(self._proc.pid, signal.SIGTERM)
            self._proc.wait(timeout=5)
            self._proc = None
            return True
        except Exception as exc:
            self.get_logger().error(f"Failed to stop scale: {exc}")
            return False

    def on_configure(self, state) -> TransitionCallbackReturn:
        _ = state
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        _ = state
        return (
            TransitionCallbackReturn.SUCCESS
            if self._start_process()
            else TransitionCallbackReturn.FAILURE
        )

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        _ = state
        return (
            TransitionCallbackReturn.SUCCESS
            if self._stop_process()
            else TransitionCallbackReturn.FAILURE
        )

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        _ = state
        return (
            TransitionCallbackReturn.SUCCESS
            if self._stop_process()
            else TransitionCallbackReturn.FAILURE
        )

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        _ = state
        return (
            TransitionCallbackReturn.SUCCESS
            if self._stop_process()
            else TransitionCallbackReturn.FAILURE
        )


def main() -> None:
    rclpy.init()
    node = ScaleLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
