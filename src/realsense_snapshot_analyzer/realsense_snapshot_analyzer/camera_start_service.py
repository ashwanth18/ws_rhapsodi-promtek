import os
import shlex
import signal
import subprocess
from typing import List, Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class RealSenseCameraLauncher(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("realsense_camera_launcher")
        self.declare_parameter("launch_package", "realsense2_camera")
        self.declare_parameter("launch_file", "rs_launch.py")
        self.declare_parameter("launch_args", "")
        self._proc: Optional[subprocess.Popen] = None

        self.get_logger().info(
            "RealSense launcher ready (lifecycle node). "
            "Use /realsense_camera_launcher/change_state to control."
        )

    def _build_command(self) -> List[str]:
        launch_pkg = str(self.get_parameter("launch_package").value)
        launch_file = str(self.get_parameter("launch_file").value)
        launch_args = str(self.get_parameter("launch_args").value).strip()
        cmd = ["ros2", "launch", launch_pkg, launch_file]
        if launch_args:
            cmd.extend(shlex.split(launch_args))
        return cmd

    def _is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _start_process(self) -> bool:
        if self._is_running():
            return True
        try:
            cmd = self._build_command()
            self.get_logger().info(f"Starting RealSense: {' '.join(cmd)}")
            self._proc = subprocess.Popen(
                cmd,
                start_new_session=True,
            )
            return True
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to start RealSense: {exc}")
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
            self.get_logger().error(f"Failed to stop RealSense: {exc}")
            return False

    def on_configure(self, state) -> TransitionCallbackReturn:
        _ = state
        self.get_logger().info("Configured RealSense launcher.")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state) -> TransitionCallbackReturn:
        _ = state
        ok = self._start_process()
        return (
            TransitionCallbackReturn.SUCCESS
            if ok
            else TransitionCallbackReturn.FAILURE
        )

    def on_deactivate(self, state) -> TransitionCallbackReturn:
        _ = state
        ok = self._stop_process()
        return (
            TransitionCallbackReturn.SUCCESS
            if ok
            else TransitionCallbackReturn.FAILURE
        )

    def on_cleanup(self, state) -> TransitionCallbackReturn:
        _ = state
        ok = self._stop_process()
        return (
            TransitionCallbackReturn.SUCCESS
            if ok
            else TransitionCallbackReturn.FAILURE
        )

    def on_shutdown(self, state) -> TransitionCallbackReturn:
        _ = state
        ok = self._stop_process()
        return (
            TransitionCallbackReturn.SUCCESS
            if ok
            else TransitionCallbackReturn.FAILURE
        )

    def shutdown(self) -> None:
        if not self._is_running():
            return
        try:
            assert self._proc is not None
            os.killpg(self._proc.pid, signal.SIGTERM)
            self._proc.wait(timeout=5)
        except Exception:
            try:
                if self._proc is not None:
                    os.killpg(self._proc.pid, signal.SIGKILL)
            except Exception:
                pass


def main() -> None:
    rclpy.init()
    node = RealSenseCameraLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
