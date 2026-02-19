import os
import shlex
import signal
import subprocess
from typing import List, Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn


class MicroRosLauncher(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("micro_ros_launcher")
        self.declare_parameter(
            "agent_args", "serial --dev /dev/ttyACM0"
        )
        self._proc: Optional[subprocess.Popen] = None

        self.get_logger().info(
            "micro-ROS launcher ready (lifecycle node)."
        )

    def _build_command(self) -> List[str]:
        args = str(self.get_parameter("agent_args").value).strip()
        cmd = ["ros2", "run", "micro_ros_agent", "micro_ros_agent"]
        if args:
            cmd.extend(shlex.split(args))
        return cmd

    def _serial_device(self) -> Optional[str]:
        args = str(self.get_parameter("agent_args").value).strip()
        if not args:
            return None
        parts = shlex.split(args)
        if "serial" not in parts:
            return None
        for flag in ("--dev", "-d"):
            if flag in parts:
                idx = parts.index(flag)
                if idx + 1 < len(parts):
                    return parts[idx + 1]
        return None

    def _check_serial_device(self) -> bool:
        device = self._serial_device()
        if not device:
            return True
        if not os.path.exists(device):
            self.get_logger().error(
                f"Serial device not found: {device}"
            )
            return False
        try:
            fd = os.open(
                device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
            )
            os.close(fd)
            return True
        except OSError as exc:
            self.get_logger().error(
                f"Serial device not ready: {device} ({exc})"
            )
            return False

    def _is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def _start_process(self) -> bool:
        if self._is_running():
            return True
        if not self._check_serial_device():
            return False
        try:
            cmd = self._build_command()
            self.get_logger().info(
                f"Starting micro-ROS agent: {' '.join(cmd)}"
            )
            self._proc = subprocess.Popen(
                cmd,
                start_new_session=True,
            )
            return True
        except Exception as exc:  # pragma: no cover
            self.get_logger().error(f"Failed to start micro-ROS: {exc}")
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
            self.get_logger().error(f"Failed to stop micro-ROS: {exc}")
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
    node = MicroRosLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()







