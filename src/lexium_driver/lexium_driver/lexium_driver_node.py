"""ROS 2 driver node for the Schneider Lexium Cobot.

Bridges MoveIt 2 to the Lexium JSON/TCP protocol:

* Publishes ``/joint_states`` (radians) and ``/lexium/status`` from the 50 Hz
  feedback stream (port 10000).
* Hosts a ``FollowJointTrajectory`` action server at
  ``/<controller_name>/follow_joint_trajectory`` -- the action name MoveIt's
  MoveItSimpleControllerManager expects -- and executes trajectories as a
  sequence of ``moveJ`` commands (port 10001).
* Provides power/enable/clear-error/stop services and continuous safety gating.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Duration as DurationMsg  # noqa: F401
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

from lexium_msgs.msg import LexiumStatus
from lexium_msgs.action import MoveSequence

from lexium_driver.lexium_tcp import LexiumClient, LexiumError

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0

CONTROL_SOURCE_REMOTE = 3


def _is_no_error_code(code: str) -> bool:
    """True when the Lexium controller reports no active error."""
    normalized = str(code).strip().lower()
    if not normalized or normalized in {"0", "ok", "none"}:
        return True
    if normalized.startswith("0x"):
        digits = normalized[2:]
        return bool(digits) and all(ch == "0" for ch in digits)
    return False


class LexiumDriver(Node):
    def __init__(self) -> None:
        super().__init__("lexium_driver")

        # ------------------------- parameters ------------------------- #
        self.ip = self.declare_parameter("ip", "192.168.88.82").value
        self.command_port = int(self.declare_parameter("command_port", 10001).value)
        self.feedback_port = int(self.declare_parameter("feedback_port", 10000).value)
        self.joints: List[str] = list(
            self.declare_parameter(
                "joints",
                ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            ).value
        )
        self.controller_name = self.declare_parameter(
            "controller_name", "jaka_zu5_controller"
        ).value

        # Per-joint calibration between URDF zero/direction and Lexium zero.
        self.joint_sign: List[float] = [
            float(s) for s in self.declare_parameter(
                "joint_sign", [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            ).value
        ]
        self.joint_offset_deg: List[float] = [
            float(o) for o in self.declare_parameter(
                "joint_offset_deg", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            ).value
        ]

        # Motion shaping.
        self.move_speed_deg = float(self.declare_parameter("move_speed_deg", 30.0).value)
        self.move_acc_deg = float(self.declare_parameter("move_acc_deg", 60.0).value)
        # Derive each moveJ speed/acc from the planned trajectory (honours the
        # MoveIt velocity/acceleration scaling sliders) instead of a fixed value.
        # move_speed_deg / move_acc_deg then act as the upper caps.
        self.speed_from_trajectory = bool(
            self.declare_parameter("speed_from_trajectory", True).value
        )
        self.min_move_speed_deg = float(
            self.declare_parameter("min_move_speed_deg", 2.0).value
        )
        # Apply the controller's global speed override (set_speed_rate) from the
        # planned trajectory's peak velocity. Unlike the per-moveJ speed cap,
        # this scales acceleration too, so the MoveIt velocity slider stays
        # visible even on short (acceleration-limited) segments.
        self.speed_rate_from_trajectory = bool(
            self.declare_parameter("speed_rate_from_trajectory", True).value
        )
        self.max_joint_speed_degps = float(
            self.declare_parameter("max_joint_speed_degps", 90.0).value
        )
        self.min_speed_rate = float(
            self.declare_parameter("min_speed_rate", 0.02).value
        )
        self.arc_transition_deg = float(
            self.declare_parameter("arc_transition_deg", 2.0).value
        )
        self.downsample_threshold_deg = float(
            self.declare_parameter("downsample_threshold_deg", 15.0).value
        )
        self.execution_mode = self.declare_parameter(
            "execution_mode", "blended"  # "blended" or "sequential"
        ).value
        self.goal_tolerance_deg = float(
            self.declare_parameter("goal_tolerance_deg", 1.0).value
        )
        self.inter_command_delay = float(
            self.declare_parameter("inter_command_delay", 0.02).value
        )

        # Safety / startup.
        self.feedback_timeout = float(
            self.declare_parameter("feedback_timeout", 1.0).value
        )
        self.require_remote_control = bool(
            self.declare_parameter("require_remote_control", True).value
        )
        self.auto_power_on = bool(self.declare_parameter("auto_power_on", False).value)
        self.auto_enable = bool(self.declare_parameter("auto_enable", False).value)
        self.speed_rate = float(self.declare_parameter("speed_rate", 0.1).value)
        self.motion_wait_timeout = float(
            self.declare_parameter("motion_wait_timeout", 60.0).value
        )
        self.command_timeout = float(
            self.declare_parameter("command_timeout", 10.0).value
        )
        self.state_command_timeout = float(
            self.declare_parameter("state_command_timeout", 45.0).value
        )
        self.state_feedback_timeout = float(
            self.declare_parameter("state_feedback_timeout", 15.0).value
        )
        self.power_on_settle_sec = float(
            self.declare_parameter("power_on_settle_sec", 3.0).value
        )
        self.power_off_settle_sec = float(
            self.declare_parameter("power_off_settle_sec", 1.0).value
        )
        self.pre_enable_settle_sec = float(
            self.declare_parameter("pre_enable_settle_sec", 2.0).value
        )
        self.enable_retry_count = int(
            self.declare_parameter("enable_retry_count", 3).value
        )
        self.enable_retry_delay = float(
            self.declare_parameter("enable_retry_delay", 2.0).value
        )

        if len(self.joints) != 6:
            self.get_logger().warn(
                f"Expected 6 joints, got {len(self.joints)}; protocol arrays are [0..5]."
            )

        # ------------------------- TCP client ------------------------- #
        self.client = LexiumClient(
            ip=self.ip,
            command_port=self.command_port,
            feedback_port=self.feedback_port,
            command_timeout=self.command_timeout,
            logger=lambda m: self.get_logger().info(m),
        )

        self._goal_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._state_sequence_active = False
        self._active_goal = None
        self._command_id = 0
        # control_source is NOT in the feedback stream; it is only available
        # via the get_control_source command, so we poll + cache it.
        self._control_source = 0

        cb_group = ReentrantCallbackGroup()

        # ------------------------- interfaces ------------------------- #
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.status_pub = self.create_publisher(LexiumStatus, "/lexium/status", 10)

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"/{self.controller_name}/follow_joint_trajectory",
            execute_callback=self._execute_trajectory,
            goal_callback=self._handle_goal,
            cancel_callback=self._handle_cancel,
            callback_group=cb_group,
        )

        # Low-level interface: a list of moves, each with its own speed/acc.
        self.move_sequence_server = ActionServer(
            self,
            MoveSequence,
            "~/move_sequence",
            execute_callback=self._execute_move_sequence,
            goal_callback=self._handle_sequence_goal,
            cancel_callback=self._handle_cancel,
            callback_group=cb_group,
        )

        self._make_cycle_trigger("bring_up", self._bring_up, cb_group)
        self._make_cycle_trigger("shut_down", self._shut_down, cb_group)
        # Low-level steps (CLI / debugging); RViz uses bring_up / shut_down.
        self._make_state_trigger(
            "power_on",
            "power_on",
            lambda fb: bool(int(fb.get("powered_on", 0))),
            cb_group,
            settle_after=self.power_on_settle_sec,
        )
        self._make_state_trigger(
            "power_off",
            "power_off",
            lambda fb: not bool(int(fb.get("powered_on", 0))),
            cb_group,
        )
        self._make_state_trigger(
            "enable",
            "enable_robot",
            lambda fb: bool(int(fb.get("enabled", 0))),
            cb_group,
        )
        self._make_state_trigger(
            "disable",
            "disable_robot",
            lambda fb: not bool(int(fb.get("enabled", 0))),
            cb_group,
        )
        self._make_trigger("clear_error", lambda: self._cmd("clear_error"), cb_group)
        self._make_trigger("stop", lambda: self._cmd("stop_program"), cb_group)

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Publish feedback-derived topics at ~50 Hz.
        self.create_timer(0.02, self._publish_state, callback_group=cb_group)
        # Poll control source (~1 Hz) since it is not in the feedback stream.
        self.create_timer(1.0, self._poll_control_source, callback_group=cb_group)

        # Connect + run startup sequence off the executor thread.
        threading.Thread(target=self._startup, name="lexium_startup", daemon=True).start()

    # ------------------------------------------------------------------ #
    # Startup / connection
    # ------------------------------------------------------------------ #
    def _startup(self) -> None:
        try:
            self.client.start()
        except OSError as exc:
            self.get_logger().error(f"Failed to connect to {self.ip}: {exc}")
            return

        # Wait for the first feedback so we know the channel is live.
        deadline = time.monotonic() + 5.0
        while self.client.feedback_age() > self.feedback_timeout:
            if time.monotonic() > deadline:
                self.get_logger().warn("No feedback received yet from port 10000.")
                break
            time.sleep(0.05)

        src = self._query_control_source()
        if src is not None:
            self.get_logger().info(f"Control source = {src} (3 == Remote).")
            if self.require_remote_control and src != CONTROL_SOURCE_REMOTE:
                self.get_logger().warn(
                    "Control source is not Remote(3). Delegate control to Remote in "
                    "EcoStruxure Cobot Expert before commanding motion."
                )

        if self.auto_power_on:
            self._safe_cmd("power_on")
            time.sleep(2.0)
        if self.auto_enable:
            self._safe_cmd("enable_robot")
            time.sleep(1.0)

        # Apply the conservative default speed override.
        self._apply_speed_rate(self.speed_rate)
        self.get_logger().info("Lexium driver ready.")

    def _query_control_source(self) -> Optional[int]:
        """Query + cache control source (absent from the feedback stream)."""
        try:
            reply = self.client.send_command({"cmdName": "get_control_source"})
            self._control_source = int(reply.get("control_source", 0))
            return self._control_source
        except (LexiumError, OSError) as exc:
            self.get_logger().warn(f"get_control_source failed: {exc}")
            return None

    def _poll_control_source(self) -> None:
        if self._state_sequence_active:
            return
        # Skip while a trajectory is executing to avoid interleaving commands.
        with self._goal_lock:
            if self._active_goal is not None:
                return
        if self.client.command_connected:
            self._query_control_source()

    # ------------------------------------------------------------------ #
    # Feedback -> ROS
    # ------------------------------------------------------------------ #
    def _publish_state(self) -> None:
        fb = self.client.get_feedback()
        fresh = self.client.feedback_age() <= self.feedback_timeout
        now = self.get_clock().now().to_msg()

        if fb is not None and "joint_position" in fb:
            js = JointState()
            js.header.stamp = now
            js.name = list(self.joints)
            js.position = self._lexium_to_ros(fb["joint_position"])
            self.joint_state_pub.publish(js)

        status = LexiumStatus()
        status.header.stamp = now
        status.connected = self.client.command_connected
        status.feedback_fresh = fresh
        status.control_source = self._control_source
        if fb is not None:
            status.powered_on = bool(int(fb.get("powered_on", 0)))
            status.enabled = bool(int(fb.get("enabled", 0)))
            status.in_position = bool(fb.get("in_position", False))
            status.protective_stop = bool(int(fb.get("protective_stop", 0)))
            status.emergency_stop = bool(int(fb.get("emergency_stop", 0)))
            status.collision_stop = bool(int(fb.get("collision_stop", 0)))
            status.on_soft_limit = bool(int(fb.get("on_soft_limit", 0)))
            status.error_code = str(fb.get("error_code", ""))
            status.error_msg = str(fb.get("error_msg", ""))
            status.command_id = int(fb.get("command_id", 0))
        self.status_pub.publish(status)

    # ------------------------------------------------------------------ #
    # Conversions
    # ------------------------------------------------------------------ #
    def _ros_to_lexium(self, positions_rad: List[float]) -> List[float]:
        """Ordered-by-self.joints radians -> Lexium degrees [j1..j6]."""
        out = []
        for i, p in enumerate(positions_rad):
            out.append(self.joint_sign[i] * (p * RAD2DEG) + self.joint_offset_deg[i])
        return out

    def _lexium_to_ros(self, positions_deg: List[float]) -> List[float]:
        out = []
        for i, d in enumerate(positions_deg):
            sign = self.joint_sign[i] if self.joint_sign[i] != 0.0 else 1.0
            out.append(((float(d) - self.joint_offset_deg[i]) / sign) * DEG2RAD)
        return out

    # ------------------------------------------------------------------ #
    # Safety helpers
    # ------------------------------------------------------------------ #
    def _active_faults(self, fb: Optional[Dict]) -> List[str]:
        if fb is None:
            return ["no_feedback"]
        faults = []
        if int(fb.get("protective_stop", 0)):
            faults.append("protective_stop")
        if int(fb.get("emergency_stop", 0)):
            faults.append("emergency_stop")
        if int(fb.get("collision_stop", 0)):
            faults.append("collision_stop")
        if int(fb.get("on_soft_limit", 0)):
            faults.append("on_soft_limit")
        return faults

    def _check_ready_for_motion(self) -> Optional[str]:
        """Return a human-readable reason if motion is unsafe, else None."""
        if not self.client.command_connected:
            return "command channel not connected"
        if self.client.feedback_age() > self.feedback_timeout:
            return "feedback stream stale (comms loss)"
        fb = self.client.get_feedback()
        if fb is None:
            return "no feedback yet"
        if self.require_remote_control:
            # Refresh from the command channel (not in the feedback stream).
            src = self._query_control_source()
            if src is None:
                src = self._control_source
            if src != CONTROL_SOURCE_REMOTE:
                return f"control source is not Remote(3), got {src}"
        if not bool(int(fb.get("powered_on", 0))):
            return "arm not powered on"
        if not bool(int(fb.get("enabled", 0))):
            return "arm not enabled"
        faults = self._active_faults(fb)
        if faults:
            return "active fault(s): " + ", ".join(faults)
        return None

    # ------------------------------------------------------------------ #
    # Action server
    # ------------------------------------------------------------------ #
    def _handle_goal(self, goal_request) -> GoalResponse:
        with self._goal_lock:
            if self._active_goal is not None:
                self.get_logger().warn("Rejecting goal: another trajectory is active.")
                return GoalResponse.REJECT
        reason = self._check_ready_for_motion()
        if reason is not None:
            self.get_logger().error(f"Rejecting goal: {reason}.")
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().error("Rejecting goal: empty trajectory.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _handle_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("Cancel requested; stopping robot.")
        self._safe_cmd("stop_program")
        return CancelResponse.ACCEPT

    def _execute_trajectory(self, goal_handle):
        with self._goal_lock:
            self._active_goal = goal_handle
        try:
            return self._run_trajectory(goal_handle)
        finally:
            # Restore the configured override if the trajectory changed it.
            if self.speed_rate_from_trajectory:
                self._apply_speed_rate(self.speed_rate)
            with self._goal_lock:
                self._active_goal = None

    def _run_trajectory(self, goal_handle):
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory

        # Map trajectory joint order -> our self.joints order.
        try:
            index_map = [traj.joint_names.index(j) for j in self.joints]
        except ValueError as exc:
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = f"trajectory missing joint: {exc}"
            self.get_logger().error(result.error_string)
            goal_handle.abort()
            return result

        waypoints_deg = []
        peak_speed_degps = []  # per-point max |joint velocity| in deg/s
        peak_acc_degps2 = []   # per-point max |joint acceleration| in deg/s^2
        for pt in traj.points:
            ordered_rad = [pt.positions[i] for i in index_map]
            waypoints_deg.append(self._ros_to_lexium(ordered_rad))
            if pt.velocities:
                peak_speed_degps.append(
                    max(abs(pt.velocities[i]) for i in index_map) * RAD2DEG
                )
            else:
                peak_speed_degps.append(0.0)
            if pt.accelerations:
                peak_acc_degps2.append(
                    max(abs(pt.accelerations[i]) for i in index_map) * RAD2DEG
                )
            else:
                peak_acc_degps2.append(0.0)

        kept_idx = self._downsample_indices(waypoints_deg)

        # Derive the controller's global speed override from the planned peak
        # velocity, so the MoveIt velocity slider scales the whole motion
        # (including acceleration) even on short segments.
        use_full_caps = False
        traj_peak = max(peak_speed_degps, default=0.0)
        if self.speed_rate_from_trajectory and traj_peak > 0.0:
            rate = traj_peak / self.max_joint_speed_degps
            rate = max(self.min_speed_rate, min(1.0, rate))
            self._apply_speed_rate(rate)
            use_full_caps = True

        self.get_logger().info(
            f"Executing trajectory: {len(traj.points)} points -> {len(kept_idx)} moveJ "
            f"waypoints (mode={self.execution_mode}, "
            f"speed_from_trajectory={self.speed_from_trajectory}, "
            f"speed_rate_from_trajectory={self.speed_rate_from_trajectory})."
        )

        last_id = 0
        prev_idx = 0
        for k, pt_idx in enumerate(kept_idx):
            abort = self._guard(goal_handle)
            if abort is not None:
                return abort

            wp = waypoints_deg[pt_idx]
            # Peak demand over the span since the previous kept waypoint.
            span_lo = prev_idx if k == 0 else kept_idx[k - 1] + 1
            speed, acc = self._segment_speed_acc(
                peak_speed_degps, peak_acc_degps2, span_lo, pt_idx, use_full_caps
            )
            prev_idx = pt_idx

            self._command_id += 1
            last_id = self._command_id
            is_last = k == len(kept_idx) - 1
            arc = (
                0.0
                if (self.execution_mode == "sequential" or is_last)
                else self.arc_transition_deg
            )
            cmd = {
                "cmdName": "moveJ",
                "relFlag": 0,
                "jointPosition": [round(v, 4) for v in wp],
                "speed": speed,
                "acc": acc,
                "arc_transition": arc,
                "command_id": last_id,
            }
            try:
                self.client.send_command(cmd)
            except (LexiumError, OSError) as exc:
                self._safe_cmd("stop_program")
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = f"moveJ failed: {exc}"
                self.get_logger().error(result.error_string)
                goal_handle.abort()
                return result

            if self.execution_mode == "sequential":
                abort = self._wait_until_reached(goal_handle, wp, last_id)
                if abort is not None:
                    return abort
            else:
                time.sleep(self.inter_command_delay)

        # Wait for the final target.
        abort = self._wait_until_reached(goal_handle, waypoints_deg[kept_idx[-1]], last_id)
        if abort is not None:
            return abort

        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "trajectory completed"
        goal_handle.succeed()
        self.get_logger().info("Trajectory completed.")
        return result

    def _segment_speed_acc(self, peak_speed, peak_acc, lo: int, hi: int,
                           use_full_caps: bool = False):
        """Pick moveJ speed/acc (deg, deg/s^2) for the span [lo..hi]."""
        if use_full_caps or not self.speed_from_trajectory:
            # Global set_speed_rate is doing the scaling; send the full caps.
            return self.move_speed_deg, self.move_acc_deg
        seg_speed = max(peak_speed[lo:hi + 1], default=0.0)
        seg_acc = max(peak_acc[lo:hi + 1], default=0.0)
        if seg_speed <= self.min_move_speed_deg:
            speed = self.move_speed_deg  # planner gave no useful velocity here
        else:
            speed = min(seg_speed, self.move_speed_deg)
        acc = self.move_acc_deg if seg_acc <= 0.0 else min(seg_acc, self.move_acc_deg)
        return max(speed, self.min_move_speed_deg), max(acc, 1.0)

    def _downsample_indices(self, waypoints_deg: List[List[float]]) -> List[int]:
        if len(waypoints_deg) <= 2:
            return list(range(len(waypoints_deg)))
        kept = [0]
        last = waypoints_deg[0]
        for i in range(1, len(waypoints_deg) - 1):
            wp = waypoints_deg[i]
            if max(abs(a - b) for a, b in zip(wp, last)) >= self.downsample_threshold_deg:
                kept.append(i)
                last = wp
        kept.append(len(waypoints_deg) - 1)
        return kept

    def _motion_problem(self, goal_handle):
        """Check for a stop condition during motion.

        Returns ``(kind, reason)`` where kind is None, "cancel", or "abort".
        """
        if not rclpy.ok():
            return "abort", "node shutting down"
        if goal_handle.is_cancel_requested:
            return "cancel", "canceled"
        if self.client.feedback_age() > self.feedback_timeout:
            return "abort", "feedback stream stale during motion (comms loss)"
        faults = self._active_faults(self.client.get_feedback())
        if faults:
            return "abort", "fault during motion: " + ", ".join(faults)
        return None, None

    def _guard(self, goal_handle):
        """Return an aborted/canceled FollowJointTrajectory Result, or None."""
        kind, reason = self._motion_problem(goal_handle)
        if kind is None:
            return None
        self._safe_cmd("stop_program")
        result = FollowJointTrajectory.Result()
        if kind == "cancel":
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "canceled"
            goal_handle.canceled()
        else:
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = reason
            self.get_logger().error(reason)
            goal_handle.abort()
        return result

    def _wait_until_reached(self, goal_handle, target_deg: List[float], command_id: int):
        """Block until in_position at target, or return an abort/cancel Result."""
        deadline = time.monotonic() + self.motion_wait_timeout
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = list(self.joints)
        while True:
            abort = self._guard(goal_handle)
            if abort is not None:
                return abort

            fb = self.client.get_feedback()
            if fb is not None and "joint_position" in fb:
                actual = [float(v) for v in fb["joint_position"]]
                self._publish_action_feedback(feedback_msg, goal_handle, actual)
                in_pos = bool(fb.get("in_position", False))
                reached = all(
                    abs(a - t) <= self.goal_tolerance_deg
                    for a, t in zip(actual, target_deg)
                )
                cur_id = int(fb.get("command_id", 0))
                if reached and (in_pos or cur_id >= command_id):
                    return None

            if time.monotonic() > deadline:
                self._safe_cmd("stop_program")
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = "timed out waiting to reach target"
                self.get_logger().error(result.error_string)
                goal_handle.abort()
                return result
            time.sleep(0.02)

    def _publish_action_feedback(self, feedback_msg, goal_handle, actual_deg) -> None:
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.actual.positions = self._lexium_to_ros(actual_deg)
        try:
            goal_handle.publish_feedback(feedback_msg)
        except Exception:  # noqa: BLE001 - feedback is best-effort
            pass

    # ------------------------------------------------------------------ #
    # MoveSequence action: list of moves, each with its own speed/acc
    # ------------------------------------------------------------------ #
    def _handle_sequence_goal(self, goal_request) -> GoalResponse:
        with self._goal_lock:
            if self._active_goal is not None:
                self.get_logger().warn("Rejecting move sequence: another goal is active.")
                return GoalResponse.REJECT
        reason = self._check_ready_for_motion()
        if reason is not None:
            self.get_logger().error(f"Rejecting move sequence: {reason}.")
            return GoalResponse.REJECT
        if not goal_request.moves:
            self.get_logger().error("Rejecting move sequence: no moves.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _execute_move_sequence(self, goal_handle):
        with self._goal_lock:
            self._active_goal = goal_handle
        try:
            return self._run_move_sequence(goal_handle)
        finally:
            with self._goal_lock:
                self._active_goal = None

    def _run_move_sequence(self, goal_handle):
        result = MoveSequence.Result()
        moves = goal_handle.request.moves
        fb_msg = MoveSequence.Feedback()
        fb_msg.total = len(moves)
        self.get_logger().info(f"Executing move sequence: {len(moves)} move(s).")

        for i, mc in enumerate(moves):
            kind, reason = self._motion_problem(goal_handle)
            if kind is not None:
                return self._finish_sequence(goal_handle, result, kind, reason, i)

            self._command_id += 1
            cid = self._command_id
            try:
                cmd = self._build_move_cmd(mc, cid)
            except ValueError as exc:
                self._safe_cmd("stop_program")
                result.success = False
                result.message = str(exc)
                result.completed = i
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            try:
                self.client.send_command(cmd)
            except (LexiumError, OSError) as exc:
                self._safe_cmd("stop_program")
                result.success = False
                result.message = f"{cmd['cmdName']} failed: {exc}"
                result.completed = i
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            fb_msg.current_index = i
            fb_msg.active_command_id = cid
            try:
                goal_handle.publish_feedback(fb_msg)
            except Exception:  # noqa: BLE001
                pass

            is_last = i == len(moves) - 1
            if float(mc.arc_transition) > 0.0 and not is_last:
                # Blend into the next move: pipeline, do not wait for standstill.
                time.sleep(self.inter_command_delay)
            else:
                kind, reason = self._wait_for_command(goal_handle, cid)
                if kind is not None:
                    return self._finish_sequence(goal_handle, result, kind, reason, i)
            result.completed = i + 1

        result.success = True
        result.message = "sequence completed"
        goal_handle.succeed()
        self.get_logger().info("Move sequence completed.")
        return result

    def _finish_sequence(self, goal_handle, result, kind: str, reason: str, completed: int):
        self._safe_cmd("stop_program")
        result.completed = completed
        result.message = reason
        if kind == "cancel":
            result.success = False
            goal_handle.canceled()
        else:
            result.success = False
            self.get_logger().error(reason)
            goal_handle.abort()
        return result

    def _build_move_cmd(self, mc, command_id: int) -> Dict:
        move_type = mc.move_type or "moveJ"
        speed = mc.speed if mc.speed > 0.0 else self.move_speed_deg
        acc = mc.acc if mc.acc > 0.0 else self.move_acc_deg
        target = [round(float(v), 4) for v in mc.target]
        cmd = {
            "speed": speed,
            "acc": acc,
            "arc_transition": float(mc.arc_transition),
            "command_id": command_id,
        }
        if move_type == "moveJ":
            if len(target) != 6:
                raise ValueError("moveJ target must have 6 joint values (deg)")
            cmd.update({"cmdName": "moveJ", "relFlag": int(mc.rel_flag),
                        "jointPosition": target})
        elif move_type == "moveL":
            if len(target) != 6:
                raise ValueError("moveL target must be [x,y,z,rx,ry,rz]")
            cmd.update({"cmdName": "moveL", "relFlag": int(mc.rel_flag),
                        "cartPosition": target})
        elif move_type == "moveTCP":
            if len(target) != 6:
                raise ValueError("moveTCP target must be [x,y,z,rx,ry,rz]")
            cmd.update({"cmdName": "moveTCP", "cartPosition": target})
        elif move_type == "moveC":
            if len(target) != 6 or len(mc.circ) != 6:
                raise ValueError("moveC needs target and circ as [x,y,z,rx,ry,rz]")
            cmd.update({"cmdName": "moveC", "relFlag": int(mc.rel_flag),
                        "circPosition": [round(float(v), 4) for v in mc.circ],
                        "cartPosition": target})
        else:
            raise ValueError(f"unknown move_type '{move_type}'")
        return cmd

    def _wait_for_command(self, goal_handle, command_id: int):
        """Wait until the controller reports this command_id done (in_position)."""
        deadline = time.monotonic() + self.motion_wait_timeout
        while True:
            kind, reason = self._motion_problem(goal_handle)
            if kind is not None:
                return kind, reason
            fb = self.client.get_feedback()
            if fb is not None:
                if bool(fb.get("in_position", False)) and int(fb.get("command_id", 0)) >= command_id:
                    return None, None
            if time.monotonic() > deadline:
                return "abort", "timed out waiting for move to complete"
            time.sleep(0.02)

    # ------------------------------------------------------------------ #
    # Services / parameters
    # ------------------------------------------------------------------ #
    def _wait_for_feedback(self, predicate, timeout: float = 8.0) -> bool:
        """Wait until feedback satisfies predicate(fb), or timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            fb = self.client.get_feedback()
            if fb is not None and predicate(fb):
                return True
            time.sleep(0.05)
        return False

    @staticmethod
    def _fb_powered(fb: Dict) -> bool:
        return bool(int(fb.get("powered_on", 0)))

    @staticmethod
    def _fb_enabled(fb: Dict) -> bool:
        return bool(int(fb.get("enabled", 0)))

    def _run_state_step(
        self,
        label: str,
        cmd_name: str,
        predicate,
        settle_after: float = 0.0,
    ) -> tuple[bool, str]:
        """Send one state command and wait for feedback confirmation."""
        try:
            self._cmd(cmd_name, timeout=self.state_command_timeout)
            if settle_after > 0.0:
                time.sleep(settle_after)
        except LexiumError as exc:
            self.get_logger().warn(f"{label}: controller rejected command ({exc})")
            if self._wait_for_feedback(predicate, timeout=2.0):
                return True, f"confirmed via feedback despite: {exc}"
            return False, str(exc)
        except (OSError, TimeoutError) as exc:
            self.get_logger().warn(
                f"{label}: command ack not received ({exc}); checking feedback"
            )
            if self._wait_for_feedback(predicate, timeout=self.state_feedback_timeout):
                return True, f"confirmed via feedback (command ack: {exc})"
            return False, str(exc)

        if self._wait_for_feedback(predicate, timeout=self.state_feedback_timeout):
            return True, "ok"
        return False, f"{label}: feedback did not confirm the new state"

    def _safe_clear_error(self) -> None:
        try:
            self._cmd("clear_error", timeout=self.state_command_timeout)
        except (LexiumError, OSError, TimeoutError):
            pass

    def _try_enable(self) -> tuple[bool, str]:
        last_msg = "enable failed"
        for attempt in range(1, self.enable_retry_count + 1):
            ok, msg = self._run_state_step(
                "enable", "enable_robot", self._fb_enabled
            )
            if ok:
                return True, msg
            last_msg = msg
            self.get_logger().warn(
                f"enable attempt {attempt}/{self.enable_retry_count}: {msg}"
            )
            if attempt < self.enable_retry_count:
                time.sleep(self.enable_retry_delay)
        return False, last_msg

    def _full_power_cycle(self) -> tuple[bool, str]:
        """Power off (if needed), then power on and settle."""
        fb = self.client.get_feedback()
        if fb is not None and self._fb_powered(fb):
            ok, msg = self._run_state_step(
                "power_off",
                "power_off",
                lambda f: not self._fb_powered(f),
            )
            if not ok:
                self.get_logger().warn(
                    f"power_off before bring-up: {msg}; continuing with power_on"
                )
            time.sleep(self.power_off_settle_sec)

        return self._run_state_step(
            "power_on",
            "power_on",
            self._fb_powered,
            settle_after=self.power_on_settle_sec,
        )

    def _bring_up(self) -> tuple[bool, str]:
        """Power on (if needed), settle, then enable — one operator action."""
        with self._state_lock:
            self._state_sequence_active = True
            try:
                src = self._query_control_source()
                if self.require_remote_control and src != CONTROL_SOURCE_REMOTE:
                    return False, (
                        f"control source is not Remote(3), got {src}. "
                        "Delegate control in Cobot Expert first."
                    )

                fb = self.client.get_feedback()
                if fb is not None and self._fb_enabled(fb):
                    return True, "already enabled"

                self._safe_clear_error()

                # Fast path when feedback says powered but not enabled.
                fb = self.client.get_feedback()
                if fb is not None and self._fb_powered(fb) and not self._fb_enabled(fb):
                    time.sleep(self.pre_enable_settle_sec)
                    ok, msg = self._try_enable()
                    if ok:
                        self.get_logger().info("Bring up complete (enabled).")
                        return True, "enabled"

                # Full power cycle (handles stale powered_on after shut_down).
                ok, msg = self._full_power_cycle()
                if not ok:
                    return False, f"bring_up failed during power cycle: {msg}"

                time.sleep(self.pre_enable_settle_sec)
                ok, msg = self._try_enable()
                if not ok:
                    return False, f"bring_up failed at enable: {msg}"

                self._query_control_source()
                self.get_logger().info("Bring up complete (powered on + enabled).")
                return True, "powered on and enabled"
            finally:
                self._state_sequence_active = False

    def _shut_down(self) -> tuple[bool, str]:
        """Disable, then power off — safe shutdown sequence."""
        with self._state_lock:
            self._state_sequence_active = True
            try:
                fb = self.client.get_feedback()
                if fb is not None and self._fb_enabled(fb):
                    ok, msg = self._run_state_step(
                        "disable",
                        "disable_robot",
                        lambda f: not self._fb_enabled(f),
                    )
                    if not ok:
                        return False, f"shut_down failed at disable: {msg}"

                fb = self.client.get_feedback()
                if fb is not None and self._fb_powered(fb):
                    ok, msg = self._run_state_step(
                        "power_off",
                        "power_off",
                        lambda f: not self._fb_powered(f),
                    )
                    if not ok:
                        return False, f"shut_down failed at power_off: {msg}"

                self.get_logger().info("Shut down complete (disabled + powered off).")
                return True, "disabled and powered off"
            finally:
                self._state_sequence_active = False

    def _make_cycle_trigger(self, name: str, action, cb_group) -> None:
        def _cb(request, response):
            try:
                success, message = action()
                response.success = success
                response.message = message
                if not success:
                    self.get_logger().error(f"{name} failed: {message}")
            except Exception as exc:  # noqa: BLE001 - surface to operator
                response.success = False
                response.message = str(exc)
                self.get_logger().error(f"{name} failed: {exc}")
            return response

        self.create_service(Trigger, f"~/{name}", _cb, callback_group=cb_group)

    def _make_state_trigger(
        self,
        name: str,
        cmd_name: str,
        predicate,
        cb_group,
        settle_after: float = 0.0,
    ) -> None:
        """Trigger service that waits for the expected feedback state."""

        def _cb(request, response):
            cmd_error = None
            try:
                self._cmd(cmd_name, timeout=self.state_command_timeout)
                if settle_after > 0.0:
                    time.sleep(settle_after)
            except (LexiumError, OSError, TimeoutError) as exc:
                cmd_error = exc
                self.get_logger().warn(
                    f"{name}: command ack not received ({exc}); checking feedback"
                )

            self._query_control_source()
            if self._wait_for_feedback(predicate, timeout=self.state_feedback_timeout):
                response.success = True
                if cmd_error is None:
                    response.message = "ok"
                else:
                    response.message = (
                        f"{name} confirmed via feedback (command ack: {cmd_error})"
                    )
                return response

            if cmd_error is not None:
                response.success = False
                response.message = str(cmd_error)
                self.get_logger().error(f"{name} failed: {cmd_error}")
            else:
                response.success = True
                response.message = (
                    f"{name} sent; feedback has not confirmed the new state yet"
                )
                self.get_logger().warn(response.message)
            return response

        self.create_service(Trigger, f"~/{name}", _cb, callback_group=cb_group)

    def _make_trigger(self, name: str, action, cb_group) -> None:
        def _cb(request, response):
            try:
                action()
                response.success = True
                response.message = "ok"
            except (LexiumError, OSError, TimeoutError) as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().error(f"{name} failed: {exc}")
            return response

        self.create_service(Trigger, f"~/{name}", _cb, callback_group=cb_group)

    def _cmd(self, cmd_name: str, timeout: Optional[float] = None) -> Dict:
        return self.client.send_command(
            {"cmdName": cmd_name}, timeout=timeout
        )

    def _safe_cmd(self, cmd_name: str) -> None:
        try:
            self.client.send_command(
                {"cmdName": cmd_name}, raise_on_error=False
            )
        except (OSError, TimeoutError) as exc:
            self.get_logger().error(f"{cmd_name} failed: {exc}")

    def _apply_speed_rate(self, rate: float) -> None:
        rate = max(0.0, min(1.0, rate))
        try:
            self.client.send_command(
                {"cmdName": "set_speed_rate", "rate_value": rate}
            )
            self.get_logger().info(f"Speed override set to {rate:.2f}.")
        except (LexiumError, OSError) as exc:
            self.get_logger().warn(f"set_speed_rate failed: {exc}")

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "speed_rate":
                self.speed_rate = float(p.value)
                self._apply_speed_rate(self.speed_rate)
        return SetParametersResult(successful=True)

    def destroy_node(self) -> bool:
        try:
            self.client.stop()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LexiumDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._safe_cmd("stop_program")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
