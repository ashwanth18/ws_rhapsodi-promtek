"""Point move_to at the active layout's targets plus twin-only draft targets.

``cell_layout_manager`` sets ``move_to_server.targets_yaml`` to the layout's
file on every apply, and move_to reloads it per goal. This node writes
``~/.cache/isaac_twin/targets_<layout>_draft.yaml`` (layout targets with the
draft entries on top) and keeps move_to on it, so drafts never touch
``config/layouts``.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_common_msgs.msg import CellLayoutActive


class DraftTargets(Node):
    def __init__(self) -> None:
        super().__init__("draft_targets")
        self.declare_parameter("draft_yaml", "")
        self.declare_parameter("target_node", "/move_to_server")
        self._draft = Path(self.get_parameter("draft_yaml").value)
        self._client = AsyncParameterClient(self, str(self.get_parameter("target_node").value))
        self._merged = ""
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(CellLayoutActive, "/cell_layout/active", self._on_layout, qos)
        # move_to respawns with its launch params, and the layout manager's own
        # set can land after the layout message: re-assert periodically.
        self.create_timer(2.0, self._enforce)

    def _on_layout(self, msg: CellLayoutActive) -> None:
        layout = yaml.safe_load(Path(msg.targets_yaml).read_text(encoding="utf-8")) or {}
        draft = yaml.safe_load(self._draft.read_text(encoding="utf-8")) or {}
        targets = dict(layout.get("targets") or {})
        targets.update(draft.get("targets") or {})
        layout["targets"] = targets
        out = Path.home() / ".cache" / "isaac_twin" / f"targets_{msg.layout_id}_draft.yaml"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")
        self._merged = str(out)
        self.get_logger().info(f"Draft targets {sorted((draft.get('targets') or {}))} over {msg.targets_yaml} -> {out}")
        self._enforce()

    def _enforce(self) -> None:
        if not self._merged or not self._client.services_are_ready():
            return
        future = self._client.get_parameters(["targets_yaml"])
        future.add_done_callback(self._check)

    def _check(self, future) -> None:
        values = future.result().values if future.result() else []
        if values and values[0].string_value == self._merged:
            return
        self._client.set_parameters([Parameter("targets_yaml", Parameter.Type.STRING, self._merged)])
        self.get_logger().info(f"move_to targets_yaml -> {self._merged}")


def main() -> None:
    rclpy.init()
    node = DraftTargets()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
