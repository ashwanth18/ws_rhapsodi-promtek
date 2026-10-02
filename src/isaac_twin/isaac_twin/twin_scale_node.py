#!/usr/bin/env python3
"""Twin scale: RS3 contents from Isaac (grams) -> ``/weight`` like the real scale.

Replaces ``weight_sim`` for the twin. Adds a first-order settling lag, noise
and display resolution so pour control laws see a scale, not a perfect sum.
"""

from __future__ import annotations

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


class TwinScale(Node):
    def __init__(self) -> None:
        super().__init__("twin_scale")
        p = self.declare_parameter
        p("input_topic", "/isaac_twin/rs3_mass_g")
        p("output_topic", "/weight")
        p("rate_hz", 20.0)
        p("lag_tau_s", 0.4)
        p("noise_std_g", 0.05)
        p("resolution_g", 0.01)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        self._tau = max(1e-3, float(g("lag_tau_s")))
        self._noise = max(0.0, float(g("noise_std_g")))
        self._res = max(0.0, float(g("resolution_g")))
        self._mass = 0.0
        self._shown = 0.0
        self._tare = 0.0
        self._last = None

        self._pub = self.create_publisher(Float64, str(g("output_topic")), 10)
        self.create_subscription(Float64, str(g("input_topic")), self._on_mass, 10)
        self.create_service(Trigger, "~/tare", self._on_tare)
        self.create_timer(1.0 / max(0.5, float(g("rate_hz"))), self._tick)

    def _on_mass(self, msg: Float64) -> None:
        self._mass = float(msg.data)

    def _on_tare(self, _req, resp):
        self._tare = self._shown
        resp.success = True
        resp.message = f"tared at {self._tare:.2f} g"
        return resp

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = 0.0 if self._last is None else (now - self._last).nanoseconds * 1e-9
        self._last = now
        if dt > 0:
            self._shown += (self._mass - self._shown) * (1.0 - math.exp(-dt / self._tau))
        value = self._shown - self._tare + (random.gauss(0.0, self._noise) if self._noise else 0.0)
        if self._res > 0:
            value = round(value / self._res) * self._res
        self._pub.publish(Float64(data=float(value)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TwinScale()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
