#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cstdio>
#include <string>

#include "robot_orchestrator/last_failure.hpp"

namespace robot_orchestrator {

// Fails the weightment when the net mass (scale_weight - container_baseline_g)
// is above batch_target + tolerance. ComputeRemaining clamps remaining at 0,
// so without this an overshoot finishes as SUCCESS.
class CheckOvershootNode : public BT::SyncActionNode {
public:
  CheckOvershootNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("batch_target"),
      BT::InputPort<double>("tolerance"),
      BT::OutputPort<double>("overshoot")
    };
  }

  BT::NodeStatus tick() override
  {
    double target = 0.0, tol = 0.0;
    if (!getInput("batch_target", target) || target <= 0.0 ||
        !getInput("tolerance", tol)) {
      setLastFailureReason(config().blackboard, "CheckOvershoot: missing batch_target or tolerance");
      return BT::NodeStatus::FAILURE;
    }
    double scale_weight = 0.0, baseline_g = 0.0;
    (void)config().blackboard->get("scale_weight", scale_weight);
    (void)config().blackboard->get("container_baseline_g", baseline_g);

    const double net = std::max(0.0, scale_weight - baseline_g);
    const double overshoot = std::max(0.0, net - target);
    setOutput("overshoot", overshoot);
    const bool over = overshoot > tol;

    try {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(node->get_logger(), "CheckOvershoot: target=%.3f tol=%.3f net=%.3f overshoot=%.3f => %s",
                  target, tol, net, overshoot, over ? "FAILURE" : "SUCCESS");
    } catch (...) {}

    if (over) {
      char reason[160];
      std::snprintf(reason, sizeof(reason),
                    "Weightment overshoot: dosed %.1f g for a %.1f g target (+%.1f g, tolerance %.1f g)",
                    net, target, overshoot, tol);
      setLastFailureReason(config().blackboard, reason);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace robot_orchestrator
