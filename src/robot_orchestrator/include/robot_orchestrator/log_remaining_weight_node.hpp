#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

class LogRemainingWeightNode : public BT::SyncActionNode {
public:
  LogRemainingWeightNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("remaining_weight"),
      BT::InputPort<double>("tolerance", 0.0, "Current tolerance in grams")
    };
  }

  BT::NodeStatus tick() override
  {
    double remaining = 0.0;
    if (!getInput("remaining_weight", remaining)) {
      return BT::NodeStatus::FAILURE;
    }

    double tolerance = 0.0;
    (void)getInput("tolerance", tolerance);

    try {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(
        node->get_logger(),
        "LogRemainingWeight: remaining_weight=%.3fg tolerance=%.3fg => %s",
        remaining,
        tolerance,
        (remaining <= tolerance) ? "skip pour" : "attempt pour");
    } catch (...) {
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace robot_orchestrator
