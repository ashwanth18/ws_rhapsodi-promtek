#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

// Reads remaining_weight from blackboard and compares to tolerance, without recomputing.
class CheckRemainingNode : public BT::SyncActionNode {
public:
  CheckRemainingNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("tolerance") };
  }

  BT::NodeStatus tick() override
  {
    double tol = 0.0;
    if (!getInput("tolerance", tol)) return BT::NodeStatus::FAILURE;
    double remaining = 0.0;
    config().blackboard->get("remaining_weight", remaining);
    try {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(node->get_logger(), "CheckRemaining: remaining=%.3f tol=%.3f => %s", remaining, tol, (remaining <= tol ? "SUCCESS" : "FAILURE"));
    } catch (...) {}
    return (remaining <= tol) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

} // namespace robot_orchestrator


