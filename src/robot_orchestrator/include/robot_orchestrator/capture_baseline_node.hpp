#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

// Captures the current scale_weight as the per-ingredient baseline.
class CaptureBaselineNode : public BT::SyncActionNode {
public:
  CaptureBaselineNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    double w = 0.0;
    auto bb = config().blackboard;
    (void)bb->get("scale_weight", w);
    bb->set("container_baseline_g", w);
    try {
      auto node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(node->get_logger(), "CaptureBaseline: baseline_g=%.3f", w);
    } catch (...) {}
    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace robot_orchestrator


