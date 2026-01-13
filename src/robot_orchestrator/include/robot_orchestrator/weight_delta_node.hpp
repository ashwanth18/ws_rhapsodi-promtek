#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

// Succeeds if (scale_weight - container_baseline_g) >= min_delta_g
class WeightDeltaNode : public BT::SyncActionNode {
public:
  WeightDeltaNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("min_delta_g", 1.0, "minimum required delta in grams") };
  }

  BT::NodeStatus tick() override
  {
    double min_delta = 1.0;
    (void)getInput("min_delta_g", min_delta);
    double w = 0.0, baseline = 0.0;
    auto bb = config().blackboard;
    (void)bb->get("scale_weight", w);
    (void)bb->get("container_baseline_g", baseline);
    const double delta = w - baseline;
    try {
      auto node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(node->get_logger(), "WeightDelta: baseline=%.3f scale=%.3f delta=%.3f (min=%.3f) => %s",
                  baseline, w, delta, min_delta, (delta >= min_delta ? "SUCCESS" : "FAILURE"));
    } catch (...) {}
    return (delta >= min_delta) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

} // namespace robot_orchestrator


