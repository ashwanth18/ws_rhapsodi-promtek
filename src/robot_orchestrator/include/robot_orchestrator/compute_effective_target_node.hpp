#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

// Computes the incremental pour target given cumulative target-to-date and current absolute scale.
// Inputs (blackboard):
//  - cumulative_target_g (double)  // desired absolute total after this container
//  - scale_weight (double)         // current absolute scale reading
// Outputs (blackboard):
//  - pour_target_g (double)        // incremental amount to pour now
class ComputeEffectiveTargetNode : public BT::SyncActionNode {
public:
  ComputeEffectiveTargetNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    double cumulative_target = 0.0;
    double scale_weight = 0.0;
    (void)config().blackboard->get("cumulative_target_g", cumulative_target);
    (void)config().blackboard->get("scale_weight", scale_weight);

    const double eff = std::max(0.0, cumulative_target - scale_weight);
    config().blackboard->set("pour_target_g", eff);

    try {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(node->get_logger(), "ComputeEffectiveTarget: cumulative=%.3f scale=%.3f -> pour_target=%.3f",
                  cumulative_target, scale_weight, eff);
    } catch (...) {}

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace robot_orchestrator


