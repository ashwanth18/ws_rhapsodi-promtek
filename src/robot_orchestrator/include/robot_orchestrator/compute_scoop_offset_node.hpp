#pragma once

#include <algorithm>

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

class ComputeScoopOffsetNode : public BT::SyncActionNode {
public:
  ComputeScoopOffsetNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("attempt_index", 0, "Zero-based scoop attempt index"),
      BT::InputPort<double>("base_offset_y", 0.0, "Initial scoop Y offset in meters"),
      BT::InputPort<double>("step_y", -0.06, "Y offset added per rescoop in meters"),
      BT::InputPort<double>("min_offset_y", -0.18, "Lower clamp for Y offset in meters"),
      BT::InputPort<double>("max_offset_y", 0.18, "Upper clamp for Y offset in meters"),
      BT::OutputPort<double>("pattern_offset_y", "Computed scoop pattern Y offset")
    };
  }

  BT::NodeStatus tick() override
  {
    const int attempt_index = getInput<int>("attempt_index").value_or(0);
    const double base_offset_y = getInput<double>("base_offset_y").value_or(0.0);
    const double step_y = getInput<double>("step_y").value_or(-0.06);
    const double min_offset_y = getInput<double>("min_offset_y").value_or(-0.18);
    const double max_offset_y = getInput<double>("max_offset_y").value_or(0.18);

    const double unclamped = base_offset_y + (std::max(0, attempt_index) * step_y);
    const double pattern_offset_y =
      std::clamp(unclamped, std::min(min_offset_y, max_offset_y), std::max(min_offset_y, max_offset_y));
    setOutput("pattern_offset_y", pattern_offset_y);

    try {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(
        node->get_logger(),
        "ComputeScoopOffset: attempt=%d base_y=%.4f step_y=%.4f -> pattern_offset_y=%.4f",
        attempt_index,
        base_offset_y,
        step_y,
        pattern_offset_y);
    } catch (...) {
    }

    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace robot_orchestrator
