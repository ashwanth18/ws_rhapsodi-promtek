#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

// Returns SUCCESS if a recent weight measurement exists (age <= max_age_s).
class WeightFreshNode : public BT::SyncActionNode {
public:
  WeightFreshNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("max_age_s", 1.0, "max allowed age in seconds") };
  }

  BT::NodeStatus tick() override
  {
    double max_age = 1.0;
    (void)getInput("max_age_s", max_age);
    double t = 0.0;
    auto bb = config().blackboard;
    (void)bb->get("scale_weight_time", t);
    try {
      auto node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      const double now_s = node->now().seconds();
      const double age = (t > 0.0) ? (now_s - t) : 1e9;
      RCLCPP_INFO(node->get_logger(), "WeightFresh: age=%.3fs (max=%.3fs) => %s", age, max_age, (age <= max_age ? "fresh" : "stale"));
      return (age <= max_age) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    } catch (...) {
      return BT::NodeStatus::FAILURE;
    }
  }
};

} // namespace robot_orchestrator


