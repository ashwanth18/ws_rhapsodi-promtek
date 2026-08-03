#pragma once

#include <algorithm>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace robot_orchestrator {

// Computes scooped mass as (container_baseline_g - scale_weight), clamped >= 0.
class MeasureScoopedMassNode : public BT::SyncActionNode {
public:
  MeasureScoopedMassNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    double baseline = 0.0, scale = 0.0;
    (void)bb->get("container_baseline_g", baseline);
    (void)bb->get("scale_weight", scale);
    const double scooped = std::max(0.0, baseline - scale);
    bb->set("scooped_mass_g", scooped);
    bb->set("lightsout_post_scoop_weight_g", scale);

    double min_scooped = 20.0;
    (void)bb->get("lightsout_min_scooped_g", min_scooped);

    try {
      auto pub = bb->get<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>(
        "lightsout_scooped_mass_pub");
      std_msgs::msg::Float32 msg;
      msg.data = static_cast<float>(scooped);
      pub->publish(msg);
    } catch (...) {}

    try {
      auto node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(node->get_logger(),
                  "MeasureScoopedMass: baseline=%.3f scale=%.3f scooped=%.3f (min=%.3f) => %s",
                  baseline, scale, scooped, min_scooped,
                  (scooped >= min_scooped ? "SUCCESS" : "FAILURE"));
    } catch (...) {}

    return (scooped >= min_scooped) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

} // namespace robot_orchestrator
