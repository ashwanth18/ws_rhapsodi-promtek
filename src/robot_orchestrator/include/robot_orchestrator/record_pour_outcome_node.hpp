#pragma once

#include <algorithm>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace robot_orchestrator {

// Records pour outcome on blackboard; never aborts the session.
class RecordPourOutcomeNode : public BT::SyncActionNode {
public:
  RecordPourOutcomeNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    std::string outcome = "unknown";

    bool achieved = false, overshoot = false, timeout = false;
    const bool has_achieved = bb->get("pour_achieved", achieved);
    const bool has_overshoot = bb->get("pour_overshoot", overshoot);
    const bool has_timeout = bb->get("pour_timeout", timeout);

    if (has_achieved || has_overshoot || has_timeout) {
      if (timeout) {
        outcome = "timeout";
      } else if (overshoot) {
        outcome = "overshoot";
      } else if (achieved) {
        outcome = "achieved";
      }
    }

    bb->set("lightsout_pour_outcome", outcome);

    double scale = 0.0, post_scoop = 0.0, total = 0.0;
    (void)bb->get("scale_weight", scale);
    (void)bb->get("lightsout_post_scoop_weight_g", post_scoop);
    (void)bb->get("lightsout_total_poured_g", total);
    const double poured = std::max(0.0, scale - post_scoop);
    total += poured;
    bb->set("lightsout_episode_poured_g", poured);
    bb->set("lightsout_total_poured_g", total);

    try {
      auto node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(
        node->get_logger(),
        "RecordPourOutcome: outcome=%s poured=%.3f total=%.3f",
        outcome.c_str(), poured, total);
      try {
        auto pub = bb->get<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>(
          "lightsout_pour_outcome_pub");
        if (pub) {
          std_msgs::msg::String msg;
          msg.data = outcome;
          pub->publish(msg);
        }
      } catch (...) {}
      try {
        auto total_pub = bb->get<rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>(
          "lightsout_total_poured_pub");
        if (total_pub) {
          std_msgs::msg::Float32 msg;
          msg.data = static_cast<float>(total);
          total_pub->publish(msg);
        }
      } catch (...) {}
    } catch (...) {}

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace robot_orchestrator
