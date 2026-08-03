#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

namespace robot_orchestrator {

// Evaluates lights-out stop conditions at episode end. Always SUCCESS.
class EvaluateStopConditionNode : public BT::SyncActionNode {
public:
  EvaluateStopConditionNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    rclcpp::Node::SharedPtr node;
    try {
      node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
    } catch (...) {
      return BT::NodeStatus::SUCCESS;
    }

    std::string stop_on = "episodes";
    double stop_value = 0.0;
    int episode_index = 0;
    int episodes = 0;
    double total_poured = 0.0;
    double session_start = 0.0;
    (void)bb->get("lightsout_stop_on", stop_on);
    (void)bb->get("lightsout_stop_value", stop_value);
    (void)bb->get("lightsout_episode_index", episode_index);
    (void)bb->get("lightsout_episodes", episodes);
    (void)bb->get("lightsout_total_poured_g", total_poured);
    (void)bb->get("lightsout_session_start_s", session_start);

    if ((stop_on == "total_weight_g" || stop_on == "duration_min") && stop_value <= 0.0) {
      RCLCPP_WARN(
        node->get_logger(),
        "EvaluateStopCondition: stop_on=%s with non-positive stop_value; using episode cap",
        stop_on.c_str());
    }

    bool stop = false;
    std::string reason;
    const double now_s = node->now().seconds();
    const double elapsed_s = (session_start > 0.0) ? (now_s - session_start) : 0.0;

    if (stop_on == "total_weight_g" && stop_value > 0.0) {
      stop = total_poured >= stop_value;
      if (stop) {
        reason = "total_weight_g";
      }
    } else if (stop_on == "duration_min" && stop_value > 0.0) {
      stop = elapsed_s >= (stop_value * 60.0);
      if (stop) {
        reason = "duration_min";
      }
    }

    // Episode cap always applies, whatever stop_on says.
    if (!stop && episodes > 0 && episode_index >= episodes) {
      stop = true;
      reason = "episodes";
    }

    if (stop) {
      bb->set("lightsout_stop_requested", true);
      bb->set("lightsout_stop_reason", reason);
      try {
        auto pub = bb->get<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>(
          "lightsout_stop_reason_pub");
        if (pub) {
          std_msgs::msg::String msg;
          msg.data = reason;
          pub->publish(msg);
        }
      } catch (...) {}
      RCLCPP_INFO(
        node->get_logger(),
        "EvaluateStopCondition: STOP reason=%s stop_on=%s stop_value=%.3f "
        "total_poured=%.3f elapsed_s=%.1f episode=%d/%d",
        reason.c_str(), stop_on.c_str(), stop_value, total_poured, elapsed_s,
        episode_index, episodes);
    }

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace robot_orchestrator
