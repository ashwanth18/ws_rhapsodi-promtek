#pragma once

#include <algorithm>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <vector>

namespace robot_orchestrator {

namespace detail {

inline std::vector<double> parseFractionsCsv(const std::string& csv)
{
  std::vector<double> out;
  std::stringstream ss(csv);
  std::string token;
  while (std::getline(ss, token, ',')) {
    if (token.empty()) {
      continue;
    }
    try {
      out.push_back(std::stod(token));
    } catch (...) {}
  }
  return out;
}

} // namespace detail

// Samples per-episode pour target from fixed mode or fraction list.
class SampleTargetWeightNode : public BT::SyncActionNode {
public:
  SampleTargetWeightNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    double scooped = 0.0;
    (void)bb->get("scooped_mass_g", scooped);
    if (scooped <= 0.0) {
      try {
        auto node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
        RCLCPP_WARN(node->get_logger(), "SampleTargetWeight: no scooped mass");
      } catch (...) {}
      return BT::NodeStatus::FAILURE;
    }

    std::string mode = "fixed";
    (void)bb->get("lightsout_target_mode", mode);

    double target = 0.0;
    double fraction = 0.0;

    if (mode == "fixed") {
      (void)bb->get("lightsout_fixed_target_g", target);
      if (target <= 0.0) {
        (void)bb->get("lightsout_target_weight_g", target);
      }
      if (scooped > 0.0) {
        fraction = target / scooped;
      }
    } else {
      std::string fractions_csv;
      (void)bb->get("lightsout_target_fractions_csv", fractions_csv);
      const auto fractions = detail::parseFractionsCsv(fractions_csv);
      int episode_index = 1;
      (void)bb->get("lightsout_episode_index", episode_index);
      const int idx = std::clamp(episode_index - 1, 0,
                                 static_cast<int>(std::max<std::size_t>(fractions.size(), 1)) - 1);
      fraction = fractions.empty() ? 0.5 : fractions[static_cast<std::size_t>(idx)];
      target = fraction * scooped;
    }

    double target_min = 0.0, target_max = 0.0;
    (void)bb->get("lightsout_target_min_g", target_min);
    (void)bb->get("lightsout_target_max_g", target_max);
    if (target_min > 0.0) {
      target = std::max(target, target_min);
    }
    if (target_max > 0.0) {
      target = std::min(target, target_max);
    }

    bb->set("lightsout_target_weight_g", target);
    bb->set("lightsout_target_fraction", fraction);

    try {
      auto pub = bb->get<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr>(
        "lightsout_target_weight_pub");
      std_msgs::msg::Float64 msg;
      msg.data = target;
      pub->publish(msg);
    } catch (...) {}

    try {
      auto node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(node->get_logger(),
                  "SampleTargetWeight: mode=%s scooped=%.3f fraction=%.3f target=%.3f",
                  mode.c_str(), scooped, fraction, target);
    } catch (...) {}

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace robot_orchestrator
