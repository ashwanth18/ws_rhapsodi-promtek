#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <thread>

namespace robot_orchestrator {

// Publishes a phase marker string (e.g. pour_start / pour_end).
class PhaseMarkerNode : public BT::SyncActionNode {
public:
  PhaseMarkerNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::string>("phase")};
  }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    if (!node_) {
      node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
    }

    std::string phase_topic = "/lightsout_training/phase";
    try { (void)bb->get("phase_topic", phase_topic); } catch (...) {}
    if (!pub_ || phase_topic != current_topic_) {
      pub_ = node_->create_publisher<std_msgs::msg::String>(phase_topic, 10);
      current_topic_ = phase_topic;
      // The publisher is created on the first marker of a run. Publishing in
      // that same tick drops the message: the bag subscription is not matched
      // yet, and this topic is not latched. Hold the tree until someone is
      // listening, or for a few seconds, then continue either way.
      const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
      if (pub_->get_subscription_count() == 0) {
        RCLCPP_INFO(
          node_->get_logger(),
          "PhaseMarker waiting for a subscriber on %s",
          current_topic_.c_str());
      }
      while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        if (pub_->get_subscription_count() > 0) {
          break;
        }
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "PhaseMarker subscribers=%zu on %s",
        pub_->get_subscription_count(),
        current_topic_.c_str());
    }

    auto phase = getInput<std::string>("phase");
    if (!phase) {
      RCLCPP_WARN(node_->get_logger(), "PhaseMarker missing 'phase' input");
      return BT::NodeStatus::FAILURE;
    }

    std_msgs::msg::String msg;
    msg.data = phase.value();
    pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "PhaseMarker: %s", msg.data.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string current_topic_;
};

} // namespace robot_orchestrator






