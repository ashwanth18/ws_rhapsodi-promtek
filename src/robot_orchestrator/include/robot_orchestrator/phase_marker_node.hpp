#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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
      pub_ = node_->create_publisher<std_msgs::msg::String>(
        "/lightsout_training/phase", 10);
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
};

} // namespace robot_orchestrator






