#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace robot_orchestrator {

// Increments and publishes the current lights-out episode index.
class EpisodeMarkerNode : public BT::SyncActionNode {
public:
  EpisodeMarkerNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    if (!node_) {
      node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      auto latched_qos = rclcpp::QoS(1).transient_local();
      pub_ = node_->create_publisher<std_msgs::msg::Int32>("/lightsout_training/episode", latched_qos);
    }
    int32_t idx = 0;
    try { bb->get("lightsout_episode_index", idx); } catch (...) { idx = 0; }
    idx += 1;
    bb->set("lightsout_episode_index", idx);
    std_msgs::msg::Int32 msg; msg.data = idx;
    pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "EpisodeMarker: episode=%d", idx);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

} // namespace robot_orchestrator

