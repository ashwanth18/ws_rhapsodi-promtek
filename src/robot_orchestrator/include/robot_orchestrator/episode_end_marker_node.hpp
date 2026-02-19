#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace robot_orchestrator {

// Publishes the current lights-out episode index as an "episode end" marker.
class EpisodeEndMarkerNode : public BT::SyncActionNode {
public:
  EpisodeEndMarkerNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    if (!node_) {
      node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      pub_ = node_->create_publisher<std_msgs::msg::Int32>("/lightsout_training/episode_end", 10);
    }
    int32_t idx = 0;
    try { bb->get("lightsout_episode_index", idx); } catch (...) { idx = 0; }
    std_msgs::msg::Int32 msg; msg.data = idx;
    pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(), "EpisodeEndMarker: episode=%d", idx);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

} // namespace robot_orchestrator







