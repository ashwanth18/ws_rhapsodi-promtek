#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <deque>
#include <memory>
#include <robot_common_msgs/msg/container_spec.hpp>

namespace robot_orchestrator {

// Logs and publishes the remaining number of containers in the queue "ingredients_q".
class QueueStatusNode : public BT::SyncActionNode {
public:
  QueueStatusNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    auto bb = config().blackboard;
    if (!node_) {
      node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
      pub_ = node_->create_publisher<std_msgs::msg::Int32>("/bt_queue_remaining", 10);
    }
    std::shared_ptr<std::deque<robot_common_msgs::msg::ContainerSpec>> q;
    (void)bb->get("ingredients_q", q);
    const int remaining = (q ? static_cast<int>(q->size()) : 0);
    RCLCPP_INFO(node_->get_logger(), "QueueStatus: %d ingredient(s) remaining", remaining);
    if (pub_) { std_msgs::msg::Int32 m; m.data = remaining; pub_->publish(m); }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

} // namespace robot_orchestrator


