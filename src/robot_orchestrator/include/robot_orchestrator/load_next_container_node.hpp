#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/container_spec.hpp>

namespace robot_orchestrator {

class LoadNextContainerNode : public BT::SyncActionNode {
public:
  LoadNextContainerNode(const std::string& name, const BT::NodeConfiguration& cfg);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace robot_orchestrator











