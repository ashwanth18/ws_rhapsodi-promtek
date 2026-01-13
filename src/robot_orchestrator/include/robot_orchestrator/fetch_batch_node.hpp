#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

// Fetches dynamic batch context (container name, expected lot, targets) and
// writes them to the blackboard via output ports.
// For now, values are read from ROS parameters; you can later replace this
// with a DB/MES query or a ROS2 service call.
class FetchBatchNode : public BT::SyncActionNode {
public:
  FetchBatchNode(const std::string& name, const BT::NodeConfiguration& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace robot_orchestrator











