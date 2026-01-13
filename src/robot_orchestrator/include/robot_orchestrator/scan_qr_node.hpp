#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/scan_qr.hpp>

namespace robot_orchestrator {

class ScanQrNode : public BT::StatefulActionNode {
public:
  using ScanQr = robot_common_msgs::action::ScanQr;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ScanQr>;

  ScanQrNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ScanQr>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> send_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<ScanQr>::WrappedResult> result_future_;
};

} // namespace robot_orchestrator




