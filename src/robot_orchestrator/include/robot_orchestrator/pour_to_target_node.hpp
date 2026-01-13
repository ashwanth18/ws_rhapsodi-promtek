#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/pour_to_target.hpp>

namespace robot_orchestrator {

class PourToTargetNode : public BT::StatefulActionNode {
public:
  using PourToTarget = robot_common_msgs::action::PourToTarget;
  using GoalHandle = rclcpp_action::ClientGoalHandle<PourToTarget>;

  PourToTargetNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<PourToTarget>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> send_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<PourToTarget>::WrappedResult> result_future_;
  rclcpp::Time start_time_;
  rclcpp::Time last_wait_log_time_;
  double goal_timeout_s_ { 60.0 };
};

} // namespace robot_orchestrator




