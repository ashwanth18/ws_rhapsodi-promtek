#pragma once

#include <behaviortree_cpp/action_node.h>
#include <future>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace robot_orchestrator {

class ExecuteScoopNode : public BT::StatefulActionNode {
public:
  using Trigger = std_srvs::srv::Trigger;

  ExecuteScoopNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr execute_client_;
  rclcpp::Client<Trigger>::SharedPtr execute_continuous_client_;
  rclcpp::SyncParametersClient::SharedPtr params_client_;
  std::shared_future<Trigger::Response::SharedPtr> result_future_;
  rclcpp::Time start_time_;
  double timeout_s_{90.0};
};

}  // namespace robot_orchestrator
