#include "robot_orchestrator/execute_scoop_node.hpp"

using namespace std::chrono_literals;

namespace robot_orchestrator {

BT::PortsList ExecuteScoopNode::providedPorts()
{
  return {
    BT::InputPort<bool>("continuous", true, "Use /execute_scoop_continuous"),
    BT::InputPort<double>("pattern_offset_y", 0.0, "scoop task-frame Y offset applied by scooping_mtc_node"),
    BT::InputPort<double>("timeout_s", 120.0, "timeout while waiting for scoop execution"),
  };
}

ExecuteScoopNode::ExecuteScoopNode(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg)
{
  auto bb = config().blackboard;
  node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
  execute_client_ = node_->create_client<Trigger>("/execute_scoop");
  execute_continuous_client_ = node_->create_client<Trigger>("/execute_scoop_continuous");
  params_client_ = std::make_shared<rclcpp::SyncParametersClient>(node_, "/scooping_mtc_node");
}

BT::NodeStatus ExecuteScoopNode::onStart()
{
  timeout_s_ = getInput<double>("timeout_s").value_or(120.0);
  const bool continuous = getInput<bool>("continuous").value_or(true);
  const double pattern_offset_y = getInput<double>("pattern_offset_y").value_or(0.0);

  if (!params_client_->wait_for_service(2s)) {
    RCLCPP_WARN(node_->get_logger(), "ExecuteScoopNode: parameter service for /scooping_mtc_node unavailable");
    return BT::NodeStatus::FAILURE;
  }

  const auto results = params_client_->set_parameters(
    {rclcpp::Parameter("pattern_offset_y", pattern_offset_y)});
  if (results.empty() || !results.front().successful) {
    const std::string reason = results.empty() ? "no response" : results.front().reason;
    RCLCPP_ERROR(
      node_->get_logger(),
      "ExecuteScoopNode: failed to set pattern_offset_y=%.4f (%s)",
      pattern_offset_y,
      reason.c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto& client = continuous ? execute_continuous_client_ : execute_client_;
  if (!client->wait_for_service(5s)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ExecuteScoopNode: scoop service %s unavailable",
      continuous ? "/execute_scoop_continuous" : "/execute_scoop");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<Trigger::Request>();
  result_future_ = client->async_send_request(request).future.share();
  start_time_ = node_->now();

  RCLCPP_INFO(
    node_->get_logger(),
    "ExecuteScoopNode: requested scoop execution (continuous=%s, pattern_offset_y=%.4f m)",
    continuous ? "true" : "false",
    pattern_offset_y);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteScoopNode::onRunning()
{
  if (result_future_.valid() && result_future_.wait_for(0s) == std::future_status::ready) {
    const auto response = result_future_.get();
    if (!response) {
      RCLCPP_ERROR(node_->get_logger(), "ExecuteScoopNode: empty service response");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "ExecuteScoopNode: scoop response success=%s msg=%s",
      response->success ? "true" : "false",
      response->message.c_str());
    return response->success ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  if ((node_->now() - start_time_).seconds() > timeout_s_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ExecuteScoopNode: timed out waiting for scoop execution after %.1f s",
      timeout_s_);
    return BT::NodeStatus::FAILURE;
  }

  rclcpp::spin_some(node_);
  return BT::NodeStatus::RUNNING;
}

void ExecuteScoopNode::onHalted()
{
  result_future_ = std::shared_future<Trigger::Response::SharedPtr>();
}

}  // namespace robot_orchestrator
