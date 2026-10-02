#include "robot_orchestrator/execute_scoop_node.hpp"

#include "robot_orchestrator/last_failure.hpp"

using namespace std::chrono_literals;

namespace robot_orchestrator {

BT::PortsList ExecuteScoopNode::providedPorts()
{
  return {
    BT::InputPort<bool>("continuous", true, "Use /execute_scoop_continuous"),
    BT::InputPort<double>("pattern_offset_y", 0.0, "scoop task-frame Y offset applied by scooping_mtc_node"),
    // X/Z are only written when the tree sets them (scoop_vision); otherwise
    // the MTC node keeps whatever value it already has.
    BT::InputPort<double>("pattern_offset_x", "optional scoop task-frame X offset"),
    BT::InputPort<double>("pattern_offset_z", "optional scoop task-frame Z offset"),
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
    setLastFailureReason(
      config().blackboard, "ExecuteScoop: scooping_mtc parameter service unavailable");
    return BT::NodeStatus::FAILURE;
  }

  std::vector<rclcpp::Parameter> offsets{rclcpp::Parameter("pattern_offset_y", pattern_offset_y)};
  double pattern_offset_x = 0.0;
  double pattern_offset_z = 0.0;
  const bool has_x = static_cast<bool>(getInput<double>("pattern_offset_x"));
  const bool has_z = static_cast<bool>(getInput<double>("pattern_offset_z"));
  if (has_x) {
    pattern_offset_x = getInput<double>("pattern_offset_x").value();
    offsets.emplace_back("pattern_offset_x", pattern_offset_x);
  }
  if (has_z) {
    pattern_offset_z = getInput<double>("pattern_offset_z").value();
    offsets.emplace_back("pattern_offset_z", pattern_offset_z);
  }

  const auto results = params_client_->set_parameters(offsets);
  for (std::size_t i = 0; i < offsets.size(); ++i) {
    if (i < results.size() && results[i].successful) {
      continue;
    }
    const std::string reason = i < results.size() ? results[i].reason : "no response";
    RCLCPP_ERROR(
      node_->get_logger(),
      "ExecuteScoopNode: failed to set %s=%.4f (%s)",
      offsets[i].get_name().c_str(),
      offsets[i].as_double(),
      reason.c_str());
    setLastFailureReason(
      config().blackboard,
      "ExecuteScoop: failed to set " + offsets[i].get_name() + " (" + reason + ")");
    return BT::NodeStatus::FAILURE;
  }

  auto& client = continuous ? execute_continuous_client_ : execute_client_;
  if (!client->wait_for_service(5s)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ExecuteScoopNode: scoop service %s unavailable",
      continuous ? "/execute_scoop_continuous" : "/execute_scoop");
    setLastFailureReason(
      config().blackboard,
      std::string("ExecuteScoop: scoop service unavailable (") +
        (continuous ? "/execute_scoop_continuous" : "/execute_scoop") + ")");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<Trigger::Request>();
  result_future_ = client->async_send_request(request).future.share();
  start_time_ = node_->now();

  const std::string x_text = has_x ? std::to_string(pattern_offset_x) : "unchanged";
  const std::string z_text = has_z ? std::to_string(pattern_offset_z) : "unchanged";
  RCLCPP_INFO(
    node_->get_logger(),
    "ExecuteScoopNode: requested scoop execution (continuous=%s, pattern_offset x=%s y=%.4f z=%s m)",
    continuous ? "true" : "false",
    x_text.c_str(),
    pattern_offset_y,
    z_text.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteScoopNode::onRunning()
{
  if (result_future_.valid() && result_future_.wait_for(0s) == std::future_status::ready) {
    const auto response = result_future_.get();
    if (!response) {
      RCLCPP_ERROR(node_->get_logger(), "ExecuteScoopNode: empty service response");
      setLastFailureReason(config().blackboard, "ExecuteScoop: empty scoop service response");
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "ExecuteScoopNode: scoop response success=%s msg=%s",
      response->success ? "true" : "false",
      response->message.c_str());
    if (!response->success) {
      const std::string detail = response->message.empty()
        ? "scoop execution failed"
        : response->message;
      setLastFailureReason(config().blackboard, "ExecuteScoop: " + detail);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

  if ((node_->now() - start_time_).seconds() > timeout_s_) {
    RCLCPP_WARN(
      node_->get_logger(),
      "ExecuteScoopNode: timed out waiting for scoop execution after %.1f s",
      timeout_s_);
    setLastFailureReason(
      config().blackboard,
      "ExecuteScoop: timed out after " + std::to_string(static_cast<int>(timeout_s_)) + "s");
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
