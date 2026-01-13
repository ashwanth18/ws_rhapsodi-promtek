#include "robot_orchestrator/scan_qr_node.hpp"
#include <sstream>

using namespace std::chrono_literals;

namespace robot_orchestrator {

BT::PortsList ScanQrNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("expected_lot")
  };
}

ScanQrNode::ScanQrNode(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg)
{
  auto bb = config().blackboard;
  node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
  client_ = rclcpp_action::create_client<ScanQr>(node_, "/scan_qr");
  RCLCPP_INFO(node_->get_logger(), "ScanQrNode: created action client to %s", "/scan_qr");
}

BT::NodeStatus ScanQrNode::onStart()
{
  // Cancel any previous goal before starting a new one (for retries)
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "ScanQrNode: canceling previous goal before retry");
    try { (void)client_->async_cancel_goal(goal_handle_); } catch (...) {}
    goal_handle_.reset();
  }

  RCLCPP_INFO(node_->get_logger(), "ScanQrNode: waiting for action server %s", "/scan_qr");
  if (!client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(), "ScanQrNode: action server %s not available", "/scan_qr");
    return BT::NodeStatus::FAILURE;
  }
  std::string expected;
  if (!getInput("expected_lot", expected)) {
    RCLCPP_ERROR(node_->get_logger(), "ScanQrNode: missing input port expected_lot");
    return BT::NodeStatus::FAILURE;
  }
  ScanQr::Goal goal; goal.expected_lot = expected;
  RCLCPP_INFO(node_->get_logger(), "ScanQrNode: sending goal expected_lot=%s", expected.c_str());
  rclcpp_action::Client<ScanQr>::SendGoalOptions opts;
  opts.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<ScanQr>> gh){
    if (!gh) { RCLCPP_ERROR(node_->get_logger(), "ScanQrNode: goal rejected by server"); }
    else { RCLCPP_INFO(node_->get_logger(), "ScanQrNode: goal accepted by server"); }
  };
  opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<ScanQr>::WrappedResult &res){
    std::string code;
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: code = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::ABORTED: code = "ABORTED"; break;
      case rclcpp_action::ResultCode::CANCELED: code = "CANCELED"; break;
      default: code = "UNKNOWN"; break;
    }
    const char* match = (res.result && res.result->match) ? "true" : "false";
    const char* lot = (res.result) ? res.result->lot.c_str() : "<none>";
    const char* msg = (res.result) ? res.result->message.c_str() : "<none>";
    RCLCPP_INFO(node_->get_logger(), "ScanQrNode: result code=%s match=%s lot=%s msg=%s", code.c_str(), match, lot, msg);
  };
  send_future_ = client_->async_send_goal(goal, opts);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ScanQrNode::onRunning()
{
  if (!goal_handle_) {
    if (send_future_.valid() && send_future_.wait_for(0s) == std::future_status::ready) {
      goal_handle_ = send_future_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "ScanQrNode: goal_handle is null after send_goal");
        return BT::NodeStatus::FAILURE;
      }
      result_future_ = client_->async_get_result(goal_handle_);
    } else {
      rclcpp::spin_some(node_);
      return BT::NodeStatus::RUNNING;
    }
  }
  if (result_future_.valid() && result_future_.wait_for(0s) == std::future_status::ready) {
    auto res = result_future_.get();
    if (res.result) {
      RCLCPP_INFO(node_->get_logger(), "ScanQrNode: result ready match=%s lot=%s msg=%s",
                  res.result->match ? "true" : "false",
                  res.result->lot.c_str(), res.result->message.c_str());
    }
    goal_handle_.reset();
    return (res.result && res.result->match) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  rclcpp::spin_some(node_);
  return BT::NodeStatus::RUNNING;
}

void ScanQrNode::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "ScanQrNode: onHalted canceling active goal");
    try { (void)client_->async_cancel_goal(goal_handle_); } catch (...) {}
    goal_handle_.reset();
  }
}

} // namespace robot_orchestrator




