#include "robot_orchestrator/pour_to_target_node.hpp"

using namespace std::chrono_literals;

namespace robot_orchestrator {

BT::PortsList PourToTargetNode::providedPorts()
{
  return {
    BT::InputPort<double>("target_weight"),
    BT::InputPort<double>("tolerance", 0.01, "target tolerance"),
    BT::InputPort<double>("max_time_s", 30.0, "timeout")
  };
}

PourToTargetNode::PourToTargetNode(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg)
{
  auto bb = config().blackboard;
  node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
  client_ = rclcpp_action::create_client<PourToTarget>(node_, "pour_to_target");
  RCLCPP_INFO(node_->get_logger(), "PourToTargetNode: created action client to %s", "pour_to_target");
}

BT::NodeStatus PourToTargetNode::onStart()
{
  double target=0, tol=0.01, max_time=30.0;
  if (!getInput("target_weight", target)) return BT::NodeStatus::FAILURE;
  getInput("tolerance", tol);
  getInput("max_time_s", max_time);

  RCLCPP_INFO(node_->get_logger(), "PourToTargetNode: waiting for action server /pour_to_target");
  if (!client_->wait_for_action_server(2s)) {
    RCLCPP_WARN(node_->get_logger(), "PourToTargetNode: action server /pour_to_target not available");
    return BT::NodeStatus::FAILURE;
  }

  // Reset client state from any previous goal
  goal_handle_.reset();
  send_future_ = std::shared_future<GoalHandle::SharedPtr>();
  result_future_ = std::shared_future<rclcpp_action::ClientGoalHandle<PourToTarget>::WrappedResult>();

  PourToTarget::Goal goal; goal.target_weight = static_cast<float>(target); goal.tolerance = static_cast<float>(tol); goal.max_time_s = static_cast<float>(max_time);
  RCLCPP_INFO(node_->get_logger(), "PourToTargetNode: sending goal target=%.3f tol=%.3f max=%.1f", target, tol, max_time);
  auto feedback_cb = [this](GoalHandle::SharedPtr, const std::shared_ptr<const PourToTarget::Feedback> fb){
    if (fb) {
      // RCLCPP_INFO(node_->get_logger(), "PourToTargetNode: fb phase=%s weight=%.3f band=%.3f err_to_band=%.3f hold_left=%.2f",
                  // fb->phase.c_str(), fb->current_weight, fb->band_threshold, fb->error_to_next_band, fb->hold_time_remaining);
    }
  };
  rclcpp_action::Client<PourToTarget>::SendGoalOptions opts;
  opts.feedback_callback = feedback_cb;
  send_future_ = client_->async_send_goal(goal, opts);
  start_time_ = node_->now();
  last_wait_log_time_ = start_time_;
  goal_timeout_s_ = max_time + 5.0; // client-side watchdog a bit longer than server timeout
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PourToTargetNode::onRunning()
{
  if (!goal_handle_) {
    if (send_future_.valid() && send_future_.wait_for(0s) == std::future_status::ready) {
      goal_handle_ = send_future_.get();
      if (!goal_handle_) {
        RCLCPP_WARN(node_->get_logger(), "PourToTargetNode: goal rejected by server");
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "PourToTargetNode: goal accepted by server");
      result_future_ = client_->async_get_result(goal_handle_);
    } else {
      auto now = node_->now();
      if ((now - last_wait_log_time_).seconds() > 2.0) {
        RCLCPP_INFO(node_->get_logger(), "PourToTargetNode: waiting for goal response...");
        last_wait_log_time_ = now;
      }
      rclcpp::spin_some(node_);
      return BT::NodeStatus::RUNNING;
    }
  }

  if (result_future_.valid() && result_future_.wait_for(0s) == std::future_status::ready) {
    auto res = result_future_.get();
    if (res.result) {
      // Treat overshoot as SUCCESS to terminate the flow, even if achieved=false
      std::string container_name;
      try { container_name = config().blackboard->get<std::string>("container_name"); } catch (...) {}
      RCLCPP_INFO(node_->get_logger(),
                  "PourToTargetNode: result achieved=%s overshoot=%s timeout=%s final_abs=%.3fg final_net=%.3fg msg=%s (container=%s)",
                  res.result->achieved?"true":"false",
                  res.result->overshoot?"true":"false",
                  res.result->timeout?"true":"false",
                  res.result->final_weight,
                  res.result->final_net_g,
                  res.result->message.c_str(),
                  container_name.c_str());
      // Do not override scale_weight here; rely on subscriber + freshness gates
      if (res.result->achieved || res.result->overshoot) {
        return BT::NodeStatus::SUCCESS;
      }
    }
    return BT::NodeStatus::FAILURE;
  }
  // Client-side watchdog to avoid indefinite stall if server never responds
  if ((node_->now() - start_time_).seconds() > goal_timeout_s_) {
    RCLCPP_WARN(node_->get_logger(), "PourToTargetNode: client watchdog timeout reached, canceling goal");
    if (goal_handle_) {
      (void)client_->async_cancel_goal(goal_handle_);
    }
    return BT::NodeStatus::FAILURE;
  }
  rclcpp::spin_some(node_);
  return BT::NodeStatus::RUNNING;
}

void PourToTargetNode::onHalted()
{
  if (goal_handle_) {
    (void)client_->async_cancel_goal(goal_handle_);
  }
}

} // namespace robot_orchestrator




