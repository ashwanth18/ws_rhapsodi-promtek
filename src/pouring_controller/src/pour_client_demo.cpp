#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/pour_to_target.hpp>

using PourToTarget = robot_common_msgs::action::PourToTarget;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pour_client_demo");

  auto client = rclcpp_action::create_client<PourToTarget>(node, "pour_to_target");
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "No pour_to_target server");
    return 1;
  }

  PourToTarget::Goal goal;
  goal.target_weight = 1.0f;
  goal.tolerance = 0.01f;
  goal.max_time_s = 30.0f;

  auto feedback_cb = [node](rclcpp_action::ClientGoalHandle<PourToTarget>::SharedPtr,
                             const std::shared_ptr<const PourToTarget::Feedback> feedback) {
    RCLCPP_INFO(node->get_logger(), "feedback: weight=%.3f phase=%s",
                feedback->current_weight, feedback->phase.c_str());
  };

  rclcpp_action::Client<PourToTarget>::SendGoalOptions options;
  options.feedback_callback = feedback_cb;
  auto result_future = client->async_send_goal(goal, options);

  auto goal_handle = result_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal rejected");
    return 1;
  }

  auto result = client->async_get_result(goal_handle);
  auto status = result.get();
  if (status.result) {
    RCLCPP_INFO(node->get_logger(), "result: achieved=%d overshoot=%d timeout=%d final=%.3f msg=%s",
                status.result->achieved, status.result->overshoot, status.result->timeout,
                status.result->final_weight, status.result->message.c_str());
  }

  rclcpp::shutdown();
  return 0;
}



