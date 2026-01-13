#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <yaml-cpp/yaml.h>

namespace robot_moveit {

class MoveToServer : public rclcpp::Node {
public:
  using MoveTo = robot_common_msgs::action::MoveTo;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveTo>;

  explicit MoveToServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<MoveTo>::SharedPtr action_server_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> named_targets_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                          std::shared_ptr<const MoveTo::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  bool load_named_targets();
  static bool yamlPoseToMsg(const YAML::Node& n, geometry_msgs::msg::PoseStamped& out);
  void deferred_init();
};

} // namespace robot_moveit


