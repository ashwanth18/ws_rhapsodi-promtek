#pragma once

#include <mutex>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>

namespace robot_moveit {

class MoveToServer : public rclcpp::Node {
public:
  using MoveTo = robot_common_msgs::action::MoveTo;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveTo>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using FollowJointTrajectoryGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit MoveToServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<MoveTo>::SharedPtr action_server_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_preview_pub_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> named_targets_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::mutex trajectory_goal_mutex_;
  FollowJointTrajectoryGoalHandle::SharedPtr active_trajectory_goal_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                          std::shared_ptr<const MoveTo::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  bool load_named_targets();
  static bool yamlPoseToMsg(const YAML::Node& n, geometry_msgs::msg::PoseStamped& out);
  void deferred_init();
  bool execute_joint_trajectory(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
    std::string& message);
  bool transform_pose_to_frame(
    const geometry_msgs::msg::PoseStamped& input,
    const std::string& target_frame,
    geometry_msgs::msg::PoseStamped& output,
    std::string& error_message);
  void publish_display_trajectory(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan);
  void publish_plan_preview_markers(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const std::string& eef_link,
    const std::string& frame_id);
};

} // namespace robot_moveit


