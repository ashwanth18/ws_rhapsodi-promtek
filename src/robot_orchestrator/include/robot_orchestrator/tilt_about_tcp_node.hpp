#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>

namespace robot_orchestrator {

// Centre a joint_5 residue-purge tilt on the vessel:
// 1) Cartesian lift + pre-shift opposite the wrist-arc swing,
// 2) publish /incline_control so the tip swings onto the original XY.
// Untilt is left to a following SetIncline(0); MoveBackToContainer then
// leaves the pour pose (no restore MoveTo — that plan also failed at cell).
class TiltAboutTcpNode : public BT::StatefulActionNode {
public:
  using MoveTo = robot_common_msgs::action::MoveTo;
  using GoalHandle = rclcpp_action::ClientGoalHandle<MoveTo>;

  TiltAboutTcpNode(const std::string& name, const BT::NodeConfiguration& config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  enum class Phase { Idle, Moving, WaitingIncline };

  bool lookup_pose(
    const std::string& target_frame,
    const std::string& source_frame,
    geometry_msgs::msg::PoseStamped& out,
    std::string& err) const;

  bool send_cartesian_shift(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& lifted,
    const geometry_msgs::msg::PoseStamped& pre);
  void publish_incline(double degrees);
  void cancel_move();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MoveTo>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr incline_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Phase phase_{Phase::Idle};
  double pending_incline_deg_{0.0};
  double incline_settle_s_{0.6};
  rclcpp::Time incline_done_time_{};

  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> send_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult> result_future_;
};

}  // namespace robot_orchestrator
