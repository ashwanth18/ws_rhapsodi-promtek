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
// 1) pre-shift the TCP opposite the expected wrist-arc swing (same orientation,
//    easy MoveIt plan),
// 2) publish /incline_control so the tip swings back onto the original point,
// 3) on restore: incline 0 then MoveTo the stashed pour pose.
//
// Pure tip-fixed IK (orientation change at fixed XYZ) fails at the pour pose
// with STOMP; this two-step approach reuses the reliable joint incline path.
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
  enum class Phase {
    Idle,
    Moving,
    WaitingIncline,
  };

  bool lookup_pose(
    const std::string& target_frame,
    const std::string& source_frame,
    geometry_msgs::msg::PoseStamped& out,
    std::string& err) const;

  bool send_move_to(const geometry_msgs::msg::PoseStamped& pose);
  void publish_incline(double degrees);
  void cancel_move();

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MoveTo>::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr incline_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Phase phase_{Phase::Idle};
  bool restore_{false};
  double pending_incline_deg_{0.0};
  double incline_settle_s_{0.6};
  rclcpp::Time incline_done_time_{};

  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> send_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult> result_future_;
};

}  // namespace robot_orchestrator
