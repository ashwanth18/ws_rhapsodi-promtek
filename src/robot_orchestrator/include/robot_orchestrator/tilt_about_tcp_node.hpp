#pragma once

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>

namespace robot_orchestrator {

// Tilt the end-effector about the current TCP position (tool tip stays fixed
// in Cartesian space) by rotating orientation around the wrist tilt axis
// (default: wrist_link z == joint_5), then sending a /move_to pose goal.
// With restore=true, replays a PoseStamped previously stashed on the
// blackboard under restore_key (used to return to the pre-tilt pose).
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
  bool lookup_pose(
    const std::string& target_frame,
    const std::string& source_frame,
    geometry_msgs::msg::PoseStamped& out,
    std::string& err) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MoveTo>::SharedPtr client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  GoalHandle::SharedPtr goal_handle_;
  std::shared_future<GoalHandle::SharedPtr> send_future_;
  std::shared_future<rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult> result_future_;
};

}  // namespace robot_orchestrator
