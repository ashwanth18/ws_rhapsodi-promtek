#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "robot_common_msgs/srv/record_target.hpp"

namespace robot_moveit {

class TargetRecorder : public rclcpp::Node {
public:
  explicit TargetRecorder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp::Service<robot_common_msgs::srv::RecordTarget>::SharedPtr srv_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mgi_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string end_effector_link_;
  std::string pose_source_; // 'tf' | 'moveit' | 'auto'

  void deferred_init();
  void handle_request(const std::shared_ptr<robot_common_msgs::srv::RecordTarget::Request> request,
                      std::shared_ptr<robot_common_msgs::srv::RecordTarget::Response> response);

  static void write_pose(YAML::Node & node, const geometry_msgs::msg::PoseStamped & ps);
  static void write_joints(YAML::Node & node,
                           const std::vector<std::string> & names,
                           const std::vector<double> & values);
  static bool save_yaml(const std::string & path, const YAML::Node & root, std::string & err);
};

} // namespace robot_moveit


