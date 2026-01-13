#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/container_spec.hpp>

namespace robot_orchestrator {

// Projects a ContainerSpec task into blackboard keys used by the flow.
// Inputs:
//  - task: robot_common_msgs::msg::ContainerSpec
// Side effects (blackboard keys set):
//  - container_name (string)
//  - expected_lot (string)
//  - remaining_weight (double)
//  - batch_weight_tolerance (double)
//  - target_x/target_y/target_z (double) if has_pose
class ProjectContainerNode : public BT::SyncActionNode {
public:
  ProjectContainerNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<robot_common_msgs::msg::ContainerSpec>("task") };
  }

  BT::NodeStatus tick() override
  {
    robot_common_msgs::msg::ContainerSpec task;
    if (!getInput("task", task)) {
      return BT::NodeStatus::FAILURE;
    }

    config().blackboard->set("container_name", task.name);
    // Derived target for QR pose based on container name
    config().blackboard->set("qr_target_name", task.name + std::string("_qr"));
    // Derived waypoint list (semicolon-separated) for QR approach and scan poses
    config().blackboard->set("qr_waypoint_names", task.name + std::string("_qr_approach;") + task.name + std::string("_qr"));
    // Derived target for scooping action based on container name
    config().blackboard->set("scooping_target_name", task.name + std::string("_scooping"));
    config().blackboard->set("expected_lot", task.expected_lot);
    // Per-container target
    const double container_target = static_cast<double>(task.target_weight);
    config().blackboard->set("container_target_g", container_target);
    config().blackboard->set("batch_weight_tolerance", static_cast<double>(task.weight_tolerance));

    if (task.has_pose) {
      config().blackboard->set("target_x", static_cast<double>(task.pose.pose.position.x));
      config().blackboard->set("target_y", static_cast<double>(task.pose.pose.position.y));
      config().blackboard->set("target_z", static_cast<double>(task.pose.pose.position.z));
      config().blackboard->set("qx", static_cast<double>(task.pose.pose.orientation.x));
      config().blackboard->set("qy", static_cast<double>(task.pose.pose.orientation.y));
      config().blackboard->set("qz", static_cast<double>(task.pose.pose.orientation.z));
      config().blackboard->set("qw", static_cast<double>(task.pose.pose.orientation.w));
      config().blackboard->set("frame_id", task.pose.header.frame_id);
    }
    else {
      // Clear pose-related keys to avoid stale values being reused elsewhere
      config().blackboard->set("target_x", 0.0);
      config().blackboard->set("target_y", 0.0);
      config().blackboard->set("target_z", 0.0);
      config().blackboard->set("qx", 0.0);
      config().blackboard->set("qy", 0.0);
      config().blackboard->set("qz", 0.0);
      config().blackboard->set("qw", 1.0);
      config().blackboard->set("frame_id", std::string(""));
    }

    try {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_INFO(node->get_logger(), "ProjectContainer: name=%s expected_lot=%s target_g=%.1f tol_g=%.1f has_pose=%s",
                  task.name.c_str(), task.expected_lot.c_str(), task.target_weight, task.weight_tolerance,
                  task.has_pose?"true":"false");
    } catch (...) {}

    return BT::NodeStatus::SUCCESS;
  }
};

} // namespace robot_orchestrator





