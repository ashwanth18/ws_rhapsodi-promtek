#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/cell_layout_active.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_orchestrator {

// Blocks motion trees until the latched layout activation matches this run.
class VerifyCellLayoutNode : public BT::SyncActionNode {
public:
  VerifyCellLayoutNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg)
  {
    auto bb = config().blackboard;
    node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
    sub_ = node_->create_subscription<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable(),
      [bb](const robot_common_msgs::msg::CellLayoutActive::SharedPtr msg) {
        bb->set("active_layout_id", msg->layout_id);
        bb->set("active_layout_hash", msg->layout_hash);
      });
    pose_fault_sub_ = node_->create_subscription<std_msgs::msg::String>(
      "/cell_layout/pose_fault", rclcpp::QoS(1).transient_local().reliable(),
      [bb](const std_msgs::msg::String::SharedPtr msg) {
        if (!msg->data.empty()) {
          bb->set("poses_provenance_ok", false);
        }
      });
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    rclcpp::spin_some(node_);
    auto bb = config().blackboard;
    std::string expected;
    std::string active;
    bool poses_provenance_ok = true;
    (void)bb->get("expected_layout_id", expected);
    (void)bb->get("active_layout_id", active);
    (void)bb->get("poses_provenance_ok", poses_provenance_ok);
    if (expected.empty() || active != expected || !poses_provenance_ok) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "VerifyCellLayout failed: expected='%s' active='%s' poses_provenance_ok=%s",
        expected.c_str(), active.c_str(), poses_provenance_ok ? "true" : "false");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<robot_common_msgs::msg::CellLayoutActive>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr pose_fault_sub_;
};

}  // namespace robot_orchestrator
