#include "robot_orchestrator/load_next_container_node.hpp"
#include <sstream>

namespace robot_orchestrator {

BT::PortsList LoadNextContainerNode::providedPorts()
{
  return {};
}

LoadNextContainerNode::LoadNextContainerNode(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::SyncActionNode(name, cfg)
{
  auto bb = cfg.blackboard;
  node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
}

BT::NodeStatus LoadNextContainerNode::tick()
{
  // containers: std::vector<robot_common_msgs::msg::ContainerSpec>
  // container_index: size_t
  std::vector<robot_common_msgs::msg::ContainerSpec> containers;
  if (!config().blackboard->get("containers", containers)) {
    if (node_) RCLCPP_INFO(node_->get_logger(), "LoadNextContainer: no containers on blackboard");
    return BT::NodeStatus::FAILURE;
  }
  std::size_t index = 0;
  config().blackboard->get("container_index", index);
  if (index >= containers.size()) {
    if (node_) RCLCPP_INFO(node_->get_logger(), "LoadNextContainer: index %zu >= size %zu -> done", index, containers.size());
    return BT::NodeStatus::FAILURE;
  }
  const auto & c = containers[index];
  config().blackboard->set("container_name", c.name);
  config().blackboard->set("expected_lot", c.expected_lot);
  if (c.has_pose) {
    config().blackboard->set("target_x", static_cast<double>(c.pose.pose.position.x));
    config().blackboard->set("target_y", static_cast<double>(c.pose.pose.position.y));
    config().blackboard->set("target_z", static_cast<double>(c.pose.pose.position.z));
  }
  // Per-container target and tolerance are required; always set them
  config().blackboard->set("remaining_weight", static_cast<double>(c.target_weight));
  config().blackboard->set("batch_weight_tolerance", static_cast<double>(c.weight_tolerance));
  config().blackboard->set("container_index", index + 1);
  if (node_) {
    std::ostringstream ss;
    ss << "LoadNextContainer: loaded name=" << c.name
       << " expected_lot=" << c.expected_lot
       << " target_g=" << c.target_weight
       << " tol_g=" << c.weight_tolerance
       << " has_pose=" << (c.has_pose ? "true" : "false");
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_orchestrator


