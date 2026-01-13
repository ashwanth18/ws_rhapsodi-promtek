#include "robot_orchestrator/fetch_batch_node.hpp"
#include <behaviortree_cpp/decorators/loop_node.h>
#include <robot_common_msgs/msg/container_spec.hpp>

namespace robot_orchestrator {

BT::PortsList FetchBatchNode::providedPorts()
{
  return {};
}

FetchBatchNode::FetchBatchNode(const std::string& name, const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config)
{
  auto bb = config.blackboard;
  node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
}

BT::NodeStatus FetchBatchNode::tick()
{
  // Expect that /bt_start_batch already set blackboard key "containers"
  // Build a shared deque for LoopNode consumption
  std::vector<robot_common_msgs::msg::ContainerSpec> containers;
  if (!config().blackboard->get("containers", containers)) {
    return BT::NodeStatus::SUCCESS; // nothing to do
  }

  using SharedQueue = BT::SharedQueue<robot_common_msgs::msg::ContainerSpec>;
  auto q = std::make_shared<std::deque<robot_common_msgs::msg::ContainerSpec>>();
  for (const auto & c : containers) {
    q->push_back(c);
  }
  config().blackboard->set("ingredients_q", q);
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "FetchBatch: initialized queue with %zu container(s)", q->size());
  }
  return BT::NodeStatus::SUCCESS;
}

} // namespace robot_orchestrator








