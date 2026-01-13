#include "robot_orchestrator/has_active_container_node.hpp"

namespace robot_orchestrator {

BT::NodeStatus HasActiveContainerNode::tick()
{
  std::string name;
  // Return SUCCESS if a current container_name exists on the blackboard
  return (config().blackboard->get<std::string>("container_name", name) && !name.empty())
    ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace robot_orchestrator











