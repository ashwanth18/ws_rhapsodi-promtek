#pragma once

#include <behaviortree_cpp/condition_node.h>

namespace robot_orchestrator {

class HasActiveContainerNode : public BT::ConditionNode {
public:
  HasActiveContainerNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::ConditionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

} // namespace robot_orchestrator











