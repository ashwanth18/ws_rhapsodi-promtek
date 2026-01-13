#pragma once

#include <behaviortree_cpp/action_node.h>

namespace robot_orchestrator {

class ComputeRemainingNode : public BT::SyncActionNode {
public:
  ComputeRemainingNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace robot_orchestrator











