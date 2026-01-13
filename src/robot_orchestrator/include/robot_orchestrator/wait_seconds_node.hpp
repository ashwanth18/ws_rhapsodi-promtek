#pragma once

#include <behaviortree_cpp/action_node.h>
#include <chrono>

namespace robot_orchestrator {

class WaitSecondsNode : public BT::StatefulActionNode {
public:
  WaitSecondsNode(const std::string& name, const BT::NodeConfiguration& config)
  : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("seconds", 0.3, "time to wait in seconds") };
  }

  BT::NodeStatus onStart() override
  {
    seconds_ = 0.3;
    (void)getInput("seconds", seconds_);
    start_tp_ = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_tp_).count();
    if (elapsed >= seconds_) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}

private:
  double seconds_ {0.3};
  std::chrono::steady_clock::time_point start_tp_ {};
};

} // namespace robot_orchestrator


