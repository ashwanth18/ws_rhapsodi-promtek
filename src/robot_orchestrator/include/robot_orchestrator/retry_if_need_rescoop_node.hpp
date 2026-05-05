#pragma once

#include <behaviortree_cpp/decorator_node.h>
#include <rclcpp/rclcpp.hpp>

#include <string>

namespace robot_orchestrator {

class RetryIfNeedRescoopNode : public BT::DecoratorNode {
public:
  RetryIfNeedRescoopNode(const std::string& name, const BT::NodeConfig& config)
  : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>(
        "num_attempts",
        -1,
        "Retry only when the rescoop flag is true. Use -1 for infinite retries."),
      BT::InputPort<std::string>(
        "flag_key",
        "pour_need_rescoop",
        "Blackboard key written by PourToTargetNode to request a rescoop"),
      BT::OutputPort<int>(
        "attempt_index",
        "Zero-based rescoop attempt index for the child sequence")
    };
  }

  void halt() override
  {
    try_reset_flag();
    attempt_count_ = 0;
    BT::DecoratorNode::halt();
  }

private:
  int attempt_count_{0};

  BT::NodeStatus tick() override
  {
    if (!child_node_) {
      throw BT::RuntimeError("RetryIfNeedRescoopNode requires a child");
    }

    if (status() == BT::NodeStatus::IDLE) {
      attempt_count_ = 0;
      try_reset_flag();
    }

    setStatus(BT::NodeStatus::RUNNING);
    setOutput("attempt_index", attempt_count_);
    const auto child_status = child_node_->executeTick();

    if (child_status == BT::NodeStatus::RUNNING || child_status == BT::NodeStatus::SKIPPED) {
      return child_status;
    }

    if (child_status == BT::NodeStatus::SUCCESS) {
      try_reset_flag();
      setOutput("attempt_index", 0);
      attempt_count_ = 0;
      resetChild();
      return BT::NodeStatus::SUCCESS;
    }

    const bool need_rescoop = read_rescoop_flag();
    const int max_attempts = read_num_attempts();
    const bool can_retry = (max_attempts < 0) || (attempt_count_ < max_attempts);

    if (need_rescoop && can_retry) {
      ++attempt_count_;
      log_retry();
      try_reset_flag();
      resetChild();
      return BT::NodeStatus::RUNNING;
    }

    try_reset_flag();
    setOutput("attempt_index", 0);
    attempt_count_ = 0;
    resetChild();
    return BT::NodeStatus::FAILURE;
  }

  int read_num_attempts() const
  {
    int max_attempts = -1;
    (void)getInput("num_attempts", max_attempts);
    return max_attempts;
  }

  std::string read_flag_key() const
  {
    std::string flag_key = "pour_need_rescoop";
    (void)getInput("flag_key", flag_key);
    return flag_key;
  }

  bool read_rescoop_flag() const
  {
    bool need_rescoop = false;
    try {
      (void)config().blackboard->get(read_flag_key(), need_rescoop);
    } catch (...) {
      need_rescoop = false;
    }
    return need_rescoop;
  }

  void try_reset_flag() const
  {
    try {
      config().blackboard->set(read_flag_key(), false);
    } catch (...) {
    }
  }

  void log_retry() const
  {
    try {
      auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
      RCLCPP_WARN(
        node->get_logger(),
        "RetryIfNeedRescoop: repeating child because %s=true (attempt %d)",
        read_flag_key().c_str(),
        attempt_count_ + 1);
    } catch (...) {
    }
  }
};

}  // namespace robot_orchestrator
