#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

namespace robot_orchestrator {

class VibrateNode : public BT::StatefulActionNode {
public:
  VibrateNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("value", 180, "raw vibration command 0..255"),
      BT::InputPort<double>("duration_s", 1.0, "duration seconds"),
      BT::InputPort<std::string>("vibration_topic", std::string("/motor_speed"), "vibration topic (Int32)")
    };
  }

  BT::NodeStatus onStart() override
  {
    int value = 0;
    double dur = 0.0;
    topic_ = getInput<std::string>("vibration_topic").value();
    if (auto v = getInput<int>("value")) { value = *v; }
    if (auto d = getInput<double>("duration_s")) { dur = *d; }

    auto bb = config().blackboard;
    node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
    if (!pub_ || pub_->get_topic_name() != topic_) {
      pub_ = node_->create_publisher<std_msgs::msg::Int32>(topic_, 10);
    }
    end_time_ = node_->now() + rclcpp::Duration::from_seconds(dur);
    const int cmd = std::max(0, std::min(255, value));
    RCLCPP_INFO(node_->get_logger(), "VibrateNode: publish %d on %s for %.2fs", cmd, topic_.c_str(), dur);
    std_msgs::msg::Int32 m; m.data = cmd;
    pub_->publish(m);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (node_->now() >= end_time_) {
      std_msgs::msg::Int32 m; m.data = 0; pub_->publish(m);
      return BT::NodeStatus::SUCCESS;
    }
    rclcpp::spin_some(node_);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (pub_) { std_msgs::msg::Int32 m; m.data = 0; pub_->publish(m); RCLCPP_INFO(node_->get_logger(), "VibrateNode: halted -> publish 0 on %s", topic_.c_str()); }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  std::string topic_;
  rclcpp::Time end_time_{};
};

} // namespace robot_orchestrator


