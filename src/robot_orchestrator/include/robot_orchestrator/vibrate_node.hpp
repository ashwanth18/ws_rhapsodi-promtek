#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace robot_orchestrator {

class VibrateNode : public BT::StatefulActionNode {
public:
  VibrateNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("value", 0.7, "normalized vibration command 0..1"),
      BT::InputPort<double>("duration_s", 1.0, "duration seconds"),
      BT::InputPort<double>("publish_rate_hz", 10.0, "keepalive publish rate in Hz"),
      BT::InputPort<std::string>(
        "vibration_topic",
        std::string("/vibration/intensity"),
        "vibration topic (Float64)")
    };
  }

  BT::NodeStatus onStart() override
  {
    double value = 0.0;
    double dur = 0.0;
    double publish_rate_hz = 10.0;
    topic_ = getInput<std::string>("vibration_topic").value();
    if (auto v = getInput<double>("value")) { value = *v; }
    if (auto d = getInput<double>("duration_s")) { dur = *d; }
    if (auto rate = getInput<double>("publish_rate_hz")) { publish_rate_hz = *rate; }

    auto bb = config().blackboard;
    node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
    if (!pub_ || pub_->get_topic_name() != topic_) {
      pub_ = node_->create_publisher<std_msgs::msg::Float64>(topic_, 10);
    }
    end_time_ = node_->now() + rclcpp::Duration::from_seconds(dur);
    command_ = std::clamp(value, 0.0, 1.0);
    publish_period_ = rclcpp::Duration::from_seconds(1.0 / std::max(1.0, publish_rate_hz));
    next_publish_time_ = node_->now() + publish_period_;
    RCLCPP_INFO(
      node_->get_logger(),
      "VibrateNode: publish %.3f on %s for %.2fs at %.1f Hz",
      command_,
      topic_.c_str(),
      dur,
      std::max(1.0, publish_rate_hz));
    std_msgs::msg::Float64 m;
    m.data = command_;
    pub_->publish(m);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (node_->now() >= end_time_) {
      std_msgs::msg::Float64 m;
      m.data = 0.0;
      pub_->publish(m);
      return BT::NodeStatus::SUCCESS;
    }
    if (node_->now() >= next_publish_time_) {
      std_msgs::msg::Float64 m;
      m.data = command_;
      pub_->publish(m);
      next_publish_time_ = node_->now() + publish_period_;
    }
    rclcpp::spin_some(node_);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    if (pub_) {
      std_msgs::msg::Float64 m;
      m.data = 0.0;
      pub_->publish(m);
      RCLCPP_INFO(node_->get_logger(), "VibrateNode: halted -> publish 0 on %s", topic_.c_str());
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  std::string topic_;
  rclcpp::Time end_time_{};
  rclcpp::Time next_publish_time_{};
  rclcpp::Duration publish_period_{0, 0};
  double command_{0.0};
};

} // namespace robot_orchestrator


