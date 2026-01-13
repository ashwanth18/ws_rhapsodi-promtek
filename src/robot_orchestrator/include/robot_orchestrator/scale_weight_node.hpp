#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mutex>

namespace robot_orchestrator {

// Subscribes once (lazily) to /weight and writes the latest value to blackboard key "scale_weight".
class ScaleWeightNode : public BT::StatefulActionNode {
public:
  ScaleWeightNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::StatefulActionNode(name, cfg) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override
  {
    auto bb = config().blackboard;
    if (!node_) {
      node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
    }
    if (!sub_) {
      std::string topic = "/weight";
      try { bb->get("weight_topic", topic); } catch (...) {}
      // Prefer sensor-style QoS to match many scale publishers
      rclcpp::QoS qos = rclcpp::SensorDataQoS();
      sub_ = node_->create_subscription<std_msgs::msg::Float64>(topic, qos,
        [this](const std_msgs::msg::Float64::SharedPtr msg){
          std::lock_guard<std::mutex> lk(mtx_);
          last_weight_ = msg->data;
          have_ = true;
        });
      RCLCPP_INFO(node_->get_logger(), "ScaleWeightNode: subscribed %s", topic.c_str());
    }
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // Pump callbacks
    if (node_) rclcpp::spin_some(node_);
    double w = 0.0;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (have_) w = last_weight_;
    }
    if (!have_) {
      // Wait until a message arrives
      return BT::NodeStatus::RUNNING;
    }
    config().blackboard->set("scale_weight", w);
    RCLCPP_INFO(node_->get_logger(), "ScaleWeightNode: scale_weight=%.3f", w);
    return BT::NodeStatus::SUCCESS;
  }

  void onHalted() override {}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
  std::mutex mtx_;
  double last_weight_{0.0};
  bool have_{false};
};

} // namespace robot_orchestrator


