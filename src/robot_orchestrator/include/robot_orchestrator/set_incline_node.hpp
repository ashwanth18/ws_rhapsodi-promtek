#pragma once

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

namespace robot_orchestrator {

// Publishes a one-shot incline command (degrees) to /incline_control.
// incline_control_node turns this into a joint_5 trajectory; unlike vibration
// there is no Teensy watchdog so a single publish is sufficient.
class SetInclineNode : public BT::SyncActionNode {
public:
  SetInclineNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("degrees", 0.0, "incline command in degrees"),
      BT::InputPort<std::string>(
        "incline_topic",
        std::string("/incline_control"),
        "incline topic (Float64 degrees)")
    };
  }

  BT::NodeStatus tick() override
  {
    double degrees = 0.0;
    std::string topic = "/incline_control";
    if (auto d = getInput<double>("degrees")) { degrees = *d; }
    if (auto t = getInput<std::string>("incline_topic")) { topic = *t; }

    auto bb = config().blackboard;
    auto node = bb->get<rclcpp::Node::SharedPtr>("ros_node");
    if (!pub_ || pub_->get_topic_name() != topic) {
      pub_ = node->create_publisher<std_msgs::msg::Float64>(topic, 10);
    }

    std_msgs::msg::Float64 msg;
    msg.data = degrees;
    pub_->publish(msg);
    RCLCPP_INFO(
      node->get_logger(),
      "SetIncline: publish %.3f deg on %s",
      degrees,
      topic.c_str());
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

} // namespace robot_orchestrator
