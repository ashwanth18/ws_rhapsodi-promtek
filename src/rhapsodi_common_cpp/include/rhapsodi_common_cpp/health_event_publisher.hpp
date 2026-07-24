// C++ counterpart to rhapsodi_common.health.HealthEventPublisher.
//
// Every C++ ROS 2 node that used to only RCLCPP_WARN/RCLCPP_ERROR on a
// fault path should also call one of these, so the fault is queryable
// fleet-wide (see robot_common_msgs/msg/HealthEvent.msg) instead of stuck
// in `docker logs`.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/health_event.hpp>

#include <sstream>
#include <string>

#include "rhapsodi_common_cpp/device_config.hpp"

namespace rhapsodi_common_cpp
{

class HealthEventPublisher
{
public:
  static constexpr uint8_t DEBUG = robot_common_msgs::msg::HealthEvent::DEBUG;
  static constexpr uint8_t INFO = robot_common_msgs::msg::HealthEvent::INFO;
  static constexpr uint8_t WARN = robot_common_msgs::msg::HealthEvent::WARN;
  static constexpr uint8_t ERROR = robot_common_msgs::msg::HealthEvent::ERROR;
  static constexpr uint8_t CRITICAL = robot_common_msgs::msg::HealthEvent::CRITICAL;

  // `node` must outlive this publisher (typical usage: member of the node
  // that owns it, constructed in that node's constructor).
  explicit HealthEventPublisher(
    rclcpp::Node * node,
    std::string component,
    std::string device_id = "")
  : node_(node), component_(std::move(component))
  {
    device_id_ = device_id.empty() ? loadDeviceConfig().device_id : device_id;
    publisher_ = node_->create_publisher<robot_common_msgs::msg::HealthEvent>(
      "/system/health_events", rclcpp::QoS(20));
  }

  void publish(
    uint8_t severity,
    const std::string & code,
    const std::string & message,
    const std::string & context_json = "{}")
  {
    robot_common_msgs::msg::HealthEvent msg;
    msg.stamp = node_->get_clock()->now();
    msg.device_id = device_id_;
    msg.component = component_;
    msg.severity = severity;
    msg.code = code;
    msg.message = message;
    msg.context_json = context_json;
    publisher_->publish(msg);
  }

  void debug(const std::string & code, const std::string & message, const std::string & context_json = "{}")
  {
    publish(DEBUG, code, message, context_json);
  }
  void info(const std::string & code, const std::string & message, const std::string & context_json = "{}")
  {
    publish(INFO, code, message, context_json);
  }
  void warn(const std::string & code, const std::string & message, const std::string & context_json = "{}")
  {
    publish(WARN, code, message, context_json);
  }
  void error(const std::string & code, const std::string & message, const std::string & context_json = "{}")
  {
    publish(ERROR, code, message, context_json);
  }
  void critical(const std::string & code, const std::string & message, const std::string & context_json = "{}")
  {
    publish(CRITICAL, code, message, context_json);
  }

private:
  rclcpp::Node * node_;
  std::string component_;
  std::string device_id_;
  rclcpp::Publisher<robot_common_msgs::msg::HealthEvent>::SharedPtr publisher_;
};

}  // namespace rhapsodi_common_cpp
