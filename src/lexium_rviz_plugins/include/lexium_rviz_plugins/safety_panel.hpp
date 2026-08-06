#ifndef LEXIUM_RVIZ_PLUGINS__SAFETY_PANEL_HPP_
#define LEXIUM_RVIZ_PLUGINS__SAFETY_PANEL_HPP_

#include <memory>
#include <string>

#include <QPushButton>
#include <QLabel>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <lexium_msgs/msg/lexium_status.hpp>

namespace lexium_rviz_plugins
{

class SafetyPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit SafetyPanel(QWidget * parent = nullptr);

  void onInitialize() override;

private Q_SLOTS:
  void onBringUp();
  void onShutDown();
  void onClearError();
  void onStop();

private:
  void statusCallback(const lexium_msgs::msg::LexiumStatus::SharedPtr msg);
  void updateUi(const lexium_msgs::msg::LexiumStatus & status);
  void callTrigger(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & label);
  std::string controlSourceLabel(int32_t source) const;
  bool hasFaults(const lexium_msgs::msg::LexiumStatus & status) const;
  bool blocksBringUp(const lexium_msgs::msg::LexiumStatus & status) const;
  std::string statusHint(const lexium_msgs::msg::LexiumStatus & status) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<lexium_msgs::msg::LexiumStatus>::SharedPtr status_sub_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr bring_up_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shut_down_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_error_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;

  QLabel * status_label_{nullptr};
  QLabel * hint_label_{nullptr};
  QLabel * message_label_{nullptr};
  QPushButton * bring_up_btn_{nullptr};
  QPushButton * shut_down_btn_{nullptr};
  QPushButton * clear_error_btn_{nullptr};
  QPushButton * stop_btn_{nullptr};

  lexium_msgs::msg::LexiumStatus last_status_;
  bool busy_{false};
  std::string driver_ns_{"/lexium_driver"};
};

}  // namespace lexium_rviz_plugins

#endif  // LEXIUM_RVIZ_PLUGINS__SAFETY_PANEL_HPP_
