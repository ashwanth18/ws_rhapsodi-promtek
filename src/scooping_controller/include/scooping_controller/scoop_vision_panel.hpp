#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <robot_common_msgs/srv/plan_scoop.hpp>
#include <std_srvs/srv/trigger.hpp>
#endif

#include <rviz_common/panel.hpp>

#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>

#include <functional>
#include <string>
#include <vector>

namespace scooping_controller
{
// Buttons for the D455 scoop_vision workflow: container calibration from the
// camera, powder capture, next-scoop planning, MoveIt preview and execution.
class ScoopVisionPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ScoopVisionPanel(QWidget* parent = nullptr);
  void onInitialize() override;

private Q_SLOTS:
  void onCheckContainer();
  void onFitContainer();
  void onApplyFit();
  void onMoveToWeighing();
  void onCapture();
  void onPlan();
  void onPreview();
  void onRun();
  void onUseAuthored();
  void onDepthCorrectionChanged();
  void onRosTimer();

private:
  using Trigger = std_srvs::srv::Trigger;
  using PlanScoop = robot_common_msgs::srv::PlanScoop;
  using MoveTo = robot_common_msgs::action::MoveTo;

  void callTrigger(
    const rclcpp::Client<Trigger>::SharedPtr& client,
    const QString& busy_text,
    std::function<void(bool, const std::string&)> done = nullptr);
  void setOffsets(double x, double y, double z, std::function<void(bool, const std::string&)> done);
  void moveTo(const std::string& target, std::function<void(bool, const std::string&)> done);
  void setBusy(bool busy);
  void setStatus(const QString& text, const QString& color = "#e5e7eb");
  void refreshButtons();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr check_client_;
  rclcpp::Client<Trigger>::SharedPtr fit_client_;
  rclcpp::Client<Trigger>::SharedPtr apply_fit_client_;
  rclcpp::Client<Trigger>::SharedPtr capture_client_;
  rclcpp::Client<PlanScoop>::SharedPtr plan_client_;
  rclcpp::Client<Trigger>::SharedPtr mtc_plan_client_;
  rclcpp::Client<Trigger>::SharedPtr mtc_execute_client_;
  rclcpp_action::Client<MoveTo>::SharedPtr move_to_client_;
  rclcpp::AsyncParametersClient::SharedPtr mtc_params_;
  rclcpp::AsyncParametersClient::SharedPtr vision_params_;

  QTimer* ros_timer_;
  QLabel* status_label_;
  QLabel* plan_label_;
  QPushButton* check_button_;
  QPushButton* fit_button_;
  QPushButton* apply_fit_button_;
  QPushButton* weighing_button_;
  QPushButton* capture_button_;
  QPushButton* plan_button_;
  QPushButton* preview_button_;
  QPushButton* run_button_;
  QPushButton* authored_button_;
  QSpinBox* depth_correction_;
  bool depth_correction_loaded_{false};
  int depth_correction_applied_mm_{-1};

  bool busy_{false};
  bool fit_pending_{false};
  bool have_plan_{false};
  double plan_x_{0.0};
  double plan_y_{0.0};
  double plan_z_{0.0};
};

}  // namespace scooping_controller
