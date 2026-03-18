#include "scooping_controller/scooping_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QVBoxLayout>

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace scooping_controller
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}
}  // namespace

ScoopingPanel::ScoopingPanel(QWidget* parent)
: rviz_common::Panel(parent)
, service_state_label_(new QLabel(this))
, status_label_(new QLabel(this))
, target_name_edit_(new QLineEdit(this))
, target_selector_combo_(new QComboBox(this))
, scoop_marker_combo_(new QComboBox(this))
, scoop_x_edit_(new QLineEdit(this))
, scoop_y_edit_(new QLineEdit(this))
, scoop_z_edit_(new QLineEdit(this))
, scoop_roll_edit_(new QLineEdit(this))
, scoop_pitch_edit_(new QLineEdit(this))
, scoop_yaw_edit_(new QLineEdit(this))
, x_edit_(new QLineEdit(this))
, y_edit_(new QLineEdit(this))
, z_edit_(new QLineEdit(this))
, roll_edit_(new QLineEdit(this))
, pitch_edit_(new QLineEdit(this))
, yaw_edit_(new QLineEdit(this))
, velocity_scale_edit_(new QLineEdit(this))
, acceleration_scale_edit_(new QLineEdit(this))
, pattern_offset_y_edit_(new QLineEdit(this))
, offset_step_edit_(new QLineEdit(this))
, save_button_(new QPushButton("Save YAML", this))
, load_button_(new QPushButton("Load YAML", this))
, plan_button_(new QPushButton("Plan Only", this))
, execute_button_(new QPushButton("Execute Scoop", this))
, execute_continuous_button_(new QPushButton("Execute Continuous", this))
, execute_waypoint_motion_button_(new QPushButton("Execute Waypoint Style", this))
, move_goal_button_(new QPushButton("Move To Goal Marker", this))
, apply_typed_pose_button_(new QPushButton("Apply Typed Pose", this))
, apply_scoop_pose_button_(new QPushButton("Apply To Selected Scoop Marker", this))
, focus_selected_scoop_button_(new QPushButton("Focus Selected Marker", this))
, show_all_scoops_button_(new QPushButton("Show All Scoop Markers", this))
, record_target_button_(new QPushButton("Record Current Pose", this))
, refresh_targets_button_(new QPushButton("Refresh Targets", this))
, load_selected_target_button_(new QPushButton("Load Selected Target", this))
, apply_motion_tuning_button_(new QPushButton("Apply Motion Settings", this))
, shift_offset_positive_button_(new QPushButton("Shift Left (+Y)", this))
, shift_offset_negative_button_(new QPushButton("Shift Right (-Y)", this))
, zero_offset_button_(new QPushButton("Zero Offset", this))
, ros_timer_(new QTimer(this))
, has_scoop_poses_(false)
, has_target_goal_pose_(false)
, updating_pose_fields_(false)
, updating_scoop_pose_fields_(false)
, current_scoop_focus_index_(-1)
, spin_tick_count_(0)
{
  auto* title = new QLabel("Scooping Controls", this);
  auto* hint = new QLabel(
    "Use the 3D markers to place approach/contact/scoop/lift/transport_ready poses.\n"
    "Use the move-goal pose fields for exact xyz/rpy edits in addition to dragging the marker.",
    this);

  title->setStyleSheet("font-weight: 600; font-size: 14px;");
  hint->setWordWrap(true);
  service_state_label_->setWordWrap(true);
  status_label_->setWordWrap(true);
  target_name_edit_->setPlaceholderText("target_name in targets.yaml");
  target_name_edit_->setText("new_target");
  target_selector_combo_->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  target_selector_combo_->setMinimumContentsLength(18);
  scoop_marker_combo_->addItems(
    QStringList()
      << "Approach"
      << "Contact"
      << "Scoop"
      << "Lift"
      << "Transport Ready");
  scoop_x_edit_->setPlaceholderText("x");
  scoop_y_edit_->setPlaceholderText("y");
  scoop_z_edit_->setPlaceholderText("z");
  scoop_roll_edit_->setPlaceholderText("roll");
  scoop_pitch_edit_->setPlaceholderText("pitch");
  scoop_yaw_edit_->setPlaceholderText("yaw");

  auto* button_row = new QHBoxLayout();
  button_row->addWidget(save_button_);
  button_row->addWidget(load_button_);
  button_row->addWidget(plan_button_);
  button_row->addWidget(execute_button_);
  button_row->addWidget(execute_continuous_button_);
  button_row->addWidget(execute_waypoint_motion_button_);

  auto* authoring_row = new QHBoxLayout();
  authoring_row->addWidget(target_name_edit_, 1);
  authoring_row->addWidget(move_goal_button_);
  authoring_row->addWidget(record_target_button_);

  auto* targets_row = new QHBoxLayout();
  targets_row->addWidget(new QLabel("Saved target", this));
  targets_row->addWidget(target_selector_combo_, 1);
  targets_row->addWidget(load_selected_target_button_);
  targets_row->addWidget(refresh_targets_button_);

  x_edit_->setPlaceholderText("x");
  y_edit_->setPlaceholderText("y");
  z_edit_->setPlaceholderText("z");
  roll_edit_->setPlaceholderText("roll");
  pitch_edit_->setPlaceholderText("pitch");
  yaw_edit_->setPlaceholderText("yaw");
  velocity_scale_edit_->setPlaceholderText("0..1");
  acceleration_scale_edit_->setPlaceholderText("0..1");
  pattern_offset_y_edit_->setPlaceholderText("meters");
  offset_step_edit_->setPlaceholderText("step");
  velocity_scale_edit_->setText("0.20");
  acceleration_scale_edit_->setText("0.20");
  pattern_offset_y_edit_->setText("0.000");
  offset_step_edit_->setText("0.010");

  auto* pose_grid = new QGridLayout();
  pose_grid->addWidget(new QLabel("X (m)", this), 0, 0);
  pose_grid->addWidget(x_edit_, 0, 1);
  pose_grid->addWidget(new QLabel("Y (m)", this), 0, 2);
  pose_grid->addWidget(y_edit_, 0, 3);
  pose_grid->addWidget(new QLabel("Z (m)", this), 0, 4);
  pose_grid->addWidget(z_edit_, 0, 5);
  pose_grid->addWidget(new QLabel("Roll (deg)", this), 1, 0);
  pose_grid->addWidget(roll_edit_, 1, 1);
  pose_grid->addWidget(new QLabel("Pitch (deg)", this), 1, 2);
  pose_grid->addWidget(pitch_edit_, 1, 3);
  pose_grid->addWidget(new QLabel("Yaw (deg)", this), 1, 4);
  pose_grid->addWidget(yaw_edit_, 1, 5);

  auto* scoop_pose_grid = new QGridLayout();
  scoop_pose_grid->addWidget(new QLabel("Marker", this), 0, 0);
  scoop_pose_grid->addWidget(scoop_marker_combo_, 0, 1, 1, 2);
  scoop_pose_grid->addWidget(focus_selected_scoop_button_, 0, 3, 1, 2);
  scoop_pose_grid->addWidget(show_all_scoops_button_, 0, 5);
  scoop_pose_grid->addWidget(new QLabel("X (m)", this), 1, 0);
  scoop_pose_grid->addWidget(scoop_x_edit_, 1, 1);
  scoop_pose_grid->addWidget(new QLabel("Y (m)", this), 1, 2);
  scoop_pose_grid->addWidget(scoop_y_edit_, 1, 3);
  scoop_pose_grid->addWidget(new QLabel("Z (m)", this), 1, 4);
  scoop_pose_grid->addWidget(scoop_z_edit_, 1, 5);
  scoop_pose_grid->addWidget(new QLabel("Roll (deg)", this), 2, 0);
  scoop_pose_grid->addWidget(scoop_roll_edit_, 2, 1);
  scoop_pose_grid->addWidget(new QLabel("Pitch (deg)", this), 2, 2);
  scoop_pose_grid->addWidget(scoop_pitch_edit_, 2, 3);
  scoop_pose_grid->addWidget(new QLabel("Yaw (deg)", this), 2, 4);
  scoop_pose_grid->addWidget(scoop_yaw_edit_, 2, 5);

  auto* motion_grid = new QGridLayout();
  motion_grid->addWidget(new QLabel("Velocity Scale", this), 0, 0);
  motion_grid->addWidget(velocity_scale_edit_, 0, 1);
  motion_grid->addWidget(new QLabel("Acceleration Scale", this), 0, 2);
  motion_grid->addWidget(acceleration_scale_edit_, 0, 3);
  motion_grid->addWidget(apply_motion_tuning_button_, 0, 4, 1, 2);
  motion_grid->addWidget(new QLabel("Pattern Offset Y (m)", this), 1, 0);
  motion_grid->addWidget(pattern_offset_y_edit_, 1, 1);
  motion_grid->addWidget(new QLabel("Offset Step (m)", this), 1, 2);
  motion_grid->addWidget(offset_step_edit_, 1, 3);
  motion_grid->addWidget(shift_offset_negative_button_, 1, 4);
  motion_grid->addWidget(shift_offset_positive_button_, 1, 5);
  motion_grid->addWidget(zero_offset_button_, 2, 4, 1, 2);

  auto* layout = new QVBoxLayout();
  layout->addWidget(title);
  layout->addWidget(hint);
  layout->addSpacing(8);
  layout->addWidget(service_state_label_);
  layout->addLayout(button_row);
  layout->addSpacing(8);
  layout->addWidget(new QLabel("Motion Tuning", this));
  layout->addLayout(motion_grid);
  layout->addSpacing(8);
  layout->addWidget(new QLabel("Scoop Marker Editor", this));
  layout->addLayout(scoop_pose_grid);
  layout->addWidget(apply_scoop_pose_button_);
  layout->addSpacing(8);
  layout->addWidget(new QLabel("Target Authoring", this));
  layout->addLayout(targets_row);
  layout->addLayout(pose_grid);
  layout->addWidget(apply_typed_pose_button_);
  layout->addLayout(authoring_row);
  layout->addSpacing(8);
  layout->addWidget(status_label_);
  layout->addStretch(1);
  setLayout(layout);

  connect(save_button_, &QPushButton::clicked, this, &ScoopingPanel::onSaveClicked);
  connect(load_button_, &QPushButton::clicked, this, &ScoopingPanel::onLoadClicked);
  connect(plan_button_, &QPushButton::clicked, this, &ScoopingPanel::onPlanClicked);
  connect(execute_button_, &QPushButton::clicked, this, &ScoopingPanel::onExecuteClicked);
  connect(
    execute_continuous_button_,
    &QPushButton::clicked,
    this,
    &ScoopingPanel::onExecuteContinuousClicked);
  connect(
    execute_waypoint_motion_button_,
    &QPushButton::clicked,
    this,
    &ScoopingPanel::onExecuteWaypointMotionClicked);
  connect(ros_timer_, &QTimer::timeout, this, &ScoopingPanel::onRosTimer);
  connect(
    scoop_marker_combo_,
    qOverload<int>(&QComboBox::currentIndexChanged),
    this,
    [this](int) {
      updateScoopEditorsFromSelection();
      if (current_scoop_focus_index_ >= 0) {
        publishScoopMarkerFocus(selectedScoopMarkerIndex());
      }
    });
  connect(move_goal_button_, &QPushButton::clicked, this, [this]() { sendMoveGoal(); });
  connect(apply_typed_pose_button_, &QPushButton::clicked, this, [this]() { applyTypedPose(); });
  connect(apply_scoop_pose_button_, &QPushButton::clicked, this, [this]() { applyTypedScoopPose(); });
  connect(focus_selected_scoop_button_, &QPushButton::clicked, this, [this]() { focusSelectedScoopMarker(); });
  connect(show_all_scoops_button_, &QPushButton::clicked, this, [this]() { showAllScoopMarkers(); });
  connect(record_target_button_, &QPushButton::clicked, this, [this]() { sendRecordTarget(); });
  connect(refresh_targets_button_, &QPushButton::clicked, this, &ScoopingPanel::onRefreshTargetsClicked);
  connect(load_selected_target_button_, &QPushButton::clicked, this, &ScoopingPanel::onLoadSelectedTargetClicked);
  connect(
    target_selector_combo_,
    &QComboBox::currentTextChanged,
    this,
    [this](const QString& text) {
      if (!text.isEmpty()) {
        target_name_edit_->setText(text);
      }
    });
  connect(apply_motion_tuning_button_, &QPushButton::clicked, this, &ScoopingPanel::onApplyMotionTuningClicked);
  connect(shift_offset_positive_button_, &QPushButton::clicked, this, &ScoopingPanel::onShiftOffsetPositiveClicked);
  connect(shift_offset_negative_button_, &QPushButton::clicked, this, &ScoopingPanel::onShiftOffsetNegativeClicked);
  connect(zero_offset_button_, &QPushButton::clicked, this, &ScoopingPanel::onZeroOffsetClicked);

  updateStatus("Waiting for RViz panel initialization...");
  service_state_label_->setText("Services: not initialized");
  save_button_->setEnabled(false);
  load_button_->setEnabled(false);
  plan_button_->setEnabled(false);
  execute_button_->setEnabled(false);
  execute_continuous_button_->setEnabled(false);
  execute_waypoint_motion_button_->setEnabled(false);
  move_goal_button_->setEnabled(false);
  apply_typed_pose_button_->setEnabled(false);
  apply_scoop_pose_button_->setEnabled(false);
  focus_selected_scoop_button_->setEnabled(false);
  show_all_scoops_button_->setEnabled(false);
  record_target_button_->setEnabled(false);
  refresh_targets_button_->setEnabled(false);
  load_selected_target_button_->setEnabled(false);
  apply_motion_tuning_button_->setEnabled(false);
  shift_offset_positive_button_->setEnabled(false);
  shift_offset_negative_button_->setEnabled(false);
  zero_offset_button_->setEnabled(false);
}

ScoopingPanel::~ScoopingPanel() = default;

void ScoopingPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("scooping_rviz_panel");
  save_client_ = node_->create_client<Trigger>("/save_scoop_poses");
  load_client_ = node_->create_client<Trigger>("/load_scoop_poses");
  plan_client_ = node_->create_client<Trigger>("/plan_scoop");
  execute_client_ = node_->create_client<Trigger>("/execute_scoop");
  execute_continuous_client_ = node_->create_client<Trigger>("/execute_scoop_continuous");
  execute_waypoint_motion_client_ = node_->create_client<Trigger>("/execute_scoop_waypoint_motion");
  record_target_client_ = node_->create_client<RecordTarget>("/record_target");
  scooping_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/scooping_mtc_node");
  move_to_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/move_to_server");
  record_target_params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/target_recorder");
  move_to_client_ = rclcpp_action::create_client<MoveTo>(node_, "/move_to");
  scoop_poses_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
    "/scoop_poses_cmd",
    rclcpp::QoS(1).reliable());
  scoop_marker_focus_pub_ = node_->create_publisher<std_msgs::msg::Int32>(
    "/scoop_marker_focus_index",
    rclcpp::QoS(1).reliable());
  target_goal_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/target_goal_pose_cmd",
    rclcpp::QoS(1).reliable());
  scoop_poses_sub_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
    "/scoop_poses",
    rclcpp::QoS(1).transient_local(),
    [this](const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      latest_scoop_poses_ = msg->poses;
      latest_scoop_frame_id_ = msg->header.frame_id;
      has_scoop_poses_ = latest_scoop_poses_.size() >= 5;
      updateScoopEditorsFromSelection();
    });
  target_goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/target_goal_pose",
    rclcpp::QoS(1).transient_local(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      latest_target_goal_pose_ = *msg;
      has_target_goal_pose_ = true;
      updatePoseEditorsFromGoal();
    });

  ros_timer_->start(150);
  updateStatus("Panel ready.");
  refreshServiceState();
  refreshTargetsFromYaml();
}

void ScoopingPanel::onSaveClicked()
{
  sendRequest("Saving scoop poses...", save_client_);
}

void ScoopingPanel::onLoadClicked()
{
  sendRequest("Loading scoop poses...", load_client_);
}

void ScoopingPanel::onExecuteClicked()
{
  sendRequest("Executing scoop task...", execute_client_);
}

void ScoopingPanel::onExecuteContinuousClicked()
{
  sendRequest("Executing continuous scoop goal...", execute_continuous_client_);
}

void ScoopingPanel::onExecuteWaypointMotionClicked()
{
  sendRequest(
    "Executing waypoint-style scoop motion...",
    execute_waypoint_motion_client_);
}

void ScoopingPanel::onApplyMotionTuningClicked()
{
  applyMotionTuning();
}

void ScoopingPanel::onShiftOffsetPositiveClicked()
{
  double step = 0.0;
  if (!readDoubleField(offset_step_edit_, "Offset step", step)) {
    return;
  }
  adjustPatternOffset(std::abs(step));
}

void ScoopingPanel::onShiftOffsetNegativeClicked()
{
  double step = 0.0;
  if (!readDoubleField(offset_step_edit_, "Offset step", step)) {
    return;
  }
  adjustPatternOffset(-std::abs(step));
}

void ScoopingPanel::onZeroOffsetClicked()
{
  setLineEditValue(pattern_offset_y_edit_, 0.0, 3);
  applyMotionTuning();
}

void ScoopingPanel::onRefreshTargetsClicked()
{
  refreshTargetsFromYaml();
}

void ScoopingPanel::onLoadSelectedTargetClicked()
{
  loadSelectedTarget();
}

void ScoopingPanel::onPlanClicked()
{
  sendRequest("Planning scoop task...", plan_client_);
}

void ScoopingPanel::onRosTimer()
{
  if (node_) {
    rclcpp::spin_some(node_);
    ++spin_tick_count_;
    if (spin_tick_count_ % 5 == 0) {
      refreshServiceState();
    }
  }
}

void ScoopingPanel::sendRequest(
  const std::string& action_name,
  const rclcpp::Client<Trigger>::SharedPtr& client)
{
  if (!client || !client->service_is_ready()) {
    updateStatus("Requested service is not available.", "#fca5a5");
    refreshServiceState();
    return;
  }

  updateStatus(QString::fromStdString(action_name), "#93c5fd");
  auto request = std::make_shared<Trigger::Request>();
  client->async_send_request(
    request,
    [this](rclcpp::Client<Trigger>::SharedFuture future) {
      try {
        const auto response = future.get();
        updateStatus(
          QString::fromStdString(response->message.empty() ? "Request finished." : response->message),
          response->success ? "#86efac" : "#fca5a5");
      } catch (const std::exception& ex) {
        updateStatus(QString("Service call failed: %1").arg(ex.what()), "#fca5a5");
      }
      refreshServiceState();
    });
}

void ScoopingPanel::sendMoveGoal()
{
  if (!has_target_goal_pose_) {
    updateStatus("Move goal marker pose not available yet.", "#fca5a5");
    return;
  }
  if (!move_to_client_ || !move_to_client_->wait_for_action_server(std::chrono::seconds(1))) {
    updateStatus("MoveTo action server is not available.", "#fca5a5");
    refreshServiceState();
    return;
  }

  MoveTo::Goal goal;
  goal.target_pose = latest_target_goal_pose_;
  goal.use_cartesian = false;
  double velocity_scale = 0.2;
  double acceleration_scale = 0.2;
  (void)readDoubleField(velocity_scale_edit_, "Velocity scale", velocity_scale);
  (void)readDoubleField(acceleration_scale_edit_, "Acceleration scale", acceleration_scale);
  goal.velocity_scaling = static_cast<float>(velocity_scale);
  goal.acceleration_scaling = static_cast<float>(acceleration_scale);

  updateStatus("Sending MoveTo goal from target marker...", "#93c5fd");
  rclcpp_action::Client<MoveTo>::SendGoalOptions options;
  options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<MoveTo>::SharedPtr& goal_handle) {
      if (!goal_handle) {
        updateStatus("MoveTo goal was rejected.", "#fca5a5");
      } else {
        updateStatus("MoveTo goal accepted.", "#93c5fd");
      }
    };
  options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult& result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED && result.result && result.result->success) {
        updateStatus(
          QString::fromStdString(
            result.result->message.empty() ? "MoveTo goal executed." : result.result->message),
          "#86efac");
      } else {
        const std::string message =
          (result.result && !result.result->message.empty()) ? result.result->message : "MoveTo execution failed.";
        updateStatus(QString::fromStdString(message), "#fca5a5");
      }
      refreshServiceState();
    };
  move_to_client_->async_send_goal(goal, options);
}

void ScoopingPanel::sendRecordTarget()
{
  if (!record_target_client_ || !record_target_client_->service_is_ready()) {
    updateStatus("RecordTarget service is not available.", "#fca5a5");
    refreshServiceState();
    return;
  }

  const QString target_name = target_name_edit_->text().trimmed();
  if (target_name.isEmpty()) {
    updateStatus("Enter a target name before recording.", "#fca5a5");
    return;
  }

  auto request = std::make_shared<RecordTarget::Request>();
  request->name = target_name.toStdString();
  request->joints = false;
  updateStatus(QString("Recording current pose as '%1'...").arg(target_name), "#93c5fd");
  record_target_client_->async_send_request(
    request,
    [this](rclcpp::Client<RecordTarget>::SharedFuture future) {
      try {
        const auto response = future.get();
        updateStatus(
          QString::fromStdString(response->message.empty() ? "RecordTarget finished." : response->message),
          response->success ? "#86efac" : "#fca5a5");
        if (response->success) {
          refreshTargetsFromYaml();
        }
      } catch (const std::exception& ex) {
        updateStatus(QString("RecordTarget failed: %1").arg(ex.what()), "#fca5a5");
      }
      refreshServiceState();
    });
}

void ScoopingPanel::applyTypedPose()
{
  if (!target_goal_cmd_pub_) {
    updateStatus("Target goal pose publisher is not initialized.", "#fca5a5");
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  QString error_message;
  if (!typedPoseToStamped(pose, error_message)) {
    updateStatus(error_message, "#fca5a5");
    return;
  }

  latest_target_goal_pose_ = pose;
  has_target_goal_pose_ = true;
  target_goal_cmd_pub_->publish(pose);
  updateStatus("Applied typed pose to move goal marker.", "#86efac");
  refreshServiceState();
}

void ScoopingPanel::applyTypedScoopPose()
{
  if (!has_scoop_poses_ || !scoop_poses_cmd_pub_) {
    updateStatus("Scoop poses are not available yet.", "#fca5a5");
    return;
  }

  const int index = selectedScoopMarkerIndex();
  if (index < 0 || index >= static_cast<int>(latest_scoop_poses_.size())) {
    updateStatus("Selected scoop marker index is out of range.", "#fca5a5");
    return;
  }

  geometry_msgs::msg::Pose pose;
  QString error_message;
  if (!typedEditorsToPose(
        scoop_x_edit_,
        scoop_y_edit_,
        scoop_z_edit_,
        scoop_roll_edit_,
        scoop_pitch_edit_,
        scoop_yaw_edit_,
        pose,
        error_message))
  {
    updateStatus(error_message, "#fca5a5");
    return;
  }

  latest_scoop_poses_[static_cast<std::size_t>(index)] = pose;
  geometry_msgs::msg::PoseArray msg;
  msg.header.frame_id = latest_scoop_frame_id_.empty() ? "base_link" : latest_scoop_frame_id_;
  msg.header.stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  msg.poses = latest_scoop_poses_;
  scoop_poses_cmd_pub_->publish(msg);
  updateStatus(
    QString("Applied typed pose to scoop marker '%1'.").arg(scoop_marker_combo_->currentText()),
    "#86efac");
}

void ScoopingPanel::updatePoseEditorsFromGoal()
{
  if (!has_target_goal_pose_) {
    return;
  }

  updating_pose_fields_ = true;
  x_edit_->setText(QString::number(latest_target_goal_pose_.pose.position.x, 'f', 4));
  y_edit_->setText(QString::number(latest_target_goal_pose_.pose.position.y, 'f', 4));
  z_edit_->setText(QString::number(latest_target_goal_pose_.pose.position.z, 'f', 4));

  const auto& q_msg = latest_target_goal_pose_.pose.orientation;
  tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  roll_edit_->setText(QString::number(radians_to_degrees(roll), 'f', 2));
  pitch_edit_->setText(QString::number(radians_to_degrees(pitch), 'f', 2));
  yaw_edit_->setText(QString::number(radians_to_degrees(yaw), 'f', 2));
  updating_pose_fields_ = false;
}

void ScoopingPanel::updateScoopEditorsFromSelection()
{
  if (!has_scoop_poses_) {
    return;
  }
  const int index = selectedScoopMarkerIndex();
  if (index < 0 || index >= static_cast<int>(latest_scoop_poses_.size())) {
    return;
  }
  updating_scoop_pose_fields_ = true;
  setEditorsFromPose(
    latest_scoop_poses_[static_cast<std::size_t>(index)],
    scoop_x_edit_,
    scoop_y_edit_,
    scoop_z_edit_,
    scoop_roll_edit_,
    scoop_pitch_edit_,
    scoop_yaw_edit_);
  updating_scoop_pose_fields_ = false;
}

void ScoopingPanel::focusSelectedScoopMarker()
{
  publishScoopMarkerFocus(selectedScoopMarkerIndex());
}

void ScoopingPanel::showAllScoopMarkers()
{
  publishScoopMarkerFocus(-1);
}

void ScoopingPanel::applyMotionTuning()
{
  double velocity_scale = 0.0;
  double acceleration_scale = 0.0;
  double pattern_offset_y = 0.0;
  if (!readDoubleField(velocity_scale_edit_, "Velocity scale", velocity_scale) ||
      !readDoubleField(acceleration_scale_edit_, "Acceleration scale", acceleration_scale) ||
      !readDoubleField(pattern_offset_y_edit_, "Pattern offset Y", pattern_offset_y))
  {
    return;
  }

  if (velocity_scale < 0.0 || velocity_scale > 1.0 ||
      acceleration_scale < 0.0 || acceleration_scale > 1.0)
  {
    updateStatus("Velocity and acceleration scale must be within [0, 1].", "#fca5a5");
    return;
  }

  const bool scoop_ready = scooping_params_client_ && scooping_params_client_->service_is_ready();
  const bool move_to_ready = move_to_params_client_ && move_to_params_client_->service_is_ready();
  if (!scoop_ready || !move_to_ready) {
    updateStatus("Parameter services are not available yet.", "#fca5a5");
    refreshServiceState();
    return;
  }

  updateStatus("Applying motion settings...", "#93c5fd");
  setLineEditValue(velocity_scale_edit_, velocity_scale, 2);
  setLineEditValue(acceleration_scale_edit_, acceleration_scale, 2);
  setLineEditValue(pattern_offset_y_edit_, pattern_offset_y, 3);

  auto pending = std::make_shared<int>(2);
  auto all_ok = std::make_shared<bool>(true);
  auto errors = std::make_shared<QStringList>();
  auto finalize = [this, pending, all_ok, errors, velocity_scale, acceleration_scale, pattern_offset_y]() {
    --(*pending);
    if (*pending > 0) {
      return;
    }
    if (*all_ok) {
      updateStatus(
        QString("Applied velocity=%1 acceleration=%2 offset_y=%3 m")
          .arg(velocity_scale, 0, 'f', 2)
          .arg(acceleration_scale, 0, 'f', 2)
          .arg(pattern_offset_y, 0, 'f', 3),
        "#86efac");
    } else {
      updateStatus(
        QString("Failed to apply motion settings: %1").arg(errors->join("; ")),
        "#fca5a5");
    }
    refreshServiceState();
  };

  scooping_params_client_->set_parameters(
    {
      rclcpp::Parameter("velocity_scaling", velocity_scale),
      rclcpp::Parameter("acceleration_scaling", acceleration_scale),
      rclcpp::Parameter("cartesian_velocity_scaling", velocity_scale),
      rclcpp::Parameter("cartesian_acceleration_scaling", acceleration_scale),
      rclcpp::Parameter("pattern_offset_y", pattern_offset_y),
    },
    [all_ok, errors, finalize](auto future) {
      try {
        const auto results = future.get();
        for (const auto& result : results) {
          if (!result.successful) {
            *all_ok = false;
            errors->append(QString::fromStdString(
              result.reason.empty() ? "scooping_mtc_node rejected parameter update" : result.reason));
          }
        }
      } catch (const std::exception& ex) {
        *all_ok = false;
        errors->append(QString("scooping_mtc_node: %1").arg(ex.what()));
      }
      finalize();
    });

  move_to_params_client_->set_parameters(
    {
      rclcpp::Parameter("velocity_scaling", velocity_scale),
      rclcpp::Parameter("acceleration_scaling", acceleration_scale),
    },
    [all_ok, errors, finalize](auto future) {
      try {
        const auto results = future.get();
        for (const auto& result : results) {
          if (!result.successful) {
            *all_ok = false;
            errors->append(QString::fromStdString(
              result.reason.empty() ? "move_to_server rejected parameter update" : result.reason));
          }
        }
      } catch (const std::exception& ex) {
        *all_ok = false;
        errors->append(QString("move_to_server: %1").arg(ex.what()));
      }
      finalize();
    });
}

void ScoopingPanel::adjustPatternOffset(double delta)
{
  double current_offset = 0.0;
  if (!readDoubleField(pattern_offset_y_edit_, "Pattern offset Y", current_offset)) {
    return;
  }
  setLineEditValue(pattern_offset_y_edit_, current_offset + delta, 3);
  applyMotionTuning();
}

void ScoopingPanel::refreshTargetsFromYaml()
{
  std::string yaml_path;
  if (!tryGetTargetsYamlPath(yaml_path)) {
    updateStatus("Could not query targets.yaml path from move_to_server.", "#fca5a5");
    refreshServiceState();
    return;
  }

  QString error_message;
  if (!loadNamedTargetsFromYaml(yaml_path, error_message)) {
    updateStatus(error_message, "#fca5a5");
    refreshServiceState();
    return;
  }

  targets_yaml_path_ = yaml_path;
  const QString previous = target_selector_combo_->currentText();
  target_selector_combo_->blockSignals(true);
  target_selector_combo_->clear();
  for (const auto& entry : named_targets_) {
    target_selector_combo_->addItem(QString::fromStdString(entry.first));
  }
  const int previous_index = target_selector_combo_->findText(previous);
  if (previous_index >= 0) {
    target_selector_combo_->setCurrentIndex(previous_index);
  } else if (target_selector_combo_->count() > 0) {
    target_selector_combo_->setCurrentIndex(0);
  }
  target_selector_combo_->blockSignals(false);
  if (!target_selector_combo_->currentText().isEmpty()) {
    target_name_edit_->setText(target_selector_combo_->currentText());
  }

  updateStatus(
    QString("Loaded %1 targets from %2")
      .arg(target_selector_combo_->count())
      .arg(QString::fromStdString(targets_yaml_path_)),
    "#86efac");
}

void ScoopingPanel::loadSelectedTarget()
{
  const QString selected = target_selector_combo_->currentText().trimmed();
  if (selected.isEmpty()) {
    updateStatus("Select a saved target first.", "#fca5a5");
    return;
  }

  const auto it = named_targets_.find(selected.toStdString());
  if (it == named_targets_.end()) {
    updateStatus("Selected target is not loaded in the panel.", "#fca5a5");
    return;
  }

  latest_target_goal_pose_ = it->second;
  has_target_goal_pose_ = true;
  target_name_edit_->setText(selected);
  updatePoseEditorsFromGoal();
  if (target_goal_cmd_pub_) {
    auto pose = latest_target_goal_pose_;
    pose.header.stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
    target_goal_cmd_pub_->publish(pose);
  }
  updateStatus(QString("Loaded target '%1' into the move goal marker.").arg(selected), "#86efac");
}

bool ScoopingPanel::tryGetTargetsYamlPath(std::string& yaml_path)
{
  return tryGetTargetsYamlPathFromClient(move_to_params_client_, yaml_path) ||
         tryGetTargetsYamlPathFromClient(record_target_params_client_, yaml_path);
}

bool ScoopingPanel::tryGetTargetsYamlPathFromClient(
  const rclcpp::AsyncParametersClient::SharedPtr& client,
  std::string& yaml_path)
{
  if (!client || !client->service_is_ready()) {
    return false;
  }

  auto future = client->get_parameters({"targets_yaml"});
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    if (future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
      break;
    }
    if (node_) {
      rclcpp::spin_some(node_);
    }
  }
  if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    return false;
  }

  try {
    const auto params = future.get();
    if (params.empty()) {
      return false;
    }
    yaml_path = params.front().as_string();
    return !yaml_path.empty();
  } catch (...) {
    return false;
  }
}

bool ScoopingPanel::loadNamedTargetsFromYaml(const std::string& yaml_path, QString& error_message)
{
  if (!std::ifstream(yaml_path).good()) {
    error_message = QString("targets.yaml not found: %1").arg(QString::fromStdString(yaml_path));
    return false;
  }

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const std::exception& ex) {
    error_message = QString("Failed to parse targets.yaml: %1").arg(ex.what());
    return false;
  }

  if (!root["targets"] || !root["targets"].IsMap()) {
    error_message = "targets.yaml has no 'targets' map.";
    return false;
  }

  named_targets_.clear();
  for (const auto& it : root["targets"]) {
    const auto name = it.first.as<std::string>();
    const auto node = it.second;
    if (node["joints"]) {
      continue;
    }
    if (!node["frame_id"] || !node["position"] || !node["orientation"]) {
      continue;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = node["frame_id"].as<std::string>();
    pose.pose.position.x = node["position"]["x"].as<double>();
    pose.pose.position.y = node["position"]["y"].as<double>();
    pose.pose.position.z = node["position"]["z"].as<double>();
    pose.pose.orientation.x = node["orientation"]["x"].as<double>();
    pose.pose.orientation.y = node["orientation"]["y"].as<double>();
    pose.pose.orientation.z = node["orientation"]["z"].as<double>();
    pose.pose.orientation.w = node["orientation"]["w"].as<double>();
    named_targets_[name] = pose;
  }

  return true;
}

bool ScoopingPanel::typedPoseToStamped(
  geometry_msgs::msg::PoseStamped& pose,
  QString& error_message) const
{
  if (!typedEditorsToPose(
        x_edit_,
        y_edit_,
        z_edit_,
        roll_edit_,
        pitch_edit_,
        yaw_edit_,
        pose.pose,
        error_message))
  {
    return false;
  }

  pose.header.frame_id =
    latest_target_goal_pose_.header.frame_id.empty() ? "base_link" : latest_target_goal_pose_.header.frame_id;
  pose.header.stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  return true;
}

bool ScoopingPanel::typedEditorsToPose(
  QLineEdit* x_edit,
  QLineEdit* y_edit,
  QLineEdit* z_edit,
  QLineEdit* roll_edit,
  QLineEdit* pitch_edit,
  QLineEdit* yaw_edit,
  geometry_msgs::msg::Pose& pose,
  QString& error_message) const
{
  bool ok_x = false;
  bool ok_y = false;
  bool ok_z = false;
  bool ok_roll = false;
  bool ok_pitch = false;
  bool ok_yaw = false;

  const double x = x_edit->text().trimmed().toDouble(&ok_x);
  const double y = y_edit->text().trimmed().toDouble(&ok_y);
  const double z = z_edit->text().trimmed().toDouble(&ok_z);
  const double roll_deg = roll_edit->text().trimmed().toDouble(&ok_roll);
  const double pitch_deg = pitch_edit->text().trimmed().toDouble(&ok_pitch);
  const double yaw_deg = yaw_edit->text().trimmed().toDouble(&ok_yaw);

  if (!(ok_x && ok_y && ok_z && ok_roll && ok_pitch && ok_yaw)) {
    error_message = "Enter valid numeric values for x, y, z, roll, pitch, and yaw.";
    return false;
  }

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  tf2::Quaternion q;
  q.setRPY(
    degrees_to_radians(roll_deg),
    degrees_to_radians(pitch_deg),
    degrees_to_radians(yaw_deg));
  q.normalize();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return true;
}

void ScoopingPanel::setEditorsFromPose(
  const geometry_msgs::msg::Pose& pose,
  QLineEdit* x_edit,
  QLineEdit* y_edit,
  QLineEdit* z_edit,
  QLineEdit* roll_edit,
  QLineEdit* pitch_edit,
  QLineEdit* yaw_edit)
{
  x_edit->setText(QString::number(pose.position.x, 'f', 4));
  y_edit->setText(QString::number(pose.position.y, 'f', 4));
  z_edit->setText(QString::number(pose.position.z, 'f', 4));

  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  roll_edit->setText(QString::number(radians_to_degrees(roll), 'f', 2));
  pitch_edit->setText(QString::number(radians_to_degrees(pitch), 'f', 2));
  yaw_edit->setText(QString::number(radians_to_degrees(yaw), 'f', 2));
}

int ScoopingPanel::selectedScoopMarkerIndex() const
{
  return scoop_marker_combo_ ? scoop_marker_combo_->currentIndex() : -1;
}

void ScoopingPanel::publishScoopMarkerFocus(int index)
{
  if (!scoop_marker_focus_pub_) {
    return;
  }
  std_msgs::msg::Int32 msg;
  msg.data = index;
  scoop_marker_focus_pub_->publish(msg);
  current_scoop_focus_index_ = index;
  updateStatus(
    index >= 0
      ? QString("Focusing scoop marker '%1'.").arg(scoop_marker_combo_->itemText(index))
      : "Showing all scoop markers.",
    "#93c5fd");
}

bool ScoopingPanel::readDoubleField(QLineEdit* edit, const QString& label, double& value)
{
  bool ok = false;
  const double parsed = edit->text().trimmed().toDouble(&ok);
  if (!ok) {
    updateStatus(QString("%1 must be a valid number.").arg(label), "#fca5a5");
    return false;
  }
  value = parsed;
  return true;
}

void ScoopingPanel::setLineEditValue(QLineEdit* edit, double value, int decimals)
{
  edit->setText(QString::number(value, 'f', decimals));
}

void ScoopingPanel::updateStatus(const QString& text, const QString& color)
{
  status_label_->setStyleSheet(QString("color: %1;").arg(color));
  status_label_->setText(QString("Status: %1").arg(text));
}

void ScoopingPanel::refreshServiceState()
{
  const bool save_ready = save_client_ && save_client_->service_is_ready();
  const bool load_ready = load_client_ && load_client_->service_is_ready();
  const bool plan_ready = plan_client_ && plan_client_->service_is_ready();
  const bool execute_ready = execute_client_ && execute_client_->service_is_ready();
  const bool execute_continuous_ready =
    execute_continuous_client_ && execute_continuous_client_->service_is_ready();
  const bool execute_waypoint_motion_ready =
    execute_waypoint_motion_client_ && execute_waypoint_motion_client_->service_is_ready();
  const bool record_ready = record_target_client_ && record_target_client_->service_is_ready();
  const bool scoop_params_ready =
    scooping_params_client_ && scooping_params_client_->service_is_ready();
  const bool move_to_params_ready =
    move_to_params_client_ && move_to_params_client_->service_is_ready();
  const bool record_target_params_ready =
    record_target_params_client_ && record_target_params_client_->service_is_ready();
  const bool tuning_ready = scoop_params_ready && move_to_params_ready;
  const bool move_ready = move_to_client_ && move_to_client_->wait_for_action_server(std::chrono::seconds(0));
  const bool scoop_edit_ready = has_scoop_poses_ && latest_scoop_poses_.size() >= 5;
  const bool targets_ready = move_to_params_ready || record_target_params_ready;

  save_button_->setEnabled(save_ready);
  load_button_->setEnabled(load_ready);
  plan_button_->setEnabled(plan_ready);
  execute_button_->setEnabled(execute_ready);
  execute_continuous_button_->setEnabled(execute_continuous_ready);
  execute_waypoint_motion_button_->setEnabled(execute_waypoint_motion_ready);
  move_goal_button_->setEnabled(move_ready && has_target_goal_pose_);
  apply_typed_pose_button_->setEnabled(has_target_goal_pose_);
  apply_scoop_pose_button_->setEnabled(scoop_edit_ready);
  focus_selected_scoop_button_->setEnabled(scoop_edit_ready);
  show_all_scoops_button_->setEnabled(scoop_edit_ready);
  record_target_button_->setEnabled(record_ready);
  refresh_targets_button_->setEnabled(targets_ready);
  load_selected_target_button_->setEnabled(targets_ready && target_selector_combo_->count() > 0);
  apply_motion_tuning_button_->setEnabled(tuning_ready);
  shift_offset_positive_button_->setEnabled(tuning_ready);
  shift_offset_negative_button_->setEnabled(tuning_ready);
  zero_offset_button_->setEnabled(tuning_ready);

  service_state_label_->setText(
    QString("Services: save=%1, load=%2, plan=%3, execute=%4, continuous=%5, waypoint=%6, move_to=%7, record=%8, scoop_editor=%9, tuning=%10, targets=%11")
      .arg(save_ready ? "ready" : "waiting")
      .arg(load_ready ? "ready" : "waiting")
      .arg(plan_ready ? "ready" : "waiting")
      .arg(execute_ready ? "ready" : "waiting")
      .arg(execute_continuous_ready ? "ready" : "waiting")
      .arg(execute_waypoint_motion_ready ? "ready" : "waiting")
      .arg(move_ready ? "ready" : "waiting")
      .arg(record_ready ? "ready" : "waiting")
      .arg(scoop_edit_ready ? "ready" : "waiting")
      .arg(tuning_ready ? "ready" : "waiting")
      .arg(targets_ready ? "ready" : "waiting"));
}
}  // namespace scooping_controller

PLUGINLIB_EXPORT_CLASS(scooping_controller::ScoopingPanel, rviz_common::Panel)
