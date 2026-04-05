#include "scooping_controller/scooping_panel.hpp"

#include <algorithm>
#include <functional>

#include <pluginlib/class_list_macros.hpp>
#include <yaml-cpp/yaml.h>

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/msg/marker.hpp>

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

QSlider* make_slider(QWidget* parent, int min_value, int max_value, int initial_value)
{
  auto* slider = new QSlider(Qt::Horizontal, parent);
  slider->setRange(min_value, max_value);
  slider->setValue(initial_value);
  slider->setSingleStep(1);
  slider->setPageStep(std::max(1, (max_value - min_value) / 20));
  return slider;
}

void bind_slider_and_edit(
  QSlider* slider,
  QLineEdit* edit,
  int decimals,
  const std::function<double(int)>& slider_to_value,
  const std::function<int(double)>& value_to_slider,
  const std::function<void()>& on_value_changed)
{
  QObject::connect(
    slider,
    &QSlider::valueChanged,
    edit,
    [slider, edit, decimals, slider_to_value, on_value_changed](int raw_value) {
      Q_UNUSED(slider);
      const QSignalBlocker blocker(edit);
      edit->setText(QString::number(slider_to_value(raw_value), 'f', decimals));
      on_value_changed();
    });

  QObject::connect(
    edit,
    &QLineEdit::textChanged,
    slider,
    [slider, value_to_slider, on_value_changed](const QString& text) {
      bool ok = false;
      const double value = text.trimmed().toDouble(&ok);
      if (!ok) {
        return;
      }

      const int clamped_value = std::clamp(
        value_to_slider(value),
        slider->minimum(),
        slider->maximum());
      if (slider->value() != clamped_value) {
        const QSignalBlocker blocker(slider);
        slider->setValue(clamped_value);
      }
      on_value_changed();
    });
}

std::vector<geometry_msgs::msg::Pose> apply_manual_pose_adjustments(
  const std::vector<geometry_msgs::msg::Pose>& input_poses,
  double offset_x,
  double offset_y,
  double offset_z,
  double sweep_scale,
  double pitch_offset_rad,
  double lift_offset_z)
{
  auto poses = input_poses;
  if (poses.size() < 5U) {
    return poses;
  }

  for (auto& pose : poses) {
    pose.position.x += offset_x;
    pose.position.y += offset_y;
    pose.position.z += offset_z;
  }

  const auto contact_pose = poses[1];
  for (std::size_t i = 2; i < poses.size(); ++i) {
    poses[i].position.x =
      contact_pose.position.x + sweep_scale * (poses[i].position.x - contact_pose.position.x);
    poses[i].position.y =
      contact_pose.position.y + sweep_scale * (poses[i].position.y - contact_pose.position.y);
    poses[i].position.z =
      contact_pose.position.z + sweep_scale * (poses[i].position.z - contact_pose.position.z);
  }

  poses[3].position.z += lift_offset_z;
  poses[4].position.z += lift_offset_z;

  if (pitch_offset_rad != 0.0) {
    tf2::Quaternion q_offset;
    q_offset.setRPY(0.0, pitch_offset_rad, 0.0);
    q_offset.normalize();
    for (auto& pose : poses) {
      tf2::Quaternion q_current;
      q_current.setX(pose.orientation.x);
      q_current.setY(pose.orientation.y);
      q_current.setZ(pose.orientation.z);
      q_current.setW(pose.orientation.w);
      tf2::Quaternion q_adjusted = q_current * q_offset;
      q_adjusted.normalize();
      pose.orientation.x = q_adjusted.x();
      pose.orientation.y = q_adjusted.y();
      pose.orientation.z = q_adjusted.z();
      pose.orientation.w = q_adjusted.w();
    }
  }

  return poses;
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
, velocity_scale_slider_(make_slider(this, 0, 100, 20))
, acceleration_scale_edit_(new QLineEdit(this))
, acceleration_scale_slider_(make_slider(this, 0, 100, 20))
, pattern_offset_x_edit_(new QLineEdit(this))
, pattern_offset_x_slider_(make_slider(this, -200, 200, 0))
, pattern_offset_y_edit_(new QLineEdit(this))
, pattern_offset_y_slider_(make_slider(this, -200, 200, 0))
, pattern_offset_z_edit_(new QLineEdit(this))
, pattern_offset_z_slider_(make_slider(this, -200, 200, 0))
, offset_step_edit_(new QLineEdit(this))
, sweep_scale_edit_(new QLineEdit(this))
, sweep_scale_slider_(make_slider(this, 25, 250, 100))
, pitch_offset_edit_(new QLineEdit(this))
, pitch_offset_slider_(make_slider(this, -450, 450, 0))
, lift_offset_z_edit_(new QLineEdit(this))
, lift_offset_z_slider_(make_slider(this, -100, 200, 0))
, post_lift_vibration_enabled_checkbox_(new QCheckBox("Enable Post-Lift Shake-Off", this))
, post_lift_vibration_duration_edit_(new QLineEdit(this))
, post_lift_vibration_duration_slider_(make_slider(this, 0, 500, 50))
, post_lift_vibration_intensity_edit_(new QLineEdit(this))
, post_lift_vibration_intensity_slider_(make_slider(this, 0, 100, 50))
, template_x_edit_(new QLineEdit(this))
, template_y_edit_(new QLineEdit(this))
, template_z_initial_edit_(new QLineEdit(this))
, template_z_final_edit_(new QLineEdit(this))
, template_sweep_length_edit_(new QLineEdit(this))
, template_pitch_edit_(new QLineEdit(this))
, template_hover_height_edit_(new QLineEdit(this))
, template_transport_pitch_edit_(new QLineEdit(this))
, template_lift_height_edit_(new QLineEdit(this))
, save_button_(new QPushButton("Save YAML", this))
, load_button_(new QPushButton("Load YAML", this))
, plan_button_(new QPushButton("Plan Only", this))
, execute_button_(new QPushButton("Execute Scoop", this))
, execute_continuous_button_(new QPushButton("Execute Continuous", this))
, execute_waypoint_motion_button_(new QPushButton("Execute Waypoint Style", this))
, preview_parameterized_button_(new QPushButton("Preview Template", this))
, plan_parameterized_button_(new QPushButton("Plan Parameterized Scoop", this))
, execute_parameterized_button_(new QPushButton("Execute Parameterized Scoop", this))
, move_goal_button_(new QPushButton("Move To Goal Marker", this))
, apply_typed_pose_button_(new QPushButton("Apply Typed Pose", this))
, apply_scoop_pose_button_(new QPushButton("Apply To Selected Scoop Marker", this))
, focus_selected_scoop_button_(new QPushButton("Focus Selected Marker", this))
, show_all_scoops_button_(new QPushButton("Show All Scoop Markers", this))
, record_target_button_(new QPushButton("Save Target To YAML", this))
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

  auto* button_grid = new QGridLayout();
  button_grid->setHorizontalSpacing(8);
  button_grid->setVerticalSpacing(6);
  button_grid->addWidget(save_button_, 0, 0);
  button_grid->addWidget(load_button_, 0, 1);
  button_grid->addWidget(plan_button_, 0, 2);
  button_grid->addWidget(execute_button_, 0, 3);
  button_grid->addWidget(execute_continuous_button_, 1, 0, 1, 2);
  button_grid->addWidget(execute_waypoint_motion_button_, 1, 2, 1, 2);

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
  pattern_offset_x_edit_->setPlaceholderText("meters");
  pattern_offset_y_edit_->setPlaceholderText("meters");
  pattern_offset_z_edit_->setPlaceholderText("meters");
  offset_step_edit_->setPlaceholderText("step");
  sweep_scale_edit_->setPlaceholderText("scale");
  pitch_offset_edit_->setPlaceholderText("deg");
  lift_offset_z_edit_->setPlaceholderText("meters");
  post_lift_vibration_duration_edit_->setPlaceholderText("seconds");
  post_lift_vibration_intensity_edit_->setPlaceholderText("0..1");
  velocity_scale_edit_->setText("0.20");
  acceleration_scale_edit_->setText("0.20");
  pattern_offset_x_edit_->setText("0.000");
  pattern_offset_y_edit_->setText("0.000");
  pattern_offset_z_edit_->setText("0.000");
  offset_step_edit_->setText("0.010");
  sweep_scale_edit_->setText("1.000");
  pitch_offset_edit_->setText("0.0");
  lift_offset_z_edit_->setText("0.000");
  post_lift_vibration_enabled_checkbox_->setChecked(true);
  post_lift_vibration_duration_edit_->setText("5.00");
  post_lift_vibration_intensity_edit_->setText("0.50");
  velocity_scale_slider_->setToolTip("0.00 to 1.00");
  acceleration_scale_slider_->setToolTip("0.00 to 1.00");
  pattern_offset_x_slider_->setToolTip("-0.200 m to 0.200 m");
  pattern_offset_y_slider_->setToolTip("-0.200 m to 0.200 m");
  pattern_offset_z_slider_->setToolTip("-0.200 m to 0.200 m");
  sweep_scale_slider_->setToolTip("0.25 to 2.50");
  pitch_offset_slider_->setToolTip("-45.0 deg to 45.0 deg");
  lift_offset_z_slider_->setToolTip("-0.100 m to 0.200 m");
  post_lift_vibration_duration_slider_->setToolTip("0.00 s to 5.00 s");
  post_lift_vibration_intensity_slider_->setToolTip("0.00 to 1.00");
  template_x_edit_->setPlaceholderText("x");
  template_y_edit_->setPlaceholderText("y");
  template_z_initial_edit_->setPlaceholderText("z_i");
  template_z_final_edit_->setPlaceholderText("z_f");
  template_sweep_length_edit_->setPlaceholderText("L");
  template_pitch_edit_->setPlaceholderText("theta");
  template_hover_height_edit_->setPlaceholderText("hover");
  template_transport_pitch_edit_->setPlaceholderText("retain");
  template_lift_height_edit_->setPlaceholderText("lift");
  template_x_edit_->setText("0.300");
  template_y_edit_->setText("0.000");
  template_z_initial_edit_->setText("0.060");
  template_z_final_edit_->setText("0.080");
  template_sweep_length_edit_->setText("0.100");
  template_pitch_edit_->setText("25.0");
  template_hover_height_edit_->setText("0.240");
  template_transport_pitch_edit_->setText("15.0");
  template_lift_height_edit_->setText("0.240");

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
  motion_grid->setHorizontalSpacing(10);
  motion_grid->setVerticalSpacing(6);
  motion_grid->setColumnStretch(1, 1);
  motion_grid->setColumnStretch(2, 4);
  auto add_tuning_row =
    [this, motion_grid](int section_index, const QString& label, QLineEdit* edit, QSlider* slider) {
      const int title_row = section_index * 2;
      const int control_row = title_row + 1;
      auto* row_label = new QLabel(label, this);
      row_label->setStyleSheet("font-weight: 600;");
      motion_grid->addWidget(row_label, title_row, 0, 1, 3);
      motion_grid->addWidget(edit, control_row, 0);
      motion_grid->addWidget(slider, control_row, 1, 1, 2);
    };
  add_tuning_row(0, "Velocity Scale", velocity_scale_edit_, velocity_scale_slider_);
  add_tuning_row(1, "Acceleration Scale", acceleration_scale_edit_, acceleration_scale_slider_);
  add_tuning_row(2, "Offset X (m)", pattern_offset_x_edit_, pattern_offset_x_slider_);
  add_tuning_row(3, "Offset Y (m)", pattern_offset_y_edit_, pattern_offset_y_slider_);
  add_tuning_row(4, "Offset Z (m)", pattern_offset_z_edit_, pattern_offset_z_slider_);
  add_tuning_row(5, "Sweep Scale", sweep_scale_edit_, sweep_scale_slider_);
  add_tuning_row(6, "Pitch Offset (deg)", pitch_offset_edit_, pitch_offset_slider_);
  add_tuning_row(7, "Lift Offset Z (m)", lift_offset_z_edit_, lift_offset_z_slider_);
  motion_grid->addWidget(post_lift_vibration_enabled_checkbox_, 16, 0, 1, 3);
  add_tuning_row(9, "Shake-Off Duration (s)", post_lift_vibration_duration_edit_, post_lift_vibration_duration_slider_);
  add_tuning_row(10, "Shake-Off Intensity", post_lift_vibration_intensity_edit_, post_lift_vibration_intensity_slider_);

  auto* motion_actions_row = new QHBoxLayout();
  motion_actions_row->addWidget(new QLabel("Offset Step (m)", this));
  motion_actions_row->addWidget(offset_step_edit_);
  motion_actions_row->addWidget(shift_offset_negative_button_);
  motion_actions_row->addWidget(shift_offset_positive_button_);
  motion_actions_row->addWidget(zero_offset_button_);
  motion_actions_row->addStretch(1);
  motion_actions_row->addWidget(apply_motion_tuning_button_);

  auto* parameterized_grid = new QGridLayout();
  parameterized_grid->addWidget(new QLabel("X (m)", this), 0, 0);
  parameterized_grid->addWidget(template_x_edit_, 0, 1);
  parameterized_grid->addWidget(new QLabel("Y (m)", this), 0, 2);
  parameterized_grid->addWidget(template_y_edit_, 0, 3);
  parameterized_grid->addWidget(new QLabel("Z_i (m)", this), 0, 4);
  parameterized_grid->addWidget(template_z_initial_edit_, 0, 5);
  parameterized_grid->addWidget(new QLabel("Z_f (m)", this), 1, 0);
  parameterized_grid->addWidget(template_z_final_edit_, 1, 1);
  parameterized_grid->addWidget(new QLabel("Sweep L (m)", this), 1, 2);
  parameterized_grid->addWidget(template_sweep_length_edit_, 1, 3);
  parameterized_grid->addWidget(new QLabel("TCP Pitch (deg)", this), 1, 4);
  parameterized_grid->addWidget(template_pitch_edit_, 1, 5);
  parameterized_grid->addWidget(new QLabel("Hover Z (m)", this), 2, 0);
  parameterized_grid->addWidget(template_hover_height_edit_, 2, 1);
  parameterized_grid->addWidget(new QLabel("Retain TCP Pitch (deg)", this), 2, 2);
  parameterized_grid->addWidget(template_transport_pitch_edit_, 2, 3);
  parameterized_grid->addWidget(new QLabel("Lift Z (m)", this), 2, 4);
  parameterized_grid->addWidget(template_lift_height_edit_, 2, 5);

  auto* parameterized_buttons = new QGridLayout();
  parameterized_buttons->setHorizontalSpacing(8);
  parameterized_buttons->setVerticalSpacing(6);
  parameterized_buttons->addWidget(preview_parameterized_button_, 0, 0);
  parameterized_buttons->addWidget(plan_parameterized_button_, 0, 1);
  parameterized_buttons->addWidget(execute_parameterized_button_, 1, 0, 1, 2);

  auto make_group_box = [this](const QString& title_text) {
    auto* box = new QGroupBox(title_text, this);
    box->setStyleSheet(
      "QGroupBox { font-weight: 600; margin-top: 12px; padding-top: 10px; }"
      "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }");
    return box;
  };

  auto* execution_box = make_group_box("Execution");
  auto* execution_layout = new QVBoxLayout(execution_box);
  execution_layout->addWidget(service_state_label_);
  execution_layout->addLayout(button_grid);
  execution_box->setLayout(execution_layout);

  auto* motion_box = make_group_box("Motion Tuning");
  auto* motion_layout = new QVBoxLayout(motion_box);
  motion_layout->addLayout(motion_grid);
  motion_layout->addSpacing(8);
  motion_layout->addLayout(motion_actions_row);
  motion_box->setLayout(motion_layout);

  auto* scoop_box = make_group_box("Scoop Marker Editor");
  auto* scoop_layout = new QVBoxLayout(scoop_box);
  scoop_layout->addLayout(scoop_pose_grid);
  scoop_layout->addWidget(apply_scoop_pose_button_);
  scoop_box->setLayout(scoop_layout);

  auto* authoring_box = make_group_box("Target Authoring");
  auto* authoring_layout = new QVBoxLayout(authoring_box);
  authoring_layout->addLayout(targets_row);
  authoring_layout->addLayout(pose_grid);
  authoring_layout->addWidget(apply_typed_pose_button_);
  authoring_layout->addLayout(authoring_row);
  authoring_box->setLayout(authoring_layout);

  auto* parameterized_box = make_group_box("Scooping Parameter Workflow");
  auto* parameterized_layout = new QVBoxLayout(parameterized_box);
  parameterized_layout->addLayout(parameterized_grid);
  parameterized_layout->addLayout(parameterized_buttons);
  parameterized_box->setLayout(parameterized_layout);

  auto* content_widget = new QWidget(this);
  auto* content_layout = new QVBoxLayout(content_widget);
  content_layout->setContentsMargins(8, 8, 8, 8);
  content_layout->setSpacing(10);
  content_layout->addWidget(title);
  content_layout->addWidget(hint);
  content_layout->addSpacing(4);
  content_layout->addWidget(execution_box);
  content_layout->addWidget(motion_box);
  content_layout->addWidget(scoop_box);
  content_layout->addWidget(authoring_box);
  content_layout->addWidget(parameterized_box);
  content_layout->addSpacing(4);
  content_layout->addWidget(status_label_);
  content_layout->addStretch(1);

  auto* scroll_area = new QScrollArea(this);
  scroll_area->setWidgetResizable(true);
  scroll_area->setWidget(content_widget);

  auto* panel_layout = new QVBoxLayout();
  panel_layout->setContentsMargins(0, 0, 0, 0);
  panel_layout->addWidget(scroll_area);
  setLayout(panel_layout);

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
  connect(
    preview_parameterized_button_,
    &QPushButton::clicked,
    this,
    &ScoopingPanel::onPreviewParameterizedClicked);
  connect(
    plan_parameterized_button_,
    &QPushButton::clicked,
    this,
    &ScoopingPanel::onPlanParameterizedClicked);
  connect(
    execute_parameterized_button_,
    &QPushButton::clicked,
    this,
    &ScoopingPanel::onExecuteParameterizedClicked);
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

  auto preview_manual_tuning = [this]() {
    if (has_scoop_poses_) {
      publishManualScoopPreviewMarkers();
    }
  };
  bind_slider_and_edit(
    velocity_scale_slider_,
    velocity_scale_edit_,
    2,
    [](int raw) { return static_cast<double>(raw) / 100.0; },
    [](double value) { return static_cast<int>(std::round(value * 100.0)); },
    preview_manual_tuning);
  bind_slider_and_edit(
    acceleration_scale_slider_,
    acceleration_scale_edit_,
    2,
    [](int raw) { return static_cast<double>(raw) / 100.0; },
    [](double value) { return static_cast<int>(std::round(value * 100.0)); },
    preview_manual_tuning);
  bind_slider_and_edit(
    pattern_offset_x_slider_,
    pattern_offset_x_edit_,
    3,
    [](int raw) { return static_cast<double>(raw) / 1000.0; },
    [](double value) { return static_cast<int>(std::round(value * 1000.0)); },
    preview_manual_tuning);
  bind_slider_and_edit(
    pattern_offset_y_slider_,
    pattern_offset_y_edit_,
    3,
    [](int raw) { return static_cast<double>(raw) / 1000.0; },
    [](double value) { return static_cast<int>(std::round(value * 1000.0)); },
    preview_manual_tuning);
  bind_slider_and_edit(
    pattern_offset_z_slider_,
    pattern_offset_z_edit_,
    3,
    [](int raw) { return static_cast<double>(raw) / 1000.0; },
    [](double value) { return static_cast<int>(std::round(value * 1000.0)); },
    preview_manual_tuning);
  bind_slider_and_edit(
    sweep_scale_slider_,
    sweep_scale_edit_,
    3,
    [](int raw) { return static_cast<double>(raw) / 100.0; },
    [](double value) { return static_cast<int>(std::round(value * 100.0)); },
    preview_manual_tuning);
  bind_slider_and_edit(
    pitch_offset_slider_,
    pitch_offset_edit_,
    1,
    [](int raw) { return static_cast<double>(raw) / 10.0; },
    [](double value) { return static_cast<int>(std::round(value * 10.0)); },
    preview_manual_tuning);
  bind_slider_and_edit(
    lift_offset_z_slider_,
    lift_offset_z_edit_,
    3,
    [](int raw) { return static_cast<double>(raw) / 1000.0; },
    [](double value) { return static_cast<int>(std::round(value * 1000.0)); },
    preview_manual_tuning);
  bind_slider_and_edit(
    post_lift_vibration_duration_slider_,
    post_lift_vibration_duration_edit_,
    2,
    [](int raw) { return static_cast<double>(raw) / 100.0; },
    [](double value) { return static_cast<int>(std::round(value * 100.0)); },
    []() {});
  bind_slider_and_edit(
    post_lift_vibration_intensity_slider_,
    post_lift_vibration_intensity_edit_,
    2,
    [](int raw) { return static_cast<double>(raw) / 100.0; },
    [](double value) { return static_cast<int>(std::round(value * 100.0)); },
    []() {});

  updateStatus("Waiting for RViz panel initialization...");
  service_state_label_->setText("Services: not initialized");
  save_button_->setEnabled(false);
  load_button_->setEnabled(false);
  plan_button_->setEnabled(false);
  execute_button_->setEnabled(false);
  execute_continuous_button_->setEnabled(false);
  execute_waypoint_motion_button_->setEnabled(false);
  preview_parameterized_button_->setEnabled(false);
  plan_parameterized_button_->setEnabled(false);
  execute_parameterized_button_->setEnabled(false);
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
  plan_parameterized_client_ = node_->create_client<Trigger>("/plan_parameterized_scoop");
  execute_client_ = node_->create_client<Trigger>("/execute_scoop");
  execute_parameterized_client_ = node_->create_client<Trigger>("/execute_parameterized_scoop");
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
  parameterized_preview_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/parameterized_scoop_preview",
    rclcpp::QoS(1).transient_local().reliable());
  manual_preview_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/manual_scoop_preview",
    rclcpp::QoS(1).transient_local().reliable());
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

void ScoopingPanel::onPreviewParameterizedClicked()
{
  previewParameterizedTemplate();
}

void ScoopingPanel::onPlanParameterizedClicked()
{
  callParameterizedTemplateService(
    "Planning parameterized scoop template...",
    plan_parameterized_client_);
}

void ScoopingPanel::onExecuteParameterizedClicked()
{
  callParameterizedTemplateService(
    "Executing parameterized scoop template...",
    execute_parameterized_client_);
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
  setLineEditValue(velocity_scale_edit_, 0.20, 2);
  setLineEditValue(acceleration_scale_edit_, 0.20, 2);
  setLineEditValue(pattern_offset_x_edit_, 0.0, 3);
  setLineEditValue(pattern_offset_y_edit_, 0.0, 3);
  setLineEditValue(pattern_offset_z_edit_, 0.0, 3);
  setLineEditValue(sweep_scale_edit_, 1.0, 3);
  setLineEditValue(pitch_offset_edit_, 0.0, 1);
  setLineEditValue(lift_offset_z_edit_, 0.0, 3);
  post_lift_vibration_enabled_checkbox_->setChecked(true);
  setLineEditValue(post_lift_vibration_duration_edit_, 5.00, 2);
  setLineEditValue(post_lift_vibration_intensity_edit_, 0.50, 2);
  setLineEditValue(offset_step_edit_, 0.010, 3);
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
    publishParameterizedPreviewMarkers();
    publishManualScoopPreviewMarkers();
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
  const QString target_name = target_name_edit_->text().trimmed();
  if (target_name.isEmpty()) {
    updateStatus("Enter a target name before saving.", "#fca5a5");
    return;
  }

  std::string yaml_path;
  if (!tryGetTargetsYamlPath(yaml_path)) {
    updateStatus("Could not query targets.yaml path.", "#fca5a5");
    refreshServiceState();
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

  updateStatus(QString("Saving target '%1' to YAML...").arg(target_name), "#93c5fd");
  if (!saveNamedTargetToYaml(yaml_path, target_name.toStdString(), pose, error_message)) {
    updateStatus(error_message, "#fca5a5");
    refreshServiceState();
    return;
  }

  QTimer::singleShot(0, this, [this]() { refreshTargetsFromYaml(); });
  updateStatus(
    QString("Saved target '%1' to %2").arg(target_name, QString::fromStdString(yaml_path)),
    "#86efac");
  refreshServiceState();
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
  double pattern_offset_x = 0.0;
  double pattern_offset_y = 0.0;
  double pattern_offset_z = 0.0;
  double sweep_scale = 0.0;
  double pitch_offset_deg = 0.0;
  double lift_offset_z = 0.0;
  double post_lift_vibration_duration = 0.0;
  double post_lift_vibration_intensity = 0.0;
  if (!readDoubleField(velocity_scale_edit_, "Velocity scale", velocity_scale) ||
      !readDoubleField(acceleration_scale_edit_, "Acceleration scale", acceleration_scale) ||
      !readDoubleField(pattern_offset_x_edit_, "Pattern offset X", pattern_offset_x) ||
      !readDoubleField(pattern_offset_y_edit_, "Pattern offset Y", pattern_offset_y) ||
      !readDoubleField(pattern_offset_z_edit_, "Pattern offset Z", pattern_offset_z) ||
      !readDoubleField(sweep_scale_edit_, "Sweep scale", sweep_scale) ||
      !readDoubleField(pitch_offset_edit_, "Pitch offset", pitch_offset_deg) ||
      !readDoubleField(lift_offset_z_edit_, "Lift offset Z", lift_offset_z) ||
      !readDoubleField(
        post_lift_vibration_duration_edit_,
        "Post-lift vibration duration",
        post_lift_vibration_duration) ||
      !readDoubleField(
        post_lift_vibration_intensity_edit_,
        "Post-lift vibration intensity",
        post_lift_vibration_intensity))
  {
    return;
  }

  if (velocity_scale < 0.0 || velocity_scale > 1.0 ||
      acceleration_scale < 0.0 || acceleration_scale > 1.0 || sweep_scale <= 0.0 ||
      post_lift_vibration_duration < 0.0 ||
      post_lift_vibration_intensity < 0.0 || post_lift_vibration_intensity > 1.0)
  {
    updateStatus(
      "Velocity/acceleration/intensity must be within [0, 1], shake-off duration must be >= 0, and sweep scale must be > 0.",
      "#fca5a5");
    return;
  }

  const double pitch_offset_rad = degrees_to_radians(pitch_offset_deg);
  const bool post_lift_vibration_enabled = post_lift_vibration_enabled_checkbox_->isChecked();

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
  setLineEditValue(pattern_offset_x_edit_, pattern_offset_x, 3);
  setLineEditValue(pattern_offset_y_edit_, pattern_offset_y, 3);
  setLineEditValue(pattern_offset_z_edit_, pattern_offset_z, 3);
  setLineEditValue(sweep_scale_edit_, sweep_scale, 3);
  setLineEditValue(pitch_offset_edit_, pitch_offset_deg, 1);
  setLineEditValue(lift_offset_z_edit_, lift_offset_z, 3);
  setLineEditValue(post_lift_vibration_duration_edit_, post_lift_vibration_duration, 2);
  setLineEditValue(post_lift_vibration_intensity_edit_, post_lift_vibration_intensity, 2);
  publishManualScoopPreviewMarkers();

  auto pending = std::make_shared<int>(2);
  auto all_ok = std::make_shared<bool>(true);
  auto errors = std::make_shared<QStringList>();
  auto finalize =
    [this,
     pending,
     all_ok,
     errors,
     velocity_scale,
     acceleration_scale,
     pattern_offset_x,
     pattern_offset_y,
     pattern_offset_z,
     sweep_scale,
     pitch_offset_deg,
     lift_offset_z,
     post_lift_vibration_enabled,
     post_lift_vibration_duration,
     post_lift_vibration_intensity]() {
    --(*pending);
    if (*pending > 0) {
      return;
    }
    if (*all_ok) {
      updateStatus(
        QString(
          "Applied v=%1 a=%2 offsets=(%3, %4, %5) m sweep=%6 pitch=%7 deg lift_z=%8 m shake_off=%9 dur=%10 s intensity=%11")
          .arg(velocity_scale, 0, 'f', 2)
          .arg(acceleration_scale, 0, 'f', 2)
          .arg(pattern_offset_x, 0, 'f', 3)
          .arg(pattern_offset_y, 0, 'f', 3)
          .arg(pattern_offset_z, 0, 'f', 3)
          .arg(sweep_scale, 0, 'f', 3)
          .arg(pitch_offset_deg, 0, 'f', 1)
          .arg(lift_offset_z, 0, 'f', 3)
          .arg(post_lift_vibration_enabled ? "on" : "off")
          .arg(post_lift_vibration_duration, 0, 'f', 2)
          .arg(post_lift_vibration_intensity, 0, 'f', 2),
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
      rclcpp::Parameter("pattern_offset_x", pattern_offset_x),
      rclcpp::Parameter("pattern_offset_y", pattern_offset_y),
      rclcpp::Parameter("pattern_offset_z", pattern_offset_z),
      rclcpp::Parameter("manual_sweep_scale", sweep_scale),
      rclcpp::Parameter("manual_pitch_offset_rad", pitch_offset_rad),
      rclcpp::Parameter("manual_lift_offset_z", lift_offset_z),
      rclcpp::Parameter("post_lift_vibration_enabled", post_lift_vibration_enabled),
      rclcpp::Parameter("post_lift_vibration_duration_s", post_lift_vibration_duration),
      rclcpp::Parameter("post_lift_vibration_intensity", post_lift_vibration_intensity),
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

void ScoopingPanel::previewParameterizedTemplate()
{
  if (!scoop_poses_cmd_pub_) {
    updateStatus("Scoop pose publisher is not initialized.", "#fca5a5");
    return;
  }

  double x = 0.0;
  double y = 0.0;
  double z_initial = 0.0;
  double z_final = 0.0;
  double sweep_length = 0.0;
  double pitch_rad = 0.0;
  double hover_height = 0.0;
  double transport_pitch_rad = 0.0;
  double lift_height = 0.0;
  if (!readParameterizedTemplateFields(
        x,
        y,
        z_initial,
        z_final,
        sweep_length,
        pitch_rad,
        hover_height,
        transport_pitch_rad,
        lift_height))
  {
    return;
  }

  geometry_msgs::msg::PoseArray msg;
  msg.header.frame_id = "base_link";
  msg.header.stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  msg.poses = generateParameterizedTemplatePoses(
    x,
    y,
    z_initial,
    z_final,
    sweep_length,
    pitch_rad,
    hover_height,
    transport_pitch_rad,
    lift_height);
  double pattern_offset_y = 0.0;
  if (readDoubleField(pattern_offset_y_edit_, "Pattern offset Y", pattern_offset_y)) {
    for (auto& pose : msg.poses) {
      pose.position.y += pattern_offset_y;
    }
  }
  scoop_poses_cmd_pub_->publish(msg);
  updateStatus("Previewed parameterized scoop template in the 5 scoop markers.", "#86efac");
}

void ScoopingPanel::callParameterizedTemplateService(
  const QString& action_name,
  const rclcpp::Client<Trigger>::SharedPtr& client)
{
  if (!client || !client->service_is_ready()) {
    updateStatus("Parameterized scoop service is not available.", "#fca5a5");
    refreshServiceState();
    return;
  }
  if (!applyParameterizedTemplateParameters()) {
    return;
  }
  sendRequest(action_name.toStdString(), client);
}

bool ScoopingPanel::applyParameterizedTemplateParameters()
{
  if (!scooping_params_client_ || !scooping_params_client_->service_is_ready()) {
    updateStatus("Scooping parameter service is not available yet.", "#fca5a5");
    refreshServiceState();
    return false;
  }

  double x = 0.0;
  double y = 0.0;
  double z_initial = 0.0;
  double z_final = 0.0;
  double sweep_length = 0.0;
  double pitch_rad = 0.0;
  double hover_height = 0.0;
  double transport_pitch_rad = 0.0;
  double lift_height = 0.0;
  if (!readParameterizedTemplateFields(
        x,
        y,
        z_initial,
        z_final,
        sweep_length,
        pitch_rad,
        hover_height,
        transport_pitch_rad,
        lift_height))
  {
    return false;
  }

  updateStatus("Applying parameterized scoop template settings...", "#93c5fd");
  auto future = scooping_params_client_->set_parameters(
    {
      rclcpp::Parameter("template_x", x),
      rclcpp::Parameter("template_y", y),
      rclcpp::Parameter("template_z_initial", z_initial),
      rclcpp::Parameter("template_z_final", z_final),
      rclcpp::Parameter("template_sweep_length", sweep_length),
      rclcpp::Parameter("template_pitch_rad", pitch_rad),
      rclcpp::Parameter("template_hover_height", hover_height),
      rclcpp::Parameter("template_transport_pitch_rad", transport_pitch_rad),
      rclcpp::Parameter("template_lift_height", lift_height),
    });

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
    updateStatus("Timed out applying parameterized scoop template settings.", "#fca5a5");
    return false;
  }

  try {
    const auto results = future.get();
    for (const auto& result : results) {
      if (!result.successful) {
        updateStatus(
          QString("Parameterized template update rejected: %1").arg(QString::fromStdString(result.reason)),
          "#fca5a5");
        return false;
      }
    }
  } catch (const std::exception& ex) {
    updateStatus(QString("Failed to apply parameterized template: %1").arg(ex.what()), "#fca5a5");
    return false;
  }

  return true;
}

void ScoopingPanel::publishParameterizedPreviewMarkers()
{
  if (!parameterized_preview_pub_) {
    return;
  }

  bool ok_x = false;
  bool ok_y = false;
  bool ok_z_initial = false;
  bool ok_z_final = false;
  bool ok_sweep_length = false;
  bool ok_pitch_deg = false;
  bool ok_hover_height = false;
  bool ok_transport_pitch_deg = false;
  bool ok_lift_height = false;
  bool ok_pattern_offset_y = false;

  const double x = template_x_edit_->text().trimmed().toDouble(&ok_x);
  const double y = template_y_edit_->text().trimmed().toDouble(&ok_y);
  const double z_initial = template_z_initial_edit_->text().trimmed().toDouble(&ok_z_initial);
  const double z_final = template_z_final_edit_->text().trimmed().toDouble(&ok_z_final);
  const double sweep_length =
    template_sweep_length_edit_->text().trimmed().toDouble(&ok_sweep_length);
  const double pitch_deg = template_pitch_edit_->text().trimmed().toDouble(&ok_pitch_deg);
  const double hover_height =
    template_hover_height_edit_->text().trimmed().toDouble(&ok_hover_height);
  const double transport_pitch_deg =
    template_transport_pitch_edit_->text().trimmed().toDouble(&ok_transport_pitch_deg);
  const double lift_height =
    template_lift_height_edit_->text().trimmed().toDouble(&ok_lift_height);
  const double pattern_offset_y =
    pattern_offset_y_edit_->text().trimmed().toDouble(&ok_pattern_offset_y);

  if (!(ok_x && ok_y && ok_z_initial && ok_z_final && ok_sweep_length && ok_pitch_deg &&
        ok_hover_height && ok_transport_pitch_deg && ok_lift_height && ok_pattern_offset_y))
  {
    return;
  }

  const auto poses = generateParameterizedTemplatePoses(
    x,
    y + pattern_offset_y,
    z_initial,
    z_final,
    sweep_length,
    degrees_to_radians(pitch_deg),
    hover_height,
    degrees_to_radians(transport_pitch_deg),
    lift_height);

  const auto stamp = node_->now();
  auto make_color = [](float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  };

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker line;
  line.header.frame_id = "base_link";
  line.header.stamp = stamp;
  line.ns = "parameterized_scoop_preview";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.006;
  line.color = make_color(0.1F, 0.8F, 1.0F, 1.0F);
  for (std::size_t index : {0U, 1U, 2U, 4U}) {
    geometry_msgs::msg::Point point;
    point.x = poses[index].position.x;
    point.y = poses[index].position.y;
    point.z = poses[index].position.z;
    line.points.push_back(point);
  }
  markers.markers.push_back(line);

  visualization_msgs::msg::Marker points;
  points.header = line.header;
  points.ns = line.ns;
  points.id = 1;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.scale.x = 0.015;
  points.scale.y = 0.015;
  points.scale.z = 0.015;
  points.color = make_color(1.0F, 0.9F, 0.2F, 1.0F);
  for (const auto& pose : poses) {
    geometry_msgs::msg::Point point;
    point.x = pose.position.x;
    point.y = pose.position.y;
    point.z = pose.position.z;
    points.points.push_back(point);
  }
  markers.markers.push_back(points);

  const std::array<QString, 5> labels = {
    "Hover", "Plunge", "Sweep End", "Retain", "Lift"};
  for (std::size_t i = 0; i < poses.size(); ++i) {
    visualization_msgs::msg::Marker text;
    text.header = line.header;
    text.ns = line.ns;
    text.id = static_cast<int>(10 + i);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = poses[i];
    text.pose.position.z += 0.03;
    text.scale.z = 0.03;
    text.color = make_color(0.95F, 0.95F, 0.95F, 1.0F);
    text.text = labels[i].toStdString();
    markers.markers.push_back(text);
  }

  visualization_msgs::msg::Marker retain_arrow;
  retain_arrow.header = line.header;
  retain_arrow.ns = line.ns;
  retain_arrow.id = 20;
  retain_arrow.type = visualization_msgs::msg::Marker::ARROW;
  retain_arrow.action = visualization_msgs::msg::Marker::ADD;
  retain_arrow.pose = poses[3];
  retain_arrow.scale.x = 0.08;
  retain_arrow.scale.y = 0.01;
  retain_arrow.scale.z = 0.01;
  retain_arrow.color = make_color(1.0F, 0.45F, 0.15F, 1.0F);
  markers.markers.push_back(retain_arrow);

  visualization_msgs::msg::Marker working_arrow;
  working_arrow.header = line.header;
  working_arrow.ns = line.ns;
  working_arrow.id = 21;
  working_arrow.type = visualization_msgs::msg::Marker::ARROW;
  working_arrow.action = visualization_msgs::msg::Marker::ADD;
  working_arrow.pose = poses[1];
  working_arrow.scale.x = 0.08;
  working_arrow.scale.y = 0.01;
  working_arrow.scale.z = 0.01;
  working_arrow.color = make_color(0.2F, 1.0F, 0.3F, 1.0F);
  markers.markers.push_back(working_arrow);

  parameterized_preview_pub_->publish(markers);
}

void ScoopingPanel::publishManualScoopPreviewMarkers()
{
  if (!manual_preview_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.header.frame_id = latest_scoop_frame_id_.empty() ? "base_link" : latest_scoop_frame_id_;
  clear.header.stamp = node_ ? node_->now() : rclcpp::Time(0, 0, RCL_ROS_TIME);
  clear.ns = "manual_scoop_preview";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  if (!(has_scoop_poses_ && latest_scoop_poses_.size() >= 5)) {
    manual_preview_pub_->publish(markers);
    return;
  }

  const auto stamp = node_->now();
  const std::string frame_id = latest_scoop_frame_id_.empty() ? "base_link" : latest_scoop_frame_id_;
  auto parse_or_default = [](QLineEdit* edit, double fallback) {
    bool ok = false;
    const double value = edit->text().trimmed().toDouble(&ok);
    return ok ? value : fallback;
  };
  const double pattern_offset_x = parse_or_default(pattern_offset_x_edit_, 0.0);
  const double pattern_offset_y = parse_or_default(pattern_offset_y_edit_, 0.0);
  const double pattern_offset_z = parse_or_default(pattern_offset_z_edit_, 0.0);
  const double sweep_scale = parse_or_default(sweep_scale_edit_, 1.0);
  const double pitch_offset_deg = parse_or_default(pitch_offset_edit_, 0.0);
  const double lift_offset_z = parse_or_default(lift_offset_z_edit_, 0.0);
  const auto preview_poses = apply_manual_pose_adjustments(
    latest_scoop_poses_,
    pattern_offset_x,
    pattern_offset_y,
    pattern_offset_z,
    sweep_scale,
    degrees_to_radians(pitch_offset_deg),
    lift_offset_z);
  auto make_color = [](float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  };

  visualization_msgs::msg::Marker line;
  line.header.frame_id = frame_id;
  line.header.stamp = stamp;
  line.ns = "manual_scoop_preview";
  line.id = 1;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.006;
  line.color = make_color(1.0F, 0.35F, 0.1F, 1.0F);
  for (std::size_t i = 0; i < 5; ++i) {
    const auto& shifted_pose = preview_poses[i];
    geometry_msgs::msg::Point point;
    point.x = shifted_pose.position.x;
    point.y = shifted_pose.position.y;
    point.z = shifted_pose.position.z;
    line.points.push_back(point);
  }
  markers.markers.push_back(line);

  visualization_msgs::msg::Marker points;
  points.header = line.header;
  points.ns = line.ns;
  points.id = 2;
  points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  points.action = visualization_msgs::msg::Marker::ADD;
  points.scale.x = 0.015;
  points.scale.y = 0.015;
  points.scale.z = 0.015;
  points.color = make_color(1.0F, 0.8F, 0.2F, 1.0F);
  for (std::size_t i = 0; i < 5; ++i) {
    const auto& shifted_pose = preview_poses[i];
    geometry_msgs::msg::Point point;
    point.x = shifted_pose.position.x;
    point.y = shifted_pose.position.y;
    point.z = shifted_pose.position.z;
    points.points.push_back(point);
  }
  markers.markers.push_back(points);

  const std::array<QString, 5> labels = {
    "Approach", "Contact", "Scoop", "Lift", "Transport Ready"};
  for (std::size_t i = 0; i < 5; ++i) {
    const auto& shifted_pose = preview_poses[i];
    visualization_msgs::msg::Marker text;
    text.header = line.header;
    text.ns = line.ns;
    text.id = static_cast<int>(10 + i);
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose = shifted_pose;
    text.pose.position.z += 0.03;
    text.scale.z = 0.03;
    text.color = make_color(0.95F, 0.95F, 0.95F, 1.0F);
    text.text = labels[i].toStdString();
    markers.markers.push_back(text);

    visualization_msgs::msg::Marker arrow;
    arrow.header = line.header;
    arrow.ns = line.ns;
    arrow.id = static_cast<int>(20 + i);
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose = shifted_pose;
    arrow.scale.x = 0.08;
    arrow.scale.y = 0.01;
    arrow.scale.z = 0.01;
    arrow.color = make_color(0.4F, 0.9F, 1.0F, 1.0F);
    markers.markers.push_back(arrow);
  }

  manual_preview_pub_->publish(markers);
}

bool ScoopingPanel::readParameterizedTemplateFields(
  double& x,
  double& y,
  double& z_initial,
  double& z_final,
  double& sweep_length,
  double& pitch_rad,
  double& hover_height,
  double& transport_pitch_rad,
  double& lift_height)
{
  double pitch_deg = 0.0;
  double transport_pitch_deg = 0.0;
  if (!readDoubleField(template_x_edit_, "Template x", x) ||
      !readDoubleField(template_y_edit_, "Template y", y) ||
      !readDoubleField(template_z_initial_edit_, "Template z_i", z_initial) ||
      !readDoubleField(template_z_final_edit_, "Template z_f", z_final) ||
      !readDoubleField(template_sweep_length_edit_, "Template sweep length", sweep_length) ||
      !readDoubleField(template_pitch_edit_, "Template theta", pitch_deg) ||
      !readDoubleField(template_hover_height_edit_, "Template hover height", hover_height) ||
      !readDoubleField(
        template_transport_pitch_edit_,
        "Template retain pitch",
        transport_pitch_deg) ||
      !readDoubleField(template_lift_height_edit_, "Template lift height", lift_height))
  {
    return false;
  }

  pitch_rad = degrees_to_radians(pitch_deg);
  transport_pitch_rad = degrees_to_radians(transport_pitch_deg);

  if (hover_height <= std::max(z_initial, z_final)) {
    updateStatus("Template hover height must be above z_i and z_f.", "#fca5a5");
    return false;
  }
  if (lift_height <= z_final) {
    updateStatus("Template lift height must be above z_f.", "#fca5a5");
    return false;
  }
  if (std::abs(pitch_rad) >= (kPi / 2.0) ||
      std::abs(transport_pitch_rad) >= (kPi / 2.0))
  {
    updateStatus("TCP pitch values must stay between -90 and 90 degrees.", "#fca5a5");
    return false;
  }

  return true;
}

std::vector<geometry_msgs::msg::Pose> ScoopingPanel::generateParameterizedTemplatePoses(
  double x,
  double y,
  double z_initial,
  double z_final,
  double sweep_length,
  double pitch_rad,
  double hover_height,
  double transport_pitch_rad,
  double lift_height) const
{
  auto make_orientation = [](double tcp_pitch_rad) {
    tf2::Quaternion q;
    q.setRPY(0.0, tcp_pitch_rad, 0.0);
    q.normalize();

    geometry_msgs::msg::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
    return orientation;
  };
  auto make_pose =
    [](double px,
       double py,
       double pz,
       const geometry_msgs::msg::Quaternion& orientation) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = px;
      pose.position.y = py;
      pose.position.z = pz;
      pose.orientation = orientation;
      return pose;
    };

  const auto working_orientation = make_orientation(pitch_rad);
  const auto retain_orientation = make_orientation(transport_pitch_rad);
  return {
    make_pose(x, y, hover_height, working_orientation),
    make_pose(x, y, z_initial, working_orientation),
    make_pose(x + sweep_length, y, z_final, working_orientation),
    make_pose(x + sweep_length, y, z_final, retain_orientation),
    make_pose(x + sweep_length, y, lift_height, retain_orientation),
  };
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

bool ScoopingPanel::saveNamedTargetToYaml(
  const std::string& yaml_path,
  const std::string& target_name,
  const geometry_msgs::msg::PoseStamped& pose,
  QString& error_message)
{
  YAML::Node root;
  try {
    if (std::ifstream(yaml_path).good()) {
      root = YAML::LoadFile(yaml_path);
    }
  } catch (const std::exception& ex) {
    error_message = QString("Failed to parse targets.yaml: %1").arg(ex.what());
    return false;
  }

  if (!root["targets"]) {
    root["targets"] = YAML::Node(YAML::NodeType::Map);
  }
  if (!root["targets"].IsMap()) {
    error_message = "targets.yaml has no 'targets' map.";
    return false;
  }

  YAML::Node entry;
  entry["frame_id"] = pose.header.frame_id;
  YAML::Node position;
  position["x"] = pose.pose.position.x;
  position["y"] = pose.pose.position.y;
  position["z"] = pose.pose.position.z;
  entry["position"] = position;

  YAML::Node orientation;
  orientation["x"] = pose.pose.orientation.x;
  orientation["y"] = pose.pose.orientation.y;
  orientation["z"] = pose.pose.orientation.z;
  orientation["w"] = pose.pose.orientation.w;
  entry["orientation"] = orientation;

  root["targets"][target_name] = entry;

  try {
    std::ofstream out(yaml_path);
    if (!out.is_open()) {
      error_message = QString("Failed to open targets.yaml for writing: %1")
        .arg(QString::fromStdString(yaml_path));
      return false;
    }
    out << root;
  } catch (const std::exception& ex) {
    error_message = QString("Failed to write targets.yaml: %1").arg(ex.what());
    return false;
  }

  named_targets_[target_name] = pose;
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
  const bool plan_parameterized_ready =
    plan_parameterized_client_ && plan_parameterized_client_->service_is_ready();
  const bool execute_ready = execute_client_ && execute_client_->service_is_ready();
  const bool execute_parameterized_ready =
    execute_parameterized_client_ && execute_parameterized_client_->service_is_ready();
  const bool execute_continuous_ready =
    execute_continuous_client_ && execute_continuous_client_->service_is_ready();
  const bool execute_waypoint_motion_ready =
    execute_waypoint_motion_client_ && execute_waypoint_motion_client_->service_is_ready();
  const bool scoop_params_ready =
    scooping_params_client_ && scooping_params_client_->service_is_ready();
  const bool move_to_params_ready =
    move_to_params_client_ && move_to_params_client_->service_is_ready();
  const bool record_target_params_ready =
    record_target_params_client_ && record_target_params_client_->service_is_ready();
  const bool tuning_ready = scoop_params_ready && move_to_params_ready;
  const bool parameterized_ready =
    scoop_params_ready && plan_parameterized_ready && execute_parameterized_ready;
  const bool move_ready = move_to_client_ && move_to_client_->wait_for_action_server(std::chrono::seconds(0));
  const bool scoop_edit_ready = has_scoop_poses_ && latest_scoop_poses_.size() >= 5;
  const bool targets_ready = move_to_params_ready || record_target_params_ready;

  save_button_->setEnabled(save_ready);
  load_button_->setEnabled(load_ready);
  plan_button_->setEnabled(plan_ready);
  plan_parameterized_button_->setEnabled(parameterized_ready);
  execute_button_->setEnabled(execute_ready);
  execute_parameterized_button_->setEnabled(parameterized_ready);
  execute_continuous_button_->setEnabled(execute_continuous_ready);
  execute_waypoint_motion_button_->setEnabled(execute_waypoint_motion_ready);
  preview_parameterized_button_->setEnabled(scoop_poses_cmd_pub_ != nullptr);
  move_goal_button_->setEnabled(move_ready && has_target_goal_pose_);
  apply_typed_pose_button_->setEnabled(has_target_goal_pose_);
  apply_scoop_pose_button_->setEnabled(scoop_edit_ready);
  focus_selected_scoop_button_->setEnabled(scoop_edit_ready);
  show_all_scoops_button_->setEnabled(scoop_edit_ready);
  record_target_button_->setEnabled(targets_ready);
  refresh_targets_button_->setEnabled(targets_ready);
  load_selected_target_button_->setEnabled(targets_ready && target_selector_combo_->count() > 0);
  apply_motion_tuning_button_->setEnabled(tuning_ready);
  shift_offset_positive_button_->setEnabled(tuning_ready);
  shift_offset_negative_button_->setEnabled(tuning_ready);
  zero_offset_button_->setEnabled(tuning_ready);

  service_state_label_->setText(
    QString("Services: save=%1, load=%2, plan=%3, plan_template=%4, execute=%5, execute_template=%6, continuous=%7, waypoint=%8, move_to=%9, scoop_editor=%10, tuning=%11, targets=%12")
      .arg(save_ready ? "ready" : "waiting")
      .arg(load_ready ? "ready" : "waiting")
      .arg(plan_ready ? "ready" : "waiting")
      .arg(plan_parameterized_ready ? "ready" : "waiting")
      .arg(execute_ready ? "ready" : "waiting")
      .arg(execute_parameterized_ready ? "ready" : "waiting")
      .arg(execute_continuous_ready ? "ready" : "waiting")
      .arg(execute_waypoint_motion_ready ? "ready" : "waiting")
      .arg(move_ready ? "ready" : "waiting")
      .arg(scoop_edit_ready ? "ready" : "waiting")
      .arg(tuning_ready ? "ready" : "waiting")
      .arg(targets_ready ? "ready" : "waiting"));
}
}  // namespace scooping_controller

PLUGINLIB_EXPORT_CLASS(scooping_controller::ScoopingPanel, rviz_common::Panel)
