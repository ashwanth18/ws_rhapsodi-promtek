#pragma once

#ifndef Q_MOC_RUN
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <robot_common_msgs/msg/pour_status.hpp>
#include <robot_common_msgs/srv/record_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#endif

#include <rviz_common/panel.hpp>

#include <QComboBox>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSlider>
#include <QTimer>

#include <map>
#include <string>
#include <vector>

namespace scooping_controller
{
class ScoopingPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit ScoopingPanel(QWidget* parent = nullptr);
  ~ScoopingPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void onSaveClicked();
  void onLoadClicked();
  void onPlanClicked();
  void onExecuteClicked();
  void onExecuteContinuousClicked();
  void onExecuteWaypointMotionClicked();
  void onPreviewParameterizedClicked();
  void onPlanParameterizedClicked();
  void onExecuteParameterizedClicked();
  void onApplyMotionTuningClicked();
  void onShiftOffsetPositiveClicked();
  void onShiftOffsetNegativeClicked();
  void onZeroOffsetClicked();
  void onRefreshTargetsClicked();
  void onLoadSelectedTargetClicked();
  void onRosTimer();

private:
  using Trigger = std_srvs::srv::Trigger;
  using MoveTo = robot_common_msgs::action::MoveTo;
  using RecordTarget = robot_common_msgs::srv::RecordTarget;

  void sendRequest(
    const std::string& action_name,
    const rclcpp::Client<Trigger>::SharedPtr& client);
  void sendMoveGoal(bool plan_only = false);
  void sendRecordTarget();
  void applyTypedPose();
  void applyTypedScoopPose();
  bool populateMoveGoal(
    MoveTo::Goal& goal,
    bool plan_only,
    QString& error_message);
  void updatePoseEditorsFromGoal();
  void updateScoopEditorsFromSelection();
  void focusSelectedScoopMarker();
  void showAllScoopMarkers();
  void applyMotionTuning();
  void adjustPatternOffset(double delta);
  void previewParameterizedTemplate();
  void callParameterizedTemplateService(
    const QString& action_name,
    const rclcpp::Client<Trigger>::SharedPtr& client);
  bool applyParameterizedTemplateParameters();
  void publishParameterizedPreviewMarkers();
  void publishManualScoopPreviewMarkers();
  bool readParameterizedTemplateFields(
    double& x,
    double& y,
    double& z_initial,
    double& z_final,
    double& sweep_length,
    double& pitch_rad,
    double& hover_height,
    double& transport_pitch_rad,
    double& lift_height);
  std::vector<geometry_msgs::msg::Pose> generateParameterizedTemplatePoses(
    double x,
    double y,
    double z_initial,
    double z_final,
    double sweep_length,
    double pitch_rad,
    double hover_height,
    double transport_pitch_rad,
    double lift_height) const;
  void refreshTargetsFromYaml();
  void loadSelectedTarget();
  QString selectedPourTargetName() const;
  bool loadNamedTargetByName(const QString& target_name);
  void loadSelectedPourTarget();
  void saveSelectedPourTarget();
  void sendPourTargetGoal(bool plan_only);
  void updatePourStatusLabels();
  void publishPourPreviewMarkers();
  bool tryGetTargetsYamlPath(std::string& yaml_path);
  bool tryGetTargetsYamlPathFromClient(
    const rclcpp::AsyncParametersClient::SharedPtr& client,
    std::string& yaml_path);
  bool loadNamedTargetsFromYaml(const std::string& yaml_path, QString& error_message);
  bool saveNamedTargetToYaml(
    const std::string& yaml_path,
    const std::string& target_name,
    const geometry_msgs::msg::PoseStamped& pose,
    QString& error_message);
  bool typedPoseToStamped(geometry_msgs::msg::PoseStamped& pose, QString& error_message) const;
  bool typedEditorsToPose(
    QLineEdit* x_edit,
    QLineEdit* y_edit,
    QLineEdit* z_edit,
    QLineEdit* roll_edit,
    QLineEdit* pitch_edit,
    QLineEdit* yaw_edit,
    geometry_msgs::msg::Pose& pose,
    QString& error_message) const;
  void setEditorsFromPose(
    const geometry_msgs::msg::Pose& pose,
    QLineEdit* x_edit,
    QLineEdit* y_edit,
    QLineEdit* z_edit,
    QLineEdit* roll_edit,
    QLineEdit* pitch_edit,
    QLineEdit* yaw_edit);
  int selectedScoopMarkerIndex() const;
  void publishScoopMarkerFocus(int index);
  bool readDoubleField(QLineEdit* edit, const QString& label, double& value);
  void setLineEditValue(QLineEdit* edit, double value, int decimals);
  void updateStatus(const QString& text, const QString& color = "#e5e7eb");
  void refreshServiceState();

  QLabel* service_state_label_;
  QLabel* status_label_;
  QLabel* preview_hint_label_;
  QLabel* pour_status_label_;
  QLabel* pour_metrics_label_;
  QLineEdit* target_name_edit_;
  QComboBox* target_selector_combo_;
  QComboBox* scoop_marker_combo_;
  QComboBox* constraint_mode_combo_;
  QComboBox* planner_combo_;
  QComboBox* pour_pose_combo_;
  QLineEdit* scoop_x_edit_;
  QLineEdit* scoop_y_edit_;
  QLineEdit* scoop_z_edit_;
  QLineEdit* scoop_roll_edit_;
  QLineEdit* scoop_pitch_edit_;
  QLineEdit* scoop_yaw_edit_;
  QLineEdit* x_edit_;
  QLineEdit* y_edit_;
  QLineEdit* z_edit_;
  QLineEdit* roll_edit_;
  QLineEdit* pitch_edit_;
  QLineEdit* yaw_edit_;
  QCheckBox* constraint_test_enabled_checkbox_;
  QLineEdit* constraint_pitch_edit_;
  QLineEdit* constraint_pitch_tolerance_edit_;
  QLineEdit* constraint_roll_tolerance_edit_;
  QLineEdit* constraint_yaw_tolerance_edit_;
  QLineEdit* velocity_scale_edit_;
  QSlider* velocity_scale_slider_;
  QLineEdit* acceleration_scale_edit_;
  QSlider* acceleration_scale_slider_;
  QLineEdit* pattern_offset_x_edit_;
  QSlider* pattern_offset_x_slider_;
  QLineEdit* pattern_offset_y_edit_;
  QSlider* pattern_offset_y_slider_;
  QLineEdit* pattern_offset_z_edit_;
  QSlider* pattern_offset_z_slider_;
  QLineEdit* offset_step_edit_;
  QLineEdit* sweep_scale_edit_;
  QSlider* sweep_scale_slider_;
  QLineEdit* pitch_offset_edit_;
  QSlider* pitch_offset_slider_;
  QLineEdit* lift_offset_z_edit_;
  QSlider* lift_offset_z_slider_;
  QCheckBox* post_lift_vibration_enabled_checkbox_;
  QLineEdit* post_lift_vibration_duration_edit_;
  QSlider* post_lift_vibration_duration_slider_;
  QLineEdit* post_lift_vibration_intensity_edit_;
  QSlider* post_lift_vibration_intensity_slider_;
  QLineEdit* template_x_edit_;
  QLineEdit* template_y_edit_;
  QLineEdit* template_z_initial_edit_;
  QLineEdit* template_z_final_edit_;
  QLineEdit* template_sweep_length_edit_;
  QLineEdit* template_pitch_edit_;
  QLineEdit* template_hover_height_edit_;
  QLineEdit* template_transport_pitch_edit_;
  QLineEdit* template_lift_height_edit_;
  QPushButton* save_button_;
  QPushButton* load_button_;
  QPushButton* plan_button_;
  QPushButton* execute_button_;
  QPushButton* execute_continuous_button_;
  QPushButton* execute_waypoint_motion_button_;
  QPushButton* preview_parameterized_button_;
  QPushButton* plan_parameterized_button_;
  QPushButton* execute_parameterized_button_;
  QPushButton* move_goal_button_;
  QPushButton* plan_move_goal_button_;
  QPushButton* plan_constrained_goal_button_;
  QPushButton* apply_typed_pose_button_;
  QPushButton* apply_scoop_pose_button_;
  QPushButton* focus_selected_scoop_button_;
  QPushButton* show_all_scoops_button_;
  QPushButton* record_target_button_;
  QPushButton* refresh_targets_button_;
  QPushButton* load_selected_target_button_;
  QPushButton* load_pour_pose_button_;
  QPushButton* save_pour_pose_button_;
  QPushButton* plan_pour_pose_button_;
  QPushButton* execute_pour_pose_button_;
  QPushButton* apply_motion_tuning_button_;
  QPushButton* shift_offset_positive_button_;
  QPushButton* shift_offset_negative_button_;
  QPushButton* zero_offset_button_;
  QTimer* ros_timer_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<Trigger>::SharedPtr save_client_;
  rclcpp::Client<Trigger>::SharedPtr load_client_;
  rclcpp::Client<Trigger>::SharedPtr plan_client_;
  rclcpp::Client<Trigger>::SharedPtr execute_client_;
  rclcpp::Client<Trigger>::SharedPtr execute_continuous_client_;
  rclcpp::Client<Trigger>::SharedPtr execute_waypoint_motion_client_;
  rclcpp::Client<Trigger>::SharedPtr plan_parameterized_client_;
  rclcpp::Client<Trigger>::SharedPtr execute_parameterized_client_;
  rclcpp::Client<RecordTarget>::SharedPtr record_target_client_;
  rclcpp::AsyncParametersClient::SharedPtr scooping_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr move_to_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr record_target_params_client_;
  rclcpp_action::Client<MoveTo>::SharedPtr move_to_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr scoop_poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_goal_sub_;
  rclcpp::Subscription<robot_common_msgs::msg::PourStatus>::SharedPtr pour_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr weight_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr vibration_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr incline_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr scoop_poses_cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr parameterized_preview_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr manual_preview_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pour_preview_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr scoop_marker_focus_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_goal_cmd_pub_;
  std::vector<geometry_msgs::msg::Pose> latest_scoop_poses_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> named_targets_;
  std::string latest_scoop_frame_id_;
  std::string targets_yaml_path_;
  geometry_msgs::msg::PoseStamped latest_target_goal_pose_;
  robot_common_msgs::msg::PourStatus latest_pour_status_;
  bool has_scoop_poses_;
  bool has_target_goal_pose_;
  bool has_pour_status_;
  bool updating_pose_fields_;
  bool updating_scoop_pose_fields_;
  int current_scoop_focus_index_;
  int spin_tick_count_;
  double latest_weight_;
  double latest_vibration_;
  double latest_incline_;
};
}  // namespace scooping_controller
