#pragma once

#ifndef Q_MOC_RUN
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <robot_common_msgs/msg/cell_layout_active.hpp>
#include <robot_common_msgs/msg/pour_status.hpp>
#include <robot_common_msgs/srv/add_layout_object.hpp>
#include <robot_common_msgs/srv/diagnose_scoop_poses.hpp>
#include <robot_common_msgs/srv/list_scoop_pose_sets.hpp>
#include <robot_common_msgs/srv/load_scoop_pose_set.hpp>
#include <robot_common_msgs/srv/record_target.hpp>
#include <robot_common_msgs/srv/remove_layout_object.hpp>
#include <robot_common_msgs/srv/save_scoop_pose_set.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
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
#include <QTabWidget>
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
  void onSaveLayoutClicked();
  void onReloadLayoutClicked();
  void onAddLayoutObjectClicked();
  void onRemoveLayoutObjectClicked();
  void onCaptureTouchPointClicked();
  void onFitTouchPointsClicked();
  void onCheckReachabilityClicked();
  void onExportPosesClicked();
  void onSavePoseSetClicked();
  void onUpdatePoseSetClicked();
  void onLoadPoseSetClicked();
  void onRefreshPoseSetsClicked();
  void onPlanarModeToggled(bool checked);
  void onContainerAlphaChanged();
  void onLayoutObjectSelected(const QString& object_id);
  void onRosTimer();

private:
  using Trigger = std_srvs::srv::Trigger;
  using MoveTo = robot_common_msgs::action::MoveTo;
  using RecordTarget = robot_common_msgs::srv::RecordTarget;
  using AddLayoutObject = robot_common_msgs::srv::AddLayoutObject;
  using RemoveLayoutObject = robot_common_msgs::srv::RemoveLayoutObject;
  using DiagnoseScoopPoses = robot_common_msgs::srv::DiagnoseScoopPoses;
  using SaveScoopPoseSet = robot_common_msgs::srv::SaveScoopPoseSet;
  using ListScoopPoseSets = robot_common_msgs::srv::ListScoopPoseSets;
  using LoadScoopPoseSet = robot_common_msgs::srv::LoadScoopPoseSet;

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
  void sendSelectedScoopMarkerGoal(bool plan_only);
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
  void refreshLayoutObjectList();
  void loadCatalogModels();
  void sendLayoutTrigger(
    const QString& action_name,
    const rclcpp::Client<Trigger>::SharedPtr& client);
  void applyContainerVisualAlpha(double alpha);
  void refreshPoseSetList();
  void updatePoseSetStatus(const QString& text, const QString& color = "#e5e7eb");
  void resetMotionTuningGeometry();
  bool readMotionTuningGeometry(
    double& offset_x,
    double& offset_y,
    double& offset_z,
    double& sweep_scale,
    double& pitch_offset_deg,
    double& lift_offset_z);
  void captureBaseScoopPosesFromLatest();
  void adoptScoopPosesFromMarkerServer(const std::vector<geometry_msgs::msg::Pose>& poses);
  void syncScoopMarkersToMotionTuning();
  bool bakeMotionTuningIntoScoopMarkers(QString& detail);
  void sendSavePoseSetRequest(const QString& bake_detail, const QString& overwrite_set_id);

  QLabel* service_state_label_;
  QLabel* status_label_;
  QLabel* preview_hint_label_;
  QLabel* pour_status_label_;
  QLabel* pour_metrics_label_;
  QLabel* layout_status_label_;
  QLabel* layout_fault_label_;
  QComboBox* layout_object_combo_;
  QComboBox* catalog_model_combo_;
  QCheckBox* planar_mode_checkbox_;
  QLineEdit* layout_object_id_edit_;
  QLineEdit* container_alpha_edit_;
  QSlider* container_alpha_slider_;
  QPushButton* container_alpha_ghost_button_;
  QPushButton* container_alpha_opaque_button_;
  QPushButton* save_layout_button_;
  QPushButton* reload_layout_button_;
  QPushButton* add_layout_object_button_;
  QPushButton* remove_layout_object_button_;
  QPushButton* capture_touch_button_;
  QPushButton* fit_touch_button_;
  QPushButton* check_reachability_button_;
  QPushButton* export_poses_button_;
  QLineEdit* pose_set_note_edit_;
  QComboBox* pose_set_combo_;
  QPushButton* save_pose_set_button_;
  QPushButton* update_pose_set_button_;
  QPushButton* load_pose_set_button_;
  QPushButton* refresh_pose_sets_button_;
  QLabel* pose_set_status_label_;
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
  QPushButton* plan_selected_scoop_button_;
  QPushButton* move_selected_scoop_button_;
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
  rclcpp::Client<Trigger>::SharedPtr export_poses_client_;
  rclcpp::Client<Trigger>::SharedPtr save_layout_client_;
  rclcpp::Client<Trigger>::SharedPtr reload_layout_client_;
  rclcpp::Client<Trigger>::SharedPtr capture_touch_client_;
  rclcpp::Client<Trigger>::SharedPtr fit_touch_client_;
  rclcpp::Client<DiagnoseScoopPoses>::SharedPtr diagnose_poses_client_;
  rclcpp::Client<SaveScoopPoseSet>::SharedPtr save_pose_set_client_;
  rclcpp::Client<ListScoopPoseSets>::SharedPtr list_pose_sets_client_;
  rclcpp::Client<LoadScoopPoseSet>::SharedPtr load_pose_set_client_;
  rclcpp::Client<AddLayoutObject>::SharedPtr add_layout_object_client_;
  rclcpp::Client<RemoveLayoutObject>::SharedPtr remove_layout_object_client_;
  rclcpp::Client<RecordTarget>::SharedPtr record_target_client_;
  rclcpp::AsyncParametersClient::SharedPtr scooping_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr move_to_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr record_target_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr layout_editor_params_client_;
  rclcpp::AsyncParametersClient::SharedPtr container_marker_params_client_;
  rclcpp_action::Client<MoveTo>::SharedPtr move_to_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr scoop_poses_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_goal_sub_;
  rclcpp::Subscription<robot_common_msgs::msg::PourStatus>::SharedPtr pour_status_sub_;
  rclcpp::Subscription<robot_common_msgs::msg::CellLayoutActive>::SharedPtr layout_active_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr layout_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr layout_fault_sub_;
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
  std::vector<geometry_msgs::msg::Pose> base_scoop_poses_;
  std::map<std::string, geometry_msgs::msg::PoseStamped> named_targets_;
  std::string latest_scoop_frame_id_;
  std::string targets_yaml_path_;
  std::string active_layout_scene_yaml_;
  std::string catalog_yaml_path_;
  QString loaded_pose_set_id_;
  geometry_msgs::msg::PoseStamped latest_target_goal_pose_;
  robot_common_msgs::msg::PourStatus latest_pour_status_;
  bool has_scoop_poses_;
  bool has_base_scoop_poses_;
  bool ignore_scoop_pose_echo_;
  bool has_target_goal_pose_;
  bool has_pour_status_;
  bool layout_preview_active_;
  bool updating_pose_fields_;
  bool updating_scoop_pose_fields_;
  int current_scoop_focus_index_;
  int spin_tick_count_;
  double latest_weight_;
  double latest_vibration_;
  double latest_incline_;
};
}  // namespace scooping_controller
