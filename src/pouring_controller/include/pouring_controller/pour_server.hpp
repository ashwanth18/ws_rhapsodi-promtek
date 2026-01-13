#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/pour_to_target.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pluginlib/class_loader.hpp>
#include "pouring_controller/control_law.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <robot_common_msgs/msg/pour_status.hpp>
#include <mutex>
#include <vector>

namespace pouring_controller {

class PourServer : public rclcpp::Node {
public:
  using PourToTarget = robot_common_msgs::action::PourToTarget;
  using GoalHandle = rclcpp_action::ServerGoalHandle<PourToTarget>;

  explicit PourServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  rclcpp_action::Server<PourToTarget>::SharedPtr action_server_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr weight_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr vibration_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr valve_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr incline_pub_;
  rclcpp::Publisher<robot_common_msgs::msg::PourStatus>::SharedPtr pour_status_pub_;
  std::mutex data_mutex_;
  double raw_weight_{0.0};
  double filtered_weight_{0.0};
  rclcpp::Time last_weight_stamp_;
  double ema_alpha_{0.2};
  double sample_rate_hz_{20.0};
  double stale_ms_{500.0};

  // Phase thresholds
  double coarse_thresh_{0.2};
  double fine_thresh_{0.05};
  double settle_time_s_{0.3};
  int hold_within_tol_count_{5};
  double final_settle_time_s_{1.0};
  double min_delta_g_{1.0};

  // Plugin
  std::shared_ptr<pouring_controller::ControlLaw> control_;
  std::unique_ptr<pluginlib::ClassLoader<pouring_controller::ControlLaw>> loader_;

  // Joint control action client
  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr traj_client_;
  std::string traj_action_server_{"/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory"};
  std::string tilt_joint_name_{};
  std::vector<std::string> controller_joint_names_;
  std::string joint_state_topic_{"/joint_states"};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::vector<std::string> last_joint_names_;
  std::vector<double> last_joint_positions_;
  std::mutex joint_mutex_;
  rclcpp::TimerBase::SharedPtr traj_client_init_timer_;
  bool traj_client_ready_{false};
  double coarse_tilt_deg_{0.0};
  double fine_tilt_deg_{0.0};
  double trickle_tilt_deg_{0.0};
  double joint_move_time_s_{2.0};
  // Per-phase vibration raw command (0..255)
  int coarse_vibration_raw_{230};
  int settle_vibration_raw_{178};
  int fine_vibration_raw_{128};
  int trickle_vibration_raw_{76};

  void sendTiltJoint(double target_deg);
  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Methods
  void weightCb(const std_msgs::msg::Float64::SharedPtr msg);

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                          std::shared_ptr<const PourToTarget::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);
};

} // namespace pouring_controller


