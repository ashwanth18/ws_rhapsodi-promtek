#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

namespace
{

std::vector<std::string> split_names(const std::string& csv)
{
  std::vector<std::string> names;
  std::stringstream ss(csv);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(item.begin(), std::find_if(item.begin(), item.end(), [](unsigned char ch) {
      return !std::isspace(ch);
    }));
    item.erase(std::find_if(item.rbegin(), item.rend(), [](unsigned char ch) {
      return !std::isspace(ch);
    }).base(), item.end());
    if (!item.empty()) {
      names.push_back(item);
    }
  }
  return names;
}

double deg_to_rad(double deg)
{
  return deg * M_PI / 180.0;
}

class InclineControlNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  InclineControlNode()
  : Node("incline_control_node")
  {
    declare_parameter<std::string>("incline_topic", "/incline_control");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("traj_action_server", "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory");
    declare_parameter<std::string>("tilt_joint_name", "joint_5");
    declare_parameter<std::string>("controller_joint_names", "joint_1,joint_2,joint_3,joint_4,joint_5,joint_6");
    declare_parameter<double>("max_incline_deg", 20.0);
    declare_parameter<double>("min_incline_deg", 0.0);
    declare_parameter<double>("incline_direction", 1.0);
    declare_parameter<double>("joint_move_time_s", 0.5);
    declare_parameter<double>("command_deadband_deg", 0.25);
    declare_parameter<double>("min_command_interval_s", 0.25);

    incline_topic_ = get_parameter("incline_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    traj_action_server_ = get_parameter("traj_action_server").as_string();
    tilt_joint_name_ = get_parameter("tilt_joint_name").as_string();
    controller_joint_names_ = split_names(get_parameter("controller_joint_names").as_string());
    min_incline_deg_ = get_parameter("min_incline_deg").as_double();
    max_incline_deg_ = get_parameter("max_incline_deg").as_double();
    incline_direction_ = get_parameter("incline_direction").as_double() >= 0.0 ? 1.0 : -1.0;
    joint_move_time_s_ = get_parameter("joint_move_time_s").as_double();
    command_deadband_deg_ = get_parameter("command_deadband_deg").as_double();
    min_command_interval_s_ = get_parameter("min_command_interval_s").as_double();

    if (tilt_joint_name_.empty()) {
      RCLCPP_WARN(get_logger(), "tilt_joint_name is empty; incline commands will be ignored");
    }
    if (controller_joint_names_.empty()) {
      controller_joint_names_.push_back(tilt_joint_name_);
    }

    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, traj_action_server_);
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(joint_mutex_);
        joint_positions_.clear();
        for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          joint_positions_[msg->name[i]] = msg->position[i];
        }
        has_joint_state_ = true;
      });

    incline_sub_ = create_subscription<std_msgs::msg::Float64>(
      incline_topic_,
      10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        handle_incline_command(msg->data);
      });

    RCLCPP_INFO(
      get_logger(),
      "InclineControlNode: topic=%s joint=%s action=%s max=%.2fdeg direction=%.0f",
      incline_topic_.c_str(),
      tilt_joint_name_.c_str(),
      traj_action_server_.c_str(),
      max_incline_deg_,
      incline_direction_);
  }

private:
  void handle_incline_command(double requested_deg)
  {
    if (tilt_joint_name_.empty()) {
      return;
    }
    const double lo = std::min(min_incline_deg_, max_incline_deg_);
    const double hi = std::max(min_incline_deg_, max_incline_deg_);
    const double incline_deg = std::clamp(requested_deg, lo, hi);
    const auto now = this->now();

    if (last_command_valid_ && std::abs(incline_deg - last_command_deg_) < command_deadband_deg_) {
      return;
    }
    if (last_command_time_.nanoseconds() != 0 &&
        (now - last_command_time_).seconds() < min_command_interval_s_) {
      return;
    }

    std::unordered_map<std::string, double> current_positions;
    {
      std::lock_guard<std::mutex> lk(joint_mutex_);
      if (!has_joint_state_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No joint state yet; ignoring incline command");
        return;
      }
      current_positions = joint_positions_;
    }

    const auto tilt_it = current_positions.find(tilt_joint_name_);
    if (tilt_it == current_positions.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Joint state does not contain tilt joint '%s'",
        tilt_joint_name_.c_str());
      return;
    }

    if (!base_initialized_) {
      base_joint_position_rad_ = tilt_it->second;
      base_initialized_ = true;
      RCLCPP_INFO(
        get_logger(),
        "InclineControlNode: captured base %s=%.4frad",
        tilt_joint_name_.c_str(),
        base_joint_position_rad_);
    }

    const double target_joint_rad =
      base_joint_position_rad_ + incline_direction_ * deg_to_rad(incline_deg);
    if (!send_trajectory(current_positions, target_joint_rad)) {
      return;
    }

    last_command_deg_ = incline_deg;
    last_command_time_ = now;
    last_command_valid_ = true;
    RCLCPP_INFO(
      get_logger(),
      "InclineControlNode: commanded incline=%.3fdeg target_%s=%.4frad",
      incline_deg,
      tilt_joint_name_.c_str(),
      target_joint_rad);

    if (std::abs(incline_deg) < command_deadband_deg_) {
      base_initialized_ = false;
    }
  }

  bool send_trajectory(
    const std::unordered_map<std::string, double>& current_positions,
    double target_joint_rad)
  {
    if (!action_client_->wait_for_action_server(250ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Trajectory action server '%s' unavailable",
        traj_action_server_.c_str());
      return false;
    }

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = controller_joint_names_;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(controller_joint_names_.size());
    for (const auto& name : controller_joint_names_) {
      const auto it = current_positions.find(name);
      if (it == current_positions.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Joint state missing controller joint '%s'",
          name.c_str());
        return false;
      }
      point.positions.push_back(name == tilt_joint_name_ ? target_joint_rad : it->second);
    }

    const double move_time = std::max(0.1, joint_move_time_s_);
    point.time_from_start = rclcpp::Duration::from_seconds(move_time);
    goal.trajectory.points.push_back(point);
    action_client_->async_send_goal(goal);
    return true;
  }

  std::string incline_topic_;
  std::string joint_state_topic_;
  std::string traj_action_server_;
  std::string tilt_joint_name_;
  std::vector<std::string> controller_joint_names_;
  double min_incline_deg_{0.0};
  double max_incline_deg_{20.0};
  double incline_direction_{1.0};
  double joint_move_time_s_{0.5};
  double command_deadband_deg_{0.25};
  double min_command_interval_s_{0.25};

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr incline_sub_;
  std::mutex joint_mutex_;
  std::unordered_map<std::string, double> joint_positions_;
  bool has_joint_state_{false};
  bool base_initialized_{false};
  double base_joint_position_rad_{0.0};
  bool last_command_valid_{false};
  double last_command_deg_{0.0};
  rclcpp::Time last_command_time_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InclineControlNode>());
  rclcpp::shutdown();
  return 0;
}
