#include "robot_moveit/move_to_server.hpp"

#include <algorithm>
#include <chrono>
#include <thread>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/trajectory_processing/trajectory_tools.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace robot_moveit {
namespace
{
geometry_msgs::msg::Quaternion make_yaw_only_quaternion(
  const geometry_msgs::msg::Quaternion& input)
{
  tf2::Quaternion q;
  tf2::fromMsg(input, q);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  tf2::Quaternion yaw_only;
  yaw_only.setRPY(0.0, 0.0, yaw);
  yaw_only.normalize();
  return tf2::toMsg(yaw_only);
}

void enforce_upright_pose(geometry_msgs::msg::PoseStamped& pose)
{
  pose.pose.orientation = make_yaw_only_quaternion(pose.pose.orientation);
}

void enforce_upright_pose(geometry_msgs::msg::Pose& pose)
{
  pose.orientation = make_yaw_only_quaternion(pose.orientation);
}

moveit_msgs::msg::Constraints make_upright_constraints(
  const std::string& frame_id,
  const std::string& link_name,
  const geometry_msgs::msg::Quaternion& desired_orientation,
  double roll_tolerance,
  double pitch_tolerance,
  double yaw_tolerance)
{
  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = frame_id;
  oc.link_name = link_name;
  oc.orientation = desired_orientation;
  oc.absolute_x_axis_tolerance = roll_tolerance;
  oc.absolute_y_axis_tolerance = pitch_tolerance;
  oc.absolute_z_axis_tolerance = yaw_tolerance;
  oc.weight = 1.0;

  moveit_msgs::msg::Constraints constraints;
  constraints.orientation_constraints.push_back(oc);
  return constraints;
}

struct MoveGroupCleanupGuard
{
  explicit MoveGroupCleanupGuard(moveit::planning_interface::MoveGroupInterface* move_group)
  : move_group(move_group)
  {
  }

  ~MoveGroupCleanupGuard()
  {
    if (move_group != nullptr) {
      move_group->clearPathConstraints();
      move_group->clearPoseTargets();
    }
  }

  moveit::planning_interface::MoveGroupInterface* move_group;
};
}  // namespace

MoveToServer::MoveToServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("move_to_server", options)
{
  this->declare_parameter<std::string>("planning_group", "arm");
  this->declare_parameter<std::string>("targets_yaml", "");
  this->declare_parameter<double>("velocity_scaling", 1.0);
  this->declare_parameter<double>("acceleration_scaling", 1.0);
  this->declare_parameter<double>("planning_time", 5.0);
  this->declare_parameter<bool>("cartesian_avoid_collisions", false);
  this->declare_parameter<bool>("constrain_upright", false);
  this->declare_parameter<double>("upright_roll_tolerance_rad", 0.0872665);
  this->declare_parameter<double>("upright_pitch_tolerance_rad", 0.0872665);
  this->declare_parameter<double>("upright_yaw_tolerance_rad", 3.14159265);
  this->declare_parameter<double>("pose_success_pos_tol_m", 0.01);
  this->declare_parameter<double>("pose_success_ang_tol_rad", 0.05);
  this->declare_parameter<std::string>(
    "trajectory_action_server",
    "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory");
  // Defer heavy init to avoid shared_from_this in constructor
  init_timer_ = this->create_wall_timer(std::chrono::milliseconds(0), std::bind(&MoveToServer::deferred_init, this));
}

void MoveToServer::deferred_init()
{
  init_timer_->cancel();
  auto group = this->get_parameter("planning_group").as_string();
  mgi_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group);
  trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    this,
    this->get_parameter("trajectory_action_server").as_string());
  // Initial load of named targets from YAML configuration
  load_named_targets();

  action_server_ = rclcpp_action::create_server<MoveTo>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    "move_to",
    std::bind(&MoveToServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&MoveToServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&MoveToServer::handle_accepted, this, std::placeholders::_1));
}

bool MoveToServer::load_named_targets()
{
  const std::string yaml_path = this->get_parameter("targets_yaml").as_string();
  if (yaml_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "targets_yaml parameter is empty; named targets disabled");
    return true;
  }
  try {
    named_targets_.clear();
    YAML::Node root = YAML::LoadFile(yaml_path);
    if (!root["targets"]) {
      RCLCPP_WARN(this->get_logger(), "YAML has no 'targets' key: %s", yaml_path.c_str());
      return true;
    }
    for (auto it : root["targets"]) {
      const std::string name = it.first.as<std::string>();
      const YAML::Node & node = it.second;
      if (node["joints"]) {
        // Ignore pure joint entries in MoveToServer (it consumes poses only)
        continue;
      }
      geometry_msgs::msg::PoseStamped pose;
      if (!yamlPoseToMsg(node, pose)) {
        RCLCPP_WARN(this->get_logger(), "Invalid pose for target %s", name.c_str());
        continue;
      }
      named_targets_[name] = pose;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Loaded %zu named targets from %s",
      named_targets_.size(),
      yaml_path.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse targets YAML: %s", e.what());
  }
  return true;
}


rclcpp_action::GoalResponse MoveToServer::handle_goal(const rclcpp_action::GoalUUID &,
                                                      std::shared_ptr<const MoveTo::Goal>)
{
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveToServer::handle_cancel(const std::shared_ptr<GoalHandle>)
{
  mgi_->stop();
  std::scoped_lock<std::mutex> lock(trajectory_goal_mutex_);
  if (active_trajectory_goal_) {
    (void)trajectory_client_->async_cancel_goal(active_trajectory_goal_);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveToServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&MoveToServer::execute, this, goal_handle)}.detach();
}

void MoveToServer::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveTo::Feedback>();
  auto result = std::make_shared<MoveTo::Result>();
  MoveGroupCleanupGuard cleanup_guard(mgi_.get());

  if (!goal->target_name.empty() || !goal->waypoint_names.empty()) {
    load_named_targets();
  }

  geometry_msgs::msg::PoseStamped target = goal->target_pose;
  if (!goal->target_name.empty()) {
    auto it = named_targets_.find(goal->target_name);
    if (it == named_targets_.end()) {
      result->success = false;
      result->message = "Unknown target_name";
      goal_handle->succeed(result);
      return;
    }
    target = it->second;
  }

  // Common planning context (eef link and reference frame should match robot config)
  std::string planning_frame = mgi_->getPlanningFrame();
  std::string eef_link_param;
  try {
    // Declare lazily if not declared
    if (!this->has_parameter("eef_link")) {
      this->declare_parameter<std::string>("eef_link", "");
    }
    eef_link_param = this->get_parameter("eef_link").as_string();
  } catch (const std::exception &){
    eef_link_param = "";
  }
  const std::string eef_link = eef_link_param.empty() ? mgi_->getEndEffectorLink() : eef_link_param;
  if (eef_link.empty()) {
    RCLCPP_WARN(this->get_logger(), "End-effector link unknown; relying on group tip link");
  } else {
    mgi_->setEndEffectorLink(eef_link);
  }
  mgi_->setPoseReferenceFrame(planning_frame);
  mgi_->setStartStateToCurrentState();

  // Apply planning/runtime tuning params
  // Per-goal overrides take precedence if > 0; otherwise use node defaults
  double vel = goal->velocity_scaling > 0.0f ? goal->velocity_scaling : this->get_parameter("velocity_scaling").as_double();
  double acc = goal->acceleration_scaling > 0.0f ? goal->acceleration_scaling : this->get_parameter("acceleration_scaling").as_double();
  double plan_time = this->get_parameter("planning_time").as_double();
  vel = std::max(0.0, std::min(1.0, vel));
  acc = std::max(0.0, std::min(1.0, acc));
  if (plan_time <= 0.0) plan_time = 5.0;
  mgi_->setMaxVelocityScalingFactor(vel);
  mgi_->setMaxAccelerationScalingFactor(acc);
  mgi_->setPlanningTime(plan_time);

  const bool constrain_upright =
    goal->constrain_upright || this->get_parameter("constrain_upright").as_bool();
  if (constrain_upright) {
    enforce_upright_pose(target);
    const double roll_tolerance =
      goal->upright_roll_tolerance > 0.0f ?
      static_cast<double>(goal->upright_roll_tolerance) :
      this->get_parameter("upright_roll_tolerance_rad").as_double();
    const double pitch_tolerance =
      goal->upright_pitch_tolerance > 0.0f ?
      static_cast<double>(goal->upright_pitch_tolerance) :
      this->get_parameter("upright_pitch_tolerance_rad").as_double();
    const double yaw_tolerance =
      goal->upright_yaw_tolerance > 0.0f ?
      static_cast<double>(goal->upright_yaw_tolerance) :
      this->get_parameter("upright_yaw_tolerance_rad").as_double();
    const auto constraints = make_upright_constraints(
      planning_frame,
      eef_link,
      target.pose.orientation,
      roll_tolerance,
      pitch_tolerance,
      yaw_tolerance);
    mgi_->setPathConstraints(constraints);
    RCLCPP_INFO(
      this->get_logger(),
      "Applying upright MoveTo constraint on %s (roll/pitch locked near zero)",
      eef_link.c_str());
  }

  // Decide execution mode based on waypoints + flag
  // Resolve waypoint_names (if any) into poses from the named_targets_ map.
  std::vector<geometry_msgs::msg::Pose> poses_from_names;
  if (!goal->waypoint_names.empty()) {
    poses_from_names.reserve(goal->waypoint_names.size());
    for (const auto & n : goal->waypoint_names) {
      auto itn = named_targets_.find(n);
      if (itn == named_targets_.end()) {
        result->success = false;
        result->message = std::string("Unknown waypoint name: ") + n;
        goal_handle->succeed(result);
        return;
      }
      if (itn->second.header.frame_id != "base_link") {
        RCLCPP_WARN(this->get_logger(), "Waypoint '%s' frame_id '%s' != 'base_link'", n.c_str(), itn->second.header.frame_id.c_str());
      }
      auto pose = itn->second.pose;
      if (constrain_upright) {
        enforce_upright_pose(pose);
      }
      poses_from_names.push_back(pose);
    }
  }
  const bool have_waypoints = !goal->waypoints.empty() || !poses_from_names.empty();
  const bool use_cartesian = goal->use_cartesian && (have_waypoints || !goal->target_name.empty() || (target.header.frame_id.size() > 0));

  auto within_pose_tolerance = [&](const geometry_msgs::msg::Pose& cur,
                                   const geometry_msgs::msg::Pose& tgt) -> bool {
    const double pos_tol = this->get_parameter("pose_success_pos_tol_m").as_double();
    const double ang_tol = this->get_parameter("pose_success_ang_tol_rad").as_double();
    const double dx = cur.position.x - tgt.position.x;
    const double dy = cur.position.y - tgt.position.y;
    const double dz = cur.position.z - tgt.position.z;
    const double pos_err = std::sqrt(dx*dx + dy*dy + dz*dz);
    tf2::Quaternion q_cur, q_tgt;
    tf2::fromMsg(cur.orientation, q_cur);
    tf2::fromMsg(tgt.orientation, q_tgt);
    tf2::Quaternion q_err = q_tgt.inverse() * q_cur;
    q_err.normalize();
    const double ang_err = 2.0 * std::acos(std::min(1.0, std::max(-1.0, static_cast<double>(q_err.w()))));
    return (pos_err <= pos_tol) && (ang_err <= ang_tol);
  };

  auto wait_and_sync_start_state = [&]() {
    auto current_state = mgi_->getCurrentState(0.5);
    if (!current_state) {
      RCLCPP_WARN(get_logger(), "CurrentStateMonitor timeout waiting for joint state; proceeding anyway");
    }
    mgi_->setStartStateToCurrentState();
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
  };

  auto execute_plan_direct = [&](const moveit::planning_interface::MoveGroupInterface::Plan& plan) -> bool {
    std::string execution_message;
    if (!execute_joint_trajectory(plan.trajectory.joint_trajectory, execution_message)) {
      RCLCPP_WARN(get_logger(), "Direct trajectory execution failed: %s", execution_message.c_str());
      return false;
    }
    return true;
  };

  if (have_waypoints) {
    // Build combined pose list: names first, then literal waypoints
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(poses_from_names.size() + goal->waypoints.size());
    poses.insert(poses.end(), poses_from_names.begin(), poses_from_names.end());
    for (const auto & ps : goal->waypoints) {
      if (ps.header.frame_id != "base_link") {
        RCLCPP_WARN(this->get_logger(), "Waypoint frame_id '%s' != 'base_link'; ensure transforms are handled upstream", ps.header.frame_id.c_str());
      }
      auto pose = ps.pose;
      if (constrain_upright) {
        enforce_upright_pose(pose);
      }
      poses.push_back(pose);
    }

    if (use_cartesian) {
      // Cartesian path through waypoints
      wait_and_sync_start_state();
      const double eef_step = (goal->eef_step > 0.0f) ? goal->eef_step : 0.01;        // meters/radians along path
      const double jump_threshold = (goal->jump_threshold >= 0.0f) ? goal->jump_threshold : 0.0; // 0 disables
      moveit_msgs::msg::RobotTrajectory traj;
      const bool avoid_collisions = this->get_parameter("cartesian_avoid_collisions").as_bool();
      double fraction = mgi_->computeCartesianPath(poses, eef_step, traj, avoid_collisions);
      if (fraction < 0.999) {
        result->success = false;
        result->message = "Cartesian path incomplete fraction=" + std::to_string(fraction);
        goal_handle->succeed(result);
        return;
      }
      // Time-parameterize the Cartesian trajectory to honor velocity/acceleration scaling
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      try {
        moveit::core::RobotModelConstPtr model = mgi_->getRobotModel();
        robot_trajectory::RobotTrajectory rt(model, mgi_->getName());
        moveit::core::RobotStatePtr start = mgi_->getCurrentState();
        rt.setRobotTrajectoryMsg(*start, traj);
        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        bool ok_param = totg.computeTimeStamps(rt, vel, acc);
        if (!ok_param) {
          // Fallback to Ruckig smoothing if TOTG fails
          trajectory_processing::applyRuckigSmoothing(rt, vel, acc);
        }
        rt.getRobotTrajectoryMsg(traj);
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Time parameterization failed (%s); executing raw trajectory", e.what());
      }
      plan.trajectory = traj;
      if (!execute_plan_direct(plan)) {
        // Check actual pose; treat near-target as success to avoid false aborts
        auto cur_pose = mgi_->getCurrentPose(eef_link);
        if (!within_pose_tolerance(cur_pose.pose, poses.back())) {
          result->success = false;
          result->message = "Execution failed";
          goal_handle->succeed(result);
          return;
        }
      }
      feedback->progress = 1.0f;
      goal_handle->publish_feedback(feedback);
    } else {
      // Sequential planning between waypoints (joint-space). Allow per-waypoint scaling.
      const std::size_t num_segments = poses.size();
      for (std::size_t i = 0; i < num_segments; ++i) {
        // If per-waypoint scaling provided, apply per segment
        if (i < goal->waypoint_velocity_scaling.size() && goal->waypoint_velocity_scaling[i] > 0.0f) {
          const double v = std::max(0.0, std::min(1.0, static_cast<double>(goal->waypoint_velocity_scaling[i])));
          mgi_->setMaxVelocityScalingFactor(v);
        } else {
          mgi_->setMaxVelocityScalingFactor(vel);
        }
        if (i < goal->waypoint_acceleration_scaling.size() && goal->waypoint_acceleration_scaling[i] > 0.0f) {
          const double a = std::max(0.0, std::min(1.0, static_cast<double>(goal->waypoint_acceleration_scaling[i])));
          mgi_->setMaxAccelerationScalingFactor(a);
        } else {
          mgi_->setMaxAccelerationScalingFactor(acc);
        }
        wait_and_sync_start_state();
        mgi_->setPoseTarget(poses[i]);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool ok = (mgi_->plan(plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
        if (!ok) {
          result->success = false;
          result->message = "Planning failed at segment " + std::to_string(i);
          goal_handle->succeed(result);
          return;
        }
        if (!execute_plan_direct(plan)) {
          // Retry with sync once
          RCLCPP_WARN(get_logger(), "Execute aborted at segment %zu; retrying after start-state sync", i);
          wait_and_sync_start_state();
          ok = (mgi_->plan(plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
          if (!ok) {
            result->success = false;
            result->message = "Execution failed at segment " + std::to_string(i);
            goal_handle->succeed(result);
            return;
          }
          if (!execute_plan_direct(plan)) {
            auto cur_pose = mgi_->getCurrentPose(eef_link);
            if (!within_pose_tolerance(cur_pose.pose, poses[i])) {
              result->success = false;
              result->message = "Execution failed at segment " + std::to_string(i);
              goal_handle->succeed(result);
              return;
            }
            RCLCPP_INFO(get_logger(), "Segment %zu within tolerance after aborted execute; continuing", i);
          }
        }
        feedback->progress = static_cast<float>((i + 1.0) / static_cast<double>(num_segments));
        goal_handle->publish_feedback(feedback);
      }
    }
  } else if (use_cartesian) {
    // No waypoints provided but Cartesian requested: use single target pose as one waypoint
    if (target.header.frame_id != "base_link") {
      RCLCPP_WARN(this->get_logger(), "Target frame_id '%s' != 'base_link'; ensure transforms are handled upstream", target.header.frame_id.c_str());
    }
    wait_and_sync_start_state();
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.push_back(target.pose);
    const double eef_step = (goal->eef_step > 0.0f) ? goal->eef_step : 0.01;
    const double jump_threshold = (goal->jump_threshold >= 0.0f) ? goal->jump_threshold : 0.0;
    moveit_msgs::msg::RobotTrajectory traj;
    const bool avoid_collisions = this->get_parameter("cartesian_avoid_collisions").as_bool();
    double fraction = mgi_->computeCartesianPath(poses, eef_step, traj, avoid_collisions);
    if (fraction < 0.999) {
      result->success = false;
      result->message = "Cartesian path incomplete fraction=" + std::to_string(fraction);
      goal_handle->succeed(result);
      return;
    }
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = traj;
    if (!execute_plan_direct(plan)) {
      auto cur_pose = mgi_->getCurrentPose(eef_link);
      if (!within_pose_tolerance(cur_pose.pose, poses.back())) {
        result->success = false;
        result->message = "Execution failed";
        goal_handle->succeed(result);
        return;
      }
    }
    feedback->progress = 1.0f;
    goal_handle->publish_feedback(feedback);
  } else {
    // Single-target joint-space plan with sync and retries
    int retries = 3;
    while (retries-- > 0) {
      wait_and_sync_start_state();
      mgi_->setPoseTarget(target.pose);
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool ok = (mgi_->plan(plan) == moveit_msgs::msg::MoveItErrorCodes::SUCCESS);
      if (!ok) {
        RCLCPP_WARN(get_logger(), "Plan failed; retries remaining: %d", retries);
        if (retries <= 0) {
          result->success = false;
          result->message = "Planning failed";
          goal_handle->succeed(result);
          return;
        }
        continue;
      }
      if (execute_plan_direct(plan)) {
        break;
      }
      RCLCPP_WARN(get_logger(), "Execute aborted; checking pose tolerance and retrying. Retries left: %d", retries);
      auto cur_pose = mgi_->getCurrentPose(eef_link);
      if (within_pose_tolerance(cur_pose.pose, target.pose)) {
        RCLCPP_INFO(get_logger(), "Within final pose tolerance despite execute abort; treating as success");
        break;
      }
      if (retries <= 0) {
        result->success = false;
        result->message = "Execution failed";
        goal_handle->succeed(result);
        return;
      }
    }
    feedback->progress = 1.0f;
    goal_handle->publish_feedback(feedback);
  }

  result->success = true;
  result->message = "Done";
  goal_handle->succeed(result);
}

bool MoveToServer::execute_joint_trajectory(
  const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
  std::string& message)
{
  if (joint_trajectory.points.empty()) {
    message = "Planned trajectory had no points";
    return false;
  }

  if (!trajectory_client_ || !trajectory_client_->wait_for_action_server(5s)) {
    message = "Trajectory action server unavailable: " +
      this->get_parameter("trajectory_action_server").as_string();
    return false;
  }

  auto stamped_trajectory = joint_trajectory;
  stamped_trajectory.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
  const auto& final_point = stamped_trajectory.points.back();
  const double duration =
    static_cast<double>(final_point.time_from_start.sec) +
    1e-9 * static_cast<double>(final_point.time_from_start.nanosec);

  FollowJointTrajectory::Goal goal;
  goal.trajectory = stamped_trajectory;

  auto send_future = trajectory_client_->async_send_goal(goal);
  if (send_future.wait_for(10s) != std::future_status::ready) {
    message = "Timed out sending trajectory goal";
    return false;
  }

  auto goal_handle = send_future.get();
  if (!goal_handle) {
    message = "Trajectory controller rejected goal";
    return false;
  }

  {
    std::scoped_lock<std::mutex> lock(trajectory_goal_mutex_);
    active_trajectory_goal_ = goal_handle;
  }

  auto result_future = trajectory_client_->async_get_result(goal_handle);
  const auto timeout = std::chrono::duration<double>(std::max(10.0, duration * 3.0 + 5.0));
  if (result_future.wait_for(std::chrono::duration_cast<std::chrono::steady_clock::duration>(timeout)) !=
      std::future_status::ready)
  {
    (void)trajectory_client_->async_cancel_goal(goal_handle);
    {
      std::scoped_lock<std::mutex> lock(trajectory_goal_mutex_);
      active_trajectory_goal_.reset();
    }
    message = "Timed out executing trajectory goal";
    return false;
  }

  const auto wrapped_result = result_future.get();
  {
    std::scoped_lock<std::mutex> lock(trajectory_goal_mutex_);
    active_trajectory_goal_.reset();
  }
  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result) {
    message = "Trajectory execution aborted";
    return false;
  }
  if (wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
    message = wrapped_result.result->error_string.empty() ?
      "Trajectory controller reported failure" :
      wrapped_result.result->error_string;
    return false;
  }

  return true;
}

bool MoveToServer::yamlPoseToMsg(const YAML::Node& n, geometry_msgs::msg::PoseStamped& out)
{
  if (!n["frame_id"]) return false;
  out.header.frame_id = n["frame_id"].as<std::string>();
  if (n["position"]) {
    out.pose.position.x = n["position"]["x"].as<double>();
    out.pose.position.y = n["position"]["y"].as<double>();
    out.pose.position.z = n["position"]["z"].as<double>();
  } else {
    return false;
  }
  if (n["orientation"]) {
    out.pose.orientation.x = n["orientation"]["x"].as<double>();
    out.pose.orientation.y = n["orientation"]["y"].as<double>();
    out.pose.orientation.z = n["orientation"]["z"].as<double>();
    out.pose.orientation.w = n["orientation"]["w"].as<double>();
  } else {
    out.pose.orientation.w = 1.0;
  }
  return true;
}

} // namespace robot_moveit


