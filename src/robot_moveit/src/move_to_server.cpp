#include "robot_moveit/move_to_server.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/trajectory_processing/trajectory_tools.hpp>
#include <algorithm>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>


namespace robot_moveit {

MoveToServer::MoveToServer(const rclcpp::NodeOptions & options)
: rclcpp::Node("move_to_server", options)
{
  this->declare_parameter<std::string>("planning_group", "arm");
  this->declare_parameter<std::string>("targets_yaml", "");
  this->declare_parameter<double>("velocity_scaling", 1.0);
  this->declare_parameter<double>("acceleration_scaling", 1.0);
  this->declare_parameter<double>("planning_time", 5.0);
  this->declare_parameter<bool>("cartesian_avoid_collisions", false);
  this->declare_parameter<double>("pose_success_pos_tol_m", 0.01);
  this->declare_parameter<double>("pose_success_ang_tol_rad", 0.05);
  // Defer heavy init to avoid shared_from_this in constructor
  init_timer_ = this->create_wall_timer(std::chrono::milliseconds(0), std::bind(&MoveToServer::deferred_init, this));
}

void MoveToServer::deferred_init()
{
  init_timer_->cancel();
  auto group = this->get_parameter("planning_group").as_string();
  mgi_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), group);
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
      poses_from_names.push_back(itn->second.pose);
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

  if (have_waypoints) {
    // Build combined pose list: names first, then literal waypoints
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(poses_from_names.size() + goal->waypoints.size());
    poses.insert(poses.end(), poses_from_names.begin(), poses_from_names.end());
    for (const auto & ps : goal->waypoints) {
      if (ps.header.frame_id != "base_link") {
        RCLCPP_WARN(this->get_logger(), "Waypoint frame_id '%s' != 'base_link'; ensure transforms are handled upstream", ps.header.frame_id.c_str());
      }
      poses.push_back(ps.pose);
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
      auto exec_res = mgi_->execute(plan);
      if (exec_res != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
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
        auto exec_res = mgi_->execute(plan);
        if (exec_res != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
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
          exec_res = mgi_->execute(plan);
          if (exec_res != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
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
    auto exec_res = mgi_->execute(plan);
    if (exec_res != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
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
      auto exec_res = mgi_->execute(plan);
      if (exec_res == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
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


