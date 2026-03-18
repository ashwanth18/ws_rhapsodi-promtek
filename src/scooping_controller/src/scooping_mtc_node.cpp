#include "scooping_controller/container_collision_objects.hpp"

#include <algorithm>
#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/task_constructor/storage.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/modify_planning_scene.h>
#include <moveit/task_constructor/task.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace mtc = moveit::task_constructor;

class ScoopingMtcNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using MoveTo = robot_common_msgs::action::MoveTo;

  ScoopingMtcNode()
  : Node("scooping_mtc_node")
  {
    this->declare_parameter<std::string>("group", "arm");
    this->declare_parameter<std::string>("ik_frame", "tool_link");
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<std::string>(
      "trajectory_controller",
      "niryo_robot_follow_joint_trajectory_controller");
    this->declare_parameter<std::string>(
      "trajectory_action_server",
      "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory");
    this->declare_parameter<double>("planning_time", 5.0);
    this->declare_parameter<double>("current_state_timeout", 20.0);
    this->declare_parameter<double>("velocity_scaling", 0.2);
    this->declare_parameter<double>("acceleration_scaling", 0.2);
    this->declare_parameter<double>("pattern_offset_y", 0.0);
    this->declare_parameter<double>("cartesian_velocity_scaling", 0.15);
    this->declare_parameter<double>("cartesian_acceleration_scaling", 0.15);
    this->declare_parameter<double>("cartesian_step", 0.005);
    this->declare_parameter<bool>("cartesian_avoid_collisions", false);
    this->declare_parameter<double>("orientation_constraint_tolerance", 0.12);
    scooping_controller::declare_container_scene_parameters(*this);
    container_scene_specs_ = scooping_controller::load_container_scene_specs(*this);

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/scoop_poses",
      rclcpp::QoS(1).transient_local(),
      std::bind(&ScoopingMtcNode::handle_pose_array, this, std::placeholders::_1));

    plan_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/plan_scoop",
      std::bind(&ScoopingMtcNode::handle_plan, this, std::placeholders::_1, std::placeholders::_2));
    execute_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/execute_scoop",
      std::bind(&ScoopingMtcNode::handle_execute, this, std::placeholders::_1, std::placeholders::_2));
    execute_continuous_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/execute_scoop_continuous",
      std::bind(&ScoopingMtcNode::handle_execute_continuous, this, std::placeholders::_1, std::placeholders::_2));
    execute_waypoint_motion_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/execute_scoop_waypoint_motion",
      std::bind(&ScoopingMtcNode::handle_execute_waypoint_motion, this, std::placeholders::_1, std::placeholders::_2));
    action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, trajectory_action_server(), action_callback_group_);
    move_to_client_ = rclcpp_action::create_client<MoveTo>(
      this, "/move_to", action_callback_group_);
    RCLCPP_INFO(this->get_logger(), "Scooping MTC executor ready for planning group '%s'", group().c_str());
  }

private:
  void handle_pose_array(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    std::scoped_lock<std::mutex> lock(poses_mutex_);
    latest_poses_ = msg->poses;
    latest_frame_id_ = msg->header.frame_id.empty() ? frame_id() : msg->header.frame_id;
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Received %zu scoop poses in frame %s",
      latest_poses_.size(), latest_frame_id_.c_str());
  }

  void handle_plan(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    plan_only(response);
  }

  void handle_execute(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    execute_mode(response, false);
  }

  void handle_execute_continuous(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    execute_mode(response, true);
  }

  void handle_execute_waypoint_motion(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::vector<geometry_msgs::msg::Pose> poses_copy;
    std::string frame_id_copy;
    {
      std::scoped_lock<std::mutex> lock(poses_mutex_);
      poses_copy = latest_poses_;
      frame_id_copy = latest_frame_id_;
    }

    if (poses_copy.size() != 5) {
      response->success = false;
      response->message =
        "Expected exactly 5 scoop poses (approach, contact, scoop, lift, transport_ready)";
      return;
    }

    apply_pattern_offset(poses_copy);
    RCLCPP_INFO(
      this->get_logger(),
      "Executing scoop as waypoint-style hybrid motion with pattern_offset_y=%.4f m",
      this->get_parameter("pattern_offset_y").as_double());
    response->success = execute_waypoint_style_sequence(poses_copy, frame_id_copy, response->message);
    if (response->success && response->message.empty()) {
      response->message =
        "Waypoint-style scoop executed: MoveTo(approach) + waypoint path(contact,scoop,lift) + MoveTo(transport_ready)";
    }
  }

  void plan_only(
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::vector<geometry_msgs::msg::Pose> poses_copy;
    std::string frame_id_copy;
    {
      std::scoped_lock<std::mutex> lock(poses_mutex_);
      poses_copy = latest_poses_;
      frame_id_copy = latest_frame_id_;
    }

    if (poses_copy.size() != 5) {
      response->success = false;
      response->message =
        "Expected exactly 5 scoop poses (approach, contact, scoop, lift, transport_ready)";
      return;
    }

    apply_pattern_offset(poses_copy);

    try {
      auto task = create_task(poses_copy, frame_id_copy);
      RCLCPP_INFO(
        this->get_logger(),
        "Planning MTC scooping task with pattern_offset_y=%.4f m",
        this->get_parameter("pattern_offset_y").as_double());
      const auto planning_result = task.plan(1);
      if (!static_cast<bool>(planning_result) || task.solutions().empty()) {
        std::ostringstream failure_stream;
        task.explainFailure(failure_stream);
        response->success = false;
        response->message = failure_stream.str().empty()
          ? "MTC planning failed"
          : failure_stream.str();
        return;
      }

      const auto& solution = *task.solutions().front();
      task.introspection().publishSolution(solution);
      response->success = true;
      response->message = "Scooping task planned. Review Motion Planning Tasks before executing.";
    } catch (const mtc::InitStageException& ex) {
      std::ostringstream failure_stream;
      failure_stream << ex;
      response->success = false;
      response->message = failure_stream.str();
      RCLCPP_ERROR(this->get_logger(), "Scooping task init failed: %s", response->message.c_str());
    } catch (const std::exception& ex) {
      response->success = false;
      response->message = ex.what();
      RCLCPP_ERROR(this->get_logger(), "Scooping task failed: %s", ex.what());
    }
  }

  void execute_mode(
    std::shared_ptr<std_srvs::srv::Trigger::Response> response,
    bool continuous)
  {
    std::vector<geometry_msgs::msg::Pose> poses_copy;
    std::string frame_id_copy;
    {
      std::scoped_lock<std::mutex> lock(poses_mutex_);
      poses_copy = latest_poses_;
      frame_id_copy = latest_frame_id_;
    }

    if (poses_copy.size() != 5) {
      response->success = false;
      response->message =
        "Expected exactly 5 scoop poses (approach, contact, scoop, lift, transport_ready)";
      return;
    }

    apply_pattern_offset(poses_copy);

    if (continuous) {
      RCLCPP_INFO(
        this->get_logger(),
        "Executing scoop as one continuous controller goal with pattern_offset_y=%.4f m",
        this->get_parameter("pattern_offset_y").as_double());
      response->success = execute_continuous_sequence(poses_copy, frame_id_copy, response->message);
      if (response->success && response->message.empty()) {
        response->message = "Scooping task executed as one continuous goal";
      }
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Executing scoop as sequential MTC stages with pattern_offset_y=%.4f m",
        this->get_parameter("pattern_offset_y").as_double());
      response->success = execute_stage_sequence(poses_copy, frame_id_copy, response->message);
      if (response->success && response->message.empty()) {
        response->message = "Scooping task executed";
      }
    }
  }

  void apply_pattern_offset(std::vector<geometry_msgs::msg::Pose>& poses) const
  {
    const double pattern_offset_y = this->get_parameter("pattern_offset_y").as_double();
    if (pattern_offset_y == 0.0) {
      return;
    }

    for (auto& pose : poses) {
      pose.position.y += pattern_offset_y;
    }
  }

  mtc::Task create_task(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg)
  {
    mtc::Task task;
    task.setName("manual scooping");
    task.loadRobotModel(shared_from_this());
    task.setProperty("group", group());
    task.setProperty("ik_frame", ik_frame());

    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    pipeline->setTimeout(this->get_parameter("planning_time").as_double());
    pipeline->setMaxVelocityScalingFactor(this->get_parameter("velocity_scaling").as_double());
    pipeline->setMaxAccelerationScalingFactor(this->get_parameter("acceleration_scaling").as_double());

    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state->setTimeout(this->get_parameter("current_state_timeout").as_double());
    task.add(std::move(current_state));
    task.add(make_container_collision_stage());
    task.add(make_move_to_stage("approach", pipeline, poses[0], frame_id_msg));
    task.add(make_move_to_stage("contact", pipeline, poses[1], frame_id_msg));
    task.add(make_move_to_stage("scoop", pipeline, poses[2], frame_id_msg));
    task.add(make_move_to_stage("lift", pipeline, poses[3], frame_id_msg));
    task.add(make_move_to_stage("transport_ready", pipeline, poses[4], frame_id_msg));
    return task;
  }

  mtc::solvers::PlannerInterfacePtr make_pipeline_planner()
  {
    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(shared_from_this());
    pipeline->setTimeout(this->get_parameter("planning_time").as_double());
    pipeline->setMaxVelocityScalingFactor(this->get_parameter("velocity_scaling").as_double());
    pipeline->setMaxAccelerationScalingFactor(this->get_parameter("acceleration_scaling").as_double());
    return pipeline;
  }

  mtc::Task create_single_stage_task(
    std::unique_ptr<mtc::Stage> stage,
    const planning_scene::PlanningScenePtr& start_scene = nullptr)
  {
    mtc::Task task;
    task.setName(stage->name());
    task.loadRobotModel(shared_from_this());
    task.setProperty("group", group());
    task.setProperty("ik_frame", ik_frame());

    if (start_scene) {
      task.add(std::make_unique<mtc::stages::FixedState>("fixed state", start_scene));
    } else {
      auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
      current_state->setTimeout(this->get_parameter("current_state_timeout").as_double());
      task.add(std::move(current_state));
      task.add(make_container_collision_stage());
    }
    task.add(std::move(stage));
    return task;
  }

  mtc::Stage::pointer make_container_collision_stage() const
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("add container collisions");
    for (const auto& object : scooping_controller::make_container_collision_objects(
           frame_id(), container_scene_specs_))
    {
      stage->addObject(object);
    }
    return stage;
  }

  mtc::Stage::pointer make_move_to_stage(
    const std::string& name,
    const mtc::solvers::PlannerInterfacePtr& planner,
    const geometry_msgs::msg::Pose& pose,
    const std::string& frame_id_msg) const
  {
    auto stage = std::make_unique<mtc::stages::MoveTo>(name, planner);
    mtc::TrajectoryExecutionInfo execution_info;
    execution_info.controller_names = {trajectory_controller()};
    stage->setGroup(group());
    stage->setIKFrame(ik_frame());
    stage->setTimeout(this->get_parameter("planning_time").as_double());
    stage->setGoal(make_pose_stamped(pose, frame_id_msg));
    stage->setTrajectoryExecutionInfo(execution_info);
    return stage;
  }

  geometry_msgs::msg::PoseStamped make_pose_stamped(
    const geometry_msgs::msg::Pose& pose,
    const std::string& frame_id_msg) const
  {
    geometry_msgs::msg::PoseStamped target;
    target.header.frame_id = frame_id_msg.empty() ? frame_id() : frame_id_msg;
    target.pose = pose;
    return target;
  }

  std::string group() const
  {
    return this->get_parameter("group").as_string();
  }

  std::string ik_frame() const
  {
    return this->get_parameter("ik_frame").as_string();
  }

  std::string frame_id() const
  {
    return this->get_parameter("frame_id").as_string();
  }

  std::string trajectory_action_server() const
  {
    return this->get_parameter("trajectory_action_server").as_string();
  }

  std::string trajectory_controller() const
  {
    return this->get_parameter("trajectory_controller").as_string();
  }

  template <typename FutureT>
  bool wait_for_future(
    FutureT& future,
    const std::chrono::steady_clock::duration& timeout) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        return true;
      }
    }
    return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }

  bool execute_move_to_goal(MoveTo::Goal goal, std::string& message) const
  {
    if (!move_to_client_->wait_for_action_server(std::chrono::seconds(5))) {
      message = "MoveTo action server unavailable";
      return false;
    }

    auto send_future = move_to_client_->async_send_goal(goal);
    if (!wait_for_future(send_future, std::chrono::seconds(10))) {
      message = "Timed out sending MoveTo waypoint-style goal";
      return false;
    }

    auto goal_handle = send_future.get();
    if (!goal_handle) {
      message = "MoveTo action server rejected waypoint-style goal";
      return false;
    }

    auto result_future = move_to_client_->async_get_result(goal_handle);
    if (!wait_for_future(result_future, std::chrono::seconds(60))) {
      (void)move_to_client_->async_cancel_goal(goal_handle);
      message = "Timed out executing MoveTo waypoint-style goal";
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result) {
      message = "MoveTo waypoint-style goal was aborted";
      return false;
    }

    if (!wrapped_result.result->success) {
      message = wrapped_result.result->message.empty()
        ? "MoveTo waypoint-style goal failed"
        : wrapped_result.result->message;
      return false;
    }

    return true;
  }

  bool execute_solution_direct(
    const mtc::SolutionBase& solution,
    std::string& message,
    std::size_t min_required_segments = 1)
  {
    moveit_task_constructor_msgs::msg::Solution solution_msg;
    solution.toMsg(solution_msg);

    if (solution_msg.sub_trajectory.empty()) {
      message = "MTC produced no executable joint trajectories";
      return false;
    }

    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
      message = "Trajectory action server unavailable: " + trajectory_action_server();
      return false;
    }

    std::size_t executed_segments = 0;
    for (std::size_t i = 0; i < solution_msg.sub_trajectory.size(); ++i) {
      const auto& sub_trajectory = solution_msg.sub_trajectory[i];
      auto joint_trajectory = sub_trajectory.trajectory.joint_trajectory;
      if (joint_trajectory.points.empty()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Skipping scoop segment %zu/%zu because it has no joint trajectory points (stage_id=%u planner=%s comment=%s)",
          i + 1,
          solution_msg.sub_trajectory.size(),
          sub_trajectory.info.stage_id,
          sub_trajectory.info.planner_id.c_str(),
          sub_trajectory.info.comment.c_str());
        continue;
      }

      const double segment_duration =
        static_cast<double>(joint_trajectory.points.back().time_from_start.sec) +
        1e-9 * static_cast<double>(joint_trajectory.points.back().time_from_start.nanosec);

      joint_trajectory.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);

      FollowJointTrajectory::Goal goal;
      goal.trajectory = joint_trajectory;

      RCLCPP_INFO(
        this->get_logger(),
        "Executing scoop segment %zu/%zu with %zu points over %.2fs",
        i + 1,
        solution_msg.sub_trajectory.size(),
        joint_trajectory.points.size(),
        segment_duration);

      auto send_future = trajectory_client_->async_send_goal(goal);
      if (!wait_for_future(send_future, std::chrono::seconds(10))) {
        message = "Timed out sending segment " + std::to_string(i + 1) + " to trajectory controller";
        return false;
      }

      auto goal_handle = send_future.get();
      if (!goal_handle) {
        message = "Trajectory controller rejected segment " + std::to_string(i + 1);
        return false;
      }

      auto result_future = trajectory_client_->async_get_result(goal_handle);
      const auto result_timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(std::max(10.0, segment_duration * 3.0 + 5.0)));
      if (!wait_for_future(result_future, result_timeout)) {
        (void)trajectory_client_->async_cancel_goal(goal_handle);
        message = "Timed out executing segment " + std::to_string(i + 1);
        return false;
      }

      const auto wrapped_result = result_future.get();
      if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result) {
        message = "Trajectory execution aborted for segment " + std::to_string(i + 1);
        return false;
      }

      if (wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
        message = wrapped_result.result->error_string.empty()
          ? "Trajectory controller reported failure on segment " + std::to_string(i + 1)
          : wrapped_result.result->error_string;
        return false;
      }

      ++executed_segments;
    }

    if (executed_segments < min_required_segments) {
      std::ostringstream ss;
      ss << "Expected at least " << min_required_segments << " executable motion segments, but got "
         << executed_segments;
      message = ss.str();
      return false;
    }

    return true;
  }

  bool execute_stage_sequence(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg,
    std::string& message)
  {
    struct StageSpec
    {
      std::string name;
      std::function<std::unique_ptr<mtc::Stage>()> make_stage;
    };

    std::vector<StageSpec> stages;
    stages.push_back(
      StageSpec{
        "approach",
        [this, &poses, &frame_id_msg]() {
          return make_move_to_stage("approach", make_pipeline_planner(), poses[0], frame_id_msg);
        }});
    stages.push_back(
      StageSpec{
        "contact",
        [this, &poses, &frame_id_msg]() { return make_move_to_stage("contact", make_pipeline_planner(), poses[1], frame_id_msg); }});
    stages.push_back(
      StageSpec{
        "scoop",
        [this, &poses, &frame_id_msg]() { return make_move_to_stage("scoop", make_pipeline_planner(), poses[2], frame_id_msg); }});
    stages.push_back(
      StageSpec{
        "lift",
        [this, &poses, &frame_id_msg]() { return make_move_to_stage("lift", make_pipeline_planner(), poses[3], frame_id_msg); }});
    stages.push_back(
      StageSpec{
        "transport_ready",
        [this, &poses, &frame_id_msg]() {
          return make_move_to_stage("transport_ready", make_pipeline_planner(), poses[4], frame_id_msg);
        }});

    for (auto& stage_spec : stages) {
      auto task = create_single_stage_task(stage_spec.make_stage());
      RCLCPP_INFO(this->get_logger(), "Planning scoop stage '%s'", stage_spec.name.c_str());
      const auto planning_result = task.plan(1);
      if (!static_cast<bool>(planning_result) || task.solutions().empty()) {
        std::ostringstream failure_stream;
        task.explainFailure(failure_stream);
        message = "Planning failed for stage '" + stage_spec.name + "'";
        if (!failure_stream.str().empty()) {
          message += ": " + failure_stream.str();
        }
        return false;
      }

      const auto& solution = *task.solutions().front();
      task.introspection().publishSolution(solution);

      RCLCPP_INFO(this->get_logger(), "Executing scoop stage '%s'", stage_spec.name.c_str());
      if (!execute_solution_direct(solution, message, 1)) {
        message = "Execution failed for stage '" + stage_spec.name + "': " + message;
        return false;
      }
    }

    return true;
  }

  bool extract_first_motion_trajectory(
    const mtc::SolutionBase& solution,
    trajectory_msgs::msg::JointTrajectory& joint_trajectory,
    std::string& message) const
  {
    moveit_task_constructor_msgs::msg::Solution solution_msg;
    solution.toMsg(solution_msg);

    for (std::size_t i = 0; i < solution_msg.sub_trajectory.size(); ++i) {
      const auto& sub_trajectory = solution_msg.sub_trajectory[i];
      if (!sub_trajectory.trajectory.joint_trajectory.points.empty()) {
        joint_trajectory = sub_trajectory.trajectory.joint_trajectory;
        return true;
      }
    }

    message = "No executable joint trajectory found in planned stage solution";
    return false;
  }

  bool extract_combined_motion_trajectory(
    const mtc::SolutionBase& solution,
    trajectory_msgs::msg::JointTrajectory& joint_trajectory,
    std::string& message) const
  {
    moveit_task_constructor_msgs::msg::Solution solution_msg;
    solution.toMsg(solution_msg);

    joint_trajectory = trajectory_msgs::msg::JointTrajectory{};
    bool found_motion = false;
    for (const auto& sub_trajectory : solution_msg.sub_trajectory) {
      const auto& current = sub_trajectory.trajectory.joint_trajectory;
      if (current.points.empty()) {
        continue;
      }
      found_motion = true;
      if (!append_joint_trajectory(current, joint_trajectory, message)) {
        return false;
      }
    }

    if (!found_motion || joint_trajectory.points.empty()) {
      message = "No executable joint trajectory found in planned scoop solution";
      return false;
    }
    return true;
  }

  static double duration_to_seconds(const builtin_interfaces::msg::Duration& duration)
  {
    return static_cast<double>(duration.sec) + 1e-9 * static_cast<double>(duration.nanosec);
  }

  static builtin_interfaces::msg::Duration seconds_to_duration(double seconds)
  {
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(seconds);
    duration.nanosec = static_cast<uint32_t>((seconds - static_cast<double>(duration.sec)) * 1e9);
    return duration;
  }

  bool append_joint_trajectory(
    const trajectory_msgs::msg::JointTrajectory& source,
    trajectory_msgs::msg::JointTrajectory& combined,
    std::string& message) const
  {
    if (source.points.empty()) {
      return true;
    }

    if (combined.joint_names.empty()) {
      combined = source;
      return true;
    }

    if (combined.joint_names != source.joint_names) {
      message = "Cannot concatenate stage trajectories with different joint order";
      return false;
    }

    const double time_offset = duration_to_seconds(combined.points.back().time_from_start);
    const std::size_t start_index = source.points.size() > 1 ? 1 : 0;
    for (std::size_t i = start_index; i < source.points.size(); ++i) {
      auto point = source.points[i];
      point.time_from_start = seconds_to_duration(time_offset + duration_to_seconds(point.time_from_start));
      combined.points.push_back(std::move(point));
    }
    return true;
  }

  bool execute_follow_joint_trajectory_goal(
    const trajectory_msgs::msg::JointTrajectory& joint_trajectory,
    std::string& message) const
  {
    if (joint_trajectory.points.empty()) {
      message = "Continuous goal had no points";
      return false;
    }

    if (!trajectory_client_->wait_for_action_server(std::chrono::seconds(5))) {
      message = "Trajectory action server unavailable: " + trajectory_action_server();
      return false;
    }

    auto stamped_trajectory = joint_trajectory;
    stamped_trajectory.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.1);
    const double duration = duration_to_seconds(stamped_trajectory.points.back().time_from_start);

    FollowJointTrajectory::Goal goal;
    goal.trajectory = stamped_trajectory;

    RCLCPP_INFO(
      this->get_logger(),
      "Executing continuous scoop goal with %zu points over %.2fs",
      stamped_trajectory.points.size(),
      duration);

    auto send_future = trajectory_client_->async_send_goal(goal);
    if (!wait_for_future(send_future, std::chrono::seconds(10))) {
      message = "Timed out sending continuous scoop goal";
      return false;
    }

    auto goal_handle = send_future.get();
    if (!goal_handle) {
      message = "Trajectory controller rejected continuous scoop goal";
      return false;
    }

    auto result_future = trajectory_client_->async_get_result(goal_handle);
    const auto result_timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(10.0, duration * 3.0 + 5.0)));
    if (!wait_for_future(result_future, result_timeout)) {
      (void)trajectory_client_->async_cancel_goal(goal_handle);
      message = "Timed out executing continuous scoop goal";
      return false;
    }

    const auto wrapped_result = result_future.get();
    if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped_result.result) {
      message = "Continuous scoop goal was aborted";
      return false;
    }

    if (wrapped_result.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL) {
      message = wrapped_result.result->error_string.empty()
        ? "Trajectory controller reported failure for continuous scoop goal"
        : wrapped_result.result->error_string;
      return false;
    }

    return true;
  }

  bool execute_continuous_sequence(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg,
    std::string& message)
  {
    auto task = create_task(poses, frame_id_msg);
    RCLCPP_INFO(this->get_logger(), "Planning continuous scoop as one 5-stage task");
    const auto planning_result = task.plan(1);
    if (!static_cast<bool>(planning_result) || task.solutions().empty()) {
      std::ostringstream failure_stream;
      task.explainFailure(failure_stream);
      message = "Planning failed for continuous scoop task";
      if (!failure_stream.str().empty()) {
        message += ": " + failure_stream.str();
      }
      return false;
    }

    const auto& solution = *task.solutions().front();
    task.introspection().publishSolution(solution);

    trajectory_msgs::msg::JointTrajectory combined_trajectory;
    if (!extract_combined_motion_trajectory(solution, combined_trajectory, message)) {
      return false;
    }
    return execute_follow_joint_trajectory_goal(combined_trajectory, message);
  }

  bool execute_waypoint_style_sequence(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg,
    std::string& message) const
  {
    if (poses.size() != 5U) {
      message = "Waypoint-style scoop requires exactly 5 poses";
      return false;
    }

    const std::string resolved_frame = frame_id_msg.empty() ? frame_id() : frame_id_msg;

    MoveTo::Goal approach_goal;
    approach_goal.target_pose = make_pose_stamped(poses[0], resolved_frame);
    approach_goal.use_cartesian = false;
    approach_goal.velocity_scaling = static_cast<float>(this->get_parameter("velocity_scaling").as_double());
    approach_goal.acceleration_scaling = static_cast<float>(this->get_parameter("acceleration_scaling").as_double());
    if (!execute_move_to_goal(approach_goal, message)) {
      message = "Waypoint-style approach failed: " + message;
      return false;
    }

    MoveTo::Goal scoop_goal;
    scoop_goal.use_cartesian = true;
    scoop_goal.eef_step = static_cast<float>(this->get_parameter("cartesian_step").as_double());
    scoop_goal.jump_threshold = 0.0F;
    scoop_goal.velocity_scaling =
      static_cast<float>(this->get_parameter("cartesian_velocity_scaling").as_double());
    scoop_goal.acceleration_scaling =
      static_cast<float>(this->get_parameter("cartesian_acceleration_scaling").as_double());
    scoop_goal.waypoints.push_back(make_pose_stamped(poses[1], resolved_frame));
    scoop_goal.waypoints.push_back(make_pose_stamped(poses[2], resolved_frame));
    scoop_goal.waypoints.push_back(make_pose_stamped(poses[3], resolved_frame));
    if (!execute_move_to_goal(scoop_goal, message)) {
      message = "Waypoint-style contact->scoop->lift path failed: " + message;
      return false;
    }

    MoveTo::Goal transport_goal;
    transport_goal.target_pose = make_pose_stamped(poses[4], resolved_frame);
    transport_goal.use_cartesian = false;
    transport_goal.velocity_scaling = static_cast<float>(this->get_parameter("velocity_scaling").as_double());
    transport_goal.acceleration_scaling = static_cast<float>(this->get_parameter("acceleration_scaling").as_double());
    if (!execute_move_to_goal(transport_goal, message)) {
      message = "Waypoint-style transport_ready move failed: " + message;
      return false;
    }

    return true;
  }

  std::mutex poses_mutex_;
  std::vector<geometry_msgs::msg::Pose> latest_poses_;
  std::vector<scooping_controller::ContainerSceneSpec> container_scene_specs_;
  std::string latest_frame_id_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_continuous_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_waypoint_motion_srv_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp_action::Client<MoveTo>::SharedPtr move_to_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScoopingMtcNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
