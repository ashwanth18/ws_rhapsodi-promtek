#include "scooping_controller/container_collision_objects.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/robot_state.hpp>
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
#include <robot_common_msgs/msg/cell_layout_active.hpp>
#include <robot_common_msgs/srv/diagnose_scoop_poses.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace mtc = moveit::task_constructor;
namespace
{
constexpr double kPi = 3.14159265358979323846;
}  // namespace

class ScoopingMtcNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using MoveTo = robot_common_msgs::action::MoveTo;

  ScoopingMtcNode()
  : Node("scooping_mtc_node")
  {
    this->declare_parameter<std::string>("group", "arm");
    this->declare_parameter<std::string>("ik_frame", "tcp_link");
    this->declare_parameter<std::string>("frame_id", "scooping_container_frame");
    this->declare_parameter<std::string>("planning_scene_frame_id", "base_link");
    this->declare_parameter<std::string>(
      "trajectory_controller",
      "niryo_robot_follow_joint_trajectory_controller");
    this->declare_parameter<std::string>(
      "trajectory_action_server",
      "/niryo_robot_follow_joint_trajectory_controller/follow_joint_trajectory");
    this->declare_parameter<double>("planning_time", 5.0);
    this->declare_parameter<double>("current_state_timeout", 20.0);
    this->declare_parameter<std::string>("planning_pipeline", "ompl");
    this->declare_parameter<std::string>("planner_id", "");
    this->declare_parameter<double>("velocity_scaling", 0.2);
    this->declare_parameter<double>("acceleration_scaling", 0.2);
    this->declare_parameter<double>("pattern_offset_x", 0.0);
    this->declare_parameter<double>("pattern_offset_y", 0.0);
    this->declare_parameter<double>("pattern_offset_z", 0.0);
    this->declare_parameter<double>("manual_sweep_scale", 1.0);
    this->declare_parameter<double>("manual_pitch_offset_rad", 0.0);
    this->declare_parameter<double>("manual_lift_offset_z", 0.0);
    this->declare_parameter<bool>("post_lift_vibration_enabled", true);
    this->declare_parameter<double>("post_lift_vibration_duration_s", 5.0);
    this->declare_parameter<double>("post_lift_vibration_intensity", 0.75);
    this->declare_parameter<double>("post_lift_vibration_publish_rate_hz", 10.0);
    this->declare_parameter<std::string>("post_lift_vibration_topic", "/vibration/intensity");
    this->declare_parameter<double>("cartesian_velocity_scaling", 0.15);
    this->declare_parameter<double>("cartesian_acceleration_scaling", 0.15);
    this->declare_parameter<double>("cartesian_step", 0.005);
    this->declare_parameter<bool>("cartesian_avoid_collisions", false);
    this->declare_parameter<double>("orientation_constraint_tolerance", 0.12);
    this->declare_parameter<double>("template_x", -0.160);
    this->declare_parameter<double>("template_y", 0.119);
    this->declare_parameter<double>("template_z_initial", 0.06);
    this->declare_parameter<double>("template_z_final", 0.08);
    this->declare_parameter<double>("template_sweep_length", 0.10);
    this->declare_parameter<double>("template_pitch_rad", 0.4363323130);
    this->declare_parameter<double>("template_hover_height", 0.24);
    this->declare_parameter<double>("template_transport_pitch_rad", kPi / 12.0);
    this->declare_parameter<double>("template_lift_height", 0.24);
    scooping_controller::declare_container_scene_parameters(*this);
    container_scene_specs_ = scooping_controller::load_container_scene_specs(*this);
    layout_sub_ = this->create_subscription<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable(),
      [this](const robot_common_msgs::msg::CellLayoutActive::SharedPtr msg) {
        try {
          container_scene_specs_ = scooping_controller::load_container_scene_specs_from_yaml(
            msg->scene_yaml_path);
        } catch (const std::exception& ex) {
          RCLCPP_ERROR(this->get_logger(), "Could not reload layout collision specs: %s", ex.what());
        }
      });

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/scoop_poses",
      rclcpp::QoS(1).transient_local(),
      std::bind(&ScoopingMtcNode::handle_pose_array, this, std::placeholders::_1));
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::scoped_lock<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_ = msg;
      });

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    plan_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/plan_scoop",
      std::bind(&ScoopingMtcNode::handle_plan, this, std::placeholders::_1, std::placeholders::_2));
    plan_parameterized_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/plan_parameterized_scoop",
      std::bind(
        &ScoopingMtcNode::handle_plan_parameterized,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    execute_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/execute_scoop",
      std::bind(&ScoopingMtcNode::handle_execute, this, std::placeholders::_1, std::placeholders::_2));
    execute_parameterized_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/execute_parameterized_scoop",
      std::bind(
        &ScoopingMtcNode::handle_execute_parameterized,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    execute_continuous_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/execute_scoop_continuous",
      std::bind(&ScoopingMtcNode::handle_execute_continuous, this, std::placeholders::_1, std::placeholders::_2));
    execute_waypoint_motion_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/execute_scoop_waypoint_motion",
      std::bind(&ScoopingMtcNode::handle_execute_waypoint_motion, this, std::placeholders::_1, std::placeholders::_2));
    diagnose_srv_ = this->create_service<robot_common_msgs::srv::DiagnoseScoopPoses>(
      "/diagnose_scoop_poses",
      std::bind(&ScoopingMtcNode::handle_diagnose, this, std::placeholders::_1, std::placeholders::_2));
    action_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, trajectory_action_server(), action_callback_group_);
    move_to_client_ = rclcpp_action::create_client<MoveTo>(
      this, "/move_to", action_callback_group_);
    vibration_pub_ = this->create_publisher<std_msgs::msg::Float64>(
      this->get_parameter("post_lift_vibration_topic").as_string(),
      10);
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

  void handle_plan_parameterized(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    plan_parameterized(response);
  }

  void handle_execute_parameterized(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    execute_parameterized(response);
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

  void handle_diagnose(
    const std::shared_ptr<robot_common_msgs::srv::DiagnoseScoopPoses::Request> /*request*/,
    std::shared_ptr<robot_common_msgs::srv::DiagnoseScoopPoses::Response> response)
  {
    diagnose_poses(*response);
  }

  void diagnose_poses(robot_common_msgs::srv::DiagnoseScoopPoses::Response& response)
  {
    static const std::vector<std::string> kMarkerNames = {
      "approach", "contact", "scoop", "lift", "transport_ready"};

    std::vector<geometry_msgs::msg::Pose> poses_copy;
    std::string frame_id_copy;
    {
      std::scoped_lock<std::mutex> lock(poses_mutex_);
      poses_copy = latest_poses_;
      frame_id_copy = latest_frame_id_.empty() ? frame_id() : latest_frame_id_;
    }
    if (poses_copy.size() != kMarkerNames.size()) {
      response.success = false;
      response.summary =
        "Expected exactly 5 scoop poses (approach, contact, scoop, lift, transport_ready); got " +
        std::to_string(poses_copy.size());
      return;
    }

    apply_pattern_offset(poses_copy);

    robot_model_loader::RobotModelLoader loader(shared_from_this(), "robot_description");
    const auto model = loader.getModel();
    if (!model) {
      response.success = false;
      response.summary = "Failed to load robot_description for scoop diagnosis";
      return;
    }
    const auto* jmg = model->getJointModelGroup(group());
    if (!jmg) {
      response.success = false;
      response.summary = "Planning group '" + group() + "' not found in robot model";
      return;
    }

    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    for (const auto& object : scooping_controller::make_container_collision_objects(
           planning_scene_frame_id(), container_scene_specs_))
    {
      if (!scene->processCollisionObjectMsg(object)) {
        RCLCPP_WARN(
          this->get_logger(),
          "Could not add collision object '%s' to diagnose scene",
          object.id.c_str());
      }
    }

    moveit::core::RobotState seed(model);
    seed.setToDefaultValues();
    {
      std::scoped_lock<std::mutex> lock(joint_state_mutex_);
      if (latest_joint_state_) {
        seed.setVariablePositions(latest_joint_state_->name, latest_joint_state_->position);
        seed.update();
      }
    }

    int ok_count = 0;
    int no_ik_count = 0;
    int collision_count = 0;
    int other_count = 0;

    for (std::size_t i = 0; i < poses_copy.size(); ++i) {
      const auto& name = kMarkerNames[i];
      response.marker_names.push_back(name);

      geometry_msgs::msg::PoseStamped stamped_in;
      stamped_in.header.stamp = this->now();
      stamped_in.header.frame_id = frame_id_copy;
      stamped_in.pose = poses_copy[i];

      geometry_msgs::msg::PoseStamped stamped_out;
      try {
        stamped_out = tf_buffer_->transform(
          stamped_in, planning_scene_frame_id(), tf2::durationFromSec(1.0));
      } catch (const tf2::TransformException& ex) {
        response.verdicts.push_back("tf_error");
        response.details.push_back(
          std::string("TF ") + frame_id_copy + " -> " + planning_scene_frame_id() +
          " failed: " + ex.what());
        ++other_count;
        continue;
      }

      Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
      target.translation() = Eigen::Vector3d(
        stamped_out.pose.position.x,
        stamped_out.pose.position.y,
        stamped_out.pose.position.z);
      target.linear() = Eigen::Quaterniond(
        stamped_out.pose.orientation.w,
        stamped_out.pose.orientation.x,
        stamped_out.pose.orientation.y,
        stamped_out.pose.orientation.z)
                          .normalized()
                          .toRotationMatrix();

      moveit::core::RobotState state = seed;
      bool ik_ok = false;
      for (int attempt = 0; attempt < 8 && !ik_ok; ++attempt) {
        if (attempt > 0) {
          state.setToRandomPositions(jmg);
          state.update();
        }
        ik_ok = state.setFromIK(jmg, target, ik_frame(), 0.15);
        if (ik_ok) {
          state.update();
        }
      }

      if (!ik_ok) {
        response.verdicts.push_back("no_ik");
        std::ostringstream detail;
        detail << "No IK for " << ik_frame() << " at "
               << planning_scene_frame_id() << " xyz=["
               << stamped_out.pose.position.x << ", "
               << stamped_out.pose.position.y << ", "
               << stamped_out.pose.position.z
               << "] (pattern offsets applied). Pose/orientation unreachable.";
        response.details.push_back(detail.str());
        ++no_ik_count;
        RCLCPP_WARN(this->get_logger(), "[diagnose] %s: no_ik", name.c_str());
        continue;
      }

      if (!state.satisfiesBounds(jmg)) {
        response.verdicts.push_back("joint_limits");
        response.details.push_back("IK found but joint limits violated for group '" + group() + "'");
        ++other_count;
        RCLCPP_WARN(this->get_logger(), "[diagnose] %s: joint_limits", name.c_str());
        continue;
      }

      collision_detection::CollisionRequest creq;
      collision_detection::CollisionResult cres;
      creq.group_name = group();
      creq.contacts = true;
      creq.max_contacts = 8;
      scene->checkCollision(creq, cres, state);
      if (cres.collision) {
        std::ostringstream detail;
        detail << "IK ok, but goal state collides. Contacts:";
        std::size_t shown = 0;
        for (const auto& contact : cres.contacts) {
          detail << " [" << contact.first.first << "↔" << contact.first.second << "]";
          if (++shown >= 5) {
            break;
          }
        }
        if (cres.contacts.empty()) {
          detail << " (no contact pairs reported)";
        }
        response.verdicts.push_back("collision_at_goal");
        response.details.push_back(detail.str());
        ++collision_count;
        RCLCPP_WARN(this->get_logger(), "[diagnose] %s: collision_at_goal", name.c_str());
        continue;
      }

      response.verdicts.push_back("ok");
      std::ostringstream detail;
      detail << "IK ok, no collision at goal in " << planning_scene_frame_id()
             << " xyz=[" << stamped_out.pose.position.x << ", "
             << stamped_out.pose.position.y << ", "
             << stamped_out.pose.position.z << "]";
      response.details.push_back(detail.str());
      ++ok_count;
      RCLCPP_INFO(this->get_logger(), "[diagnose] %s: ok", name.c_str());
    }

    std::ostringstream summary;
    summary << "Scoop pose diagnose: ok=" << ok_count
            << " no_ik=" << no_ik_count
            << " collision_at_goal=" << collision_count
            << " other=" << other_count
            << ". pattern_offset_y="
            << this->get_parameter("pattern_offset_y").as_double() << " m.";
    for (std::size_t i = 0; i < response.marker_names.size(); ++i) {
      summary << " | " << response.marker_names[i] << "=" << response.verdicts[i];
    }
    response.summary = summary.str();
    response.success = (no_ik_count == 0 && collision_count == 0 && other_count == 0);
    RCLCPP_INFO(this->get_logger(), "%s", response.summary.c_str());
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
    if (poses.size() < 5U) {
      return;
    }

    const double pattern_offset_x = this->get_parameter("pattern_offset_x").as_double();
    const double pattern_offset_y = this->get_parameter("pattern_offset_y").as_double();
    const double pattern_offset_z = this->get_parameter("pattern_offset_z").as_double();
    const double sweep_scale = this->get_parameter("manual_sweep_scale").as_double();
    const double pitch_offset_rad =
      this->get_parameter("manual_pitch_offset_rad").as_double();
    const double lift_offset_z = this->get_parameter("manual_lift_offset_z").as_double();

    for (auto& pose : poses) {
      pose.position.x += pattern_offset_x;
      pose.position.y += pattern_offset_y;
      pose.position.z += pattern_offset_z;
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

    if (pitch_offset_rad == 0.0) {
      return;
    }

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

  bool validate_parameterized_template(std::string& message) const
  {
    const double z_initial = this->get_parameter("template_z_initial").as_double();
    const double z_final = this->get_parameter("template_z_final").as_double();
    const double hover_height = this->get_parameter("template_hover_height").as_double();
    const double lift_height = this->get_parameter("template_lift_height").as_double();
    const double pitch_rad = this->get_parameter("template_pitch_rad").as_double();
    const double transport_pitch_rad =
      this->get_parameter("template_transport_pitch_rad").as_double();

    if (hover_height <= std::max(z_initial, z_final)) {
      message = "Template hover height must be above both scoop depths";
      return false;
    }
    if (lift_height <= z_final) {
      message = "Template lift height must be above the final sweep depth";
      return false;
    }
    if (std::abs(pitch_rad) >= (kPi / 2.0)) {
      message = "Template TCP pitch must be within (-pi/2, pi/2) radians";
      return false;
    }
    if (std::abs(transport_pitch_rad) >= (kPi / 2.0)) {
      message = "Template retain TCP pitch must be within (-pi/2, pi/2) radians";
      return false;
    }
    return true;
  }

  geometry_msgs::msg::Quaternion make_template_orientation(double tcp_pitch_rad) const
  {
    tf2::Quaternion q;
    q.setRPY(0.0, tcp_pitch_rad, 0.0);
    q.normalize();

    geometry_msgs::msg::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();
    return orientation;
  }

  std::vector<geometry_msgs::msg::Pose> make_parameterized_template_poses() const
  {
    const double x = this->get_parameter("template_x").as_double();
    const double y = this->get_parameter("template_y").as_double();
    const double z_initial = this->get_parameter("template_z_initial").as_double();
    const double z_final = this->get_parameter("template_z_final").as_double();
    const double sweep_length = this->get_parameter("template_sweep_length").as_double();
    const double pitch_rad = this->get_parameter("template_pitch_rad").as_double();
    const double hover_height = this->get_parameter("template_hover_height").as_double();
    const double transport_pitch_rad =
      this->get_parameter("template_transport_pitch_rad").as_double();
    const double lift_height = this->get_parameter("template_lift_height").as_double();

    const auto working_orientation = make_template_orientation(pitch_rad);
    const auto retain_orientation = make_template_orientation(transport_pitch_rad);

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

    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(5);
    poses.push_back(make_pose(x, y, hover_height, working_orientation));
    poses.push_back(make_pose(x, y, z_initial, working_orientation));
    poses.push_back(make_pose(x + sweep_length, y, z_final, working_orientation));
    poses.push_back(make_pose(x + sweep_length, y, z_final, retain_orientation));
    poses.push_back(make_pose(x + sweep_length, y, lift_height, retain_orientation));
    apply_pattern_offset(poses);
    return poses;
  }

  void plan_parameterized(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string validation_message;
    if (!validate_parameterized_template(validation_message)) {
      response->success = false;
      response->message = validation_message;
      return;
    }

    const auto poses = make_parameterized_template_poses();
    try {
      auto task = create_parameterized_task(poses, frame_id());
      RCLCPP_INFO(
        this->get_logger(),
        "Planning parameterized scoop template with pattern_offset_y=%.4f m",
        this->get_parameter("pattern_offset_y").as_double());
      const auto planning_result = task.plan(1);
      if (!static_cast<bool>(planning_result) || task.solutions().empty()) {
        std::ostringstream failure_stream;
        task.explainFailure(failure_stream);
        response->success = false;
        response->message = failure_stream.str().empty()
          ? "Parameterized scoop planning failed"
          : failure_stream.str();
        return;
      }

      const auto& solution = *task.solutions().front();
      task.introspection().publishSolution(solution);
      response->success = true;
      response->message = "Parameterized scoop planned. Review Motion Planning Tasks before executing.";
    } catch (const mtc::InitStageException& ex) {
      std::ostringstream failure_stream;
      failure_stream << ex;
      response->success = false;
      response->message = failure_stream.str();
    } catch (const std::exception& ex) {
      response->success = false;
      response->message = ex.what();
    }
  }

  void execute_parameterized(std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::string validation_message;
    if (!validate_parameterized_template(validation_message)) {
      response->success = false;
      response->message = validation_message;
      return;
    }

    const auto poses = make_parameterized_template_poses();
    RCLCPP_INFO(
      this->get_logger(),
      "Executing parameterized scoop template with pattern_offset_y=%.4f m",
      this->get_parameter("pattern_offset_y").as_double());
    response->success = execute_parameterized_sequence(poses, frame_id(), response->message);
    if (response->success && response->message.empty()) {
      response->message = "Parameterized scoop executed";
    }
  }

  mtc::Task create_task(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg)
  {
    return create_task_with_transport_option(poses, frame_id_msg, true);
  }

  mtc::Task create_task_up_to_lift(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg)
  {
    return create_task_with_transport_option(poses, frame_id_msg, false);
  }

  mtc::Task create_task_with_transport_option(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg,
    bool include_transport_ready)
  {
    mtc::Task task;
    task.setName("manual scooping");
    task.loadRobotModel(shared_from_this());
    task.setProperty("group", group());
    task.setProperty("ik_frame", ik_frame());

    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(
      shared_from_this(),
      this->get_parameter("planning_pipeline").as_string(),
      this->get_parameter("planner_id").as_string());
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
    if (include_transport_ready) {
      task.add(make_move_to_stage("transport_ready", pipeline, poses[4], frame_id_msg));
    }
    return task;
  }

  mtc::Task create_parameterized_task(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg)
  {
    mtc::Task task;
    task.setName("parameterized scooping");
    task.loadRobotModel(shared_from_this());
    task.setProperty("group", group());
    task.setProperty("ik_frame", ik_frame());

    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(
      shared_from_this(),
      this->get_parameter("planning_pipeline").as_string(),
      this->get_parameter("planner_id").as_string());
    pipeline->setTimeout(this->get_parameter("planning_time").as_double());
    pipeline->setMaxVelocityScalingFactor(this->get_parameter("velocity_scaling").as_double());
    pipeline->setMaxAccelerationScalingFactor(this->get_parameter("acceleration_scaling").as_double());

    auto current_state = std::make_unique<mtc::stages::CurrentState>("current state");
    current_state->setTimeout(this->get_parameter("current_state_timeout").as_double());
    task.add(std::move(current_state));
    task.add(make_container_collision_stage());
    task.add(make_move_to_stage("hover_start", pipeline, poses[0], frame_id_msg));
    task.add(make_move_to_stage("plunge", pipeline, poses[1], frame_id_msg));
    task.add(make_move_to_stage("sweep_end", pipeline, poses[2], frame_id_msg));
    task.add(make_move_to_stage("retain", pipeline, poses[3], frame_id_msg));
    task.add(make_move_to_stage("lift_out", pipeline, poses[4], frame_id_msg));
    return task;
  }

  mtc::solvers::PlannerInterfacePtr make_pipeline_planner()
  {
    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(
      shared_from_this(),
      this->get_parameter("planning_pipeline").as_string(),
      this->get_parameter("planner_id").as_string());
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
           planning_scene_frame_id(), container_scene_specs_))
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

  std::string planning_scene_frame_id() const
  {
    return this->get_parameter("planning_scene_frame_id").as_string();
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
      if (stage_spec.name == "lift" && !maybe_run_post_lift_vibration(message)) {
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

  bool maybe_run_post_lift_vibration(std::string& message) const
  {
    if (!this->get_parameter("post_lift_vibration_enabled").as_bool()) {
      return true;
    }

    const double duration_s = this->get_parameter("post_lift_vibration_duration_s").as_double();
    const double intensity = std::clamp(
      this->get_parameter("post_lift_vibration_intensity").as_double(),
      0.0,
      1.0);
    const double publish_rate_hz = std::max(
      1.0,
      this->get_parameter("post_lift_vibration_publish_rate_hz").as_double());

    if (duration_s <= 0.0 || intensity <= 0.0) {
      return true;
    }
    if (!vibration_pub_) {
      message = "Post-lift vibration publisher is not available";
      return false;
    }

    std_msgs::msg::Float64 vibration_msg;
    vibration_msg.data = intensity;
    RCLCPP_INFO(
      this->get_logger(),
      "Running post-lift shake-off at intensity %.3f for %.2fs at %.1f Hz",
      intensity,
      duration_s,
      publish_rate_hz);
    const auto publish_period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    const auto end_time = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration_s);
    while (std::chrono::steady_clock::now() < end_time) {
      vibration_pub_->publish(vibration_msg);
      const auto remaining = end_time - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {
        break;
      }
      std::this_thread::sleep_for(
        std::min(publish_period, std::chrono::duration<double>(remaining)));
    }
    vibration_msg.data = 0.0;
    vibration_pub_->publish(vibration_msg);
    return true;
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
    auto task = create_task_up_to_lift(poses, frame_id_msg);
    RCLCPP_INFO(this->get_logger(), "Planning continuous scoop through lift before shake-off");
    const auto planning_result = task.plan(1);
    if (!static_cast<bool>(planning_result) || task.solutions().empty()) {
      std::ostringstream failure_stream;
      task.explainFailure(failure_stream);
      message = "Planning failed for continuous scoop pre-lift task";
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
    if (!execute_follow_joint_trajectory_goal(combined_trajectory, message)) {
      return false;
    }
    if (!maybe_run_post_lift_vibration(message)) {
      return false;
    }

    MoveTo::Goal transport_goal;
    transport_goal.target_pose = make_pose_stamped(poses[4], frame_id_msg.empty() ? frame_id() : frame_id_msg);
    transport_goal.use_cartesian = false;
    transport_goal.velocity_scaling = static_cast<float>(this->get_parameter("velocity_scaling").as_double());
    transport_goal.acceleration_scaling =
      static_cast<float>(this->get_parameter("acceleration_scaling").as_double());
    if (!execute_move_to_goal(transport_goal, message)) {
      message = "Continuous transport_ready move failed: " + message;
      return false;
    }
    return true;
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
    if (!maybe_run_post_lift_vibration(message)) {
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

  bool execute_parameterized_sequence(
    const std::vector<geometry_msgs::msg::Pose>& poses,
    const std::string& frame_id_msg,
    std::string& message) const
  {
    if (poses.size() != 5U) {
      message = "Parameterized scoop requires exactly 5 generated poses";
      return false;
    }

    const std::string resolved_frame = frame_id_msg.empty() ? frame_id() : frame_id_msg;

    MoveTo::Goal hover_goal;
    hover_goal.target_pose = make_pose_stamped(poses[0], resolved_frame);
    hover_goal.use_cartesian = false;
    hover_goal.velocity_scaling = static_cast<float>(this->get_parameter("velocity_scaling").as_double());
    hover_goal.acceleration_scaling =
      static_cast<float>(this->get_parameter("acceleration_scaling").as_double());
    if (!execute_move_to_goal(hover_goal, message)) {
      message = "Parameterized hover approach failed: " + message;
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
      message = "Parameterized plunge/sweep/retain/lift path failed: " + message;
      return false;
    }

    if (!maybe_run_post_lift_vibration(message)) {
      return false;
    }

    MoveTo::Goal transport_goal;
    transport_goal.target_pose = make_pose_stamped(poses[4], resolved_frame);
    transport_goal.use_cartesian = false;
    transport_goal.velocity_scaling = static_cast<float>(this->get_parameter("velocity_scaling").as_double());
    transport_goal.acceleration_scaling =
      static_cast<float>(this->get_parameter("acceleration_scaling").as_double());
    if (!execute_move_to_goal(transport_goal, message)) {
      message = "Parameterized transport_ready move failed: " + message;
      return false;
    }

    return true;
  }

  std::mutex poses_mutex_;
  std::vector<geometry_msgs::msg::Pose> latest_poses_;
  std::vector<scooping_controller::ContainerSceneSpec> container_scene_specs_;
  rclcpp::Subscription<robot_common_msgs::msg::CellLayoutActive>::SharedPtr layout_sub_;
  std::string latest_frame_id_;

  std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr plan_parameterized_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_parameterized_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_continuous_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_waypoint_motion_srv_;
  rclcpp::Service<robot_common_msgs::srv::DiagnoseScoopPoses>::SharedPtr diagnose_srv_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;
  rclcpp_action::Client<MoveTo>::SharedPtr move_to_client_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vibration_pub_;
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
