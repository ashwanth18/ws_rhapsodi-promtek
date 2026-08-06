#include "robot_orchestrator/tilt_about_tcp_node.hpp"

#include <cmath>
#include <sstream>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace robot_orchestrator {

namespace {

constexpr double kDegToRad = M_PI / 180.0;

}  // namespace

BT::PortsList TiltAboutTcpNode::providedPorts()
{
  return {
    BT::InputPort<double>("degrees", 0.0, "tilt angle in degrees (ignored when restore=true)"),
    // Matches compose INCLINE_DIRECTION default (-1.0) so the pour-side tilt
    // sense is reproduced when pivoting about the TCP instead of joint_5.
    BT::InputPort<double>("direction", -1.0, "sign applied to degrees (+1 or -1)"),
    BT::InputPort<std::string>("base_frame", std::string("base_link"), "planning / TF parent frame"),
    BT::InputPort<std::string>("tcp_frame", std::string("tcp_link"), "tool tip frame held fixed"),
    BT::InputPort<std::string>(
      "axis_frame", std::string("wrist_link"), "frame whose local axis is the tilt axis"),
    BT::InputPort<double>("axis_x", 0.0, "tilt axis x in axis_frame"),
    BT::InputPort<double>("axis_y", 0.0, "tilt axis y in axis_frame"),
    BT::InputPort<double>("axis_z", 1.0, "tilt axis z in axis_frame (joint_5 = wrist z)"),
    BT::InputPort<double>("velocity_scaling", 0.4, "MoveTo velocity scaling [0,1]"),
    BT::InputPort<double>("acceleration_scaling", 0.4, "MoveTo acceleration scaling [0,1]"),
    BT::InputPort<std::string>("planner_id", std::string(), "optional MoveIt planner id"),
    BT::InputPort<bool>("restore", false, "if true, replay pose stashed under restore_key"),
    BT::InputPort<std::string>(
      "restore_key",
      std::string("lightsout_purge_pretilt_pose"),
      "blackboard key for pre-tilt PoseStamped"),
  };
}

TiltAboutTcpNode::TiltAboutTcpNode(
  const std::string& name,
  const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg)
{
  auto bb = config().blackboard;
  node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
  client_ = rclcpp_action::create_client<MoveTo>(node_, "/move_to");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  RCLCPP_INFO(node_->get_logger(), "TiltAboutTcpNode: created action client to /move_to");
}

bool TiltAboutTcpNode::lookup_pose(
  const std::string& target_frame,
  const std::string& source_frame,
  geometry_msgs::msg::PoseStamped& out,
  std::string& err) const
{
  try {
    const auto tf = tf_buffer_->lookupTransform(
      target_frame, source_frame, tf2::TimePointZero, 500ms);
    out.header = tf.header;
    out.header.frame_id = target_frame;
    out.pose.position.x = tf.transform.translation.x;
    out.pose.position.y = tf.transform.translation.y;
    out.pose.position.z = tf.transform.translation.z;
    out.pose.orientation = tf.transform.rotation;
    return true;
  } catch (const tf2::TransformException& ex) {
    err = ex.what();
    return false;
  }
}

BT::NodeStatus TiltAboutTcpNode::onStart()
{
  goal_handle_.reset();
  send_future_ = {};
  result_future_ = {};

  const bool restore = getInput<bool>("restore").value_or(false);
  const std::string restore_key =
    getInput<std::string>("restore_key").value_or("lightsout_purge_pretilt_pose");
  const std::string base_frame =
    getInput<std::string>("base_frame").value_or("base_link");

  geometry_msgs::msg::PoseStamped target_pose;

  if (restore) {
    auto bb = config().blackboard;
    geometry_msgs::msg::PoseStamped stashed;
    if (!bb->get(restore_key, stashed)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "TiltAboutTcp: restore requested but blackboard key '%s' missing",
        restore_key.c_str());
      return BT::NodeStatus::FAILURE;
    }
    target_pose = stashed;
    RCLCPP_INFO(
      node_->get_logger(),
      "TiltAboutTcp: restore pose from '%s' (%.4f, %.4f, %.4f)",
      restore_key.c_str(),
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z);
  } else {
    const double degrees = getInput<double>("degrees").value_or(0.0);
    const double direction = getInput<double>("direction").value_or(-1.0);
    const std::string tcp_frame =
      getInput<std::string>("tcp_frame").value_or("tcp_link");
    const std::string axis_frame =
      getInput<std::string>("axis_frame").value_or("wrist_link");
    const double axis_x = getInput<double>("axis_x").value_or(0.0);
    const double axis_y = getInput<double>("axis_y").value_or(0.0);
    const double axis_z = getInput<double>("axis_z").value_or(1.0);

    geometry_msgs::msg::PoseStamped tcp_pose;
    std::string err;
    if (!lookup_pose(base_frame, tcp_frame, tcp_pose, err)) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "TiltAboutTcp: TF %s <- %s failed: %s",
        base_frame.c_str(),
        tcp_frame.c_str(),
        err.c_str());
      return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::TransformStamped wrist_tf;
    try {
      wrist_tf = tf_buffer_->lookupTransform(
        base_frame, axis_frame, tf2::TimePointZero, 500ms);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "TiltAboutTcp: TF %s <- %s failed: %s",
        base_frame.c_str(),
        axis_frame.c_str(),
        ex.what());
      return BT::NodeStatus::FAILURE;
    }

    // Stash pre-tilt TCP pose so restore can return to the same tip pose.
    config().blackboard->set(restore_key, tcp_pose);

    tf2::Quaternion q_wrist;
    tf2::fromMsg(wrist_tf.transform.rotation, q_wrist);
    tf2::Matrix3x3 R_base_wrist(q_wrist);
    tf2::Vector3 axis_local(axis_x, axis_y, axis_z);
    if (axis_local.length2() < 1e-12) {
      RCLCPP_ERROR(node_->get_logger(), "TiltAboutTcp: axis vector is zero");
      return BT::NodeStatus::FAILURE;
    }
    tf2::Vector3 axis_base = R_base_wrist * axis_local.normalized();

    const double angle_rad = direction * degrees * kDegToRad;
    tf2::Quaternion q_delta;
    q_delta.setRotation(axis_base, angle_rad);

    tf2::Quaternion q_tcp;
    tf2::fromMsg(tcp_pose.pose.orientation, q_tcp);
    tf2::Quaternion q_target = q_delta * q_tcp;
    q_target.normalize();

    target_pose = tcp_pose;
    target_pose.pose.orientation = tf2::toMsg(q_target);

    RCLCPP_INFO(
      node_->get_logger(),
      "TiltAboutTcp: degrees=%.2f direction=%.1f angle_rad=%.4f "
      "tcp=(%.4f,%.4f,%.4f) axis_base=(%.3f,%.3f,%.3f) stash='%s'",
      degrees,
      direction,
      angle_rad,
      target_pose.pose.position.x,
      target_pose.pose.position.y,
      target_pose.pose.position.z,
      axis_base.x(),
      axis_base.y(),
      axis_base.z(),
      restore_key.c_str());
  }

  if (!client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(), "TiltAboutTcp: /move_to action server unavailable");
    return BT::NodeStatus::FAILURE;
  }

  MoveTo::Goal goal;
  goal.target_pose = target_pose;
  goal.use_cartesian = false;
  goal.plan_only = false;
  if (auto vs = getInput<double>("velocity_scaling"); vs && *vs > 0.0) {
    goal.velocity_scaling = static_cast<float>(*vs);
  }
  if (auto as = getInput<double>("acceleration_scaling"); as && *as > 0.0) {
    goal.acceleration_scaling = static_cast<float>(*as);
  }
  if (auto planner = getInput<std::string>("planner_id"); planner && !planner->empty()) {
    goal.planner_id = *planner;
  }

  rclcpp_action::Client<MoveTo>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveTo>> gh) {
      if (!gh) {
        RCLCPP_ERROR(node_->get_logger(), "TiltAboutTcp: goal rejected by server");
      } else {
        RCLCPP_INFO(node_->get_logger(), "TiltAboutTcp: goal accepted by server");
      }
    };
  send_future_ = client_->async_send_goal(goal, opts);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TiltAboutTcpNode::onRunning()
{
  if (!goal_handle_) {
    if (send_future_.valid() && send_future_.wait_for(0s) == std::future_status::ready) {
      goal_handle_ = send_future_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "TiltAboutTcp: goal_handle null after send");
        return BT::NodeStatus::FAILURE;
      }
      result_future_ = client_->async_get_result(goal_handle_);
    } else {
      rclcpp::spin_some(node_);
      return BT::NodeStatus::RUNNING;
    }
  }

  if (result_future_.valid() && result_future_.wait_for(0s) == std::future_status::ready) {
    auto res = result_future_.get();
    const bool ok = res.result && res.result->success;
    RCLCPP_INFO(
      node_->get_logger(),
      "TiltAboutTcp: result success=%s msg=%s",
      ok ? "true" : "false",
      res.result ? res.result->message.c_str() : "<none>");
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  rclcpp::spin_some(node_);
  return BT::NodeStatus::RUNNING;
}

void TiltAboutTcpNode::onHalted()
{
  try {
    if (goal_handle_) {
      RCLCPP_INFO(node_->get_logger(), "TiltAboutTcp: canceling active goal on halt");
      (void)client_->async_cancel_goal(goal_handle_);
    }
  } catch (...) {
  }
  goal_handle_.reset();
  send_future_ = {};
  result_future_ = {};
}

}  // namespace robot_orchestrator
