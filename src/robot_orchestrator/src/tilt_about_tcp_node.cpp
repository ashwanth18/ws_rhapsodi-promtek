#include "robot_orchestrator/tilt_about_tcp_node.hpp"

#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace robot_orchestrator {

namespace {

constexpr double kDegToRad = M_PI / 180.0;

tf2::Vector3 pose_position(const geometry_msgs::msg::Pose& pose)
{
  return {pose.position.x, pose.position.y, pose.position.z};
}

void set_pose_position(geometry_msgs::msg::Pose& pose, const tf2::Vector3& p)
{
  pose.position.x = p.x();
  pose.position.y = p.y();
  pose.position.z = p.z();
}

}  // namespace

BT::PortsList TiltAboutTcpNode::providedPorts()
{
  return {
    BT::InputPort<double>("degrees", 0.0, "tilt angle in degrees"),
    BT::InputPort<double>("direction", -1.0, "sign applied to degrees (+1 or -1)"),
    BT::InputPort<std::string>("base_frame", std::string("base_link"), "planning / TF parent frame"),
    BT::InputPort<std::string>("tcp_frame", std::string("tcp_link"), "tool tip frame"),
    BT::InputPort<std::string>(
      "axis_frame", std::string("wrist_link"), "frame whose origin lies on the tilt axis"),
    BT::InputPort<double>("axis_x", 0.0, "tilt axis x in axis_frame"),
    BT::InputPort<double>("axis_y", 0.0, "tilt axis y in axis_frame"),
    BT::InputPort<double>("axis_z", 1.0, "tilt axis z in axis_frame (joint_5 = wrist z)"),
    BT::InputPort<double>("lift_m", 0.04, "Cartesian lift before pre-shift (metres)"),
    BT::InputPort<double>("velocity_scaling", 0.4, "MoveTo velocity scaling [0,1]"),
    BT::InputPort<double>("acceleration_scaling", 0.4, "MoveTo acceleration scaling [0,1]"),
    BT::InputPort<double>("eef_step", 0.01, "Cartesian eef step (metres)"),
    BT::InputPort<std::string>("planning_pipeline", std::string("ompl"), "unused for cartesian; kept for logs"),
    BT::InputPort<double>(
      "incline_settle_s", 0.6, "seconds to wait after publishing /incline_control"),
    BT::InputPort<std::string>(
      "incline_topic", std::string("/incline_control"), "Float64 incline topic (degrees)"),
    // restore kept so older trees still parse; ignored (untilt is SetIncline 0).
    BT::InputPort<bool>("restore", false, "ignored; untilt via SetIncline(0) in the tree"),
    BT::InputPort<std::string>(
      "restore_key",
      std::string("lightsout_purge_pretilt_pose"),
      "unused stash key (kept for port compatibility)"),
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

void TiltAboutTcpNode::publish_incline(double degrees)
{
  const std::string topic =
    getInput<std::string>("incline_topic").value_or("/incline_control");
  if (!incline_pub_ || incline_pub_->get_topic_name() != topic) {
    incline_pub_ = node_->create_publisher<std_msgs::msg::Float64>(topic, 10);
  }
  std_msgs::msg::Float64 msg;
  msg.data = degrees;
  incline_pub_->publish(msg);
  RCLCPP_INFO(
    node_->get_logger(),
    "TiltAboutTcp: publish incline %.3f deg on %s",
    degrees,
    topic.c_str());
}

bool TiltAboutTcpNode::send_cartesian_shift(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& lifted,
  const geometry_msgs::msg::PoseStamped& pre)
{
  if (!client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(), "TiltAboutTcp: /move_to unavailable");
    return false;
  }

  MoveTo::Goal goal;
  goal.use_cartesian = true;
  goal.plan_only = false;
  goal.eef_step = static_cast<float>(getInput<double>("eef_step").value_or(0.01));
  goal.jump_threshold = 0.0f;
  // Waypoints: lift straight up, then horizontal to pre-shift (still raised).
  // Incline from the raised pre pose; tip XY recentres over the vessel.
  goal.waypoints = {lifted, pre};
  // Also set target_pose to final for servers that prefer it.
  goal.target_pose = pre;
  if (auto vs = getInput<double>("velocity_scaling"); vs && *vs > 0.0) {
    goal.velocity_scaling = static_cast<float>(*vs);
  }
  if (auto as = getInput<double>("acceleration_scaling"); as && *as > 0.0) {
    goal.acceleration_scaling = static_cast<float>(*as);
  }

  (void)start;
  goal_handle_.reset();
  send_future_ = {};
  result_future_ = {};

  rclcpp_action::Client<MoveTo>::SendGoalOptions opts;
  opts.goal_response_callback =
    [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveTo>> gh) {
      if (!gh) {
        RCLCPP_ERROR(node_->get_logger(), "TiltAboutTcp: MoveTo goal rejected");
      } else {
        RCLCPP_INFO(node_->get_logger(), "TiltAboutTcp: Cartesian pre-shift accepted");
      }
    };
  send_future_ = client_->async_send_goal(goal, opts);
  phase_ = Phase::Moving;
  return true;
}

void TiltAboutTcpNode::cancel_move()
{
  try {
    if (goal_handle_) {
      (void)client_->async_cancel_goal(goal_handle_);
    }
  } catch (...) {
  }
  goal_handle_.reset();
  send_future_ = {};
  result_future_ = {};
}

BT::NodeStatus TiltAboutTcpNode::onStart()
{
  phase_ = Phase::Idle;
  cancel_move();

  // Older trees may still call restore=true; untilt is SetIncline(0) in the tree.
  if (getInput<bool>("restore").value_or(false)) {
    publish_incline(0.0);
    incline_settle_s_ = std::max(0.1, getInput<double>("incline_settle_s").value_or(0.6));
    incline_done_time_ = node_->now() + rclcpp::Duration::from_seconds(incline_settle_s_);
    phase_ = Phase::WaitingIncline;
    pending_incline_deg_ = 0.0;
    RCLCPP_INFO(node_->get_logger(), "TiltAboutTcp: restore degenerates to incline 0");
    return BT::NodeStatus::RUNNING;
  }

  incline_settle_s_ = std::max(0.1, getInput<double>("incline_settle_s").value_or(0.6));
  const std::string base_frame =
    getInput<std::string>("base_frame").value_or("base_link");
  const double degrees = getInput<double>("degrees").value_or(0.0);
  const double direction = getInput<double>("direction").value_or(-1.0);
  const std::string tcp_frame =
    getInput<std::string>("tcp_frame").value_or("tcp_link");
  const std::string axis_frame =
    getInput<std::string>("axis_frame").value_or("wrist_link");
  const double axis_x = getInput<double>("axis_x").value_or(0.0);
  const double axis_y = getInput<double>("axis_y").value_or(0.0);
  const double axis_z = getInput<double>("axis_z").value_or(1.0);
  const double lift_m = std::max(0.0, getInput<double>("lift_m").value_or(0.04));

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

  tf2::Quaternion q_wrist;
  tf2::fromMsg(wrist_tf.transform.rotation, q_wrist);
  const tf2::Matrix3x3 R_base_wrist(q_wrist);
  tf2::Vector3 axis_local(axis_x, axis_y, axis_z);
  if (axis_local.length2() < 1e-12) {
    RCLCPP_ERROR(node_->get_logger(), "TiltAboutTcp: axis vector is zero");
    return BT::NodeStatus::FAILURE;
  }
  const tf2::Vector3 axis_base = (R_base_wrist * axis_local).normalized();
  const tf2::Vector3 pivot(
    wrist_tf.transform.translation.x,
    wrist_tf.transform.translation.y,
    wrist_tf.transform.translation.z);

  const double angle_rad = direction * degrees * kDegToRad;
  const tf2::Quaternion q_delta(axis_base, angle_rad);
  const tf2::Vector3 tcp = pose_position(tcp_pose.pose);
  // Pre-shift so joint incline about the wrist axis returns tip XY to `tcp`.
  const tf2::Vector3 tcp_pre = pivot + tf2::quatRotate(q_delta.inverse(), tcp - pivot);

  geometry_msgs::msg::PoseStamped lifted = tcp_pose;
  lifted.pose.position.z += lift_m;

  geometry_msgs::msg::PoseStamped pre = tcp_pose;
  set_pose_position(pre.pose, tcp_pre);
  pre.pose.position.z = tcp.z() + lift_m;  // keep raised; incline from here

  const double shift_xy = std::hypot(tcp_pre.x() - tcp.x(), tcp_pre.y() - tcp.y());
  RCLCPP_INFO(
    node_->get_logger(),
    "TiltAboutTcp: cartesian lift=%.0fmm shift_xy=%.1fmm then incline %.1fdeg "
    "(tcp=(%.4f,%.4f,%.4f) pre=(%.4f,%.4f,%.4f))",
    lift_m * 1000.0,
    shift_xy * 1000.0,
    degrees,
    tcp.x(),
    tcp.y(),
    tcp.z(),
    pre.pose.position.x,
    pre.pose.position.y,
    pre.pose.position.z);

  pending_incline_deg_ = degrees;
  if (!send_cartesian_shift(tcp_pose, lifted, pre)) {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TiltAboutTcpNode::onRunning()
{
  if (phase_ == Phase::WaitingIncline) {
    if (node_->now() < incline_done_time_) {
      rclcpp::spin_some(node_);
      return BT::NodeStatus::RUNNING;
    }
    phase_ = Phase::Idle;
    RCLCPP_INFO(node_->get_logger(), "TiltAboutTcp: incline settle done");
    return BT::NodeStatus::SUCCESS;
  }

  if (phase_ != Phase::Moving) {
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_handle_) {
    if (send_future_.valid() && send_future_.wait_for(0s) == std::future_status::ready) {
      goal_handle_ = send_future_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "TiltAboutTcp: MoveTo goal_handle null");
        phase_ = Phase::Idle;
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
      "TiltAboutTcp: Cartesian success=%s msg=%s",
      ok ? "true" : "false",
      res.result ? res.result->message.c_str() : "<none>");
    goal_handle_.reset();
    send_future_ = {};
    result_future_ = {};
    if (!ok) {
      phase_ = Phase::Idle;
      return BT::NodeStatus::FAILURE;
    }

    publish_incline(pending_incline_deg_);
    incline_done_time_ = node_->now() + rclcpp::Duration::from_seconds(incline_settle_s_);
    phase_ = Phase::WaitingIncline;
    return BT::NodeStatus::RUNNING;
  }

  rclcpp::spin_some(node_);
  return BT::NodeStatus::RUNNING;
}

void TiltAboutTcpNode::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "TiltAboutTcp: halted");
  cancel_move();
  if (std::abs(pending_incline_deg_) > 1e-3 && phase_ != Phase::Idle) {
    publish_incline(0.0);
  }
  phase_ = Phase::Idle;
}

}  // namespace robot_orchestrator
