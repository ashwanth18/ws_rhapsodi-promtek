#include "robot_orchestrator/move_to_node.hpp"
#include <sstream>

using namespace std::chrono_literals;

namespace robot_orchestrator {

BT::PortsList MoveToNode::providedPorts()
{
  return {
    BT::InputPort<std::string>("target_name"),
    BT::InputPort<double>("target_x"),
    BT::InputPort<double>("target_y"),
    BT::InputPort<double>("target_z"),
    BT::InputPort<double>("qx", 0.0, "orientation x"),
    BT::InputPort<double>("qy", 0.0, "orientation y"),
    BT::InputPort<double>("qz", 0.0, "orientation z"),
    BT::InputPort<double>("qw", 1.0, "orientation w"),
    BT::InputPort<std::string>("frame_id", std::string("base_link"), "pose frame"),
    BT::InputPort<bool>("use_cartesian", false, "use Cartesian path"),
    BT::InputPort<double>("eef_step", 0.01, "Cartesian eef step"),
    BT::InputPort<double>("jump_threshold", 0.0, "Cartesian jump threshold"),
    // New optional ports matching updated action:
    BT::InputPort<std::vector<std::string>>("waypoint_names", {}, "list of named waypoints"),
    BT::InputPort<double>("velocity_scaling", 0.0, "per-goal velocity scaling [0,1] (0=use param)"),
    BT::InputPort<double>("acceleration_scaling", 0.0, "per-goal acceleration scaling [0,1] (0=use param)"),
    BT::InputPort<std::vector<double>>("waypoint_velocity_scaling", {}, "per-waypoint velocity scaling for joint-space"),
    BT::InputPort<std::vector<double>>("waypoint_acceleration_scaling", {}, "per-waypoint acceleration scaling for joint-space")
  };
}

MoveToNode::MoveToNode(const std::string& name, const BT::NodeConfiguration& cfg)
: BT::StatefulActionNode(name, cfg)
{
  auto bb = config().blackboard;
  node_ = bb->get<rclcpp::Node::SharedPtr>("ros_node");
  client_ = rclcpp_action::create_client<MoveTo>(node_, "/move_to");
  RCLCPP_INFO(node_->get_logger(), "MoveToNode: created action client to %s", "/move_to");
}

BT::NodeStatus MoveToNode::onStart()
{
  RCLCPP_INFO(node_->get_logger(), "MoveToNode: waiting for action server %s", "/move_to");
  if (!client_->wait_for_action_server(10s)) {
    RCLCPP_ERROR(node_->get_logger(), "MoveToNode: action server %s not available", "/move_to");
    return BT::NodeStatus::FAILURE;
  }

  MoveTo::Goal goal;
  std::string name;
  if (getInput("target_name", name) && !name.empty()) {
    goal.target_name = name;
  } else {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = getInput<std::string>("frame_id").value();
    pose.pose.position.x = getInput<double>("target_x").value();
    pose.pose.position.y = getInput<double>("target_y").value();
    pose.pose.position.z = getInput<double>("target_z").value();
    pose.pose.orientation.x = getInput<double>("qx").value();
    pose.pose.orientation.y = getInput<double>("qy").value();
    pose.pose.orientation.z = getInput<double>("qz").value();
    pose.pose.orientation.w = getInput<double>("qw").value();
    goal.target_pose = pose;
  }

  // Optional Cartesian controls
  if (auto cart = getInput<bool>("use_cartesian"); cart) {
    goal.use_cartesian = *cart;
  }
  if (auto es = getInput<double>("eef_step"); es) {
    goal.eef_step = static_cast<float>(*es);
  }
  if (auto jt = getInput<double>("jump_threshold"); jt) {
    goal.jump_threshold = static_cast<float>(*jt);
  }

  // New: optional named waypoints and speed scalings
  if (auto wpn = getInput<std::vector<std::string>>("waypoint_names"); wpn) {
    std::vector<std::string> filtered;
    filtered.reserve(wpn->size());
    for (const auto& s : *wpn) {
      if (!s.empty() && s != "json:[]") {
        filtered.push_back(s);
      }
    }
    if (!filtered.empty()) {
      goal.waypoint_names = std::move(filtered);
    }
  }
  if (auto vs = getInput<double>("velocity_scaling"); vs) {
    goal.velocity_scaling = static_cast<float>(*vs);
  }
  if (auto as = getInput<double>("acceleration_scaling"); as) {
    goal.acceleration_scaling = static_cast<float>(*as);
  }
  if (auto wv = getInput<std::vector<double>>("waypoint_velocity_scaling"); wv && !wv->empty()) {
    goal.waypoint_velocity_scaling.assign(wv->begin(), wv->end());
  }
  if (auto wa = getInput<std::vector<double>>("waypoint_acceleration_scaling"); wa && !wa->empty()) {
    goal.waypoint_acceleration_scaling.assign(wa->begin(), wa->end());
  }

  // Log goal summary
  {
    std::ostringstream ss;
    ss << "MoveToNode: sending goal: ";
    if (!goal.target_name.empty()) {
      ss << "target_name=" << goal.target_name;
    } else {
      ss << "pose(x=" << goal.target_pose.pose.position.x
         << ", y=" << goal.target_pose.pose.position.y
         << ", z=" << goal.target_pose.pose.position.z << ")";
    }
    ss << ", use_cartesian=" << (goal.use_cartesian ? "true" : "false");
    ss << ", waypoints(names)=" << goal.waypoint_names.size();
    RCLCPP_INFO(node_->get_logger(), "%s", ss.str().c_str());
  }

  rclcpp_action::Client<MoveTo>::SendGoalOptions opts;
  opts.goal_response_callback = [this](std::shared_ptr<rclcpp_action::ClientGoalHandle<MoveTo>> gh){
    if (!gh) {
      RCLCPP_ERROR(node_->get_logger(), "MoveToNode: goal rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "MoveToNode: goal accepted by server");
    }
  };
  opts.result_callback = [this](const rclcpp_action::ClientGoalHandle<MoveTo>::WrappedResult &res){
    std::string code;
    switch (res.code) {
      case rclcpp_action::ResultCode::SUCCEEDED: code = "SUCCEEDED"; break;
      case rclcpp_action::ResultCode::ABORTED: code = "ABORTED"; break;
      case rclcpp_action::ResultCode::CANCELED: code = "CANCELED"; break;
      default: code = "UNKNOWN"; break;
    }
    RCLCPP_INFO(node_->get_logger(), "MoveToNode: result code=%s success=%s msg=%s",
                code.c_str(), (res.result && res.result->success) ? "true" : "false",
                res.result ? res.result->message.c_str() : "<none>");
  };
  send_future_ = client_->async_send_goal(goal, opts);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToNode::onRunning()
{
  if (!goal_handle_) {
    if (send_future_.valid() && send_future_.wait_for(0s) == std::future_status::ready) {
      goal_handle_ = send_future_.get();
      if (!goal_handle_) {
        RCLCPP_ERROR(node_->get_logger(), "MoveToNode: goal_handle is null after send_goal");
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
    if (res.result) {
      RCLCPP_INFO(node_->get_logger(), "MoveToNode: result ready success=%s msg=%s",
                  res.result->success ? "true" : "false", res.result->message.c_str());
    }
    return (res.result && res.result->success) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
  rclcpp::spin_some(node_);
  return BT::NodeStatus::RUNNING;
}

void MoveToNode::onHalted()
{
  // Cancel any active goal to avoid overlap with subsequent goals
  try {
    if (goal_handle_) {
      RCLCPP_INFO(node_->get_logger(), "MoveToNode: canceling active goal on halt");
      (void)client_->async_cancel_goal(goal_handle_);
    }
  } catch (...) {}
}

} // namespace robot_orchestrator




