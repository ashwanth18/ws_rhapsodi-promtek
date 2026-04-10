#include "scooping_controller/container_collision_objects.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Transform.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

namespace
{
using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

constexpr double kTcpOffsetX = 0.183;
constexpr double kTcpOffsetY = 0.0;
constexpr double kTcpOffsetZ = -0.072;
constexpr char kDefaultGoalFrame[] = "base_link";
constexpr char kDefaultScoopFrame[] = "scooping_container_frame";
constexpr char kDefaultTaskContainerId[] = "scooping_container";

struct MarkerSeed
{
  std::string marker_name;
  std::string label;
  geometry_msgs::msg::Pose pose{rosidl_runtime_cpp::MessageInitialization::ALL};
  std::array<float, 3> color{};
};

std::array<MarkerSeed, 5> default_marker_seeds()
{
  std::array<MarkerSeed, 5> seeds{};

  seeds[0].marker_name = "approach_marker";
  seeds[0].label = "Approach";
  seeds[0].pose.position.x = 0.32;
  seeds[0].pose.position.y = 0.18;
  seeds[0].pose.position.z = 0.22;
  seeds[0].pose.orientation.y = 0.70710678;
  seeds[0].pose.orientation.w = 0.70710678;
  seeds[0].color = {0.2f, 0.7f, 1.0f};

  seeds[1].marker_name = "contact_marker";
  seeds[1].label = "Contact";
  seeds[1].pose = seeds[0].pose;
  seeds[1].pose.position.z = 0.16;
  seeds[1].color = {1.0f, 0.7f, 0.1f};

  seeds[2].marker_name = "scoop_marker";
  seeds[2].label = "Scoop";
  seeds[2].pose = seeds[1].pose;
  seeds[2].pose.position.x = 0.36;
  seeds[2].pose.position.z = 0.15;
  seeds[2].color = {0.2f, 0.9f, 0.2f};

  seeds[3].marker_name = "lift_marker";
  seeds[3].label = "Lift";
  seeds[3].pose = seeds[2].pose;
  seeds[3].pose.position.z = 0.24;
  seeds[3].color = {0.9f, 0.3f, 0.8f};

  seeds[4].marker_name = "transport_ready_marker";
  seeds[4].label = "Transport Ready";
  seeds[4].pose = seeds[3].pose;
  seeds[4].pose.position.z = 0.30;
  seeds[4].pose.orientation.x = 0.0;
  seeds[4].pose.orientation.y = 0.0;
  seeds[4].pose.orientation.z = 0.0;
  seeds[4].pose.orientation.w = 1.0;
  seeds[4].color = {1.0f, 0.45f, 0.15f};

  return seeds;
}

MarkerSeed default_goal_marker_seed()
{
  MarkerSeed seed;
  seed.marker_name = "target_goal_marker";
  seed.label = "Move Goal";
  seed.pose.position.x = 0.30;
  seed.pose.position.y = 0.0;
  seed.pose.position.z = 0.30;
  seed.pose.orientation.y = 0.70710678;
  seed.pose.orientation.w = 0.70710678;
  seed.color = {1.0f, 0.3f, 0.3f};
  return seed;
}

std::string expand_user_path(const std::string& path)
{
  if (path.rfind("~/", 0) == 0) {
    const char* home = std::getenv("HOME");
    if (home != nullptr) {
      return std::string(home) + path.substr(1);
    }
  }
  return path;
}

tf2::Transform tf_from_pose(const geometry_msgs::msg::Pose& pose)
{
  tf2::Quaternion q(
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z,
    pose.orientation.w);
  return tf2::Transform(q, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::msg::Pose pose_from_tf(const tf2::Transform& transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.getOrigin().x();
  pose.position.y = transform.getOrigin().y();
  pose.position.z = transform.getOrigin().z();
  pose.orientation.x = transform.getRotation().x();
  pose.orientation.y = transform.getRotation().y();
  pose.orientation.z = transform.getRotation().z();
  pose.orientation.w = transform.getRotation().w();
  return pose;
}

geometry_msgs::msg::Pose transform_pose(
  const geometry_msgs::msg::Pose& pose,
  const tf2::Transform& frame_transform)
{
  return pose_from_tf(frame_transform * tf_from_pose(pose));
}

Marker make_cube_marker(
  const std::array<float, 3>& color,
  double size_x,
  double size_y,
  double size_z)
{
  Marker marker;
  marker.type = Marker::CUBE;
  marker.scale.x = size_x;
  marker.scale.y = size_y;
  marker.scale.z = size_z;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 0.85f;
  return marker;
}

Marker make_origin_marker(const std::array<float, 3>& color, double size)
{
  Marker marker = make_cube_marker(color, size, size, size);
  marker.pose.orientation.w = 1.0;
  marker.color.a = 1.0f;
  return marker;
}

Marker make_tool_mesh_marker(
  const std::array<float, 3>& color,
  const std::string& mesh_resource,
  bool apply_visual_offset = true,
  bool draw_relative_to_tcp = false)
{
  Marker marker;
  marker.type = Marker::MESH_RESOURCE;
  marker.mesh_resource = mesh_resource;
  marker.mesh_use_embedded_materials = false;
  marker.scale.x = 0.001;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;
  if (draw_relative_to_tcp) {
    marker.pose.position.x = -kTcpOffsetX;
    marker.pose.position.y = -kTcpOffsetY;
    marker.pose.position.z = -kTcpOffsetZ;
  } else if (apply_visual_offset) {
    marker.pose.position.y = -0.038;
    marker.pose.position.z = -0.03;
  }
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 0.85f;
  return marker;
}

Marker make_text_marker(
  const std::string& text,
  const std::array<float, 3>& color,
  double z_offset,
  double scale_z)
{
  Marker marker;
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.pose.position.z = z_offset;
  marker.scale.z = scale_z;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = 1.0f;
  marker.text = text;
  return marker;
}

InteractiveMarkerControl make_axis_control(const std::string& axis, bool rotate)
{
  InteractiveMarkerControl control;
  control.orientation.w = 1.0;
  control.name = rotate ? "rotate_" + axis : "move_" + axis;
  control.interaction_mode =
    rotate ? InteractiveMarkerControl::ROTATE_AXIS : InteractiveMarkerControl::MOVE_AXIS;

  if (axis == "x") {
    control.orientation.x = 1.0;
  } else if (axis == "y") {
    control.orientation.z = 1.0;
  } else {
    control.orientation.y = 1.0;
  }
  return control;
}
}  // namespace

class ScoopingMarkerServer : public rclcpp::Node
{
public:
  ScoopingMarkerServer()
  : Node("scooping_marker_server")
  {
    this->declare_parameter<std::string>("scoop_frame_id", kDefaultScoopFrame);
    this->declare_parameter<std::string>("goal_frame_id", kDefaultGoalFrame);
    this->declare_parameter<std::string>("task_container_id", kDefaultTaskContainerId);
    this->declare_parameter<std::string>("poses_yaml", "~/.ros/scooping_controller/poses.yaml");
    this->declare_parameter<std::string>("seed_poses_yaml", "");
    this->declare_parameter<bool>("auto_load_poses_on_startup", true);
    this->declare_parameter<std::string>(
      "tool_mesh_resource",
      "package://niryo_robot_description/meshes/ned3pro/stl/niryo_scoop_v4-ros.STL");
    scooping_controller::declare_container_scene_parameters(*this);
    scoop_frame_id_ = this->get_parameter("scoop_frame_id").as_string();
    goal_frame_id_ = this->get_parameter("goal_frame_id").as_string();
    poses_yaml_path_ = expand_user_path(this->get_parameter("poses_yaml").as_string());
    seed_poses_yaml_path_ = expand_user_path(this->get_parameter("seed_poses_yaml").as_string());
    tool_mesh_resource_ = this->get_parameter("tool_mesh_resource").as_string();
    seeds_ = default_marker_seeds();
    goal_seed_ = default_goal_marker_seed();
    goal_pose_ = goal_seed_.pose;
    initialize_scoop_frame_transform();
    for (auto& seed : seeds_) {
      seed.pose = transform_pose_to_scoop_frame(seed.pose);
    }

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "/scoop_poses", rclcpp::QoS(1).transient_local());
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/target_goal_pose", rclcpp::QoS(1).transient_local());
    goal_pose_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/target_goal_pose_cmd",
      rclcpp::QoS(1).reliable(),
      std::bind(&ScoopingMarkerServer::handle_goal_pose_command, this, std::placeholders::_1));
    scoop_poses_cmd_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "/scoop_poses_cmd",
      rclcpp::QoS(1).reliable(),
      std::bind(&ScoopingMarkerServer::handle_scoop_poses_command, this, std::placeholders::_1));
    scoop_focus_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "/scoop_marker_focus_index",
      rclcpp::QoS(1).reliable(),
      std::bind(&ScoopingMarkerServer::handle_scoop_focus_command, this, std::placeholders::_1));
    legend_pub_ = this->create_publisher<MarkerArray>(
      "/scooping_legend", rclcpp::QoS(1).transient_local());

    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/save_scoop_poses",
      std::bind(&ScoopingMarkerServer::handle_save_request, this, std::placeholders::_1, std::placeholders::_2));
    load_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/load_scoop_poses",
      std::bind(&ScoopingMarkerServer::handle_load_request, this, std::placeholders::_1, std::placeholders::_2));

    init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(0),
      std::bind(&ScoopingMarkerServer::deferred_init, this));
  }

private:
  void deferred_init()
  {
    init_timer_->cancel();
    scoop_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "scooping_markers", shared_from_this());
    goal_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "move_goal_marker", shared_from_this());

    for (const auto& seed : seeds_) {
      poses_.push_back(seed.pose);
    }

    if (this->get_parameter("auto_load_poses_on_startup").as_bool() &&
        !std::filesystem::exists(poses_yaml_path_))
    {
      std::string seed_message;
      if (seed_startup_poses(seed_message)) {
        update_status(seed_message, {0.75f, 0.95f, 0.75f});
        RCLCPP_INFO(
          this->get_logger(),
          "%s",
          seed_message.c_str());
      } else {
        RCLCPP_WARN(
          this->get_logger(),
          "Failed to seed scoop poses at startup: %s",
          seed_message.c_str());
      }
    }

    bool loaded_saved_poses = false;
    if (this->get_parameter("auto_load_poses_on_startup").as_bool() &&
        std::filesystem::exists(poses_yaml_path_))
    {
      std::string load_message;
      loaded_saved_poses = load_poses_from_yaml(load_message);
      if (loaded_saved_poses) {
        update_status(load_message, {0.75f, 0.95f, 0.75f});
        RCLCPP_INFO(this->get_logger(), "%s", load_message.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Auto-load scoop poses failed: %s", load_message.c_str());
      }
    }
    if (!loaded_saved_poses) {
      rebuild_scoop_markers();
    }
    insert_goal_marker();

    scoop_server_->applyChanges();
    goal_server_->applyChanges();
    publish_pose_array();
    publish_goal_pose();
    publish_legend();
    if (!loaded_saved_poses) {
      update_status("RViz panel ready. Drag markers and use the Scooping Controls panel.");
    }
    RCLCPP_INFO(this->get_logger(), "Scooping marker server ready with %zu markers", poses_.size());
  }

  void insert_pose_marker(std::size_t index)
  {
    const auto& seed = seeds_[index];
    InteractiveMarker int_marker;
    int_marker.header.frame_id = scoop_frame_id_;
    int_marker.name = seed.marker_name;
    int_marker.description = seed.label;
    int_marker.scale = 0.18;
    int_marker.pose = poses_[index];

    InteractiveMarkerControl visual_control;
    visual_control.always_visible = true;
    visual_control.markers.push_back(make_origin_marker({1.0f, 1.0f, 1.0f}, 0.010));
    if (tool_mesh_resource_.empty()) {
      visual_control.markers.push_back(make_cube_marker(seed.color, 0.035, 0.035, 0.035));
    } else {
      visual_control.markers.push_back(
        make_tool_mesh_marker(seed.color, tool_mesh_resource_, false, true));
    }
    visual_control.markers.push_back(
      make_text_marker(seed.label + " (TCP)", seed.color, 0.07, 0.04));
    int_marker.controls.push_back(visual_control);

    for (const auto& axis : {"x", "y", "z"}) {
      int_marker.controls.push_back(make_axis_control(axis, false));
      int_marker.controls.push_back(make_axis_control(axis, true));
    }

    scoop_server_->insert(
      int_marker,
      std::bind(&ScoopingMarkerServer::process_feedback, this, std::placeholders::_1));
  }

  void insert_goal_marker()
  {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = goal_frame_id_;
    int_marker.name = goal_seed_.marker_name;
    int_marker.description = goal_seed_.label;
    int_marker.scale = 0.20;
    int_marker.pose = goal_pose_;

    InteractiveMarkerControl visual_control;
    visual_control.always_visible = true;
    visual_control.markers.push_back(make_origin_marker({1.0f, 1.0f, 1.0f}, 0.012));
    if (tool_mesh_resource_.empty()) {
      visual_control.markers.push_back(make_cube_marker(goal_seed_.color, 0.04, 0.04, 0.04));
    } else {
      visual_control.markers.push_back(
        make_tool_mesh_marker(goal_seed_.color, tool_mesh_resource_, false, true));
    }
    visual_control.markers.push_back(
      make_text_marker(goal_seed_.label + " (TCP)", goal_seed_.color, 0.08, 0.045));
    int_marker.controls.push_back(visual_control);

    for (const auto& axis : {"x", "y", "z"}) {
      int_marker.controls.push_back(make_axis_control(axis, false));
      int_marker.controls.push_back(make_axis_control(axis, true));
    }

    goal_server_->insert(
      int_marker,
      std::bind(&ScoopingMarkerServer::process_feedback, this, std::placeholders::_1));
  }

  void process_feedback(const Feedback::ConstSharedPtr& feedback)
  {
    if (feedback->marker_name == goal_seed_.marker_name) {
      goal_pose_ = feedback->pose;
      publish_goal_pose();
      RCLCPP_INFO(
        this->get_logger(),
        "Updated %s to [%.3f, %.3f, %.3f]",
        feedback->marker_name.c_str(),
        feedback->pose.position.x,
        feedback->pose.position.y,
        feedback->pose.position.z);
      update_status("Updated move goal marker", {1.0f, 0.8f, 0.3f});
      return;
    }

    for (std::size_t i = 0; i < seeds_.size(); ++i) {
      if (feedback->marker_name == seeds_[i].marker_name) {
        poses_[i] = feedback->pose;
        publish_pose_array();
        RCLCPP_INFO(
          this->get_logger(),
          "Updated %s to [%.3f, %.3f, %.3f]",
          feedback->marker_name.c_str(),
          feedback->pose.position.x,
          feedback->pose.position.y,
          feedback->pose.position.z);
        update_status("Updated " + seeds_[i].label + " pose");
        break;
      }
    }
  }

  void publish_pose_array()
  {
    geometry_msgs::msg::PoseArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = scoop_frame_id_;
    msg.poses = poses_;
    pose_pub_->publish(msg);
  }

  void publish_goal_pose()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = goal_frame_id_;
    msg.pose = goal_pose_;
    goal_pose_pub_->publish(msg);
  }

  void rebuild_scoop_markers()
  {
    if (!scoop_server_) {
      return;
    }
    for (const auto& seed : seeds_) {
      scoop_server_->erase(seed.marker_name);
    }
    for (std::size_t i = 0; i < seeds_.size(); ++i) {
      if (focus_index_ >= 0 && static_cast<int>(i) != focus_index_) {
        continue;
      }
      insert_pose_marker(i);
    }
    scoop_server_->applyChanges();
  }

  void handle_goal_pose_command(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN(this->get_logger(), "Ignoring target goal command with empty frame_id");
      return;
    }
    if (msg->header.frame_id != goal_frame_id_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring target goal command in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(),
        goal_frame_id_.c_str());
      return;
    }

    goal_pose_ = msg->pose;
    if (goal_server_) {
      goal_server_->setPose(goal_seed_.marker_name, goal_pose_);
      goal_server_->applyChanges();
    }
    publish_goal_pose();
    update_status("Updated move goal marker from typed pose", {1.0f, 0.8f, 0.3f});
  }

  void handle_scoop_poses_command(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (msg->poses.size() != seeds_.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring scoop pose command with %zu poses; expected %zu",
        msg->poses.size(),
        seeds_.size());
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != scoop_frame_id_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring scoop pose command in frame '%s'; expected '%s'",
        msg->header.frame_id.c_str(),
        scoop_frame_id_.c_str());
      return;
    }

    poses_ = msg->poses;
    rebuild_scoop_markers();
    publish_pose_array();
    update_status("Updated selected scoop marker from typed pose", {0.95f, 0.95f, 0.95f});
  }

  void handle_scoop_focus_command(const std_msgs::msg::Int32::SharedPtr msg)
  {
    int requested_index = msg->data;
    if (requested_index < -1 || requested_index >= static_cast<int>(seeds_.size())) {
      requested_index = -1;
    }
    focus_index_ = requested_index;
    rebuild_scoop_markers();
    update_status(
      focus_index_ >= 0
        ? "Showing only " + seeds_[static_cast<std::size_t>(focus_index_)].label + " marker"
        : "Showing all scoop markers",
      {0.95f, 0.95f, 0.95f});
  }

  void publish_legend()
  {
    MarkerArray legend;
    const auto stamp = this->now();
    std::int32_t id = 0;

    legend.markers.push_back(make_legend_text(
      id++, "Scooping Workflow", 0.13, -0.50, 0.34, {1.0f, 1.0f, 1.0f}, 0.055));

    for (std::size_t i = 0; i < seeds_.size(); ++i) {
      const double y = -0.50 - static_cast<double>(i) * 0.055;
      legend.markers.push_back(make_legend_cube(id++, 0.02, y, 0.26, seeds_[i].color));
      legend.markers.push_back(make_legend_text(
        id++,
        seeds_[i].label,
        0.08,
        y,
        0.26,
        seeds_[i].color,
        0.04));
    }

    legend.markers.push_back(make_legend_text(
      id++,
      "Panel actions: Save YAML | Load YAML | Execute Scoop",
      0.18,
      -0.74,
      0.25,
      {0.95f, 0.95f, 0.95f},
      0.035));
    legend.markers.push_back(make_legend_cube(id++, 0.02, -0.72, 0.20, goal_seed_.color));
    legend.markers.push_back(make_legend_text(
      id++,
      "Move Goal marker: white cube is tcp_link, mesh is offset from it",
      0.24,
      -0.72,
      0.20,
      goal_seed_.color,
      0.03));
    legend.markers.push_back(make_legend_text(
      id++,
      status_text_,
      0.22,
      -0.80,
      0.21,
      status_color_,
      0.032));

    for (auto& marker : legend.markers) {
      marker.header.frame_id = scoop_frame_id_;
      marker.header.stamp = stamp;
      marker.ns = "scooping_legend";
      marker.action = Marker::ADD;
    }

    legend_pub_->publish(legend);
  }

  Marker make_legend_cube(
    int id,
    double x,
    double y,
    double z,
    const std::array<float, 3>& color) const
  {
    Marker marker = make_cube_marker(color, 0.03, 0.03, 0.03);
    marker.id = id;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  Marker make_legend_text(
    int id,
    const std::string& text,
    double x,
    double y,
    double z,
    const std::array<float, 3>& color,
    double scale_z) const
  {
    Marker marker = make_text_marker(text, color, 0.0, scale_z);
    marker.id = id;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void handle_save_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = save_poses_to_yaml(response->message);
    update_status(response->message);
  }

  void handle_load_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = load_poses_from_yaml(response->message);
    update_status(response->message);
  }

  bool save_poses_to_yaml(std::string& message)
  {
    try {
      const std::filesystem::path path(poses_yaml_path_);
      if (!path.parent_path().empty()) {
        std::filesystem::create_directories(path.parent_path());
      }

      YAML::Node root;
      root["frame_id"] = scoop_frame_id_;
      YAML::Node markers(YAML::NodeType::Sequence);
      for (std::size_t i = 0; i < seeds_.size(); ++i) {
        YAML::Node entry;
        entry["name"] = seeds_[i].marker_name;
        entry["label"] = seeds_[i].label;
        entry["pose"]["position"]["x"] = poses_[i].position.x;
        entry["pose"]["position"]["y"] = poses_[i].position.y;
        entry["pose"]["position"]["z"] = poses_[i].position.z;
        entry["pose"]["orientation"]["x"] = poses_[i].orientation.x;
        entry["pose"]["orientation"]["y"] = poses_[i].orientation.y;
        entry["pose"]["orientation"]["z"] = poses_[i].orientation.z;
        entry["pose"]["orientation"]["w"] = poses_[i].orientation.w;
        markers.push_back(entry);
      }
      root["markers"] = markers;

      std::ofstream out(poses_yaml_path_, std::ios::out | std::ios::trunc);
      if (!out.is_open()) {
        message = "Failed to open pose YAML for writing: " + poses_yaml_path_;
        return false;
      }

      out << root;
      message = "Saved scoop poses to " + poses_yaml_path_;
      return true;
    } catch (const std::exception& ex) {
      message = std::string("Failed to save scoop poses: ") + ex.what();
      return false;
    }
  }

  bool seed_startup_poses(std::string& message)
  {
    try {
      const std::filesystem::path destination(poses_yaml_path_);
      if (!destination.parent_path().empty()) {
        std::filesystem::create_directories(destination.parent_path());
      }

      if (!seed_poses_yaml_path_.empty() && std::filesystem::exists(seed_poses_yaml_path_)) {
        std::filesystem::copy_file(
          seed_poses_yaml_path_,
          poses_yaml_path_,
          std::filesystem::copy_options::overwrite_existing);
        message =
          "Seeded scoop poses from " + seed_poses_yaml_path_ + " to " + poses_yaml_path_;
        return true;
      }

      if (!seed_poses_yaml_path_.empty()) {
        RCLCPP_WARN(
          this->get_logger(),
          "Configured seed_poses_yaml does not exist: %s. Falling back to built-in defaults.",
          seed_poses_yaml_path_.c_str());
      }

      return save_poses_to_yaml(message);
    } catch (const std::exception& ex) {
      message = std::string("Failed to seed scoop poses: ") + ex.what();
      return false;
    }
  }

  bool load_poses_from_yaml(std::string& message)
  {
    try {
      if (!std::filesystem::exists(poses_yaml_path_)) {
        message = "Pose YAML not found: " + poses_yaml_path_;
        return false;
      }

      const YAML::Node root = YAML::LoadFile(poses_yaml_path_);
      const std::string source_frame =
        root["frame_id"] ? root["frame_id"].as<std::string>() : scoop_frame_id_;
      const auto markers = root["markers"];
      if (!markers || !markers.IsSequence()) {
        message = "Pose YAML is missing a markers sequence";
        return false;
      }

      std::size_t updated = 0;
      for (const auto& marker : markers) {
        const auto name_node = marker["name"];
        const auto pose_node = marker["pose"];
        if (!name_node || !pose_node) {
          continue;
        }

        const auto marker_name = name_node.as<std::string>();
        const auto index = find_pose_index(marker_name);
        if (index < 0) {
          continue;
        }

        geometry_msgs::msg::Pose pose;
        pose.position.x = pose_node["position"]["x"].as<double>();
        pose.position.y = pose_node["position"]["y"].as<double>();
        pose.position.z = pose_node["position"]["z"].as<double>();
        pose.orientation.x = pose_node["orientation"]["x"].as<double>();
        pose.orientation.y = pose_node["orientation"]["y"].as<double>();
        pose.orientation.z = pose_node["orientation"]["z"].as<double>();
        pose.orientation.w = pose_node["orientation"]["w"].as<double>();

        poses_[static_cast<std::size_t>(index)] = convert_loaded_scoop_pose(pose, source_frame);
        ++updated;
      }

      rebuild_scoop_markers();
      publish_pose_array();
      message = "Loaded " + std::to_string(updated) + " scoop poses from " + poses_yaml_path_;
      return updated > 0;
    } catch (const std::exception& ex) {
      message = std::string("Failed to load scoop poses: ") + ex.what();
      return false;
    }
  }

  void initialize_scoop_frame_transform()
  {
    if (scoop_frame_id_ == goal_frame_id_) {
      has_scoop_frame_transform_ = false;
      return;
    }

    const auto scene_specs = scooping_controller::load_container_scene_specs(*this);
    const auto task_container_id = this->get_parameter("task_container_id").as_string();
    const auto it = std::find_if(
      scene_specs.begin(),
      scene_specs.end(),
      [&task_container_id](const scooping_controller::ContainerSceneSpec& spec) {
        return spec.id == task_container_id;
      });
    if (it == scene_specs.end()) {
      throw std::runtime_error("Missing scooping container scene spec for scoop frame transform");
    }

    geometry_msgs::msg::Pose task_frame_pose = it->pose;
    task_frame_pose.orientation.x = 0.0;
    task_frame_pose.orientation.y = 0.0;
    task_frame_pose.orientation.z = 0.0;
    task_frame_pose.orientation.w = 1.0;
    scoop_frame_from_goal_ = tf_from_pose(task_frame_pose);
    has_scoop_frame_transform_ = true;
  }

  geometry_msgs::msg::Pose transform_pose_to_scoop_frame(const geometry_msgs::msg::Pose& pose) const
  {
    if (!has_scoop_frame_transform_) {
      return pose;
    }
    return transform_pose(pose, scoop_frame_from_goal_.inverse());
  }

  geometry_msgs::msg::Pose convert_loaded_scoop_pose(
    const geometry_msgs::msg::Pose& pose,
    const std::string& source_frame) const
  {
    if (source_frame.empty() || source_frame == scoop_frame_id_) {
      return pose;
    }
    if (source_frame == goal_frame_id_) {
      return transform_pose_to_scoop_frame(pose);
    }
    throw std::runtime_error(
      "Unsupported scoop pose frame '" + source_frame + "' in " + poses_yaml_path_);
  }

  int find_pose_index(const std::string& marker_name) const
  {
    for (std::size_t i = 0; i < seeds_.size(); ++i) {
      if (seeds_[i].marker_name == marker_name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  }

  void update_status(
    const std::string& message,
    const std::array<float, 3>& color = {0.95f, 0.95f, 0.95f})
  {
    status_text_ = message;
    status_color_ = color;
    publish_legend();
  }

  std::string scoop_frame_id_;
  std::string goal_frame_id_;
  std::string poses_yaml_path_;
  std::string seed_poses_yaml_path_;
  std::string tool_mesh_resource_;
  tf2::Transform scoop_frame_from_goal_;
  bool has_scoop_frame_transform_{false};
  std::array<MarkerSeed, 5> seeds_{};
  MarkerSeed goal_seed_{};
  geometry_msgs::msg::Pose goal_pose_{};
  int focus_index_{-1};
  std::string status_text_{"Waiting for marker updates"};
  std::array<float, 3> status_color_{0.95f, 0.95f, 0.95f};
  rclcpp::TimerBase::SharedPtr init_timer_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> scoop_server_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> goal_server_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr scoop_poses_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr scoop_focus_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_cmd_sub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr legend_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_srv_;
  std::vector<geometry_msgs::msg::Pose> poses_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScoopingMarkerServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
