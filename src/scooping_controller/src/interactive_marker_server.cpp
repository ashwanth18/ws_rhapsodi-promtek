#include "scooping_controller/container_collision_objects.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/cell_layout_active.hpp>
#include <robot_common_msgs/srv/list_scoop_pose_sets.hpp>
#include <robot_common_msgs/srv/load_scoop_pose_set.hpp>
#include <robot_common_msgs/srv/save_scoop_pose_set.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
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

// Keep the RViz tool mesh aligned with the real tcp_link definition in
// `niryo_ned3pro.urdf.xacro` (`tool_tcp_joint`).
constexpr double kTcpOffsetX = 0.15825;
constexpr double kTcpOffsetY = 0.0;
constexpr double kTcpOffsetZ = -0.09356;
constexpr char kDefaultGoalFrame[] = "base_link";
constexpr char kDefaultScoopFrame[] = "scooping_container_frame";
constexpr char kDefaultTaskContainerId[] = "rs6";
constexpr char kDefaultPoseSetId[] = "default";

std::string utc_timestamp_compact()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y%m%dT%H%M%SZ", &tm);
  return buf;
}

std::string utc_iso8601()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

std::string sanitize_pose_set_note(const std::string& note)
{
  std::string out;
  out.reserve(note.size());
  for (char ch : note) {
    if (std::isalnum(static_cast<unsigned char>(ch))) {
      out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(ch))));
    } else if (ch == '-' || ch == '_' || ch == ' ') {
      if (out.empty() || out.back() == '-') {
        continue;
      }
      out.push_back('-');
    }
    if (out.size() >= 40U) {
      break;
    }
  }
  while (!out.empty() && out.back() == '-') {
    out.pop_back();
  }
  return out;
}

bool is_safe_pose_set_id(const std::string& set_id)
{
  if (set_id.empty() || set_id == "." || set_id == ".." ||
      set_id.find('/') != std::string::npos ||
      set_id.find('\\') != std::string::npos)
  {
    return false;
  }
  return std::all_of(set_id.begin(), set_id.end(), [](unsigned char ch) {
    return std::isalnum(ch) || ch == '-' || ch == '_' || ch == 'T' || ch == 'Z';
  });
}

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
  const std::array<double, 3>& tcp_visual_offset_xyz,
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
    marker.pose.position.x = -tcp_visual_offset_xyz[0];
    marker.pose.position.y = -tcp_visual_offset_xyz[1];
    marker.pose.position.z = -tcp_visual_offset_xyz[2];
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
    this->declare_parameter<std::string>("layout_id", "");
    this->declare_parameter<std::string>("tool_id", "");
    this->declare_parameter<std::string>("authored_in", "");
    this->declare_parameter<std::string>("robot_key", "");
    this->declare_parameter<bool>("poses_provenance_ok", false);
    this->declare_parameter<std::string>("layouts_dir", "");
    this->declare_parameter<std::string>("poses_env", "real");
    this->declare_parameter<std::string>("poses_yaml", "");
    this->declare_parameter<std::string>("seed_poses_yaml", "");
    this->declare_parameter<bool>("auto_load_poses_on_startup", true);
    this->declare_parameter<std::string>(
      "tool_mesh_resource",
      "package://niryo_robot_description/meshes/ned3pro/stl/niryo_scoop_v4-ros.STL");
    this->declare_parameter<std::vector<double>>(
      "tcp_visual_offset_xyz",
      {kTcpOffsetX, kTcpOffsetY, kTcpOffsetZ});
    scooping_controller::declare_container_scene_parameters(*this);
    scoop_frame_id_ = this->get_parameter("scoop_frame_id").as_string();
    goal_frame_id_ = this->get_parameter("goal_frame_id").as_string();
    layouts_dir_ = expand_user_path(this->get_parameter("layouts_dir").as_string());
    poses_env_ = this->get_parameter("poses_env").as_string();
    if (poses_env_.empty()) {
      poses_env_ = "real";
    }
    refresh_pose_paths(this->get_parameter("layout_id").as_string());
    tool_mesh_resource_ = this->get_parameter("tool_mesh_resource").as_string();
    const auto tcp_visual_offset =
      this->get_parameter("tcp_visual_offset_xyz").as_double_array();
    if (tcp_visual_offset.size() == tcp_visual_offset_xyz_.size()) {
      std::copy(
        tcp_visual_offset.begin(),
        tcp_visual_offset.end(),
        tcp_visual_offset_xyz_.begin());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "tcp_visual_offset_xyz must contain exactly 3 values; using defaults");
    }
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
    pose_fault_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/cell_layout/pose_fault", rclcpp::QoS(1).transient_local().reliable());
    layout_sub_ = this->create_subscription<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ScoopingMarkerServer::handle_active_layout, this, std::placeholders::_1));

    save_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/save_scoop_poses",
      std::bind(&ScoopingMarkerServer::handle_save_request, this, std::placeholders::_1, std::placeholders::_2));
    load_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/load_scoop_poses",
      std::bind(&ScoopingMarkerServer::handle_load_request, this, std::placeholders::_1, std::placeholders::_2));
    export_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/export_scoop_poses",
      std::bind(&ScoopingMarkerServer::handle_export_request, this, std::placeholders::_1, std::placeholders::_2));
    save_pose_set_srv_ = this->create_service<robot_common_msgs::srv::SaveScoopPoseSet>(
      "/save_scoop_pose_set",
      std::bind(
        &ScoopingMarkerServer::handle_save_pose_set_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    list_pose_sets_srv_ = this->create_service<robot_common_msgs::srv::ListScoopPoseSets>(
      "/list_scoop_pose_sets",
      std::bind(
        &ScoopingMarkerServer::handle_list_pose_sets_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    load_pose_set_srv_ = this->create_service<robot_common_msgs::srv::LoadScoopPoseSet>(
      "/load_scoop_pose_set",
      std::bind(
        &ScoopingMarkerServer::handle_load_pose_set_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

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
      loaded_saved_poses = load_poses_from_yaml(
        poses_yaml_path_, load_message, /*strict_authored_in=*/true);
      if (loaded_saved_poses) {
        update_status(load_message, {0.75f, 0.95f, 0.75f});
        RCLCPP_INFO(this->get_logger(), "%s", load_message.c_str());
      } else if (
        !seed_poses_yaml_path_.empty() &&
        std::filesystem::exists(seed_poses_yaml_path_))
      {
        // Device-local cache may be stale after a layout change; try versioned seed.
        loaded_saved_poses = load_poses_from_yaml(
          seed_poses_yaml_path_, load_message, /*strict_authored_in=*/false);
        if (loaded_saved_poses) {
          std::string save_message;
          (void)save_poses_to_yaml(poses_yaml_path_, save_message);
          update_status(load_message, {0.75f, 0.95f, 0.75f});
          RCLCPP_INFO(this->get_logger(), "%s", load_message.c_str());
        } else {
          RCLCPP_WARN(
            this->get_logger(), "Auto-load scoop poses failed: %s", load_message.c_str());
        }
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
        make_tool_mesh_marker(
          seed.color, tool_mesh_resource_, tcp_visual_offset_xyz_, false, true));
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
        make_tool_mesh_marker(
          goal_seed_.color, tool_mesh_resource_, tcp_visual_offset_xyz_, false, true));
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
    // KEEP_ALIVE carries the client's last-known pose and was stomping Load Pose
    // Set / programmatic updates. Only apply intentional drag events.
    if (feedback->event_type == Feedback::KEEP_ALIVE ||
        feedback->event_type == Feedback::MENU_SELECT ||
        feedback->event_type == Feedback::BUTTON_CLICK)
    {
      return;
    }
    if (feedback->event_type != Feedback::POSE_UPDATE &&
        feedback->event_type != Feedback::MOUSE_UP &&
        feedback->event_type != Feedback::MOUSE_DOWN)
    {
      return;
    }
    if (this->now() < ignore_marker_feedback_until_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "[pose_set] ignoring marker feedback during post-load gate (%s event=%u)",
        feedback->marker_name.c_str(),
        static_cast<unsigned>(feedback->event_type));
      return;
    }

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
        poses_[i] = clamp_to_envelope(feedback->pose);
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
    scoop_server_->applyChanges();
    for (std::size_t i = 0; i < seeds_.size(); ++i) {
      if (focus_index_ >= 0 && static_cast<int>(i) != focus_index_) {
        continue;
      }
      insert_pose_marker(i);
    }
    scoop_server_->applyChanges();
  }

  void push_poses_to_rviz_markers()
  {
    // Gate client KEEP_ALIVE / stale POSE_UPDATE so they cannot undo the load.
    ignore_marker_feedback_until_ = this->now() + rclcpp::Duration::from_seconds(0.75);
    focus_index_ = -1;
    if (!scoop_server_) {
      publish_pose_array();
      return;
    }
    // Prefer setPose (keeps controls, forces RViz pose sync); fall back to rebuild.
    bool all_set = true;
    for (std::size_t i = 0; i < seeds_.size(); ++i) {
      if (!scoop_server_->setPose(seeds_[i].marker_name, poses_[i])) {
        all_set = false;
        break;
      }
    }
    if (all_set) {
      scoop_server_->applyChanges();
      RCLCPP_INFO(
        this->get_logger(),
        "[pose_set] pushed %zu poses to RViz via InteractiveMarkerServer::setPose",
        poses_.size());
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "[pose_set] setPose incomplete; rebuilding scoop interactive markers");
      rebuild_scoop_markers();
    }
    publish_pose_array();
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
    for (auto& pose : poses_) {
      pose = clamp_to_envelope(pose);
    }
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
    RCLCPP_INFO(this->get_logger(), "[pose_set] /save_scoop_poses → save_pose_set");
    std::string set_id;
    std::string path;
    response->success = save_pose_set(
      /*note=*/"", /*overwrite_set_id=*/"", response->message, set_id, path);
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "[pose_set] %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "[pose_set] %s", response->message.c_str());
    }
    update_status(
      response->message,
      response->success ? std::array<float, 3>{0.75f, 0.95f, 0.75f}
                        : std::array<float, 3>{0.95f, 0.45f, 0.45f});
  }

  void handle_load_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[pose_set] /load_scoop_poses cache=%s seed=%s",
      poses_yaml_path_.c_str(),
      seed_poses_yaml_path_.c_str());
    // Prefer the device-local cache; if missing/refused, reseed from the
    // versioned layout poses so Load picks up repo edits.
    bool loaded = false;
    if (std::filesystem::exists(poses_yaml_path_)) {
      loaded = load_poses_from_yaml(
        poses_yaml_path_, response->message, /*strict_authored_in=*/true);
    } else {
      response->message = "Device-local poses not found: " + poses_yaml_path_;
      RCLCPP_WARN(this->get_logger(), "[pose_set] %s", response->message.c_str());
    }
    if (!loaded && !seed_poses_yaml_path_.empty() &&
        std::filesystem::exists(seed_poses_yaml_path_))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "[pose_set] cache load failed (%s); falling back to seed %s",
        response->message.c_str(),
        seed_poses_yaml_path_.c_str());
      loaded = load_poses_from_yaml(
        seed_poses_yaml_path_, response->message, /*strict_authored_in=*/false);
      if (loaded) {
        std::string save_message;
        if (save_poses_to_yaml(poses_yaml_path_, save_message)) {
          response->message =
            "Reseeded from versioned layout poses: " + response->message;
        } else {
          RCLCPP_WARN(this->get_logger(), "[pose_set] %s", save_message.c_str());
        }
      }
    }
    response->success = loaded;
    if (loaded) {
      RCLCPP_INFO(this->get_logger(), "[pose_set] %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "[pose_set] %s", response->message.c_str());
    }
    update_status(
      response->message,
      loaded ? std::array<float, 3>{0.75f, 0.95f, 0.75f}
             : std::array<float, 3>{0.95f, 0.45f, 0.45f});
  }

  void handle_export_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Export no longer overwrites layout poses.yaml; it creates a timestamped set.
    RCLCPP_INFO(this->get_logger(), "[pose_set] /export_scoop_poses → save_pose_set(note=export)");
    std::string set_id;
    std::string path;
    response->success = save_pose_set(
      /*note=*/"export", /*overwrite_set_id=*/"", response->message, set_id, path);
    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "[pose_set] %s", response->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "[pose_set] %s", response->message.c_str());
    }
    update_status(
      response->message,
      response->success ? std::array<float, 3>{0.75f, 0.95f, 0.75f}
                        : std::array<float, 3>{0.95f, 0.45f, 0.45f});
  }

  void handle_save_pose_set_request(
    const std::shared_ptr<robot_common_msgs::srv::SaveScoopPoseSet::Request> request,
    std::shared_ptr<robot_common_msgs::srv::SaveScoopPoseSet::Response> response)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[pose_set] SaveScoopPoseSet request note='%s' set_id='%s'",
      request->note.c_str(),
      request->set_id.c_str());
    response->success = save_pose_set(
      request->note,
      request->set_id,
      response->message,
      response->set_id,
      response->path);
    if (response->success) {
      RCLCPP_INFO(
        this->get_logger(),
        "[pose_set] SaveScoopPoseSet OK set_id=%s path=%s — %s",
        response->set_id.c_str(),
        response->path.c_str(),
        response->message.c_str());
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "[pose_set] SaveScoopPoseSet FAILED — %s",
        response->message.c_str());
    }
    update_status(
      response->message,
      response->success ? std::array<float, 3>{0.75f, 0.95f, 0.75f}
                        : std::array<float, 3>{0.95f, 0.45f, 0.45f});
  }

  void handle_list_pose_sets_request(
    const std::shared_ptr<robot_common_msgs::srv::ListScoopPoseSets::Request> /*request*/,
    std::shared_ptr<robot_common_msgs::srv::ListScoopPoseSets::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "[pose_set] ListScoopPoseSets request");
    response->success = list_pose_sets(
      response->message, response->set_ids, response->created_at, response->notes);
    if (response->success) {
      RCLCPP_INFO(
        this->get_logger(),
        "[pose_set] ListScoopPoseSets OK count=%zu — %s",
        response->set_ids.size(),
        response->message.c_str());
      for (std::size_t i = 0; i < response->set_ids.size(); ++i) {
        const auto note =
          i < response->notes.size() ? response->notes[i] : std::string{};
        const auto created =
          i < response->created_at.size() ? response->created_at[i] : std::string{};
        RCLCPP_INFO(
          this->get_logger(),
          "[pose_set]   [%zu] id=%s created=%s note=%s",
          i,
          response->set_ids[i].c_str(),
          created.empty() ? "-" : created.c_str(),
          note.empty() ? "-" : note.c_str());
      }
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "[pose_set] ListScoopPoseSets FAILED — %s",
        response->message.c_str());
    }
  }

  void handle_load_pose_set_request(
    const std::shared_ptr<robot_common_msgs::srv::LoadScoopPoseSet::Request> request,
    std::shared_ptr<robot_common_msgs::srv::LoadScoopPoseSet::Response> response)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "[pose_set] LoadScoopPoseSet request set_id='%s' layout_id=%s "
      "task_container_id=%s tool_id=%s active_hash=%s seed=%s cache=%s",
      request->set_id.c_str(),
      this->get_parameter("layout_id").as_string().c_str(),
      this->get_parameter("task_container_id").as_string().c_str(),
      this->get_parameter("tool_id").as_string().c_str(),
      active_layout_hash_.c_str(),
      seed_poses_yaml_path_.c_str(),
      poses_yaml_path_.c_str());
    response->success = load_pose_set(request->set_id, response->message);
    if (response->success) {
      RCLCPP_INFO(
        this->get_logger(),
        "[pose_set] LoadScoopPoseSet OK — %s",
        response->message.c_str());
      for (std::size_t i = 0; i < poses_.size() && i < seeds_.size(); ++i) {
        RCLCPP_INFO(
          this->get_logger(),
          "[pose_set]   marker %s xyz=[%.4f, %.4f, %.4f]",
          seeds_[i].marker_name.c_str(),
          poses_[i].position.x,
          poses_[i].position.y,
          poses_[i].position.z);
      }
    } else {
      RCLCPP_ERROR(
        this->get_logger(),
        "[pose_set] LoadScoopPoseSet FAILED set_id='%s' — %s",
        request->set_id.c_str(),
        response->message.c_str());
    }
    update_status(
      response->message,
      response->success ? std::array<float, 3>{0.75f, 0.95f, 0.75f}
                        : std::array<float, 3>{0.95f, 0.45f, 0.45f});
  }

  void refresh_pose_paths(const std::string& layout_id)
  {
    const auto explicit_poses = this->get_parameter("poses_yaml").as_string();
    const auto explicit_seed = this->get_parameter("seed_poses_yaml").as_string();

    if (!layout_id.empty()) {
      poses_yaml_path_ = expand_user_path(
        "~/.ros/scooping_controller/poses_" + poses_env_ + "_" + layout_id + ".yaml");
    } else if (!explicit_poses.empty()) {
      poses_yaml_path_ = expand_user_path(explicit_poses);
    } else {
      poses_yaml_path_ = expand_user_path(
        "~/.ros/scooping_controller/poses_" + poses_env_ + ".yaml");
    }

    if (!layouts_dir_.empty() && !layout_id.empty()) {
      seed_poses_yaml_path_ =
        (std::filesystem::path(layouts_dir_) / layout_id / "poses.yaml").string();
    } else if (!explicit_seed.empty()) {
      seed_poses_yaml_path_ = expand_user_path(explicit_seed);
    } else {
      seed_poses_yaml_path_.clear();
    }

    this->set_parameters({
      rclcpp::Parameter("poses_yaml", poses_yaml_path_),
      rclcpp::Parameter("seed_poses_yaml", seed_poses_yaml_path_)});
  }

  std::filesystem::path pose_sets_dir() const
  {
    const auto layout_id = this->get_parameter("layout_id").as_string();
    return std::filesystem::path(layouts_dir_) / layout_id / "poses" / "sets";
  }

  bool save_poses_to_yaml(
    const std::string& path,
    std::string& message,
    const std::string& set_id = "",
    const std::string& created_at = "",
    const std::string& note = "")
  {
    try {
      if (path.empty()) {
        message = "Pose YAML path is empty";
        return false;
      }
      const std::filesystem::path dest(path);
      if (!dest.parent_path().empty()) {
        std::filesystem::create_directories(dest.parent_path());
      }

      YAML::Node root;
      root["frame_id"] = scoop_frame_id_;
      root["layout_id"] = this->get_parameter("layout_id").as_string();
      root["task_container_id"] = this->get_parameter("task_container_id").as_string();
      root["tool_id"] = this->get_parameter("tool_id").as_string();
      root["container_spec_hash"] = active_layout_hash_;
      root["authored_in"] = this->get_parameter("authored_in").as_string();
      root["robot_key"] = this->get_parameter("robot_key").as_string();
      if (!set_id.empty()) {
        root["set_id"] = set_id;
      }
      if (!created_at.empty()) {
        root["created_at"] = created_at;
      }
      if (!note.empty() || !set_id.empty()) {
        root["note"] = note;
      }
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

      std::ofstream out(path, std::ios::out | std::ios::trunc);
      if (!out.is_open()) {
        message = "Failed to open pose YAML for writing: " + path;
        return false;
      }

      out << root;
      message = "Saved scoop poses to " + path;
      return true;
    } catch (const std::exception& ex) {
      message = std::string("Failed to save scoop poses: ") + ex.what();
      return false;
    }
  }

  bool save_pose_set(
    const std::string& note,
    const std::string& overwrite_set_id,
    std::string& message,
    std::string& set_id,
    std::string& path)
  {
    const auto layout_id = this->get_parameter("layout_id").as_string();
    if (layouts_dir_.empty() || layout_id.empty()) {
      message = "Saving a pose set requires layouts_dir and an active layout_id";
      RCLCPP_ERROR(
        this->get_logger(),
        "[pose_set] save rejected: layouts_dir='%s' layout_id='%s'",
        layouts_dir_.c_str(),
        layout_id.c_str());
      return false;
    }
    if (poses_.size() != seeds_.size()) {
      message = "Scoop poses are not initialized";
      RCLCPP_ERROR(
        this->get_logger(),
        "[pose_set] save rejected: poses=%zu seeds=%zu",
        poses_.size(),
        seeds_.size());
      return false;
    }

    const auto sets_dir = pose_sets_dir();
    std::error_code ec;
    std::filesystem::create_directories(sets_dir, ec);
    if (ec) {
      message = "Failed to create pose sets directory: " + sets_dir.string() + " (" + ec.message() + ")";
      RCLCPP_ERROR(this->get_logger(), "[pose_set] %s", message.c_str());
      return false;
    }

    std::string created_at = utc_iso8601();
    std::string note_to_write = note;
    const bool overwrite = !overwrite_set_id.empty();
    if (overwrite) {
      if (overwrite_set_id == kDefaultPoseSetId) {
        message = "Refusing to overwrite the layout default poses.yaml via pose-set update";
        RCLCPP_ERROR(this->get_logger(), "[pose_set] %s", message.c_str());
        return false;
      }
      if (!is_safe_pose_set_id(overwrite_set_id)) {
        message = "Invalid pose set id for update: " + overwrite_set_id;
        RCLCPP_ERROR(this->get_logger(), "[pose_set] %s", message.c_str());
        return false;
      }
      set_id = overwrite_set_id;
      path = (sets_dir / (set_id + ".yaml")).string();
      if (!std::filesystem::exists(path)) {
        message = "Pose set does not exist to update: " + set_id;
        RCLCPP_ERROR(this->get_logger(), "[pose_set] %s", message.c_str());
        return false;
      }
      try {
        const YAML::Node existing = YAML::LoadFile(path);
        if (existing["created_at"] && existing["created_at"].IsScalar()) {
          created_at = existing["created_at"].as<std::string>();
        }
        if (note_to_write.empty() && existing["note"] && existing["note"].IsScalar()) {
          note_to_write = existing["note"].as<std::string>();
        }
      } catch (const std::exception& ex) {
        RCLCPP_WARN(
          this->get_logger(),
          "[pose_set] could not read existing metadata for %s (%s); rewriting with new created_at",
          set_id.c_str(),
          ex.what());
      }
    } else {
      const auto stamp = utc_timestamp_compact();
      const auto sanitized = sanitize_pose_set_note(note);
      set_id = sanitized.empty() ? stamp : (stamp + "_" + sanitized);
      path = (sets_dir / (set_id + ".yaml")).string();
    }

    RCLCPP_INFO(
      this->get_logger(),
      "[pose_set] %s set_id=%s created_at=%s path=%s",
      overwrite ? "updating" : "writing",
      set_id.c_str(),
      created_at.c_str(),
      path.c_str());
    if (!save_poses_to_yaml(path, message, set_id, created_at, note_to_write)) {
      RCLCPP_ERROR(this->get_logger(), "[pose_set] write failed — %s", message.c_str());
      return false;
    }

    // Refresh device working cache for session continuity; never touch poses.yaml.
    std::string cache_message;
    if (!save_poses_to_yaml(poses_yaml_path_, cache_message)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[pose_set] set saved but device cache refresh failed — %s",
        cache_message.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "[pose_set] refreshed device cache %s (default poses.yaml untouched)",
        poses_yaml_path_.c_str());
    }
    message = (overwrite ? "Updated pose set '" : "Saved pose set '") + set_id + "' to " + path +
      " (default poses.yaml unchanged)";
    return true;
  }

  bool list_pose_sets(
    std::string& message,
    std::vector<std::string>& set_ids,
    std::vector<std::string>& created_at,
    std::vector<std::string>& notes)
  {
    set_ids.clear();
    created_at.clear();
    notes.clear();

    const auto layout_id = this->get_parameter("layout_id").as_string();
    if (layouts_dir_.empty() || layout_id.empty()) {
      message = "Listing pose sets requires layouts_dir and an active layout_id";
      RCLCPP_ERROR(
        this->get_logger(),
        "[pose_set] list rejected: layouts_dir='%s' layout_id='%s'",
        layouts_dir_.c_str(),
        layout_id.c_str());
      return false;
    }

    set_ids.push_back(kDefaultPoseSetId);
    created_at.push_back("");
    notes.push_back("layout default (poses.yaml)");

    const auto sets_dir = pose_sets_dir();
    if (std::filesystem::exists(sets_dir) && std::filesystem::is_directory(sets_dir)) {
      std::vector<std::filesystem::directory_entry> entries;
      for (const auto& entry : std::filesystem::directory_iterator(sets_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
          entries.push_back(entry);
        }
      }
      std::sort(
        entries.begin(), entries.end(),
        [](const std::filesystem::directory_entry& a,
           const std::filesystem::directory_entry& b) {
          return a.path().filename().string() > b.path().filename().string();
        });
      for (const auto& entry : entries) {
        const auto stem = entry.path().stem().string();
        std::string created;
        std::string note;
        try {
          const auto root = YAML::LoadFile(entry.path().string());
          if (root["created_at"]) {
            created = root["created_at"].as<std::string>();
          }
          if (root["note"]) {
            note = root["note"].as<std::string>();
          }
        } catch (const std::exception& ex) {
          RCLCPP_WARN(
            this->get_logger(),
            "Could not read pose set metadata from %s: %s",
            entry.path().c_str(),
            ex.what());
        }
        set_ids.push_back(stem);
        created_at.push_back(created);
        notes.push_back(note);
      }
    }

    message = "Found " + std::to_string(set_ids.size()) + " pose set(s) including default";
    return true;
  }

  bool load_pose_set(const std::string& set_id, std::string& message)
  {
    const auto layout_id = this->get_parameter("layout_id").as_string();
    if (layouts_dir_.empty() || layout_id.empty()) {
      message = "Loading a pose set requires layouts_dir and an active layout_id";
      RCLCPP_ERROR(
        this->get_logger(),
        "[pose_set] load rejected: layouts_dir='%s' layout_id='%s'",
        layouts_dir_.c_str(),
        layout_id.c_str());
      return false;
    }
    if (set_id.empty()) {
      message = "Pose set id is empty";
      RCLCPP_ERROR(this->get_logger(), "[pose_set] load rejected: empty set_id");
      return false;
    }

    std::string path;
    if (set_id == kDefaultPoseSetId) {
      path = seed_poses_yaml_path_;
      if (path.empty() || !std::filesystem::exists(path)) {
        message = "Default poses.yaml not found for layout " + layout_id;
        RCLCPP_ERROR(
          this->get_logger(),
          "[pose_set] default seed missing path='%s'",
          path.c_str());
        return false;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "[pose_set] resolving set_id=default → seed %s",
        path.c_str());
    } else {
      if (!is_safe_pose_set_id(set_id)) {
        message = "Invalid pose set id";
        RCLCPP_ERROR(
          this->get_logger(),
          "[pose_set] load rejected: unsafe set_id='%s'",
          set_id.c_str());
        return false;
      }
      path = (pose_sets_dir() / (set_id + ".yaml")).string();
      if (!std::filesystem::exists(path)) {
        message = "Pose set not found: " + path;
        RCLCPP_ERROR(this->get_logger(), "[pose_set] %s", message.c_str());
        return false;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "[pose_set] resolving set_id=%s → %s",
        set_id.c_str(),
        path.c_str());
    }

    // Pose-set loads are layout-scoped; adopt container/tool from the file so a
    // stale default (e.g. rs6) does not refuse lightsout rs3 seeds.
    if (!load_poses_from_yaml(
          path, message, /*strict_authored_in=*/false, /*adopt_file_contract=*/true))
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "[pose_set] load_poses_from_yaml failed for %s — %s",
        path.c_str(),
        message.c_str());
      return false;
    }
    std::string cache_message;
    if (!save_poses_to_yaml(poses_yaml_path_, cache_message)) {
      RCLCPP_WARN(
        this->get_logger(),
        "[pose_set] markers loaded but device cache refresh failed — %s",
        cache_message.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "[pose_set] device cache refreshed → %s",
        poses_yaml_path_.c_str());
    }
    message = "Loaded pose set '" + set_id + "' from " + path;
    if (!poses_.empty()) {
      message += " · approach [" +
        std::to_string(poses_[0].position.x) + ", " +
        std::to_string(poses_[0].position.y) + ", " +
        std::to_string(poses_[0].position.z) + "]";
    }
    update_status(message, {0.75f, 0.95f, 0.75f});
    return true;
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

      return save_poses_to_yaml(poses_yaml_path_, message);
    } catch (const std::exception& ex) {
      message = std::string("Failed to seed scoop poses: ") + ex.what();
      return false;
    }
  }

  bool load_poses_from_yaml(
    const std::string& path,
    std::string& message,
    bool strict_authored_in,
    bool adopt_file_contract = false)
  {
    try {
      if (path.empty() || !std::filesystem::exists(path)) {
        message = "Pose YAML not found: " + path;
        return false;
      }

      const YAML::Node root = YAML::LoadFile(path);
      const auto fail_provenance = [this, &message, &path](const std::string& reason) {
          message = "Refusing pose YAML provenance: " + reason;
          RCLCPP_ERROR(
            this->get_logger(),
            "[pose_set] provenance refuse path=%s reason=%s "
            "(layout_id=%s task_container_id=%s tool_id=%s frame=%s active_hash=%s)",
            path.c_str(),
            reason.c_str(),
            this->get_parameter("layout_id").as_string().c_str(),
            this->get_parameter("task_container_id").as_string().c_str(),
            this->get_parameter("tool_id").as_string().c_str(),
            scoop_frame_id_.c_str(),
            active_layout_hash_.c_str());
          std_msgs::msg::String fault;
          fault.data = message;
          pose_fault_pub_->publish(fault);
          this->set_parameter(rclcpp::Parameter("poses_provenance_ok", false));
          return false;
        };
      for (const auto& field : {"layout_id", "task_container_id", "frame_id", "tool_id",
                                "container_spec_hash", "authored_in"}) {
        if (!root[field]) {
          return fail_provenance(std::string("missing ") + field);
        }
      }
      const auto expected_robot = this->get_parameter("robot_key").as_string();
      if (!expected_robot.empty() && root["robot_key"]) {
        const auto file_robot = root["robot_key"].as<std::string>();
        if (!file_robot.empty() && file_robot != expected_robot) {
          return fail_provenance(
            "robot_key mismatch (file=" + file_robot + ", expected=" + expected_robot + ")");
        }
      }
      const auto expected_layout = this->get_parameter("layout_id").as_string();
      const auto expected_container =
        this->get_parameter("task_container_id").as_string();
      const auto expected_tool = this->get_parameter("tool_id").as_string();
      const auto expected_authored = this->get_parameter("authored_in").as_string();
      const auto file_hash = root["container_spec_hash"].as<std::string>();
      const auto file_layout = root["layout_id"].as<std::string>();
      const auto file_container = root["task_container_id"].as<std::string>();
      const auto file_frame = root["frame_id"].as<std::string>();
      const auto file_tool = root["tool_id"].as<std::string>();
      if (file_layout != expected_layout || file_frame != scoop_frame_id_) {
        return fail_provenance("layout or frame mismatch");
      }
      if (adopt_file_contract) {
        if (file_container != expected_container ||
            (!expected_tool.empty() && file_tool != expected_tool) ||
            (expected_tool.empty() && !file_tool.empty()))
        {
          this->set_parameters({
            rclcpp::Parameter("task_container_id", file_container),
            rclcpp::Parameter("tool_id", file_tool)});
          RCLCPP_INFO(
            this->get_logger(),
            "Adopted pose-set contract task_container_id=%s tool_id=%s",
            file_container.c_str(),
            file_tool.c_str());
        }
      } else if (
        file_container != expected_container ||
        (!expected_tool.empty() && file_tool != expected_tool))
      {
        return fail_provenance("layout, container, frame, or tool mismatch");
      }
      // Versioned seeds may carry an empty hash until first export. For the
      // strict device-local path, refuse empty vs non-empty. For seed fallback
      // (strict_authored_in=false), allow empty hash when layout_id already
      // matched so the first apply can bootstrap and re-stamp.
      // Pose-set loads (adopt_file_contract) ignore hash so authoring preview
      // (layout_hash=preview) can still switch between saved sets and default.
      if (!adopt_file_contract) {
        if (!file_hash.empty() && file_hash != active_layout_hash_) {
          return fail_provenance("container_spec_hash mismatch");
        }
        if (!active_layout_hash_.empty() && file_hash.empty() && strict_authored_in) {
          return fail_provenance("container_spec_hash missing for active layout");
        }
      }
      if (strict_authored_in && !expected_authored.empty() &&
          root["authored_in"].as<std::string>() != expected_authored) {
        return fail_provenance("authored_in mismatch");
      }
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

        poses_[static_cast<std::size_t>(index)] =
          clamp_to_envelope(convert_loaded_scoop_pose(pose, source_frame));
        ++updated;
      }

      push_poses_to_rviz_markers();
      this->set_parameter(rclcpp::Parameter("poses_provenance_ok", true));
      // Clear any latched pose_fault so VerifyCellLayout can pass after a
      // prior cache/seed refuse in the same apply (transient_local QoS).
      {
        std_msgs::msg::String cleared;
        cleared.data = "";
        pose_fault_pub_->publish(cleared);
      }
      message = "Loaded " + std::to_string(updated) + " scoop poses from " + path;
      if (updated > 0 && !poses_.empty()) {
        message += " (approach xyz=" +
          std::to_string(poses_[0].position.x) + "," +
          std::to_string(poses_[0].position.y) + "," +
          std::to_string(poses_[0].position.z) + ")";
      }
      RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
      return updated > 0;
    } catch (const std::exception& ex) {
      message = std::string("Failed to load scoop poses: ") + ex.what();
      return false;
    }
  }

  void handle_active_layout(const robot_common_msgs::msg::CellLayoutActive::SharedPtr layout)
  {
    const bool layout_identity_changed =
      layout->layout_id != applied_layout_id_ ||
      layout->layout_hash != active_layout_hash_;

    active_layout_hash_ = layout->layout_hash;
    this->set_parameters({
      rclcpp::Parameter("layout_id", layout->layout_id),
      rclcpp::Parameter("task_container_id", layout->task_container_id),
      rclcpp::Parameter("tool_id", layout->tool_id)});
    refresh_pose_paths(layout->layout_id);
    try {
      const auto root = YAML::LoadFile(layout->scene_yaml_path);
      const auto envelope = root["scoop_envelope"];
      if (!envelope) {
        throw std::runtime_error("layout missing scoop_envelope");
      }
      envelope_ = {envelope["x_min"].as<double>(), envelope["x_max"].as<double>(),
        envelope["y_min"].as<double>(), envelope["y_max"].as<double>(),
        envelope["z_min"].as<double>(), envelope["z_max"].as<double>()};
      has_envelope_ = true;
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(this->get_logger(), "Invalid active layout envelope: %s", ex.what());
      has_envelope_ = false;
    }

    // Only (re)load scoop poses when the active layout identity changes.
    // Republishes of the same preview/applied layout were stomping Load Pose Set
    // and unsaved marker drags by reloading the device cache.
    if (!layout_identity_changed) {
      RCLCPP_DEBUG(
        this->get_logger(),
        "Ignoring duplicate /cell_layout/active for %s hash=%s (poses unchanged)",
        layout->layout_id.c_str(),
        layout->layout_hash.c_str());
      return;
    }
    applied_layout_id_ = layout->layout_id;

    // Prefer the layout-scoped device-local cache; on provenance failure fall
    // back to the versioned seed under layouts_dir/<id>/poses.yaml.
    std::string load_message;
    bool loaded = false;
    if (std::filesystem::exists(poses_yaml_path_)) {
      loaded = load_poses_from_yaml(
        poses_yaml_path_, load_message, /*strict_authored_in=*/true);
      if (!loaded) {
        RCLCPP_WARN(
          this->get_logger(),
          "Device-local poses refused for layout %s: %s",
          layout->layout_id.c_str(),
          load_message.c_str());
      }
    }
    if (!loaded && !seed_poses_yaml_path_.empty() &&
        std::filesystem::exists(seed_poses_yaml_path_))
    {
      loaded = load_poses_from_yaml(
        seed_poses_yaml_path_, load_message, /*strict_authored_in=*/false);
      if (loaded) {
        // Materialize a layout-scoped device-local cache from the versioned seed.
        std::string save_message;
        if (!save_poses_to_yaml(poses_yaml_path_, save_message)) {
          RCLCPP_WARN(this->get_logger(), "%s", save_message.c_str());
        }
        load_message = "Reseeded from versioned layout poses: " + load_message;
      }
    }
    if (loaded) {
      update_status(load_message, {0.75f, 0.95f, 0.75f});
      RCLCPP_INFO(this->get_logger(), "%s", load_message.c_str());
    } else {
      this->set_parameter(rclcpp::Parameter("poses_provenance_ok", false));
      std_msgs::msg::String fault;
      fault.data = load_message.empty()
        ? ("No provenance-matched poses for layout " + layout->layout_id)
        : load_message;
      pose_fault_pub_->publish(fault);
      update_status(fault.data, {0.95f, 0.45f, 0.45f});
      RCLCPP_ERROR(this->get_logger(), "%s", fault.data.c_str());
    }
  }

  geometry_msgs::msg::Pose clamp_to_envelope(geometry_msgs::msg::Pose pose) const
  {
    if (has_envelope_) {
      pose.position.x = std::clamp(pose.position.x, envelope_[0], envelope_[1]);
      pose.position.y = std::clamp(pose.position.y, envelope_[2], envelope_[3]);
      pose.position.z = std::clamp(pose.position.z, envelope_[4], envelope_[5]);
    }
    return pose;
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
  std::string layouts_dir_;
  std::string poses_env_;
  std::string poses_yaml_path_;
  std::string seed_poses_yaml_path_;
  std::string tool_mesh_resource_;
  std::string active_layout_hash_;
  std::string applied_layout_id_;
  std::array<double, 6> envelope_{};
  bool has_envelope_{false};
  std::array<double, 3> tcp_visual_offset_xyz_{
    kTcpOffsetX, kTcpOffsetY, kTcpOffsetZ};
  tf2::Transform scoop_frame_from_goal_;
  bool has_scoop_frame_transform_{false};
  std::array<MarkerSeed, 5> seeds_{};
  MarkerSeed goal_seed_{};
  geometry_msgs::msg::Pose goal_pose_{};
  int focus_index_{-1};
  rclcpp::Time ignore_marker_feedback_until_{0, 0, RCL_ROS_TIME};
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
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pose_fault_pub_;
  rclcpp::Subscription<robot_common_msgs::msg::CellLayoutActive>::SharedPtr layout_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr load_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr export_srv_;
  rclcpp::Service<robot_common_msgs::srv::SaveScoopPoseSet>::SharedPtr save_pose_set_srv_;
  rclcpp::Service<robot_common_msgs::srv::ListScoopPoseSets>::SharedPtr list_pose_sets_srv_;
  rclcpp::Service<robot_common_msgs::srv::LoadScoopPoseSet>::SharedPtr load_pose_set_srv_;
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
