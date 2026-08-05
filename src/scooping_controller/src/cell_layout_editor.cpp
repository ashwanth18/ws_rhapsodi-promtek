#include "scooping_controller/container_collision_objects.hpp"
#include "scooping_controller/touch_off.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/cell_layout_active.hpp>
#include <robot_common_msgs/srv/add_layout_object.hpp>
#include <robot_common_msgs/srv/preview_cell_layout.hpp>
#include <robot_common_msgs/srv/remove_layout_object.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

std::string expand_user_path(const std::string& path)
{
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char* home = std::getenv("HOME");
  if (!home) {
    return path;
  }
  return std::string(home) + path.substr(1);
}

std::string today_iso()
{
  const auto now = std::chrono::system_clock::now();
  const std::time_t t = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tm);
  return buf;
}

std::array<double, 3> quat_to_rpy_deg(const geometry_msgs::msg::Quaternion& q)
{
  tf2::Quaternion quat(q.x, q.y, q.z, q.w);
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  return {roll * kRadToDeg, pitch * kRadToDeg, yaw * kRadToDeg};
}

geometry_msgs::msg::Quaternion rpy_deg_to_quat_msg(const std::array<double, 3>& rpy)
{
  const auto q = scooping_controller::rpy_deg_to_quaternion(rpy);
  geometry_msgs::msg::Quaternion msg;
  msg.x = q[0];
  msg.y = q[1];
  msg.z = q[2];
  msg.w = q[3];
  return msg;
}

bool path_is_deploy_bundle(const std::filesystem::path& path)
{
  const auto normalized = path.lexically_normal().string();
  return normalized.find("/ws/config/layouts") != std::string::npos;
}

struct CatalogModel
{
  std::string model_id;
  std::string display_name;
  std::string category;
  std::string geometry_type;
  std::string mesh_resource;
  std::string mesh_units{"m"};
  std::array<double, 3> default_scale_xyz{1.0, 1.0, 1.0};
  std::array<double, 3> default_dimensions_xyz{0.0, 0.0, 0.0};
  std::array<double, 3> default_color_rgb{0.7, 0.7, 0.7};
  double rim_height_m{0.0};
};

}  // namespace

class CellLayoutEditor : public rclcpp::Node
{
public:
  using InteractiveMarker = visualization_msgs::msg::InteractiveMarker;
  using InteractiveMarkerControl = visualization_msgs::msg::InteractiveMarkerControl;
  using InteractiveMarkerFeedback = visualization_msgs::msg::InteractiveMarkerFeedback;
  using Marker = visualization_msgs::msg::Marker;
  using AddObject = robot_common_msgs::srv::AddLayoutObject;
  using RemoveObject = robot_common_msgs::srv::RemoveLayoutObject;
  using Preview = robot_common_msgs::srv::PreviewCellLayout;
  using Trigger = std_srvs::srv::Trigger;

  CellLayoutEditor()
  : Node("cell_layout_editor"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("frame_id", "base_link");
    declare_parameter<std::string>("layouts_dir", "");
    declare_parameter<std::string>("catalog_yaml", "");
    declare_parameter<std::string>("layout_yaml", "");
    declare_parameter<std::string>("layout_id", "");
    declare_parameter<std::string>("robot_key", "");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("tcp_frame", "tcp_link");
    declare_parameter<std::string>("operator_name", "");
    declare_parameter<bool>("planar_mode", true);
    declare_parameter<std::string>("selected_object_id", "");
    declare_parameter<double>("table_top_z", 0.0);

    frame_id_ = get_parameter("frame_id").as_string();
    layouts_dir_ = expand_user_path(get_parameter("layouts_dir").as_string());
    catalog_yaml_ = expand_user_path(get_parameter("catalog_yaml").as_string());
    layout_yaml_ = expand_user_path(get_parameter("layout_yaml").as_string());
    layout_id_ = get_parameter("layout_id").as_string();
    robot_key_ = get_parameter("robot_key").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    tcp_frame_ = get_parameter("tcp_frame").as_string();
    operator_name_ = get_parameter("operator_name").as_string();
    if (operator_name_.empty()) {
      const char* user = std::getenv("USER");
      operator_name_ = user ? user : "unknown";
    }
    planar_mode_ = get_parameter("planar_mode").as_bool();
    selected_object_id_ = get_parameter("selected_object_id").as_string();
    table_top_z_ = get_parameter("table_top_z").as_double();

    load_catalog();

    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/cell_layout/editor_status", rclcpp::QoS(1).transient_local().reliable());
    // Keep the same display topic as container_marker_publisher so the RViz
    // "Container" display keeps working while layout_edit replaces that node.
    marker_pub_ = create_publisher<Marker>(
      "/container_marker", rclcpp::QoS(1).transient_local().reliable());
    layout_sub_ = create_subscription<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable(),
      [this](const robot_common_msgs::msg::CellLayoutActive::SharedPtr msg) {
        handle_active_layout(msg);
      });

    preview_client_ = create_client<Preview>("/cell_layout/preview_objects");

    save_srv_ = create_service<Trigger>(
      "/cell_layout/save_objects",
      [this](const std::shared_ptr<Trigger::Request> req,
             std::shared_ptr<Trigger::Response> res) { handle_save(req, res); });
    reload_srv_ = create_service<Trigger>(
      "/cell_layout/reload_objects",
      [this](const std::shared_ptr<Trigger::Request> req,
             std::shared_ptr<Trigger::Response> res) { handle_reload(req, res); });
    add_srv_ = create_service<AddObject>(
      "/cell_layout/add_object",
      [this](const std::shared_ptr<AddObject::Request> req,
             std::shared_ptr<AddObject::Response> res) { handle_add(req, res); });
    remove_srv_ = create_service<RemoveObject>(
      "/cell_layout/remove_object",
      [this](const std::shared_ptr<RemoveObject::Request> req,
             std::shared_ptr<RemoveObject::Response> res) { handle_remove(req, res); });
    capture_srv_ = create_service<Trigger>(
      "/cell_layout/capture_touch_point",
      [this](const std::shared_ptr<Trigger::Request> req,
             std::shared_ptr<Trigger::Response> res) { handle_capture(req, res); });
    fit_srv_ = create_service<Trigger>(
      "/cell_layout/fit_touch_points",
      [this](const std::shared_ptr<Trigger::Request> req,
             std::shared_ptr<Trigger::Response> res) { handle_fit(req, res); });

    // Parameter callback for panel-driven planar toggle / selected object.
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
          if (param.get_name() == "planar_mode") {
            planar_mode_ = param.as_bool();
            rebuild_markers();
          } else if (param.get_name() == "selected_object_id") {
            selected_object_id_ = param.as_string();
          } else if (param.get_name() == "operator_name") {
            operator_name_ = param.as_string();
          }
        }
        return result;
      });

    init_timer_ = create_wall_timer(
      std::chrono::milliseconds(0),
      [this]() { deferred_init(); });
  }

private:
  void deferred_init()
  {
    init_timer_->cancel();
    server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
      "cell_layout_markers", shared_from_this());

    if (!layout_yaml_.empty() && std::filesystem::exists(layout_yaml_)) {
      load_from_yaml(layout_yaml_);
    } else if (!layouts_dir_.empty() && !layout_id_.empty()) {
      const auto path = std::filesystem::path(layouts_dir_) / (layout_id_ + ".yaml");
      if (std::filesystem::exists(path)) {
        layout_yaml_ = path.string();
        load_from_yaml(layout_yaml_);
      }
    }
    publish_status("Cell layout editor ready");
  }

  void load_catalog()
  {
    catalog_.clear();
    rim_by_mesh_.clear();
    if (catalog_yaml_.empty() || !std::filesystem::exists(catalog_yaml_)) {
      RCLCPP_WARN(get_logger(), "Catalog not found at '%s'", catalog_yaml_.c_str());
      return;
    }
    try {
      const auto root = YAML::LoadFile(catalog_yaml_);
      for (const auto& entry : root["models"]) {
        CatalogModel model;
        model.model_id = entry["model_id"].as<std::string>();
        model.display_name = entry["display_name"] ?
          entry["display_name"].as<std::string>() : model.model_id;
        model.category = entry["category"] ? entry["category"].as<std::string>() : "";
        model.geometry_type = entry["geometry_type"].as<std::string>();
        model.mesh_resource = entry["mesh_resource"] ?
          entry["mesh_resource"].as<std::string>() : "";
        model.mesh_units = entry["mesh_units"] ?
          entry["mesh_units"].as<std::string>() : "m";
        if (entry["default_scale_xyz"]) {
          for (std::size_t i = 0; i < 3; ++i) {
            model.default_scale_xyz[i] = entry["default_scale_xyz"][i].as<double>();
          }
        }
        if (entry["default_dimensions_xyz"]) {
          for (std::size_t i = 0; i < 3; ++i) {
            model.default_dimensions_xyz[i] =
              entry["default_dimensions_xyz"][i].as<double>();
          }
        }
        if (entry["default_color_rgb"]) {
          for (std::size_t i = 0; i < 3; ++i) {
            model.default_color_rgb[i] = entry["default_color_rgb"][i].as<double>();
          }
        }
        model.rim_height_m = entry["rim_height_m"] ?
          entry["rim_height_m"].as<double>() : 0.0;
        catalog_[model.model_id] = model;
        if (!model.mesh_resource.empty()) {
          rim_by_mesh_[model.mesh_resource] = model.rim_height_m;
        }
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu catalog models", catalog_.size());
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load catalog: %s", ex.what());
    }
  }

  const CatalogModel* find_model(const std::string& model_id) const
  {
    const auto it = catalog_.find(model_id);
    return it == catalog_.end() ? nullptr : &it->second;
  }

  double rim_height_for_spec(const scooping_controller::ContainerSceneSpec& spec) const
  {
    if (!spec.mesh_resource.empty()) {
      const auto it = rim_by_mesh_.find(spec.mesh_resource);
      if (it != rim_by_mesh_.end()) {
        return it->second;
      }
    }
    return 0.0;
  }

  void handle_active_layout(
    const robot_common_msgs::msg::CellLayoutActive::SharedPtr msg)
  {
    // Ignore preview republishes of our own scratch YAML to avoid feedback loops.
    if (msg->preview && !preview_path_.empty() &&
        msg->scene_yaml_path == preview_path_)
    {
      return;
    }
    layout_id_ = msg->layout_id;
    if (!msg->preview) {
      // Prefer the versioned layout YAML as the save target.
      if (!layouts_dir_.empty() && !layout_id_.empty()) {
        layout_yaml_ =
          (std::filesystem::path(layouts_dir_) / (layout_id_ + ".yaml")).string();
      } else {
        layout_yaml_ = msg->scene_yaml_path;
      }
      load_from_yaml(msg->scene_yaml_path);
    }
  }

  void load_from_yaml(const std::string& path)
  {
    try {
      specs_ = scooping_controller::load_container_scene_specs_from_yaml(path);
      dirty_ = false;
      update_table_top_from_specs();
      if (selected_object_id_.empty() && !specs_.empty()) {
        selected_object_id_ = specs_.front().id;
      }
      rebuild_markers();
      publish_status("Loaded " + std::to_string(specs_.size()) + " objects from " + path);
    } catch (const std::exception& ex) {
      RCLCPP_ERROR(get_logger(), "Could not load layout: %s", ex.what());
      publish_status(std::string("Load failed: ") + ex.what());
    }
  }

  void update_table_top_from_specs()
  {
    for (const auto& spec : specs_) {
      if (spec.id == "table" && spec.geometry_type == "box") {
        table_top_z_ = spec.pose.position.z + spec.dimensions[2] * 0.5;
        return;
      }
    }
  }

  void rebuild_markers()
  {
    if (server_) {
      server_->clear();
      for (std::size_t i = 0; i < specs_.size(); ++i) {
        if (!specs_[i].enabled) {
          continue;
        }
        insert_marker(i);
      }
      server_->applyChanges();
    }
    publish_container_markers();
  }

  void publish_container_markers()
  {
    if (!marker_pub_) {
      return;
    }
    Marker clear;
    clear.header.frame_id = frame_id_;
    clear.header.stamp = now();
    clear.action = Marker::DELETEALL;
    marker_pub_->publish(clear);

    for (std::size_t i = 0; i < specs_.size(); ++i) {
      const auto& spec = specs_[i];
      if (!spec.enabled) {
        continue;
      }
      Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = spec.id;
      marker.id = static_cast<int>(i);
      marker.action = Marker::ADD;
      marker.pose = spec.pose;
      marker.type = spec.geometry_type == "box" ? Marker::CUBE : Marker::MESH_RESOURCE;
      if (spec.geometry_type == "box") {
        marker.scale.x = spec.dimensions[0];
        marker.scale.y = spec.dimensions[1];
        marker.scale.z = spec.dimensions[2];
      } else {
        marker.scale.x = spec.scale[0];
        marker.scale.y = spec.scale[1];
        marker.scale.z = spec.scale[2];
        marker.mesh_resource = spec.mesh_resource;
        marker.mesh_use_embedded_materials = false;
      }
      marker.color.r = spec.color[0];
      marker.color.g = spec.color[1];
      marker.color.b = spec.color[2];
      marker.color.a = 1.0F;
      marker_pub_->publish(marker);
    }
  }

  Marker make_visual(const scooping_controller::ContainerSceneSpec& spec) const
  {
    Marker marker;
    marker.type = spec.geometry_type == "box" ? Marker::CUBE : Marker::MESH_RESOURCE;
    marker.pose.orientation.w = 1.0;
    if (spec.geometry_type == "box") {
      marker.scale.x = spec.dimensions[0];
      marker.scale.y = spec.dimensions[1];
      marker.scale.z = spec.dimensions[2];
    } else {
      // Mesh scale for RViz InteractiveMarker visual (mesh_units already applied
      // by load_container_scene_specs_from_yaml into scale).
      marker.scale.x = spec.scale[0];
      marker.scale.y = spec.scale[1];
      marker.scale.z = spec.scale[2];
      marker.mesh_resource = spec.mesh_resource;
      marker.mesh_use_embedded_materials = false;
    }
    marker.color.r = spec.color[0];
    marker.color.g = spec.color[1];
    marker.color.b = spec.color[2];
    marker.color.a = 0.85F;
    return marker;
  }

  static InteractiveMarkerControl make_axis_control(
    const std::string& axis, bool rotate)
  {
    // Match scooping_marker_server axis convention (y uses z quat, z uses y quat).
    InteractiveMarkerControl control;
    control.orientation.w = 1.0;
    control.name = (rotate ? "rotate_" : "move_") + axis;
    control.interaction_mode = rotate ?
      InteractiveMarkerControl::ROTATE_AXIS :
      InteractiveMarkerControl::MOVE_AXIS;
    if (axis == "x") {
      control.orientation.x = 1.0;
    } else if (axis == "y") {
      control.orientation.z = 1.0;
    } else {
      control.orientation.y = 1.0;
    }
    return control;
  }

  void insert_marker(std::size_t index)
  {
    const auto& spec = specs_[index];
    InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id_;
    int_marker.name = spec.id;
    int_marker.description = spec.id + " (drag)";
    // Larger scale → bigger axis rings / easier to grab.
    int_marker.scale = 0.45;
    int_marker.pose = spec.pose;

    // Primary control: the object itself is draggable (not just tiny axis arrows).
    // Planar → click-drag on XY; 6-DOF → free 3D drag+rotate.
    InteractiveMarkerControl grab;
    grab.always_visible = true;
    grab.name = "grab";
    if (planar_mode_ && spec.id != "table") {
      // XY plane (ROS interactive-marker convention: y component selects Z-normal).
      grab.orientation.w = 1.0;
      grab.orientation.y = 1.0;
      grab.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    } else {
      grab.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE_3D;
    }

    Marker footprint;
    footprint.type = Marker::CUBE;
    footprint.scale.x = 0.16;
    footprint.scale.y = 0.16;
    footprint.scale.z = 0.06;
    footprint.pose.orientation.w = 1.0;
    footprint.color.r = spec.color[0];
    footprint.color.g = spec.color[1];
    footprint.color.b = spec.color[2];
    footprint.color.a = 0.7F;
    grab.markers.push_back(footprint);
    grab.markers.push_back(make_visual(spec));
    Marker label;
    label.type = Marker::TEXT_VIEW_FACING;
    label.text = spec.id;
    label.scale.z = 0.05;
    label.color.r = 1.0F;
    label.color.g = 1.0F;
    label.color.b = 1.0F;
    label.color.a = 1.0F;
    label.pose.position.z = 0.14;
    label.pose.orientation.w = 1.0;
    grab.markers.push_back(label);
    int_marker.controls.push_back(grab);

    if (planar_mode_ && spec.id != "table") {
      // Visible yaw ring + XY arrows as secondary handles.
      InteractiveMarkerControl yaw = make_axis_control("z", true);
      yaw.orientation.w = 1.0;
      yaw.orientation.x = 0.0;
      yaw.orientation.y = 1.0;
      yaw.orientation.z = 0.0;
      int_marker.controls.push_back(yaw);
      int_marker.controls.push_back(make_axis_control("x", false));
      int_marker.controls.push_back(make_axis_control("y", false));
    } else {
      for (const auto& axis : {"x", "y", "z"}) {
        int_marker.controls.push_back(make_axis_control(axis, false));
        int_marker.controls.push_back(make_axis_control(axis, true));
      }
    }

    server_->insert(
      int_marker,
      [this](const InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
        process_feedback(feedback);
      });
  }

  std::size_t index_of(const std::string& object_id) const
  {
    for (std::size_t i = 0; i < specs_.size(); ++i) {
      if (specs_[i].id == object_id) {
        return i;
      }
    }
    return specs_.size();
  }

  void process_feedback(const InteractiveMarkerFeedback::ConstSharedPtr& feedback)
  {
    if (feedback->event_type != InteractiveMarkerFeedback::POSE_UPDATE &&
        feedback->event_type != InteractiveMarkerFeedback::MOUSE_UP)
    {
      return;
    }
    const auto index = index_of(feedback->marker_name);
    if (index >= specs_.size()) {
      return;
    }
    auto pose = feedback->pose;
    if (planar_mode_ && specs_[index].id != "table") {
      const double rim = rim_height_for_spec(specs_[index]);
      pose.position.z = table_top_z_ + rim;
      // Keep only yaw in planar mode.
      const auto rpy = quat_to_rpy_deg(pose.orientation);
      pose.orientation = rpy_deg_to_quat_msg({0.0, 0.0, rpy[2]});
    }
    specs_[index].pose = pose;
    selected_object_id_ = specs_[index].id;
    dirty_ = true;
    if (feedback->event_type == InteractiveMarkerFeedback::MOUSE_UP) {
      server_->setPose(feedback->marker_name, pose);
      server_->applyChanges();
      publish_preview();
    }
  }

  std::string preview_yaml_path() const
  {
    return expand_user_path(
      "~/.ros/scooping_controller/layout_preview_" +
      (layout_id_.empty() ? "scratch" : layout_id_) + ".yaml");
  }

  bool write_objects_yaml(
    const std::string& path,
    bool stamp_authoring,
    std::string& message) const
  {
    try {
      YAML::Node root;
      if (!layout_yaml_.empty() && std::filesystem::exists(layout_yaml_)) {
        root = YAML::LoadFile(layout_yaml_);
      } else {
        root["schema_version"] = 2;
        root["layout_id"] = layout_id_;
      }

      YAML::Node objects(YAML::NodeType::Sequence);
      for (const auto& spec : specs_) {
        YAML::Node obj;
        obj["id"] = spec.id;
        obj["enabled"] = spec.enabled;
        obj["geometry_type"] = spec.geometry_type;
        if (spec.geometry_type == "mesh") {
          obj["mesh_resource"] = spec.mesh_resource;
          // Prefer mm units in committed layouts (matches existing files).
          obj["mesh_units"] = "mm";
          // Reverse the mm->m scale applied by the loader when writing back.
          obj["scale_xyz"] = YAML::Node(YAML::NodeType::Sequence);
          for (double value : spec.scale) {
            obj["scale_xyz"].push_back(value / 0.001);
          }
        } else {
          obj["scale_xyz"] = YAML::Node(YAML::NodeType::Sequence);
          for (double value : spec.scale) {
            obj["scale_xyz"].push_back(value);
          }
        }
        obj["position_xyz"] = YAML::Node(YAML::NodeType::Sequence);
        obj["position_xyz"].push_back(spec.pose.position.x);
        obj["position_xyz"].push_back(spec.pose.position.y);
        obj["position_xyz"].push_back(spec.pose.position.z);
        const auto rpy = quat_to_rpy_deg(spec.pose.orientation);
        obj["orientation"]["rpy_deg"] = YAML::Node(YAML::NodeType::Sequence);
        obj["orientation"]["rpy_deg"].push_back(rpy[0]);
        obj["orientation"]["rpy_deg"].push_back(rpy[1]);
        obj["orientation"]["rpy_deg"].push_back(rpy[2]);
        obj["dimensions_xyz"] = YAML::Node(YAML::NodeType::Sequence);
        for (double value : spec.dimensions) {
          obj["dimensions_xyz"].push_back(value);
        }
        obj["color_rgb"] = YAML::Node(YAML::NodeType::Sequence);
        for (float value : spec.color) {
          obj["color_rgb"].push_back(static_cast<double>(value));
        }
        // Preserve per-object calibration if present in the original file.
        if (root["objects"] && root["objects"].IsSequence()) {
          for (const auto& prior : root["objects"]) {
            if (prior["id"] && prior["id"].as<std::string>() == spec.id &&
                prior["calibration"])
            {
              obj["calibration"] = prior["calibration"];
            }
          }
        }
        const auto cal_it = object_calibration_.find(spec.id);
        if (cal_it != object_calibration_.end()) {
          obj["calibration"] = cal_it->second;
        }
        objects.push_back(obj);
      }
      root["objects"] = objects;

      if (stamp_authoring) {
        root["authoring"]["method"] = "rviz_drag";
        root["authoring"]["robot_key"] = robot_key_;
        root["authoring"]["operator"] = operator_name_;
        root["authoring"]["date"] = today_iso();
      }

      std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
      std::ofstream out(path);
      if (!out) {
        message = "Failed to open " + path + " for writing";
        return false;
      }
      out << root;
      message = "Wrote " + path;
      return true;
    } catch (const std::exception& ex) {
      message = ex.what();
      return false;
    }
  }

  void publish_preview()
  {
    preview_path_ = preview_yaml_path();
    std::string message;
    if (!write_objects_yaml(preview_path_, false, message)) {
      publish_status("Preview write failed: " + message);
      return;
    }
    if (!preview_client_->service_is_ready()) {
      publish_status("Preview service not ready; geometry saved to scratch only");
      return;
    }
    auto request = std::make_shared<Preview::Request>();
    request->scene_yaml_path = preview_path_;
    preview_client_->async_send_request(request);
    publish_status("Preview published for " + layout_id_);
  }

  void handle_save(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    if (layout_yaml_.empty()) {
      response->success = false;
      response->message = "No layout_yaml configured";
      return;
    }
    if (path_is_deploy_bundle(layout_yaml_)) {
      response->success = false;
      response->message =
        "Refusing to save into deploy bundle path (/ws/config/layouts). "
        "Author against the git tree (config/layouts/).";
      publish_status(response->message);
      return;
    }
    std::string message;
    if (!write_objects_yaml(layout_yaml_, true, message)) {
      response->success = false;
      response->message = message;
      publish_status(message);
      return;
    }
    dirty_ = false;
    response->success = true;
    response->message = message;
    publish_status(message);
    // Re-apply from the saved file so /cell_layout/active leaves preview mode.
    // The manager's apply service is the proper channel; for now re-publish
    // preview with the saved path is wrong — call apply via layout_id if set.
  }

  void handle_reload(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    if (layout_yaml_.empty() || !std::filesystem::exists(layout_yaml_)) {
      response->success = false;
      response->message = "No layout YAML to reload";
      return;
    }
    load_from_yaml(layout_yaml_);
    object_calibration_.clear();
    touch_points_.clear();
    response->success = true;
    response->message = "Reloaded " + layout_yaml_;
  }

  std::string unique_object_id(const std::string& preferred) const
  {
    if (index_of(preferred) >= specs_.size()) {
      return preferred;
    }
    for (int suffix = 2; suffix < 1000; ++suffix) {
      const auto candidate = preferred + "_" + std::to_string(suffix);
      if (index_of(candidate) >= specs_.size()) {
        return candidate;
      }
    }
    return preferred + "_new";
  }

  void handle_add(
    const std::shared_ptr<AddObject::Request> request,
    std::shared_ptr<AddObject::Response> response)
  {
    const auto* model = find_model(request->model_id);
    if (!model) {
      response->success = false;
      response->message = "Unknown model_id: " + request->model_id;
      return;
    }
    std::string object_id = request->object_id;
    if (object_id.empty()) {
      // Prefer a short category-based id over the catalog model_id.
      object_id = model->category.empty() ? model->model_id : model->category + "_container";
      object_id = unique_object_id(object_id);
    } else if (index_of(object_id) < specs_.size()) {
      response->success = false;
      response->message = "Object id already exists: " + object_id;
      return;
    }
    scooping_controller::ContainerSceneSpec spec;
    spec.id = object_id;
    spec.enabled = true;
    spec.geometry_type = model->geometry_type;
    spec.mesh_resource = model->mesh_resource;
    spec.scale = model->default_scale_xyz;
    if (model->mesh_units == "mm") {
      for (auto& value : spec.scale) {
        value *= 0.001;
      }
    }
    spec.dimensions = model->default_dimensions_xyz;
    if (spec.geometry_type == "box" &&
        spec.dimensions[0] <= 0.0 && spec.dimensions[1] <= 0.0 && spec.dimensions[2] <= 0.0)
    {
      spec.dimensions = {0.2, 0.2, 0.05};
    }
    spec.color = {
      static_cast<float>(model->default_color_rgb[0]),
      static_cast<float>(model->default_color_rgb[1]),
      static_cast<float>(model->default_color_rgb[2])};
    const double z = table_top_z_ + model->rim_height_m;
    // Offset each new spawn so stacked adds are not on top of each other.
    const double y_offset = 0.12 * static_cast<double>(specs_.size());
    spec.pose = scooping_controller::make_pose(
      {0.40, y_offset, z}, {0.0, 0.0, 0.0, 1.0});
    specs_.push_back(spec);
    selected_object_id_ = object_id;
    set_parameter(rclcpp::Parameter("selected_object_id", object_id));
    dirty_ = true;
    rebuild_markers();
    publish_preview();
    response->success = true;
    response->message =
      "Added " + object_id + " from " + model->model_id +
      " at (0.40, " + std::to_string(y_offset) + ", " + std::to_string(z) +
      ") — look in front of the robot / Container display";
    publish_status(response->message);
  }

  void handle_remove(
    const std::shared_ptr<RemoveObject::Request> request,
    std::shared_ptr<RemoveObject::Response> response)
  {
    const auto index = index_of(request->object_id);
    if (index >= specs_.size()) {
      response->success = false;
      response->message = "Unknown object_id: " + request->object_id;
      return;
    }
    specs_.erase(specs_.begin() + static_cast<std::ptrdiff_t>(index));
    touch_points_.erase(request->object_id);
    object_calibration_.erase(request->object_id);
    if (selected_object_id_ == request->object_id) {
      selected_object_id_ = specs_.empty() ? "" : specs_.front().id;
    }
    dirty_ = true;
    rebuild_markers();
    publish_preview();
    response->success = true;
    response->message = "Removed " + request->object_id;
    publish_status(response->message);
  }

  void handle_capture(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    if (selected_object_id_.empty()) {
      response->success = false;
      response->message = "No selected_object_id; set the parameter first";
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, tcp_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
      const std::array<double, 3> point = {
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z};
      auto& buffer = touch_points_[selected_object_id_];
      if (buffer.size() >= 3U) {
        buffer.clear();
      }
      buffer.push_back(point);
      std::ostringstream stream;
      stream << "Captured touch point " << buffer.size() << "/3 for "
             << selected_object_id_ << " at ["
             << point[0] << ", " << point[1] << ", " << point[2] << "]";
      response->success = true;
      response->message = stream.str();
      publish_status(response->message);
    } catch (const std::exception& ex) {
      response->success = false;
      response->message = std::string("TF lookup failed: ") + ex.what();
      publish_status(response->message);
    }
  }

  void handle_fit(
    const std::shared_ptr<Trigger::Request> /*request*/,
    std::shared_ptr<Trigger::Response> response)
  {
    if (selected_object_id_.empty()) {
      response->success = false;
      response->message = "No selected_object_id";
      return;
    }
    const auto it = touch_points_.find(selected_object_id_);
    if (it == touch_points_.end() || it->second.size() != 3U) {
      response->success = false;
      response->message = "Need 3 captured touch points before fit";
      return;
    }
    try {
      const auto fit = scooping_controller::fit_touch_points(it->second);
      const auto index = index_of(selected_object_id_);
      if (index >= specs_.size()) {
        response->success = false;
        response->message = "Selected object missing from scene";
        return;
      }
      specs_[index].pose.position.x = fit.centroid[0];
      specs_[index].pose.position.y = fit.centroid[1];
      specs_[index].pose.position.z = fit.centroid[2];
      specs_[index].pose.orientation =
        rpy_deg_to_quat_msg({0.0, 0.0, fit.yaw_deg});

      YAML::Node cal;
      cal["method"] = "touch_off_3point";
      cal["date"] = today_iso();
      cal["operator"] = operator_name_;
      cal["residual_m"] = fit.residual_m;
      cal["plane_normal"] = YAML::Node(YAML::NodeType::Sequence);
      for (double value : fit.plane_normal) {
        cal["plane_normal"].push_back(value);
      }
      object_calibration_[selected_object_id_] = cal;
      dirty_ = true;
      rebuild_markers();
      publish_preview();

      std::ostringstream stream;
      stream << "Fitted " << selected_object_id_ << " yaw=" << fit.yaw_deg
             << "deg residual=" << fit.residual_m << "m";
      response->success = true;
      response->message = stream.str();
      publish_status(response->message);
      it->second.clear();
    } catch (const std::exception& ex) {
      response->success = false;
      response->message = ex.what();
      publish_status(response->message);
    }
  }

  void publish_status(const std::string& text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "%s", text.c_str());
  }

  std::string frame_id_;
  std::string layouts_dir_;
  std::string catalog_yaml_;
  std::string layout_yaml_;
  std::string layout_id_;
  std::string robot_key_;
  std::string base_frame_;
  std::string tcp_frame_;
  std::string operator_name_;
  std::string selected_object_id_;
  std::string preview_path_;
  bool planar_mode_{true};
  bool dirty_{false};
  double table_top_z_{0.0};

  std::unordered_map<std::string, CatalogModel> catalog_;
  std::unordered_map<std::string, double> rim_by_mesh_;
  std::vector<scooping_controller::ContainerSceneSpec> specs_;
  std::unordered_map<std::string, std::vector<std::array<double, 3>>> touch_points_;
  std::unordered_map<std::string, YAML::Node> object_calibration_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<robot_common_msgs::msg::CellLayoutActive>::SharedPtr layout_sub_;
  rclcpp::Client<Preview>::SharedPtr preview_client_;
  rclcpp::Service<Trigger>::SharedPtr save_srv_;
  rclcpp::Service<Trigger>::SharedPtr reload_srv_;
  rclcpp::Service<AddObject>::SharedPtr add_srv_;
  rclcpp::Service<RemoveObject>::SharedPtr remove_srv_;
  rclcpp::Service<Trigger>::SharedPtr capture_srv_;
  rclcpp::Service<Trigger>::SharedPtr fit_srv_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CellLayoutEditor>());
  rclcpp::shutdown();
  return 0;
}
