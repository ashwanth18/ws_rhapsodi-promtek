#pragma once

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <boost/variant/get.hpp>
#include <geometric_shapes/geometric_shapes/mesh_operations.h>
#include <geometric_shapes/geometric_shapes/shape_messages.h>
#include <geometric_shapes/geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <yaml-cpp/yaml.h>

namespace scooping_controller
{
struct ContainerSceneSpec
{
  std::string id;
  bool enabled{true};
  std::string geometry_type;
  std::string mesh_resource;
  std::array<double, 3> scale;
  std::array<double, 3> dimensions;
  geometry_msgs::msg::Pose pose;
  std::array<float, 3> color;
};

inline geometry_msgs::msg::Pose make_pose(
  const std::array<double, 3>& position,
  const std::array<double, 4>& orientation)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  pose.orientation.x = orientation[0];
  pose.orientation.y = orientation[1];
  pose.orientation.z = orientation[2];
  pose.orientation.w = orientation[3];
  return pose;
}

inline std::array<double, 4> rpy_deg_to_quaternion(const std::array<double, 3>& rpy)
{
  constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
  const double cr = std::cos(rpy[0] * kDegToRad / 2.0);
  const double sr = std::sin(rpy[0] * kDegToRad / 2.0);
  const double cp = std::cos(rpy[1] * kDegToRad / 2.0);
  const double sp = std::sin(rpy[1] * kDegToRad / 2.0);
  const double cy = std::cos(rpy[2] * kDegToRad / 2.0);
  const double sy = std::sin(rpy[2] * kDegToRad / 2.0);
  return {sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy};
}

template<std::size_t N>
inline std::array<double, N> yaml_array(const YAML::Node& node, const std::string& name)
{
  const auto value = node[name];
  if (!value || !value.IsSequence() || value.size() != N) {
    throw std::runtime_error("Layout object is missing " + name + " with " + std::to_string(N) + " values");
  }
  std::array<double, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = value[i].as<double>();
  }
  return result;
}

inline std::vector<ContainerSceneSpec> load_container_scene_specs_from_yaml(const std::string& path)
{
  const auto root = YAML::LoadFile(path);
  const auto objects = root["objects"];
  if (!objects || !objects.IsSequence() || objects.size() == 0U) {
    throw std::runtime_error("Layout " + path + " must contain a non-empty objects list");
  }
  std::vector<ContainerSceneSpec> specs;
  for (const auto& object : objects) {
    for (const auto& required : {"id", "enabled", "geometry_type", "position_xyz", "orientation",
                                 "scale_xyz", "dimensions_xyz", "color_rgb"}) {
      if (!object[required]) {
        throw std::runtime_error("Layout " + path + " object missing required field " + std::string(required));
      }
    }
    ContainerSceneSpec spec;
    spec.id = object["id"].as<std::string>();
    spec.enabled = object["enabled"].as<bool>();
    spec.geometry_type = object["geometry_type"].as<std::string>();
    spec.mesh_resource = object["mesh_resource"] ? object["mesh_resource"].as<std::string>() : "";
    if (spec.geometry_type != "mesh" && spec.geometry_type != "box") {
      throw std::runtime_error("Unsupported geometry_type for " + spec.id);
    }
    if (spec.geometry_type == "mesh" && spec.mesh_resource.empty()) {
      throw std::runtime_error("Mesh object " + spec.id + " has no mesh_resource");
    }
    const auto orientation = object["orientation"];
    const auto quat = orientation["quat_xyzw"] ? yaml_array<4>(orientation, "quat_xyzw") :
      (orientation["rpy_deg"] ? rpy_deg_to_quaternion(yaml_array<3>(orientation, "rpy_deg")) :
      throw std::runtime_error("Object " + spec.id + " must provide quat_xyzw or rpy_deg"));
    spec.pose = make_pose(yaml_array<3>(object, "position_xyz"), quat);
    spec.scale = yaml_array<3>(object, "scale_xyz");
    if (object["mesh_units"] && object["mesh_units"].as<std::string>() == "mm") {
      for (auto& value : spec.scale) { value *= 0.001; }
    }
    spec.dimensions = yaml_array<3>(object, "dimensions_xyz");
    const auto color = yaml_array<3>(object, "color_rgb");
    spec.color = {static_cast<float>(color[0]), static_cast<float>(color[1]), static_cast<float>(color[2])};
    specs.push_back(spec);
  }
  return specs;
}

inline std::vector<ContainerSceneSpec> default_container_scene_specs()
{
  std::vector<ContainerSceneSpec> specs;

  ContainerSceneSpec scooping;
  scooping.id = "rs6";
  scooping.geometry_type = "mesh";
  scooping.mesh_resource = "package://scooping_controller/models/scooping_container/meshes/rs6_container.STL";
  scooping.scale = {0.001, 0.001, 0.001};
  scooping.dimensions = {0.0, 0.0, 0.0};
  scooping.pose = make_pose({0.460, -0.119, 0.00}, {0.0, 0.0, 1.0, 0.0});
  scooping.color = {0.7F, 0.7F, 0.7F};
  specs.push_back(scooping);

  ContainerSceneSpec weighing;
  weighing.id = "rs3";
  weighing.geometry_type = "mesh";
  weighing.mesh_resource = "package://scooping_controller/models/weighing_container/meshes/rs3_container.STL";
  weighing.scale = {0.001, 0.001, 0.001};
  weighing.dimensions = {0.0, 0.0, 0.0};
  weighing.pose = make_pose({0.00, -0.4218, 0.105}, {0.0, 0.0, 0.70710678, 0.70710678});
  weighing.color = {0.4F, 0.8F, 0.4F};
  specs.push_back(weighing);

  ContainerSceneSpec table;
  table.id = "table";
  table.geometry_type = "box";
  table.mesh_resource = "";
  table.scale = {1.0, 1.0, 1.0};
  table.dimensions = {1.02, 0.61, 0.03};
  // Top at z=-0.003 so base_link collision (floor at z=0) is not coplanar.
  table.pose = make_pose({0.0, 0.0, -0.018}, {0.0, 0.0, 0.0, 1.0});
  table.color = {0.55F, 0.39F, 0.22F};
  specs.push_back(table);

  return specs;
}

template<std::size_t N>
inline std::array<double, N> get_array_parameter(
  const rclcpp::Node& node,
  const std::string& name,
  const std::array<double, N>& fallback)
{
  const auto values = node.get_parameter(name).as_double_array();
  if (values.size() != N) {
    RCLCPP_WARN(
      node.get_logger(),
      "Parameter '%s' has %zu values, expected %zu. Using fallback.",
      name.c_str(),
      values.size(),
      N);
    return fallback;
  }

  std::array<double, N> result{};
  for (std::size_t i = 0; i < N; ++i) {
    result[i] = values[i];
  }
  return result;
}

inline std::array<float, 3> get_color_parameter(
  const rclcpp::Node& node,
  const std::string& name,
  const std::array<float, 3>& fallback)
{
  const auto values = node.get_parameter(name).as_double_array();
  if (values.size() != 3U) {
    RCLCPP_WARN(
      node.get_logger(),
      "Parameter '%s' has %zu values, expected 3. Using fallback.",
      name.c_str(),
      values.size());
    return fallback;
  }

  return {
    static_cast<float>(values[0]),
    static_cast<float>(values[1]),
    static_cast<float>(values[2])};
}

inline void declare_container_scene_parameters(rclcpp::Node& node)
{
  for (const auto& spec : default_container_scene_specs()) {
    const std::string prefix = spec.id + "_";
    node.declare_parameter<bool>(prefix + "enabled", spec.enabled);
    node.declare_parameter<std::string>(prefix + "geometry_type", spec.geometry_type);
    node.declare_parameter<std::string>(prefix + "mesh_resource", spec.mesh_resource);
    node.declare_parameter<std::vector<double>>(
      prefix + "position_xyz",
      {spec.pose.position.x, spec.pose.position.y, spec.pose.position.z});
    node.declare_parameter<std::vector<double>>(
      prefix + "orientation_xyzw",
      {
        spec.pose.orientation.x,
        spec.pose.orientation.y,
        spec.pose.orientation.z,
        spec.pose.orientation.w});
    node.declare_parameter<std::vector<double>>(
      prefix + "scale_xyz",
      {spec.scale[0], spec.scale[1], spec.scale[2]});
    node.declare_parameter<std::vector<double>>(
      prefix + "dimensions_xyz",
      {spec.dimensions[0], spec.dimensions[1], spec.dimensions[2]});
    node.declare_parameter<std::vector<double>>(
      prefix + "color_rgb",
      {
        static_cast<double>(spec.color[0]),
        static_cast<double>(spec.color[1]),
        static_cast<double>(spec.color[2])});
  }
}

inline std::vector<ContainerSceneSpec> load_container_scene_specs(const rclcpp::Node& node)
{
  std::vector<ContainerSceneSpec> specs = default_container_scene_specs();
  for (auto& spec : specs) {
    const std::string prefix = spec.id + "_";
    spec.enabled = node.get_parameter(prefix + "enabled").as_bool();
    spec.geometry_type = node.get_parameter(prefix + "geometry_type").as_string();
    spec.mesh_resource = node.get_parameter(prefix + "mesh_resource").as_string();
    spec.pose = make_pose(
      get_array_parameter(node, prefix + "position_xyz", std::array<double, 3>{
        spec.pose.position.x,
        spec.pose.position.y,
        spec.pose.position.z}),
      get_array_parameter(node, prefix + "orientation_xyzw", std::array<double, 4>{
        spec.pose.orientation.x,
        spec.pose.orientation.y,
        spec.pose.orientation.z,
        spec.pose.orientation.w}));
    spec.scale = get_array_parameter(node, prefix + "scale_xyz", spec.scale);
    spec.dimensions = get_array_parameter(node, prefix + "dimensions_xyz", spec.dimensions);
    spec.color = get_color_parameter(node, prefix + "color_rgb", spec.color);
  }
  return specs;
}

inline std::vector<moveit_msgs::msg::CollisionObject> make_container_collision_objects(
  const std::string& frame_id,
  const std::vector<ContainerSceneSpec>& specs)
{
  std::vector<moveit_msgs::msg::CollisionObject> objects;
  for (const auto& spec : specs) {
    if (!spec.enabled) {
      continue;
    }
    moveit_msgs::msg::CollisionObject object;
    object.id = spec.id;
    object.header.frame_id = frame_id;
    object.operation = moveit_msgs::msg::CollisionObject::ADD;

    if (spec.geometry_type == "box") {
      shape_msgs::msg::SolidPrimitive primitive;
      primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
      primitive.dimensions = {
        spec.dimensions[0],
        spec.dimensions[1],
        spec.dimensions[2]};
      object.primitives.push_back(primitive);
      object.primitive_poses.push_back(spec.pose);
      objects.push_back(object);
      continue;
    }

    std::unique_ptr<shapes::Mesh> mesh(shapes::createMeshFromResource(
      spec.mesh_resource,
      Eigen::Vector3d(spec.scale[0], spec.scale[1], spec.scale[2])));
    if (!mesh) {
      continue;
    }

    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(mesh.get(), shape_msg)) {
      continue;
    }

    object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    object.mesh_poses.push_back(spec.pose);
    objects.push_back(object);
  }
  return objects;
}

inline std::vector<std::string> disabled_container_scene_ids(
  const std::vector<ContainerSceneSpec>& specs)
{
  std::vector<std::string> ids;
  for (const auto& spec : specs) {
    if (!spec.enabled) {
      ids.push_back(spec.id);
    }
  }
  return ids;
}

inline std::vector<moveit_msgs::msg::CollisionObject> make_container_collision_objects(
  const std::string& frame_id)
{
  return make_container_collision_objects(frame_id, default_container_scene_specs());
}
}  // namespace scooping_controller
