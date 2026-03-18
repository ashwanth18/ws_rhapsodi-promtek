#pragma once

#include <array>
#include <memory>
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

namespace scooping_controller
{
struct ContainerSceneSpec
{
  std::string id;
  std::string mesh_resource;
  std::array<double, 3> scale;
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

inline std::vector<ContainerSceneSpec> default_container_scene_specs()
{
  std::vector<ContainerSceneSpec> specs;

  ContainerSceneSpec scooping;
  scooping.id = "scooping_container";
  scooping.mesh_resource = "package://scooping_controller/models/scooping_container/meshes/container.STL";
  scooping.scale = {0.0004, 0.0004, 0.0003};
  scooping.pose = make_pose({0.10, -0.30, 0.00}, {0.0, 0.0, 0.0, 1.0});
  scooping.color = {0.7F, 0.7F, 0.7F};
  specs.push_back(scooping);

  ContainerSceneSpec weighing;
  weighing.id = "weighing_container";
  weighing.mesh_resource = "package://scooping_controller/models/weighing_container/meshes/container.STL";
  weighing.scale = {0.0001, 0.0001, 0.0001};
  weighing.pose = make_pose({-0.075, -0.40, 0.00}, {0.0, 0.0, -0.70710678, 0.70710678});
  weighing.color = {0.4F, 0.8F, 0.4F};
  specs.push_back(weighing);

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

    moveit_msgs::msg::CollisionObject object;
    object.id = spec.id;
    object.header.frame_id = frame_id;
    object.meshes.push_back(boost::get<shape_msgs::msg::Mesh>(shape_msg));
    object.mesh_poses.push_back(spec.pose);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    objects.push_back(object);
  }
  return objects;
}

inline std::vector<moveit_msgs::msg::CollisionObject> make_container_collision_objects(
  const std::string& frame_id)
{
  return make_container_collision_objects(frame_id, default_container_scene_specs());
}
}  // namespace scooping_controller
