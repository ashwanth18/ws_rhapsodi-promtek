#include "scooping_controller/container_collision_objects.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ContainerMarkerPublisher : public rclcpp::Node
{
public:
  ContainerMarkerPublisher()
  : Node("container_marker_publisher")
  {
    this->declare_parameter<std::string>("frame_id", "base_link");
    scooping_controller::declare_container_scene_parameters(*this);

    frame_id_ = this->get_parameter("frame_id").as_string();
    scene_specs_ = scooping_controller::load_container_scene_specs(*this);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/container_marker",
      rclcpp::QoS(1).transient_local().reliable());

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publish_marker(); });

    publish_marker();
  }

private:
  static visualization_msgs::msg::Marker make_marker(
    const rclcpp::Time& stamp,
    int id,
    const std::string& frame_id,
    const scooping_controller::ContainerSceneSpec& spec)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = spec.id;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = spec.pose;
    marker.scale.x = spec.scale[0];
    marker.scale.y = spec.scale[1];
    marker.scale.z = spec.scale[2];
    marker.color.r = spec.color[0];
    marker.color.g = spec.color[1];
    marker.color.b = spec.color[2];
    marker.color.a = 1.0F;
    marker.mesh_resource = spec.mesh_resource;
    marker.mesh_use_embedded_materials = false;
    return marker;
  }

  void publish_marker()
  {
    const auto stamp = this->now();
    for (std::size_t i = 0; i < scene_specs_.size(); ++i) {
      marker_pub_->publish(make_marker(
        stamp,
        static_cast<int>(i),
        frame_id_,
        scene_specs_[i]));
    }
  }

  std::string frame_id_;
  std::vector<scooping_controller::ContainerSceneSpec> scene_specs_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContainerMarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
