#include "scooping_controller/container_collision_objects.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/cell_layout_active.hpp>
#include <visualization_msgs/msg/marker.hpp>

class ContainerMarkerPublisher : public rclcpp::Node
{
public:
  ContainerMarkerPublisher()
  : Node("container_marker_publisher")
  {
    this->declare_parameter<std::string>("frame_id", "base_link");
    // Authoring-friendly default: see scoop markers through the mesh.
    this->declare_parameter<double>("visual_alpha", 0.35);
    scooping_controller::declare_container_scene_parameters(*this);

    frame_id_ = this->get_parameter("frame_id").as_string();
    visual_alpha_ = clamp_alpha(this->get_parameter("visual_alpha").as_double());
    scene_specs_ = scooping_controller::load_container_scene_specs(*this);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
      "/container_marker",
      rclcpp::QoS(1).transient_local().reliable());
    layout_sub_ = this->create_subscription<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable(),
      [this](const robot_common_msgs::msg::CellLayoutActive::SharedPtr msg) {
        try {
          scene_specs_ = scooping_controller::load_container_scene_specs_from_yaml(msg->scene_yaml_path);
          publish_marker();
        } catch (const std::exception& ex) {
          RCLCPP_ERROR(this->get_logger(), "Could not load layout markers: %s", ex.what());
        }
      });

    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
          if (param.get_name() == "visual_alpha") {
            visual_alpha_ = clamp_alpha(param.as_double());
            publish_marker();
          }
        }
        return result;
      });

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      [this]() { publish_marker(); });

    publish_marker();
  }

private:
  static double clamp_alpha(double alpha)
  {
    if (alpha < 0.0) {
      return 0.0;
    }
    if (alpha > 1.0) {
      return 1.0;
    }
    return alpha;
  }

  visualization_msgs::msg::Marker make_marker(
    const rclcpp::Time& stamp,
    int id,
    const std::string& frame_id,
    const scooping_controller::ContainerSceneSpec& spec) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = spec.id;
    marker.id = id;
    marker.type = spec.geometry_type == "box"
      ? visualization_msgs::msg::Marker::CUBE
      : visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = spec.pose;
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
    marker.color.a = static_cast<float>(visual_alpha_);
    return marker;
  }

  void publish_marker()
  {
    const auto stamp = this->now();
    for (std::size_t i = 0; i < scene_specs_.size(); ++i) {
      if (!scene_specs_[i].enabled) {
        continue;
      }
      marker_pub_->publish(make_marker(
        stamp,
        static_cast<int>(i),
        frame_id_,
        scene_specs_[i]));
    }
  }

  std::string frame_id_;
  double visual_alpha_{0.35};
  std::vector<scooping_controller::ContainerSceneSpec> scene_specs_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<robot_common_msgs::msg::CellLayoutActive>::SharedPtr layout_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ContainerMarkerPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
