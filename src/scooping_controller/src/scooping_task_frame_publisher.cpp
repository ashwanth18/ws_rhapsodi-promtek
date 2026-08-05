#include "scooping_controller/container_collision_objects.hpp"

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/cell_layout_active.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

class ScoopingTaskFramePublisher : public rclcpp::Node
{
public:
  ScoopingTaskFramePublisher()
  : Node("scooping_task_frame_publisher")
  {
    this->declare_parameter<std::string>("parent_frame_id", "base_link");
    this->declare_parameter<std::string>("child_frame_id", "scooping_container_frame");
    this->declare_parameter<std::string>("task_container_id", "rs6");
    scooping_controller::declare_container_scene_parameters(*this);

    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publish_task_frame(
      scooping_controller::load_container_scene_specs(*this),
      this->get_parameter("task_container_id").as_string());
    layout_sub_ = this->create_subscription<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable(),
      [this](const robot_common_msgs::msg::CellLayoutActive::SharedPtr msg) {
        try {
          publish_task_frame(
            scooping_controller::load_container_scene_specs_from_yaml(msg->scene_yaml_path),
            msg->task_container_id);
        } catch (const std::exception& ex) {
          RCLCPP_ERROR(this->get_logger(), "Could not publish layout task frame: %s", ex.what());
        }
      });
  }

private:
  void publish_task_frame(
    const std::vector<scooping_controller::ContainerSceneSpec>& scene_specs,
    const std::string& task_container_id)
  {
    const auto parent_frame_id = this->get_parameter("parent_frame_id").as_string();
    const auto child_frame_id = this->get_parameter("child_frame_id").as_string();
    const auto it = std::find_if(
      scene_specs.begin(),
      scene_specs.end(),
      [&task_container_id](const scooping_controller::ContainerSceneSpec& spec) {
        return spec.id == task_container_id;
      });
    if (it == scene_specs.end()) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Could not find container '%s' for task frame publication",
        task_container_id.c_str());
      throw std::runtime_error("Missing task container frame source");
    }

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = parent_frame_id;
    transform.child_frame_id = child_frame_id;
    transform.transform.translation.x = it->pose.position.x;
    transform.transform.translation.y = it->pose.position.y;
    transform.transform.translation.z = it->pose.position.z;
    transform.transform.rotation = it->pose.orientation;
    broadcaster_->sendTransform(transform);

    RCLCPP_INFO(
      this->get_logger(),
      "Publishing task frame '%s' from container '%s' relative to '%s'",
      child_frame_id.c_str(),
      task_container_id.c_str(),
      parent_frame_id.c_str());
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::Subscription<robot_common_msgs::msg::CellLayoutActive>::SharedPtr layout_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScoopingTaskFramePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
