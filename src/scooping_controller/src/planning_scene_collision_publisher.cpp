#include "scooping_controller/container_collision_objects.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/msg/cell_layout_active.hpp>

class PlanningSceneCollisionPublisher : public rclcpp::Node
{
public:
  PlanningSceneCollisionPublisher()
  : Node("planning_scene_collision_publisher")
  {
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<double>("republish_period_s", 5.0);
    scooping_controller::declare_container_scene_parameters(*this);

    frame_id_ = this->get_parameter("frame_id").as_string();
    scene_specs_ = scooping_controller::load_container_scene_specs(*this);
    planning_scene_interface_ =
      std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    layout_sub_ = this->create_subscription<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable(),
      [this](const robot_common_msgs::msg::CellLayoutActive::SharedPtr msg) {
        try {
          scene_specs_ = scooping_controller::load_container_scene_specs_from_yaml(msg->scene_yaml_path);
          planning_scene_interface_->removeCollisionObjects(
            scooping_controller::disabled_container_scene_ids(scene_specs_));
          sync_collision_objects();
        } catch (const std::exception& ex) {
          RCLCPP_ERROR(this->get_logger(), "Rejected cell layout '%s': %s", msg->layout_id.c_str(), ex.what());
        }
      });

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(this->get_parameter("republish_period_s").as_double())),
      std::bind(&PlanningSceneCollisionPublisher::sync_collision_objects, this));

    sync_collision_objects();
  }

private:
  void sync_collision_objects()
  {
    const auto desired_objects = scooping_controller::make_container_collision_objects(
      frame_id_, scene_specs_);

    std::vector<std::string> desired_ids;
    desired_ids.reserve(desired_objects.size());
    for (const auto& object : desired_objects) {
      desired_ids.push_back(object.id);
    }

    const auto known_objects = planning_scene_interface_->getObjects(desired_ids);
    bool needs_update = known_objects.size() != desired_objects.size();
    if (!needs_update) {
      for (const auto& object : desired_objects) {
        if (known_objects.find(object.id) == known_objects.end()) {
          needs_update = true;
          break;
        }
      }
    }

    if (!needs_update) {
      return;
    }

    if (planning_scene_interface_->applyCollisionObjects(desired_objects)) {
      RCLCPP_INFO(
        this->get_logger(),
        "Applied %zu container collision objects to MoveIt planning scene",
        desired_objects.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to apply container collision objects to planning scene");
    }
  }

  std::string frame_id_;
  std::vector<scooping_controller::ContainerSceneSpec> scene_specs_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Subscription<robot_common_msgs::msg::CellLayoutActive>::SharedPtr layout_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanningSceneCollisionPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
