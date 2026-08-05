#include "scooping_controller/container_collision_objects.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
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
    get_planning_scene_ = create_client<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene");
    apply_planning_scene_ = create_client<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene");
    layout_sub_ = this->create_subscription<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable(),
      [this](const robot_common_msgs::msg::CellLayoutActive::SharedPtr msg) {
        try {
          scene_specs_ = scooping_controller::load_container_scene_specs_from_yaml(msg->scene_yaml_path);
          planning_scene_interface_->removeCollisionObjects(
            scooping_controller::disabled_container_scene_ids(scene_specs_));
          // Force re-apply: same object ids (table/rs3/…) can change pose/size
          // across layouts; ID-only equality would leave stale collisions.
          sync_collision_objects(/*force=*/true);
        } catch (const std::exception& ex) {
          RCLCPP_ERROR(this->get_logger(), "Rejected cell layout '%s': %s", msg->layout_id.c_str(), ex.what());
        }
      });

    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(this->get_parameter("republish_period_s").as_double())),
      [this]() { sync_collision_objects(/*force=*/false); });

    sync_collision_objects(/*force=*/true);
  }

private:
  static bool same_collision_object(
    const moveit_msgs::msg::CollisionObject& a,
    const moveit_msgs::msg::CollisionObject& b)
  {
    if (a.id != b.id || a.header.frame_id != b.header.frame_id) {
      return false;
    }
    if (a.primitives.size() != b.primitives.size() ||
        a.primitive_poses.size() != b.primitive_poses.size() ||
        a.meshes.size() != b.meshes.size() ||
        a.mesh_poses.size() != b.mesh_poses.size()) {
      return false;
    }
    constexpr double kEps = 1e-6;
    for (size_t i = 0; i < a.primitive_poses.size(); ++i) {
      const auto& pa = a.primitive_poses[i].position;
      const auto& pb = b.primitive_poses[i].position;
      if (std::abs(pa.x - pb.x) > kEps || std::abs(pa.y - pb.y) > kEps ||
          std::abs(pa.z - pb.z) > kEps) {
        return false;
      }
      if (a.primitives[i].type != b.primitives[i].type ||
          a.primitives[i].dimensions != b.primitives[i].dimensions) {
        return false;
      }
    }
    for (size_t i = 0; i < a.mesh_poses.size(); ++i) {
      const auto& pa = a.mesh_poses[i].position;
      const auto& pb = b.mesh_poses[i].position;
      if (std::abs(pa.x - pb.x) > kEps || std::abs(pa.y - pb.y) > kEps ||
          std::abs(pa.z - pb.z) > kEps) {
        return false;
      }
    }
    return true;
  }

  void sync_collision_objects(bool force)
  {
    const auto desired_objects = scooping_controller::make_container_collision_objects(
      frame_id_, scene_specs_);

    std::vector<std::string> desired_ids;
    desired_ids.reserve(desired_objects.size());
    for (const auto& object : desired_objects) {
      desired_ids.push_back(object.id);
    }

    bool needs_update = force;
    if (!needs_update) {
      const auto known_objects = planning_scene_interface_->getObjects(desired_ids);
      needs_update = known_objects.size() != desired_objects.size();
      if (!needs_update) {
        for (const auto& object : desired_objects) {
          const auto it = known_objects.find(object.id);
          if (it == known_objects.end() || !same_collision_object(object, it->second)) {
            needs_update = true;
            break;
          }
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
      allow_mount_table_collisions(desired_ids);
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to apply container collision objects to planning scene");
    }
  }

  void allow_mount_table_collisions(const std::vector<std::string>& object_ids)
  {
    // Robot is bolted to the table: allow base/arm AABBs vs the table box only.
    if (std::find(object_ids.begin(), object_ids.end(), "table") == object_ids.end()) {
      return;
    }
    if (!get_planning_scene_->wait_for_service(std::chrono::seconds(2)) ||
        !apply_planning_scene_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(this->get_logger(), "Planning scene services unavailable for table ACM");
      return;
    }
    auto get_req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    get_req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto get_fut = get_planning_scene_->async_send_request(get_req);
    if (get_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Timed out reading ACM for table allow-list");
      return;
    }
    const auto get_res = get_fut.get();
    if (!get_res) {
      return;
    }
    auto names = get_res->scene.allowed_collision_matrix.entry_names;
    auto values = get_res->scene.allowed_collision_matrix.entry_values;
    if (std::find(names.begin(), names.end(), "table") == names.end()) {
      const size_t n = names.size();
      for (size_t i = 0; i < values.size(); ++i) {
        values[i].enabled.push_back(
          names[i] == "base_link" || names[i] == "arm_link");
      }
      moveit_msgs::msg::AllowedCollisionEntry table_row;
      table_row.enabled.assign(n + 1, false);
      for (size_t i = 0; i < n; ++i) {
        if (names[i] == "base_link" || names[i] == "arm_link") {
          table_row.enabled[i] = true;
        }
      }
      names.push_back("table");
      values.push_back(table_row);
    } else {
      const size_t table_i = static_cast<size_t>(
        std::distance(names.begin(), std::find(names.begin(), names.end(), "table")));
      for (size_t i = 0; i < names.size() && i < values.size(); ++i) {
        if (names[i] == "base_link" || names[i] == "arm_link") {
          if (table_i < values[i].enabled.size()) {
            values[i].enabled[table_i] = true;
          }
          if (i < values[table_i].enabled.size()) {
            values[table_i].enabled[i] = true;
          }
        }
      }
    }
    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.allowed_collision_matrix.entry_names = names;
    scene.allowed_collision_matrix.entry_values = values;
    auto apply_req = std::make_shared<moveit_msgs::srv::ApplyPlanningScene::Request>();
    apply_req->scene = scene;
    auto apply_fut = apply_planning_scene_->async_send_request(apply_req);
    if (apply_fut.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Timed out allowing base/arm vs table collisions");
      return;
    }
    const auto apply_res = apply_fut.get();
    if (!apply_res || !apply_res->success) {
      RCLCPP_WARN(this->get_logger(), "Failed to allow base/arm vs table collisions");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Allowed base_link/arm_link collisions with table");
  }

  std::string frame_id_;
  std::vector<scooping_controller::ContainerSceneSpec> scene_specs_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_planning_scene_;
  rclcpp::Client<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr apply_planning_scene_;
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
