#include <filesystem>
#include <cstdint>
#include <future>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <robot_common_msgs/action/move_to.hpp>
#include <robot_common_msgs/msg/cell_layout_active.hpp>
#include <robot_common_msgs/srv/apply_cell_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

class CellLayoutManager : public rclcpp::Node
{
public:
  using Apply = robot_common_msgs::srv::ApplyCellLayout;
  using MoveTo = robot_common_msgs::action::MoveTo;

  CellLayoutManager() : Node("cell_layout_manager")
  {
    declare_parameter<std::string>("layouts_dir", "/ws/config/layouts");
    declare_parameter<std::string>("initial_layout_id", "");
    active_pub_ = create_publisher<robot_common_msgs::msg::CellLayoutActive>(
      "/cell_layout/active", rclcpp::QoS(1).transient_local().reliable());
    run_sub_ = create_subscription<std_msgs::msg::String>(
      "/orchestrator/run_state", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr state) { run_state_ = state->data; });
    scoop_pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/scoop_poses", rclcpp::QoS(1).transient_local(),
      [this](const geometry_msgs::msg::PoseArray::SharedPtr poses) { scoop_poses_ = *poses; });
    apply_srv_ = create_service<Apply>("/cell_layout/apply",
      std::bind(&CellLayoutManager::apply, this, std::placeholders::_1, std::placeholders::_2));
    move_to_client_ = rclcpp_action::create_client<MoveTo>(this, "/move_to");
    marker_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "scooping_marker_server");
    move_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "move_to_server");
    load_poses_client_ = create_client<std_srvs::srv::Trigger>("/load_scoop_poses");
    const auto initial = get_parameter("initial_layout_id").as_string();
    if (!initial.empty()) {
      apply_layout(initial, false);
    }
  }

private:
  static std::string digest(const std::string& bytes)
  {
    // FNV-1a is stable across processes; backend tooling uses SHA-256 for audit-grade hashes.
    std::uint64_t hash = 14695981039346656037ULL;
    for (unsigned char byte : bytes) {
      hash = (hash ^ byte) * 1099511628211ULL;
    }
    std::ostringstream stream;
    stream << std::hex << hash;
    return stream.str();
  }

  void apply(const std::shared_ptr<Apply::Request> request, std::shared_ptr<Apply::Response> response)
  {
    if (run_state_ == "running" || run_state_ == "starting") {
      response->success = false;
      response->message = "Refusing layout change while orchestrator is " + run_state_;
      return;
    }
    const auto result = apply_layout(request->layout_id, true);
    response->success = result.success;
    response->message = result.message;
    response->layout_hash = result.hash;
    response->preflight_ok = result.preflight_ok;
  }

  struct Result { bool success{false}; bool preflight_ok{false}; std::string message; std::string hash; };

  Result apply_layout(const std::string& layout_id, bool preflight)
  {
    Result result;
    try {
      const auto path = std::filesystem::path(get_parameter("layouts_dir").as_string()) / (layout_id + ".yaml");
      const auto root = YAML::LoadFile(path.string());
      if (!root["objects"] || !root["targets_yaml"] || !root["poses_yaml"] ||
          !root["task_container_id"] || !root["tool_id"]) {
        throw std::runtime_error("layout is missing required activation fields");
      }
      const auto hash = digest(YAML::Dump(root));
      const auto targets = (path.parent_path() / root["targets_yaml"].as<std::string>()).lexically_normal().string();
      const auto poses = (path.parent_path() / root["poses_yaml"].as<std::string>()).lexically_normal().string();
      robot_common_msgs::msg::CellLayoutActive active;
      active.layout_id = layout_id;
      active.layout_hash = hash;
      active.scene_yaml_path = path.string();
      active.targets_yaml = targets;
      active.poses_yaml = poses;
      active.task_container_id = root["task_container_id"].as<std::string>();
      active.tool_id = root["tool_id"].as<std::string>();
      active.scoop_cartesian_avoid_collisions =
        root["scoop_cartesian_avoid_collisions"] ?
        root["scoop_cartesian_avoid_collisions"].as<bool>() : false;
      active.authored_in_required = true;
      active_pub_->publish(active);
      move_params_->set_parameters({
        rclcpp::Parameter("targets_yaml", targets),
        rclcpp::Parameter("cartesian_avoid_collisions",
          root["scoop_cartesian_avoid_collisions"] ? root["scoop_cartesian_avoid_collisions"].as<bool>() : false)});
      marker_params_->set_parameters({rclcpp::Parameter("poses_yaml", poses)});
      if (load_poses_client_->wait_for_service(std::chrono::seconds(2))) {
        load_poses_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      }
      result.preflight_ok = !preflight || preflight_targets(root);
      result.success = !preflight || result.preflight_ok;
      result.message = result.success ? "Activated " + layout_id + " (plan-only preflight passed)" :
        "Activated " + layout_id + " but plan-only preflight failed";
      result.hash = hash;
    } catch (const std::exception& ex) {
      result.message = ex.what();
    }
    return result;
  }

  bool preflight_targets(const YAML::Node& root)
  {
    if (!move_to_client_->wait_for_action_server(std::chrono::seconds(3))) {
      RCLCPP_ERROR(get_logger(), "MoveTo unavailable for plan-only layout preflight");
      return false;
    }
    const std::vector<std::string> targets = {
      "MoveToScoopingContainer", "MoveToWeighingContainer", "ReturnHome",
      root["safe_retreat_target"].as<std::string>()};
    for (const auto& target : targets) {
      MoveTo::Goal goal;
      goal.target_name = target;
      goal.plan_only = true;  // This action never executes hardware motion.
      if (!plan_only_goal(goal)) {
        return false;
      }
    }
    if (scoop_poses_.poses.size() != 5U) {
      RCLCPP_ERROR(get_logger(), "Expected five scoop poses for layout preflight");
      return false;
    }
    for (const auto& pose : scoop_poses_.poses) {
      MoveTo::Goal goal;
      goal.target_pose.header = scoop_poses_.header;
      goal.target_pose.pose = pose;
      goal.plan_only = true;
      if (!plan_only_goal(goal)) {
        return false;
      }
    }
    return true;
  }

  bool plan_only_goal(const MoveTo::Goal& goal)
  {
    auto accepted = move_to_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), accepted, std::chrono::seconds(10)) !=
        rclcpp::FutureReturnCode::SUCCESS || !accepted.get()) {
      return false;
    }
    auto result = move_to_client_->async_get_result(accepted.get());
    return rclcpp::spin_until_future_complete(get_node_base_interface(), result, std::chrono::seconds(30)) ==
      rclcpp::FutureReturnCode::SUCCESS && result.get().result->success;
  }

  std::string run_state_{"idle"};
  rclcpp::Publisher<robot_common_msgs::msg::CellLayoutActive>::SharedPtr active_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr run_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr scoop_pose_sub_;
  geometry_msgs::msg::PoseArray scoop_poses_;
  rclcpp::Service<Apply>::SharedPtr apply_srv_;
  rclcpp_action::Client<MoveTo>::SharedPtr move_to_client_;
  rclcpp::AsyncParametersClient::SharedPtr marker_params_, move_params_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr load_poses_client_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CellLayoutManager>());
  rclcpp::shutdown();
  return 0;
}
