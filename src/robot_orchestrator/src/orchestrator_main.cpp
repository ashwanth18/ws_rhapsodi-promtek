#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <robot_common_msgs/msg/system_status.hpp>
#include <sstream>
#include <robot_common_msgs/srv/start_batch.hpp>
#include <robot_common_msgs/srv/start_lights_out.hpp>
#include <robot_common_msgs/srv/start_webhook_weightment.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <memory>
#include <mutex>
#include <thread>
#include "robot_orchestrator/register.hpp"
#include "robot_orchestrator/mode_start.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rhapsodi_common_cpp/health_event_publisher.hpp>

namespace {

struct PendingRun {
  std::string tree_id;
  std::string active_mode;
  std::string phase_topic;
};

void publishString(
  const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr & pub,
  const std::string & value)
{
  std_msgs::msg::String msg;
  msg.data = value;
  pub->publish(msg);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;
  robot_orchestrator::RegisterNodes(factory);

  // Optional tree_file: preferred default for idle display / legacy compose.
  // All XMLs under share/bt_trees/ are registered so modes can hot-switch.
  std::string tree_file;
  auto node_tmp = rclcpp::Node::make_shared("bt_tree_loader");
  node_tmp->declare_parameter<std::string>("tree_file", "");
  node_tmp->get_parameter("tree_file", tree_file);

  const std::string share =
    ament_index_cpp::get_package_share_directory("robot_orchestrator");
  const std::filesystem::path bt_trees_dir = std::filesystem::path(share) / "bt_trees";

  try {
    robot_orchestrator::registerBehaviorTreesFromDirectory(factory, bt_trees_dir);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      node_tmp->get_logger(),
      "Failed to register BehaviorTrees from %s: %s",
      bt_trees_dir.c_str(), e.what());
    throw;
  }

  // If tree_file points outside the share directory, register it too.
  if (!tree_file.empty()) {
    const std::filesystem::path tree_path(tree_file);
    std::error_code ec;
    const auto canonical_trees = std::filesystem::weakly_canonical(bt_trees_dir, ec);
    const auto canonical_file = std::filesystem::weakly_canonical(tree_path, ec);
    const bool under_share =
      !ec &&
      canonical_file.string().rfind(canonical_trees.string(), 0) == 0;
    if (!under_share && std::filesystem::exists(tree_path)) {
      try {
        factory.registerBehaviorTreeFromFile(tree_path);
        RCLCPP_INFO(
          node_tmp->get_logger(),
          "Also registered external tree_file: %s", tree_file.c_str());
      } catch (const std::exception & e) {
        RCLCPP_FATAL(
          node_tmp->get_logger(),
          "Failed to register tree_file=%s: %s", tree_file.c_str(), e.what());
        throw;
      }
    }
  }

  const auto registered = factory.registeredBehaviorTrees();
  {
    std::ostringstream oss;
    for (size_t i = 0; i < registered.size(); ++i) {
      if (i) {
        oss << ", ";
      }
      oss << registered[i];
    }
    RCLCPP_INFO(
      node_tmp->get_logger(),
      "Registered BehaviorTrees (%zu): %s",
      registered.size(), oss.str().c_str());
  }
  if (registered.empty()) {
    RCLCPP_FATAL(node_tmp->get_logger(), "No BehaviorTrees registered; refusing to start");
    throw std::runtime_error("No BehaviorTrees registered");
  }

  std::string preferred_tree_id = "WebhookWeightment";
  if (!tree_file.empty()) {
    preferred_tree_id = robot_orchestrator::treeIdHintFromPath(tree_file);
  }
  {
    const bool preferred_registered =
      std::find(registered.begin(), registered.end(), preferred_tree_id) !=
      registered.end();
    if (!preferred_registered) {
      preferred_tree_id = registered.front();
    }
  }

  // Create shared ROS node and blackboard (tree instantiated per run).
  auto ros_node = rclcpp::Node::make_shared("robot_orchestrator");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", ros_node);
  ros_node->declare_parameter<std::string>("lightsout_layout_id", "lightsout-single-vessel");
  ros_node->declare_parameter<std::string>("webhook_layout_id", "dual-container");
  ros_node->declare_parameter<std::string>("batch_layout_id", "dual-container");
  // End-of-episode scoop residue purge (tilt + vibrate). Cell tuning, not per-run.
  ros_node->declare_parameter<bool>("lightsout_purge_enabled", true);
  ros_node->declare_parameter<double>("lightsout_purge_incline_deg", 20.0);
  ros_node->declare_parameter<double>("lightsout_purge_vibration", 0.8);
  ros_node->declare_parameter<double>("lightsout_purge_duration_s", 10.0);
  ros_node->declare_parameter<std::string>("lightsout_purge_target", "");
  // true = TiltAboutTcp (tip fixed); false = legacy joint_5 /incline_control tilt.
  ros_node->declare_parameter<bool>("lightsout_purge_hold_tcp", true);
  blackboard->set("poses_provenance_ok", true);

  // Status publisher
  auto status_pub = ros_node->create_publisher<robot_common_msgs::msg::SystemStatus>("/system_status", 10);
  rhapsodi_common_cpp::HealthEventPublisher health(ros_node.get(), "robot_orchestrator");

  auto latched_qos = rclcpp::QoS(1).transient_local();

  // Lights-out training status/metadata publishers
  auto lightsout_active_pub = ros_node->create_publisher<std_msgs::msg::Bool>("/lightsout_training/active", latched_qos);
  auto lightsout_meta_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/metadata", latched_qos);
  auto lightsout_run_id_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/run_id", latched_qos);
  auto lightsout_batch_id_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/batch_id", latched_qos);
  auto lightsout_ingredient_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/ingredient_id", latched_qos);
  auto lightsout_target_pub = ros_node->create_publisher<std_msgs::msg::Float64>("/lightsout_training/target_weight_g", latched_qos);
  auto lightsout_mode_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/mode", latched_qos);
  auto lightsout_robot_id_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/robot_id", latched_qos);
  auto lightsout_episodes_total_pub = ros_node->create_publisher<std_msgs::msg::Int32>("/lightsout_training/episodes_total", latched_qos);
  auto lightsout_powder_id_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/powder_id", latched_qos);
  auto lightsout_lot_code_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/lot_code", latched_qos);
  auto lightsout_operator_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/operator", latched_qos);
  auto lightsout_notes_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/notes", latched_qos);
  auto lightsout_scooped_mass_pub = ros_node->create_publisher<std_msgs::msg::Float32>("/lightsout_training/scooped_mass_g", latched_qos);
  auto lightsout_pour_outcome_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/pour_outcome", latched_qos);
  auto lightsout_total_poured_pub = ros_node->create_publisher<std_msgs::msg::Float32>("/lightsout_training/total_poured_g", latched_qos);
  auto lightsout_stop_reason_pub = ros_node->create_publisher<std_msgs::msg::String>("/lightsout_training/stop_reason", latched_qos);
  blackboard->set("lightsout_target_weight_pub", lightsout_target_pub);
  blackboard->set("lightsout_scooped_mass_pub", lightsout_scooped_mass_pub);
  blackboard->set("lightsout_pour_outcome_pub", lightsout_pour_outcome_pub);
  blackboard->set("lightsout_total_poured_pub", lightsout_total_poured_pub);
  blackboard->set("lightsout_stop_reason_pub", lightsout_stop_reason_pub);
  auto webhook_active_pub = ros_node->create_publisher<std_msgs::msg::Bool>("/webhook_run/active", latched_qos);
  auto webhook_meta_pub = ros_node->create_publisher<std_msgs::msg::String>("/webhook_run/metadata", latched_qos);
  auto run_state_pub = ros_node->create_publisher<std_msgs::msg::String>("/orchestrator/run_state", latched_qos);
  auto active_mode_pub = ros_node->create_publisher<std_msgs::msg::String>("/orchestrator/active_mode", latched_qos);
  publishString(run_state_pub, "idle");
  publishString(active_mode_pub, "idle");
  RCLCPP_INFO(
    ros_node->get_logger(),
    "Orchestrator idle; preferred_tree_id=%s (tree_file=%s). Trees created per run.",
    preferred_tree_id.c_str(),
    tree_file.empty() ? "<unset>" : tree_file.c_str());

  // Subscribe to weight topic and keep blackboard updated continuously
  ros_node->declare_parameter<std::string>("weight_topic", "/weight");
  auto weight_topic = ros_node->get_parameter("weight_topic").as_string();
  auto weight_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
    weight_topic, rclcpp::SensorDataQoS(),
    [blackboard, ros_node](const std_msgs::msg::Float64::SharedPtr msg){
      blackboard->set("scale_weight", static_cast<double>(msg->data));
      blackboard->set("scale_weight_time", ros_node->now().seconds());
    });

  // Per-run tree + loggers (recreated when a start is accepted).
  std::unique_ptr<BT::Tree> tree;
  std::unique_ptr<BT::StdCoutLogger> cout_logger;
  std::unique_ptr<BT::Groot2Publisher> groot2;

  // DIY start/pause/stop services
  std::atomic<bool> start_requested{false};
  std::atomic<bool> pause_requested{false};
  std::atomic<bool> stop_requested{false};
  std::atomic<bool> lightsout_active{false};
  std::atomic<bool> webhook_active{false};
  // True while the main loop is ticking a tree (run_state == running).
  std::atomic<bool> run_active{false};

  std::mutex pending_mu;
  PendingRun pending_run;

  // ---------------------------------------------------------------------------
  // CONCURRENCY GUARD (Phase 2 bugfix — revertible)
  // Previously every start callback set accepted=true with no check, allowing a
  // second start to overwrite blackboard mid-tick. Reject when a start is
  // already pending or a tree is actively ticking. Remove this helper (and the
  // call sites) to restore the old always-accept behavior.
  // ---------------------------------------------------------------------------
  auto tryClaimStart = [&](std::string & reject_message) -> bool {
    if (run_active.load() || start_requested.load()) {
      reject_message =
        "Rejected: orchestrator already has an active or pending run "
        "(stop it first, or wait for completion)";
      return false;
    }
    bool expected = false;
    if (!start_requested.compare_exchange_strong(expected, true)) {
      reject_message =
        "Rejected: orchestrator already has an active or pending run "
        "(stop it first, or wait for completion)";
      return false;
    }
    if (run_active.load()) {
      start_requested.store(false);
      reject_message =
        "Rejected: orchestrator already has an active or pending run "
        "(stop it first, or wait for completion)";
      return false;
    }
    return true;
  };

  auto beginPendingRun = [&](const std::string & tree_id,
                             const std::string & active_mode,
                             const std::string & phase_topic) {
    robot_orchestrator::clearModeBlackboardKeys(blackboard);
    blackboard->set("phase_topic", phase_topic);
    {
      std::lock_guard<std::mutex> lock(pending_mu);
      pending_run.tree_id = tree_id;
      pending_run.active_mode = active_mode;
      pending_run.phase_topic = phase_topic;
    }
    pause_requested = false;
    stop_requested = false;
    publishString(active_mode_pub, active_mode);
  };

  // Service to dump ASCII tree to logs on demand
  auto dump_srv = ros_node->create_service<std_srvs::srv::Trigger>(
    "bt_dump",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp){
      if (!tree) {
        std::ostringstream oss;
        oss << "No active tree. Registered: ";
        const auto ids = factory.registeredBehaviorTrees();
        for (size_t i = 0; i < ids.size(); ++i) {
          if (i) {
            oss << ", ";
          }
          oss << ids[i];
        }
        resp->success = true;
        resp->message = oss.str();
        RCLCPP_INFO(ros_node->get_logger(), "%s", resp->message.c_str());
        return;
      }
      std::stringstream ss;
      BT::printTreeRecursively(tree->rootNode(), ss);
      RCLCPP_INFO(ros_node->get_logger(), "BT ASCII Tree:\n%s", ss.str().c_str());
      resp->success = true;
      resp->message = "Tree printed to log";
    });

  using StartBatch = robot_common_msgs::srv::StartBatch;
  auto start_batch_srv = ros_node->create_service<StartBatch>(
    "bt_start_batch",
    [&](const std::shared_ptr<StartBatch::Request> req,
        std::shared_ptr<StartBatch::Response> resp){
      std::string reject_message;
      if (!tryClaimStart(reject_message)) {
        resp->accepted = false;
        resp->message = reject_message;
        RCLCPP_WARN(ros_node->get_logger(), "bt_start_batch: %s", reject_message.c_str());
        return;
      }
      beginPendingRun("Main", "batch", "/lightsout_training/phase");
      blackboard->set(
        "expected_layout_id",
        ros_node->get_parameter("batch_layout_id").as_string());
      blackboard->set("containers", req->containers);
      blackboard->set("container_index", static_cast<std::size_t>(0));
      blackboard->set("container_name", std::string(""));
      blackboard->set("expected_lot", std::string(""));
      resp->accepted = true;
      resp->message = "Batch accepted (tree_id=Main)";
    });

  using StartLightsOut = robot_common_msgs::srv::StartLightsOut;
  auto start_lightsout_srv = ros_node->create_service<StartLightsOut>(
    "bt_start_lightsout",
    [&, lightsout_active_pub,
      lightsout_run_id_pub, lightsout_batch_id_pub, lightsout_ingredient_pub,
      lightsout_target_pub, lightsout_mode_pub, lightsout_robot_id_pub,
      lightsout_episodes_total_pub, lightsout_powder_id_pub, lightsout_lot_code_pub,
      lightsout_operator_pub, lightsout_notes_pub, lightsout_scooped_mass_pub](
        const std::shared_ptr<StartLightsOut::Request> req,
                    std::shared_ptr<StartLightsOut::Response> resp){
      std::string reject_message;
      if (!tryClaimStart(reject_message)) {
        resp->accepted = false;
        resp->message = reject_message;
        RCLCPP_WARN(ros_node->get_logger(), "bt_start_lightsout: %s", reject_message.c_str());
        return;
      }

      const int episodes = std::max(1, req->episodes);
      const double target_g = static_cast<double>(req->target_weight_g);
      const double tolerance_g = std::max(0.1, target_g * 0.02);
      const auto now = std::chrono::system_clock::now();
      const std::time_t now_t = std::chrono::system_clock::to_time_t(now);
      std::tm tm{};
      gmtime_r(&now_t, &tm);
      char ts_buf[32];
      std::strftime(ts_buf, sizeof(ts_buf), "%Y%m%dT%H%M%SZ", &tm);
      const std::string run_id = std::string(ts_buf);

      const std::string container_target =
        req->container_target.empty() ? req->powder_name : req->container_target;
      const std::string pour_target =
        req->pour_target.empty() ? container_target : req->pour_target;

      std::string fractions_csv;
      for (std::size_t i = 0; i < req->target_fractions.size(); ++i) {
        if (i > 0) {
          fractions_csv += ',';
        }
        fractions_csv += std::to_string(req->target_fractions[i]);
      }

      beginPendingRun("LightsOut", "lightsout", "/lightsout_training/phase");
      blackboard->set(
        "expected_layout_id",
        ros_node->get_parameter("lightsout_layout_id").as_string());
      const std::string stop_on =
        req->stop_on.empty() ? std::string("episodes") : req->stop_on;
      blackboard->set("lightsout_powder_id", req->powder_id);
      blackboard->set("lightsout_powder_name", req->powder_name);
      blackboard->set("lightsout_container_name", container_target);
      blackboard->set("lightsout_pour_target", pour_target);
      blackboard->set("lightsout_lot_code", req->lot_code);
      blackboard->set("lightsout_operator", req->operator_name);
      blackboard->set("lightsout_notes", req->notes);
      blackboard->set("lightsout_stop_on", stop_on);
      blackboard->set("lightsout_stop_value", static_cast<double>(req->stop_value));
      blackboard->set("lightsout_stop_requested", false);
      blackboard->set("lightsout_session_start_s", ros_node->now().seconds());
      blackboard->set("lightsout_total_poured_g", 0.0);
      blackboard->set("lightsout_stop_reason", std::string(""));
      blackboard->set("lightsout_post_scoop_weight_g", 0.0);
      blackboard->set("lightsout_tolerance_frac", 0.02);
      blackboard->set("lightsout_target_mode",
                      req->target_mode.empty() ? std::string("fixed") : req->target_mode);
      blackboard->set("lightsout_fixed_target_g", target_g);
      blackboard->set("lightsout_target_fractions_csv", fractions_csv);
      blackboard->set("lightsout_min_scooped_g",
                      req->min_scooped_g > 0.0f
                        ? static_cast<double>(req->min_scooped_g)
                        : 20.0);
      blackboard->set("lightsout_target_min_g", static_cast<double>(req->target_min_g));
      blackboard->set("lightsout_target_max_g", static_cast<double>(req->target_max_g));
      blackboard->set("lightsout_rng_seed", req->rng_seed);
      blackboard->set("lightsout_target_weight_g", target_g);
      blackboard->set("lightsout_tolerance_g", tolerance_g);
      blackboard->set("lightsout_episodes", episodes);
      blackboard->set("lightsout_batch_id", req->batch_id);
      blackboard->set("lightsout_episode_index", 0);
      blackboard->set("enable_scoop", static_cast<bool>(req->enable_scoop));

      const bool purge_enabled =
        ros_node->get_parameter("lightsout_purge_enabled").as_bool();
      const std::string purge_target =
        ros_node->get_parameter("lightsout_purge_target").as_string();
      const bool purge_hold_tcp =
        ros_node->get_parameter("lightsout_purge_hold_tcp").as_bool();
      blackboard->set("lightsout_purge_enabled", purge_enabled);
      blackboard->set(
        "lightsout_purge_incline_deg",
        ros_node->get_parameter("lightsout_purge_incline_deg").as_double());
      blackboard->set(
        "lightsout_purge_vibration",
        ros_node->get_parameter("lightsout_purge_vibration").as_double());
      blackboard->set(
        "lightsout_purge_duration_s",
        ros_node->get_parameter("lightsout_purge_duration_s").as_double());
      blackboard->set("lightsout_purge_target", purge_target);
      // Empty target => purge in place at the pour pose (no extra MoveTo).
      blackboard->set("lightsout_purge_pose_enabled", !purge_target.empty());
      blackboard->set("lightsout_purge_hold_tcp", purge_hold_tcp);
      // Complementary flag so the BT tree can select the legacy joint tilt
      // without relying on script negation.
      blackboard->set("lightsout_purge_use_joint_incline", !purge_hold_tcp);

      lightsout_active = true;

      std_msgs::msg::Bool active_msg;
      active_msg.data = true;
      lightsout_active_pub->publish(active_msg);

      std_msgs::msg::String run_msg;
      run_msg.data = run_id;
      lightsout_run_id_pub->publish(run_msg);

      std_msgs::msg::String batch_msg;
      batch_msg.data = req->batch_id;
      lightsout_batch_id_pub->publish(batch_msg);

      std_msgs::msg::String ingredient_msg;
      ingredient_msg.data = req->powder_name;
      lightsout_ingredient_pub->publish(ingredient_msg);

      std_msgs::msg::String powder_id_msg;
      powder_id_msg.data = req->powder_id;
      lightsout_powder_id_pub->publish(powder_id_msg);

      std_msgs::msg::String lot_code_msg;
      lot_code_msg.data = req->lot_code;
      lightsout_lot_code_pub->publish(lot_code_msg);

      std_msgs::msg::String operator_msg;
      operator_msg.data = req->operator_name;
      lightsout_operator_pub->publish(operator_msg);

      std_msgs::msg::String notes_msg;
      notes_msg.data = req->notes;
      lightsout_notes_pub->publish(notes_msg);

      std_msgs::msg::Float64 target_msg;
      target_msg.data = target_g;
      lightsout_target_pub->publish(target_msg);

      std_msgs::msg::String mode_msg;
      mode_msg.data = "lightsout";
      lightsout_mode_pub->publish(mode_msg);

      if (!ros_node->has_parameter("robot_id")) {
        ros_node->declare_parameter<std::string>("robot_id", "robot-1");
      }
      std_msgs::msg::String robot_msg;
      robot_msg.data = ros_node->get_parameter("robot_id").as_string();
      lightsout_robot_id_pub->publish(robot_msg);

      std_msgs::msg::Int32 episodes_msg;
      episodes_msg.data = episodes;
      lightsout_episodes_total_pub->publish(episodes_msg);

      resp->accepted = true;
      resp->message = "Lights-out training accepted (tree_id=LightsOut)";
    });

  using StartWebhookWeightment = robot_common_msgs::srv::StartWebhookWeightment;
  auto start_webhook_weightment_srv = ros_node->create_service<StartWebhookWeightment>(
    "bt_start_webhook_weightment",
    [&, webhook_active_pub, webhook_meta_pub](
      const std::shared_ptr<StartWebhookWeightment::Request> req,
      std::shared_ptr<StartWebhookWeightment::Response> resp){
      std::string reject_message;
      if (!tryClaimStart(reject_message)) {
        resp->accepted = false;
        resp->message = reject_message;
        RCLCPP_WARN(
          ros_node->get_logger(), "bt_start_webhook_weightment: %s", reject_message.c_str());
        return;
      }

      const double target_g = static_cast<double>(req->target_weight_g);
      // Honor caller-supplied tolerance when > 0; otherwise 2% of target
      // (floor 0.1 g) so a zero/unset float32 still gets a usable band.
      const double tolerance_g =
        (static_cast<double>(req->tolerance_g) > 0.0)
          ? static_cast<double>(req->tolerance_g)
          : std::max(0.1, target_g * 0.02);
      if (!ros_node->has_parameter("robot_id")) {
        ros_node->declare_parameter<std::string>("robot_id", "robot-1");
      }
      const std::string robot_id = ros_node->get_parameter("robot_id").as_string();

      auto json_escape = [](const std::string & value) {
        std::string out;
        out.reserve(value.size());
        for (const char ch : value) {
          switch (ch) {
            case '\\': out += "\\\\"; break;
            case '"': out += "\\\""; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default: out += ch; break;
          }
        }
        return out;
      };

      // MES-family / mock share WebhookWeightment; active_mode aligns with RunSpec.
      beginPendingRun("WebhookWeightment", "mes-condor", "/webhook_run/phase");
      blackboard->set(
        "expected_layout_id",
        ros_node->get_parameter("webhook_layout_id").as_string());
      blackboard->set("webhook_run_id", req->run_id);
      blackboard->set("webhook_weightment_id", req->weightment_id);
      blackboard->set("webhook_batch_id", req->batch_id);
      blackboard->set("webhook_ingredient_id", req->ingredient_id);
      blackboard->set("webhook_location_id", req->location_id);
      blackboard->set("webhook_location_code", req->location_code);
      blackboard->set("webhook_pickup_target_name", req->pickup_target_name);
      blackboard->set("webhook_weigh_target_name", req->weigh_target_name);
      blackboard->set("webhook_return_target_name", req->return_target_name);
      blackboard->set("webhook_target_weight_g", target_g);
      blackboard->set("webhook_tolerance_g", tolerance_g);
      blackboard->set("webhook_expected_lot", req->expected_lot);

      webhook_active = true;

      std_msgs::msg::Bool active_msg;
      active_msg.data = true;
      webhook_active_pub->publish(active_msg);

      std_msgs::msg::String meta_msg;
      meta_msg.data =
        std::string("{") +
        "\"run_id\":\"" + json_escape(req->run_id) + "\"," +
        "\"weightment_id\":\"" + json_escape(req->weightment_id) + "\"," +
        "\"batch_id\":\"" + json_escape(req->batch_id) + "\"," +
        "\"ingredient_id\":\"" + json_escape(req->ingredient_id) + "\"," +
        "\"location_id\":\"" + json_escape(req->location_id) + "\"," +
        "\"location_code\":\"" + json_escape(req->location_code) + "\"," +
        "\"pickup_target_name\":\"" + json_escape(req->pickup_target_name) + "\"," +
        "\"weigh_target_name\":\"" + json_escape(req->weigh_target_name) + "\"," +
        "\"return_target_name\":\"" + json_escape(req->return_target_name) + "\"," +
        "\"expected_lot\":\"" + json_escape(req->expected_lot) + "\"," +
        "\"target_weight_g\":" + std::to_string(target_g) + "," +
        "\"tolerance_g\":" + std::to_string(tolerance_g) + "," +
        "\"mode\":\"webhook\"," +
        "\"robot_id\":\"" + json_escape(robot_id) + "\"" +
        "}";
      webhook_meta_pub->publish(meta_msg);

      RCLCPP_INFO(
        ros_node->get_logger(),
        "Webhook weightment accepted: run_id=%s weightment_id=%s batch_id=%s ingredient_id=%s pickup=%s weigh=%s return=%s target_g=%.3f tolerance_g=%.3f tree_id=WebhookWeightment",
        req->run_id.c_str(),
        req->weightment_id.c_str(),
        req->batch_id.c_str(),
        req->ingredient_id.c_str(),
        req->pickup_target_name.c_str(),
        req->weigh_target_name.c_str(),
        req->return_target_name.c_str(),
        target_g,
        tolerance_g);

      resp->accepted = true;
      resp->message = "Webhook weightment accepted (tree_id=WebhookWeightment)";
    });

  auto pause_srv = ros_node->create_service<std_srvs::srv::Trigger>(
    "bt_pause",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp){
      pause_requested = true;
      resp->success = true;
      resp->message = "BT pause requested";
    });

  auto resume_srv = ros_node->create_service<std_srvs::srv::Trigger>(
    "bt_resume",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp){
      pause_requested = false;
      resp->success = true;
      resp->message = "BT resume requested";
    });

  auto stop_srv = ros_node->create_service<std_srvs::srv::Trigger>(
    "bt_stop",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp){
      stop_requested = true;
      resp->success = true;
      resp->message = "BT stop requested";
    });

  // Optional auto-restart after finish
  ros_node->declare_parameter<bool>("auto_restart", false);

  while (rclcpp::ok()) {
    bool auto_restart = ros_node->get_parameter("auto_restart").as_bool();

    // Wait for start unless auto_restart is enabled
    if (!auto_restart) {
      while (rclcpp::ok() && !start_requested.load()) {
        rclcpp::spin_some(ros_node);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      if (!rclcpp::ok()) {
        break;
      }
    } else if (!start_requested.load()) {
      // auto_restart with no prior pending run: seed preferred tree once
      beginPendingRun(preferred_tree_id, "batch", "/lightsout_training/phase");
      start_requested = true;
    }

    PendingRun this_run;
    {
      std::lock_guard<std::mutex> lock(pending_mu);
      this_run = pending_run;
    }
    if (this_run.tree_id.empty()) {
      this_run.tree_id = preferred_tree_id;
      this_run.active_mode = "batch";
      this_run.phase_topic = "/lightsout_training/phase";
    }

    // Mark running before clearing the start latch so the concurrency guard
    // never sees a window where both flags are false mid-handoff.
    run_active = true;
    start_requested = false;
    pause_requested = false;
    stop_requested = false;

    // Create tree for this run from the resolved tree_id.
    cout_logger.reset();
    groot2.reset();
    tree.reset();
    try {
      tree = std::make_unique<BT::Tree>(
        factory.createTree(this_run.tree_id, blackboard));
    } catch (const std::exception & e) {
      run_active = false;
      RCLCPP_ERROR(
        ros_node->get_logger(),
        "Failed to create tree_id=%s: %s",
        this_run.tree_id.c_str(), e.what());
      health.error(
        "bt_tree_create_failure",
        "Failed to create behavior tree tree_id=" + this_run.tree_id,
        "{\"tree_id\":\"" + this_run.tree_id + "\",\"run_mode\":\"" +
          this_run.active_mode + "\"}");
      publishString(run_state_pub, "failed");
      publishString(active_mode_pub, "idle");
      lightsout_active = false;
      webhook_active = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    BT::printTreeRecursively(tree->rootNode(), std::cout);
    cout_logger = std::make_unique<BT::StdCoutLogger>(*tree);
    groot2 = std::make_unique<BT::Groot2Publisher>(*tree, 1666);

    RCLCPP_INFO(
      ros_node->get_logger(),
      "Starting run: tree_id=%s active_mode=%s phase_topic=%s",
      this_run.tree_id.c_str(),
      this_run.active_mode.c_str(),
      this_run.phase_topic.c_str());

    publishString(run_state_pub, "running");
    publishString(active_mode_pub, this_run.active_mode);

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    bool stopped = false;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      if (stop_requested.load()) {
        tree->haltTree();
        stopped = true;
        break;
      }
      if (!pause_requested.load()) {
        status = tree->tickOnce();
      }
      // Publish status each tick
      robot_common_msgs::msg::SystemStatus st;
      st.phase = pause_requested.load() ? std::string("PAUSED") : std::string("RUNNING");
      // queue_remaining
      try {
        std::vector<robot_common_msgs::msg::ContainerSpec> containers;
        std::size_t index = 0;
        (void)blackboard->get("containers", containers);
        (void)blackboard->get("container_index", index);
        if (containers.size() > index) {
          st.queue_remaining = static_cast<uint32_t>(containers.size() - index);
        } else {
          st.queue_remaining = 0U;
        }
      } catch (...) { st.queue_remaining = 0U; }
      // current container and targets
      try { st.container_name = blackboard->get<std::string>("container_name"); } catch (...) {}
      try { st.container_target_g = static_cast<float>(blackboard->get<double>("container_target_g")); } catch (...) {}
      try { st.weight_tolerance_g = static_cast<float>(blackboard->get<double>("batch_weight_tolerance")); } catch (...) {}
      try { st.scale_weight_g = static_cast<float>(blackboard->get<double>("scale_weight")); } catch (...) {}
      status_pub->publish(st);
      rclcpp::spin_some(ros_node);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    run_active = false;

    // After a cycle, re-arm start_requested only for auto_restart
    if (auto_restart) {
      start_requested = true;
    }

    const std::string completed_run_mode = this_run.active_mode.empty()
      ? (lightsout_active.load() ? "lightsout" :
         (webhook_active.load() ? "mes-condor" : "unknown"))
      : this_run.active_mode;

    if (lightsout_active.load()) {
      lightsout_active = false;
      std_msgs::msg::Bool active_msg;
      active_msg.data = false;
      lightsout_active_pub->publish(active_msg);
    }
    if (webhook_active.load()) {
      webhook_active = false;
      std_msgs::msg::Bool active_msg;
      active_msg.data = false;
      webhook_active_pub->publish(active_msg);
    }

    {
      std::string run_state;
      if (stopped) {
        run_state = "stopped";
      } else if (status == BT::NodeStatus::SUCCESS) {
        run_state = "succeeded";
      } else if (status == BT::NodeStatus::FAILURE) {
        run_state = "failed";
        health.error(
          "bt_tree_failure",
          "Behavior tree returned FAILURE for tree_id=" + this_run.tree_id,
          "{\"tree_id\":\"" + this_run.tree_id + "\",\"run_mode\":\"" +
            completed_run_mode + "\"}");
      } else {
        run_state = "idle";
      }
      publishString(run_state_pub, run_state);
    }
    publishString(active_mode_pub, "idle");

    // Brief idle between cycles
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}
