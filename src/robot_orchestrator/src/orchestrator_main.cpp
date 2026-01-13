#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <robot_common_msgs/msg/system_status.hpp>
#include <sstream>
#include <robot_common_msgs/srv/start_batch.hpp>
#include <atomic>
#include "robot_orchestrator/register.hpp"
#include <std_msgs/msg/float64.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;
  robot_orchestrator::RegisterNodes(factory);

  // Load external XML tree (param "tree_file" or default from package share)
  std::string tree_file;
  auto node_tmp = rclcpp::Node::make_shared("bt_tree_loader");
  node_tmp->declare_parameter<std::string>("tree_file", "");
  node_tmp->get_parameter("tree_file", tree_file);
  if (tree_file.empty()) {
    const std::string share = ament_index_cpp::get_package_share_directory("robot_orchestrator");
    tree_file = share + "/bt_trees/main.xml";
  }

  // Create shared ROS node and blackboard
  auto ros_node = rclcpp::Node::make_shared("robot_orchestrator");
  auto blackboard = BT::Blackboard::create();
  blackboard->set("ros_node", ros_node);

  // Status publisher
  auto status_pub = ros_node->create_publisher<robot_common_msgs::msg::SystemStatus>("/system_status", 10);

  // Subscribe to weight topic and keep blackboard updated continuously
  ros_node->declare_parameter<std::string>("weight_topic", "/weight");
  auto weight_topic = ros_node->get_parameter("weight_topic").as_string();
  auto weight_sub = ros_node->create_subscription<std_msgs::msg::Float64>(
    weight_topic, rclcpp::SensorDataQoS(),
    [blackboard, ros_node](const std_msgs::msg::Float64::SharedPtr msg){
      blackboard->set("scale_weight", static_cast<double>(msg->data));
      blackboard->set("scale_weight_time", ros_node->now().seconds());
    });

  // Build tree
  auto tree = factory.createTreeFromFile(tree_file, blackboard);
  // Print tree structure (ASCII) on startup
  BT::printTreeRecursively(tree.rootNode(), std::cout);
  BT::StdCoutLogger logger(tree);
  BT::Groot2Publisher groot2(tree, 1666);

  // Service to dump ASCII tree to logs on demand
  auto dump_srv = ros_node->create_service<std_srvs::srv::Trigger>(
    "bt_dump",
    [&](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp){
      std::stringstream ss;
      BT::printTreeRecursively(tree.rootNode(), ss);
      RCLCPP_INFO(ros_node->get_logger(), "BT ASCII Tree:\n%s", ss.str().c_str());
      resp->success = true;
      resp->message = "Tree printed to log";
    });

  // DIY start/pause/stop services
  std::atomic<bool> start_requested{false};
  std::atomic<bool> pause_requested{false};
  std::atomic<bool> stop_requested{false};

  using StartBatch = robot_common_msgs::srv::StartBatch;
  auto start_batch_srv = ros_node->create_service<StartBatch>(
    "bt_start_batch",
    [&, blackboard](const std::shared_ptr<StartBatch::Request> req,
                    std::shared_ptr<StartBatch::Response> resp){
      // Store the full container list and reset index
      blackboard->set("containers", req->containers);
      blackboard->set("container_index", static_cast<std::size_t>(0));
      // Clear any currently active container
      blackboard->set("container_name", std::string(""));
      blackboard->set("expected_lot", std::string(""));
      start_requested = true;
      pause_requested = false;
      stop_requested = false;
      resp->accepted = true;
      resp->message = "Batch accepted";
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
    }

    // Reset flags for this run
    pause_requested = false;
    stop_requested = false;

    // Reset tree before running
    tree.haltTree();

    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
      if (stop_requested.load()) {
        tree.haltTree();
        break;
      }
      if (!pause_requested.load()) {
        status = tree.tickOnce();
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

    // After a cycle, clear start request unless auto-restart
    if (!auto_restart) {
      start_requested = false;
    }

    // Brief idle between cycles
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  rclcpp::shutdown();
  return 0;
}


