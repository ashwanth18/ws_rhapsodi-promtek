#pragma once

#include <behaviortree_cpp/action_node.h>

#include <chrono>
#include <future>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <robot_common_msgs/srv/plan_scoop.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "robot_orchestrator/last_failure.hpp"

namespace robot_orchestrator {

// Calls a scoop_vision service without blocking the tree; FAILURE on timeout,
// missing service, or success=false. Wrap in ForceSuccess / Fallback: vision
// is optional and the authored scoop is always a valid fallback.
template <typename ServiceT>
class ScoopVisionCallNode : public BT::StatefulActionNode {
public:
  ScoopVisionCallNode(
    const std::string& name, const BT::NodeConfiguration& cfg, const std::string& service)
  : BT::StatefulActionNode(name, cfg), service_(service)
  {
    node_ = config().blackboard->template get<rclcpp::Node::SharedPtr>("ros_node");
    client_ = node_->template create_client<ServiceT>(service_);
  }

  BT::NodeStatus onStart() override
  {
    using namespace std::chrono_literals;
    timeout_s_ = this->template getInput<double>("timeout_s").value_or(15.0);
    if (!client_->wait_for_service(1s)) {
      RCLCPP_WARN(node_->get_logger(), "%s: %s unavailable", this->name().c_str(), service_.c_str());
      setLastFailureReason(config().blackboard, this->name() + ": " + service_ + " unavailable");
      return BT::NodeStatus::FAILURE;
    }
    auto request = std::make_shared<typename ServiceT::Request>();
    fillRequest(*request);
    future_ = client_->async_send_request(request).future.share();
    start_ = node_->now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    using namespace std::chrono_literals;
    if (future_.valid() && future_.wait_for(0s) == std::future_status::ready) {
      const auto response = future_.get();
      if (!response) {
        setLastFailureReason(config().blackboard, this->name() + ": empty response");
        return BT::NodeStatus::FAILURE;
      }
      return handleResponse(*response) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    if ((node_->now() - start_).seconds() > timeout_s_) {
      RCLCPP_WARN(node_->get_logger(), "%s: %s timed out", this->name().c_str(), service_.c_str());
      setLastFailureReason(config().blackboard, this->name() + ": " + service_ + " timed out");
      return BT::NodeStatus::FAILURE;
    }
    rclcpp::spin_some(node_);
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override { future_ = {}; }

protected:
  virtual void fillRequest(typename ServiceT::Request& /*request*/) {}
  virtual bool handleResponse(const typename ServiceT::Response& response) = 0;

  rclcpp::Node::SharedPtr node_;

private:
  std::string service_;
  typename rclcpp::Client<ServiceT>::SharedPtr client_;
  std::shared_future<typename ServiceT::Response::SharedPtr> future_;
  rclcpp::Time start_;
  double timeout_s_{15.0};
};

// Fuse D455 frames into the powder height map. Run it while the arm is
// clear of the camera's view of the task container (at the weighing vessel).
class CaptureScoopSurfaceNode : public ScoopVisionCallNode<std_srvs::srv::Trigger> {
public:
  CaptureScoopSurfaceNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : ScoopVisionCallNode(name, cfg, "/scoop_vision/capture") {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout_s", 15.0, "capture timeout")};
  }

protected:
  bool handleResponse(const std_srvs::srv::Trigger::Response& response) override
  {
    RCLCPP_INFO(
      node_->get_logger(), "CaptureScoopSurface: %s", response.message.c_str());
    if (!response.success) {
      setLastFailureReason(config().blackboard, "CaptureScoopSurface: " + response.message);
    }
    return response.success;
  }
};

// Next scoop as a shift of the authored poses; feed the outputs to ExecuteScoop.
class PlanScoopFromVisionNode : public ScoopVisionCallNode<robot_common_msgs::srv::PlanScoop> {
public:
  PlanScoopFromVisionNode(const std::string& name, const BT::NodeConfiguration& cfg)
  : ScoopVisionCallNode(name, cfg, "/scoop_vision/plan") {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("timeout_s", 15.0, "planning timeout"),
      BT::InputPort<double>("max_heightmap_age_s", 0.0, "0 = scoop_vision default"),
      BT::OutputPort<double>("pattern_offset_x", "scoop task-frame X shift (m)"),
      BT::OutputPort<double>("pattern_offset_y", "scoop task-frame Y shift (m)"),
      BT::OutputPort<double>("pattern_offset_z", "scoop task-frame Z shift (m)"),
      BT::OutputPort<double>("predicted_fill_ratio", "predicted fill / bowl capacity"),
      BT::OutputPort<bool>("container_empty", "best reachable scoop is nearly empty"),
    };
  }

protected:
  void fillRequest(robot_common_msgs::srv::PlanScoop::Request& request) override
  {
    request.max_heightmap_age_s = getInput<double>("max_heightmap_age_s").value_or(0.0);
  }

  bool handleResponse(const robot_common_msgs::srv::PlanScoop::Response& response) override
  {
    setOutput("container_empty", response.container_empty);
    if (!response.success) {
      RCLCPP_WARN(node_->get_logger(), "PlanScoopFromVision: %s", response.message.c_str());
      setLastFailureReason(config().blackboard, "PlanScoopFromVision: " + response.message);
      return false;
    }
    setOutput("pattern_offset_x", response.pattern_offset_x);
    setOutput("pattern_offset_y", response.pattern_offset_y);
    setOutput("pattern_offset_z", response.pattern_offset_z);
    setOutput("predicted_fill_ratio", response.predicted_fill_ratio);
    RCLCPP_INFO(
      node_->get_logger(),
      "PlanScoopFromVision: offset=(%.3f, %.3f, %.3f) m fill=%.0f%% pen=%.0f mm surface=%.3f m",
      response.pattern_offset_x, response.pattern_offset_y, response.pattern_offset_z,
      100.0 * response.predicted_fill_ratio, 1000.0 * response.max_penetration_m,
      response.surface_height_m);
    return true;
  }
};

}  // namespace robot_orchestrator
