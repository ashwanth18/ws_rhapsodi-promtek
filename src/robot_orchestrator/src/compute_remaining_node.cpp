#include "robot_orchestrator/compute_remaining_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robot_orchestrator {

BT::PortsList ComputeRemainingNode::providedPorts()
{
  return {
    BT::InputPort<double>("batch_target"),
    BT::InputPort<double>("tolerance"),
    BT::OutputPort<double>("remaining")
  };
}

BT::NodeStatus ComputeRemainingNode::tick()
{
  double batch_target = 0.0, tol = 0.0;
  // Use the explicit BT input first so webhook flows honor their requested target.
  if (!getInput("batch_target", batch_target) || batch_target <= 0.0) {
    // Fall back to blackboard-backed container flows when no explicit input is provided.
    if (!config().blackboard->get("container_target_g", batch_target)) {
      // Final fallback to previous remaining target for legacy/recompute use.
      (void)config().blackboard->get("remaining_weight", batch_target);
    }
  }
  if (batch_target <= 0.0) return BT::NodeStatus::FAILURE;
  if (!getInput("tolerance", tol)) return BT::NodeStatus::FAILURE;

  // Live scale and per-ingredient baseline
  double scale_weight = 0.0;
  (void)config().blackboard->get("scale_weight", scale_weight);
  double baseline_g = 0.0;
  (void)config().blackboard->get("container_baseline_g", baseline_g);

  // Net weight added for this ingredient
  const double net_added = std::max(0.0, scale_weight - baseline_g);
  double remaining = batch_target - net_added;
  if (remaining < 0.0) remaining = 0.0;
  setOutput("remaining", remaining);

  // Logging
  try {
    auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("ros_node");
    RCLCPP_INFO(node->get_logger(), "ComputeRemaining: target=%.3f tol=%.3f scale=%.3f baseline=%.3f net=%.3f -> remaining=%.3f (%s)",
                batch_target, tol, scale_weight, baseline_g, net_added, remaining,
                (remaining <= tol ? "SUCCESS" : "FAILURE"));
  } catch (...) {}

  return (remaining <= tol) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

} // namespace robot_orchestrator


