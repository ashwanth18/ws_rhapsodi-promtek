#include <behaviortree_cpp/bt_factory.h>
#include "robot_orchestrator/pour_to_target_node.hpp"
#include "robot_orchestrator/move_to_node.hpp"
#include "robot_orchestrator/scan_qr_node.hpp"
#include "robot_orchestrator/has_active_container_node.hpp"
#include "robot_orchestrator/load_next_container_node.hpp"
#include "robot_orchestrator/compute_remaining_node.hpp"
#include "robot_orchestrator/fetch_batch_node.hpp"
#include "robot_orchestrator/project_container_node.hpp"
#include "robot_orchestrator/vibrate_node.hpp"
#include "robot_orchestrator/scale_weight_node.hpp"
#include "robot_orchestrator/queue_status_node.hpp"
#include "robot_orchestrator/compute_effective_target_node.hpp"
#include "robot_orchestrator/wait_seconds_node.hpp"
#include "robot_orchestrator/check_remaining_node.hpp"
#include "robot_orchestrator/weight_fresh_node.hpp"
#include "robot_orchestrator/capture_baseline_node.hpp"
#include "robot_orchestrator/weight_delta_node.hpp"
#include <behaviortree_cpp/decorators/loop_node.h>
#include <behaviortree_cpp/decorators/consume_queue.h>
#include <robot_common_msgs/msg/container_spec.hpp>

namespace robot_orchestrator {

void RegisterNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<PourToTargetNode>("PourToTarget");
  factory.registerNodeType<MoveToNode>("MoveTo");
  factory.registerNodeType<ScanQrNode>("ScanQr");
  // Legacy nodes no longer used in Loop-based tree, keep registered for compatibility
  factory.registerNodeType<HasActiveContainerNode>("HasActiveContainer");
  factory.registerNodeType<LoadNextContainerNode>("LoadNextContainer");
  factory.registerNodeType<ComputeRemainingNode>("ComputeRemaining");
  factory.registerNodeType<FetchBatchNode>("FetchBatch");
  factory.registerNodeType<ProjectContainerNode>("ProjectContainer");
  factory.registerNodeType<VibrateNode>("Vibrate");
  factory.registerNodeType<ScaleWeightNode>("ScaleWeight");
  factory.registerNodeType<QueueStatusNode>("QueueStatus");
  factory.registerNodeType<ComputeEffectiveTargetNode>("ComputeEffectiveTarget");
  factory.registerNodeType<WaitSecondsNode>("WaitSeconds");
  factory.registerNodeType<CheckRemainingNode>("CheckRemaining");
  factory.registerNodeType<WeightFreshNode>("WeightFresh");
  factory.registerNodeType<CaptureBaselineNode>("CaptureBaseline");
  factory.registerNodeType<WeightDeltaNode>("WeightDelta");

  // Register LoopNode for ContainerSpec queue consumption
  factory.registerNodeType<BT::LoopNode<robot_common_msgs::msg::ContainerSpec>>("LoopContainerSpec");
}

} // namespace robot_orchestrator


