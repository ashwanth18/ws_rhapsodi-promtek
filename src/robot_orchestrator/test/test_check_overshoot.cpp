#include <gtest/gtest.h>

#include <behaviortree_cpp/bt_factory.h>

#include "robot_orchestrator/check_overshoot_node.hpp"
#include "robot_orchestrator/last_failure.hpp"
#include "robot_orchestrator/register.hpp"

namespace {

constexpr const char * kTree = R"(
<root BTCPP_format="4" main_tree_to_execute="T">
  <BehaviorTree ID="T">
    <CheckOvershoot batch_target="{target}" tolerance="{tol}" overshoot="{over}"/>
  </BehaviorTree>
</root>)";

struct Result {
  BT::NodeStatus status;
  double overshoot;
  std::string reason;
};

Result run(double target, double tol, double scale, double baseline)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<robot_orchestrator::CheckOvershootNode>("CheckOvershoot");
  auto bb = BT::Blackboard::create();
  bb->set("target", target);
  bb->set("tol", tol);
  bb->set("scale_weight", scale);
  bb->set("container_baseline_g", baseline);
  robot_orchestrator::clearLastFailureReason(bb);
  auto tree = factory.createTreeFromText(kTree, bb);
  const auto status = tree.tickOnce();
  double over = -1.0;
  (void)bb->get("over", over);
  return {status, over, robot_orchestrator::getLastFailureReason(bb)};
}

}  // namespace

TEST(CheckOvershoot, WithinToleranceSucceeds)
{
  const auto r = run(20.0, 1.0, 120.8, 100.0);
  EXPECT_EQ(r.status, BT::NodeStatus::SUCCESS);
  EXPECT_NEAR(r.overshoot, 0.8, 1e-9);
  EXPECT_TRUE(r.reason.empty());
}

TEST(CheckOvershoot, UnderTargetSucceeds)
{
  const auto r = run(20.0, 1.0, 110.0, 100.0);
  EXPECT_EQ(r.status, BT::NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(r.overshoot, 0.0);
}

TEST(CheckOvershoot, BeyondToleranceFailsWithReason)
{
  const auto r = run(20.0, 1.0, 137.5, 100.0);
  EXPECT_EQ(r.status, BT::NodeStatus::FAILURE);
  EXPECT_NEAR(r.overshoot, 17.5, 1e-9);
  EXPECT_NE(r.reason.find("overshoot"), std::string::npos);
  EXPECT_NE(r.reason.find("37.5 g for a 20.0 g target"), std::string::npos);
}

TEST(CheckOvershoot, WebhookTreeLoads)
{
  BT::BehaviorTreeFactory factory;
  robot_orchestrator::RegisterNodes(factory);
  factory.registerBehaviorTreeFromFile(std::string(BT_TREES_DIR) + "/webhook_weightment.xml");
  rclcpp::init(0, nullptr);
  auto bb = BT::Blackboard::create();
  bb->set("ros_node", std::make_shared<rclcpp::Node>("test_check_overshoot"));
  try {
    auto tree = factory.createTree("WebhookWeightment", bb);
  } catch (const std::exception & e) {
    ADD_FAILURE() << e.what();
  }
  rclcpp::shutdown();
}

TEST(CheckOvershoot, MissingTargetFails)
{
  const auto r = run(0.0, 1.0, 100.0, 100.0);
  EXPECT_EQ(r.status, BT::NodeStatus::FAILURE);
  EXPECT_FALSE(r.reason.empty());
}
