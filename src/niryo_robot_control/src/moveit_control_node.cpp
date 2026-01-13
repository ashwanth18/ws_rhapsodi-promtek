#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

class NiryoMoveItControl : public rclcpp::Node
{
public:
  NiryoMoveItControl() : Node("niryo_moveit_control")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing Niryo MoveIt Control Node");
    
    // Initialize MoveIt interfaces
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      std::shared_ptr<rclcpp::Node>(this), "arm");
    planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
    
    // Set planning parameters
    move_group_interface_->setPlanningTime(5.0);
    move_group_interface_->setMaxVelocityScalingFactor(0.5);
    move_group_interface_->setMaxAccelerationScalingFactor(0.5);
    
    // Get robot info
    RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_interface_->getPlanningFrame().c_str());
    RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_interface_->getEndEffectorLink().c_str());
    RCLCPP_INFO(this->get_logger(), "Available planning groups: %s", 
                move_group_interface_->getName().c_str());
    
    // Create timer for periodic actions
    timer_ = this->create_wall_timer(10000ms, std::bind(&NiryoMoveItControl::execute_demo_sequence, this));
    
    // Create service for manual control
    manual_control_service_ = this->create_service<std_srvs::srv::Trigger>(
      "execute_demo", 
      std::bind(&NiryoMoveItControl::manual_control_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    RCLCPP_INFO(this->get_logger(), "Niryo MoveIt Control Node initialized successfully!");
  }

private:
  void execute_demo_sequence()
  {
    RCLCPP_INFO(this->get_logger(), "=== Starting Demo Sequence ===");
    
    // 1. Move to home position
    move_to_home_position();
    
    // 2. Joint space planning
    execute_joint_space_motion();
    
    // 3. Cartesian planning
    execute_cartesian_motion();
    
    // 4. Pose target planning
    execute_pose_target_motion();
    
    // 5. Back to home
    move_to_home_position();
    
    RCLCPP_INFO(this->get_logger(), "=== Demo Sequence Completed ===");
  }
  
  void move_to_home_position()
  {
    RCLCPP_INFO(this->get_logger(), "Moving to home position...");
    
    // Define home position (adjust these values based on your robot)
    std::vector<double> home_joint_values = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    
    move_group_interface_->setJointValueTarget(home_joint_values);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Planning to home position successful");
      move_group_interface_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Moved to home position");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan to home position");
    }
  }
  
  void execute_joint_space_motion()
  {
    RCLCPP_INFO(this->get_logger(), "Executing joint space motion...");
    
    // Define target joint positions (adjust for your robot)
    std::vector<double> target_joint_values = {0.5, 0.3, -0.2, 0.0, 0.5, 0.0};
    
    move_group_interface_->setJointValueTarget(target_joint_values);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Joint space planning successful");
      move_group_interface_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Joint space motion executed");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan joint space motion");
    }
  }
  
  void execute_cartesian_motion()
  {
    RCLCPP_INFO(this->get_logger(), "Executing Cartesian motion...");
    
    // Get current pose
    geometry_msgs::msg::PoseStamped current_pose = move_group_interface_->getCurrentPose();
    
    // Define waypoints for Cartesian path
    std::vector<geometry_msgs::msg::Pose> waypoints;
    
    // Start from current pose
    geometry_msgs::msg::Pose target_pose = current_pose.pose;
    
    // Move up
    target_pose.position.z += 0.1;
    waypoints.push_back(target_pose);
    
    // Move forward
    target_pose.position.x += 0.1;
    waypoints.push_back(target_pose);
    
    // Move down
    target_pose.position.z -= 0.1;
    waypoints.push_back(target_pose);
    
    // Move back
    target_pose.position.x -= 0.1;
    waypoints.push_back(target_pose);
    
    // Execute Cartesian path
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    
    double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    if (fraction > 0.8)
    {
      RCLCPP_INFO(this->get_logger(), "Cartesian path computed successfully (%.2f%% achieved)", fraction * 100.0);
      
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory = trajectory;
      move_group_interface_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Cartesian motion executed");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to compute Cartesian path (%.2f%% achieved)", fraction * 100.0);
    }
  }
  
  void execute_pose_target_motion()
  {
    RCLCPP_INFO(this->get_logger(), "Executing pose target motion...");
    
    // Define target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.4;
    target_pose.orientation.w = 1.0;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    
    move_group_interface_->setPoseTarget(target_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Pose target planning successful");
      move_group_interface_->execute(plan);
      RCLCPP_INFO(this->get_logger(), "Pose target motion executed");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan pose target motion");
    }
  }
  
  void manual_control_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Manual control triggered");
    execute_demo_sequence();
    response->success = true;
    response->message = "Demo sequence executed successfully";
  }
  
  // Member variables
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr manual_control_service_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<NiryoMoveItControl>();
  
  RCLCPP_INFO(node->get_logger(), "Niryo MoveIt Control Node started");
  RCLCPP_INFO(node->get_logger(), "Demo sequence will run every 10 seconds");
  RCLCPP_INFO(node->get_logger(), "You can also trigger it manually via the /execute_demo service");
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  return 0;
} 