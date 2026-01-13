#include "robot_moveit/target_recorder.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_moveit::TargetRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}








