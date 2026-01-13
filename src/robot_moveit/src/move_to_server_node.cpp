#include "robot_moveit/move_to_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_moveit::MoveToServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}













