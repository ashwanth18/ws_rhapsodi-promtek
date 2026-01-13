#include "pouring_controller/pour_server.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<pouring_controller::PourServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}













